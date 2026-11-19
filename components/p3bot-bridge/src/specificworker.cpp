/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <limits>
#include <locale>
#include <fstream>
#include "specificworker.h"
#include <set>

#pragma region ROBOCOMP_METHODS

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	delete robot;
}

void SpecificWorker::initialize()
{
    std::cout << "Initialize worker" << std::endl;
    if(this->startup_check_flag)
    {
        this->startup_check();
        return;
    }

    // ── Config ───────────────────────────────────────────────────────────────────────────────────
    try { pars.points3D = configLoader.get<bool>("Camera.ZED.points3D"); } catch(...) {}
    try { pars.humans   = configLoader.get<bool>("Humans"); } catch(...) {}
    try { pars.delay    = configLoader.get<bool>("Delay"); } catch(...) {}
    try { pars.do_joystick = configLoader.get<bool>("Joystick"); } catch(...) {}

    // Source and rate of the published base velocity. Strings must be quoted in etc/config.
    std::string velocity_source = "wheels";
    try { velocity_source = configLoader.get<std::string>("FullPose.VelocitySource"); } catch(...) {}
    fullpose_velocity_source_ = velocity_source == "supervisor" ? VelocitySource::Supervisor
                                                                : VelocitySource::Wheels;
    try { fullpose_publish_period_ms = configLoader.get<int>("FullPose.PublishPeriod"); } catch(...) {}
    if (fullpose_publish_period_ms < 1) fullpose_publish_period_ms = 1;

    // ── Synthetic sensor error — see the header for why the two channels differ ──
    try { sensor_noise_enabled_ = configLoader.get<bool>("SensorNoise.Enabled"); } catch(...) {}
    try { wheel_sigma_v_ = configLoader.get<double>("SensorNoise.WheelSigmaV"); } catch(...) {}
    try { wheel_sigma_v_floor_ = configLoader.get<double>("SensorNoise.WheelSigmaVFloor"); } catch(...) {}
    try { wheel_sigma_w_floor_ = configLoader.get<double>("SensorNoise.WheelSigmaWFloor"); } catch(...) {}
    try { wheel_sigma_w_ = configLoader.get<double>("SensorNoise.WheelSigmaW"); } catch(...) {}
    try { wheel_scale_v_ = configLoader.get<double>("SensorNoise.WheelScaleV"); } catch(...) {}
    try { wheel_scale_w_ = configLoader.get<double>("SensorNoise.WheelScaleW"); } catch(...) {}
    try { gyro_sigma_    = configLoader.get<double>("SensorNoise.GyroSigma"); } catch(...) {}
    try { gyro_bias_     = configLoader.get<double>("SensorNoise.GyroBias"); } catch(...) {}
    try { gyro_scale_    = configLoader.get<double>("SensorNoise.GyroScale"); } catch(...) {}
    try { acc_sigma_     = configLoader.get<double>("SensorNoise.AccSigma"); } catch(...) {}
    if (sensor_noise_enabled_)
        std::printf("[SensorNoise] ON — wheels: sigma_v=%.5g m/sqrt(s) sigma_w=%.5g rad/sqrt(s) "
                    "scale_v=%+.4f scale_w=%+.4f | gyro: sigma=%.5g rad/sqrt(s) bias=%+.5g rad/s "
                    "scale=%+.4f\n"
                    "[SensorNoise] ⚠ this bridge is NO LONGER publishing ground truth; every "
                    "consumer sees the corrupted stream, which is the point.\n",
                    wheel_sigma_v_, wheel_sigma_w_, wheel_scale_v_, wheel_scale_w_,
                    gyro_sigma_, gyro_bias_, gyro_scale_);
    if (sensor_noise_enabled_)
        std::printf("[SensorNoise] acc: sigma=%.5g m/s^2/sqrt(Hz)\n", acc_sigma_);

    try { nominal_gyro_var_ = configLoader.get<double>("IMU.NominalGyroVar"); } catch(...) {}
    try { nominal_acc_var_  = configLoader.get<double>("IMU.NominalAccVar"); } catch(...) {}
    try { nominal_vel_var_  = configLoader.get<double>("FullPose.NominalVelVar"); } catch(...) {}
    try { nominal_rot_var_  = configLoader.get<double>("FullPose.NominalRotVar"); } catch(...) {}

    // Heavy sensors sample independently of the step rate; absent from the config they follow it,
    // which is the historical behaviour.
    const int compute_period = this->getPeriod("Compute");
    sensor_period       = compute_period;
    imu_step_period_ms_ = compute_period;
    try { sensor_period = configLoader.get<int>("Period.Sensors"); } catch(...) {}
    if(sensor_period < compute_period)
    {
        std::cout << "Period.Sensors (" << sensor_period << ") is below Period.Compute ("
                  << compute_period << "); clamping to Period.Compute." << std::endl;
        sensor_period = compute_period;
    }
    std::cout << "Compute period: " << compute_period << " ms, sensor period: "
              << sensor_period << " ms" << std::endl;

    // Per-sensor rates. Webots samples at multiples of the step, so Hz is rounded to the nearest one
    // (step 8 ms: 30 Hz -> 32 ms = 31.25 Hz, 20 Hz -> 48 ms = 20.8 Hz, 10 Hz -> 96 ms = 10.4 Hz).
    const auto hz_to_period = [compute_period](double hz)
    {
        const int steps = std::max(1, static_cast<int>(std::lround(1000.0 / hz / compute_period)));
        return steps * compute_period;
    };
    double zed_hz = 30.0, cam360_hz = 30.0;
    int lidar_profile_hz = 20;
    try { zed_hz = configLoader.get<double>("Sensors.ZedHz"); } catch(...) {}
    try { cam360_hz = configLoader.get<double>("Sensors.Camera360Hz"); } catch(...) {}
    try { lidar_profile_hz = configLoader.get<int>("Sensors.LidarProfileHz"); } catch(...) {}
    if(lidar_profile_hz != 20 and lidar_profile_hz != 10)
    {
        std::cout << "Sensors.LidarProfileHz must be 20 (900 columns) or 10 (1800 columns); using 20." << std::endl;
        lidar_profile_hz = 20;
    }
    zed_period    = hz_to_period(zed_hz);
    cam360_period = hz_to_period(cam360_hz);
    lidar_period  = hz_to_period(lidar_profile_hz);
    std::cout << "Sensor periods: zed " << zed_period << " ms, camera360 " << cam360_period
              << " ms, lidar " << lidar_period << " ms (profile " << lidar_profile_hz << " Hz)" << std::endl;
    lidar_profile_hz_ = lidar_profile_hz;

    // ── Supervisor and robot node ────────────────────────────────────────────────────────────────
    robot = new webots::Supervisor();
    // getSelf(), not getFromDef("..."): play_room_human.wbt instantiates two P3Bots (DEF shadow_0 and
    // DEF human), and each bridge must resolve the one it is the controller of.
    robotNode = robot->getSelf();
    if(robotNode != nullptr)
    {
        // Resolved once: getField() is a name lookup against the simulator.
        robotTranslationField = robotNode->getField("translation");
        robotRotationField    = robotNode->getField("rotation");
    }

    // ── Base motors and encoders ─────────────────────────────────────────────────────────────────
    // Index i is motor "wheel<i+1>", which is also the order the kinematics in
    // apply_pending_speed_command()/update_wheel_odometry() is written against.
    const char *motorNames[4]  = {"wheel1", "wheel2", "wheel3", "wheel4"};
    const char *sensorNames[4] = {"wheel1sensor", "wheel2sensor", "wheel3sensor", "wheel4sensor"};
    int wheels_found = 0;
    for (int i = 0; i < 4; i++)
    {
        motors[i] = robot->getMotor(motorNames[i]);
        if (motors[i] == nullptr) continue;
        ++wheels_found;
        // The proto names the encoders explicitly, so ask for them by name rather than through
        // getPositionSensor(): that is what a world author reading the proto would expect to match.
        ps[i] = robot->getPositionSensor(sensorNames[i]);
        if (ps[i] == nullptr) ps[i] = motors[i]->getPositionSensor();
        // Encoders feed the control loop, so they stay at the STEP rate, not the heavy-sensor rate.
        if (ps[i]) ps[i]->enable(compute_period);
        motors[i]->setPosition(INFINITY);   // velocity mode
        motors[i]->setVelocity(0);
    }
    if (wheels_found == 0)
        std::cout << "No wheel motors in this world — the base interfaces will be inert. "
                     "(Expected in arm_table.wbt.)" << std::endl;
    else if (wheels_found < 4)
        std::cout << "Only " << wheels_found << "/4 wheel motors found — check protos/P3Bot.proto "
                     "against the loaded world." << std::endl;

    // Wheel odometry needs every encoder. Fall back LOUDLY rather than silently, because a quiet
    // revert to the supervisor is indistinguishable from the wheel path not working.
    if (fullpose_velocity_source_ == VelocitySource::Wheels)
        for (int i = 0; i < 4; i++)
            if (ps[i] == nullptr)
            {
                std::cout << "No position sensor for " << motorNames[i]
                          << "; falling back to FullPose.VelocitySource = supervisor." << std::endl;
                fullpose_velocity_source_ = VelocitySource::Supervisor;
                velocity_source = "supervisor";
                break;
            }

    // ── Kinova arms ──────────────────────────────────────────────────────────────────────────────
    // Both arms are optional: arm_table.wbt carries only the right one, and the P3Bot proto's `arms`
    // field can be overridden by a world.
    const auto open_arm = [this, compute_period](const std::string &prefix,
                                                 std::vector<webots::Motor*> &arm_motors,
                                                 std::vector<webots::PositionSensor*> &arm_sensors)
    {
        std::vector<webots::Motor*> m;
        std::vector<webots::PositionSensor*> s;
        for (size_t i = 0; i < kinovaMotorNames.size(); ++i)
        {
            m.push_back(robot->getMotor(prefix + kinovaMotorNames[i]));
            s.push_back(robot->getPositionSensor(prefix + kinovaSensorNames[i]));
        }
        // All-or-nothing: a half-resolved arm is a world/proto mismatch, and pretending to serve it
        // would hand the caller joint angles for the joints that happened to exist.
        if (std::ranges::any_of(m, [](auto *p){ return p == nullptr; }))
        {
            std::cout << "Arm '" << prefix << "' not present in this world." << std::endl;
            return false;
        }
        for (auto *sensor : s)
            if (sensor) sensor->enable(compute_period);
        arm_motors  = std::move(m);
        arm_sensors = std::move(s);
        return true;
    };
    const bool have_arm_r = open_arm("Right_", kinovaArmRMotors, kinovaArmRSensors);
    const bool have_arm_l = open_arm("Left_",  kinovaArmLMotors, kinovaArmLSensors);

    // ── Grippers: slider limits from the arm node, falling back to the proto defaults ──
    const auto open_hand = [this, compute_period](const std::string &prefix, const std::string &arm_def,
                                                  Hand &hand)
    {
        hand.right_motor  = robot->getMotor(prefix + "Hand_LinearMotor_Right");
        hand.left_motor   = robot->getMotor(prefix + "Hand_LinearMotor_Left");
        hand.right_sensor = robot->getPositionSensor(prefix + "Hand_PositionSensor_Right");
        hand.left_sensor  = robot->getPositionSensor(prefix + "Hand_PositionSensor_Left");
        if (not hand.valid()) return;

        if (webots::Node *arm = robot->getFromDef(arm_def); arm != nullptr)
        {
            if (webots::Field *f = arm->getField("armsMinPosition"); f != nullptr)
                hand.min_position = static_cast<float>(f->getSFFloat());
            if (webots::Field *f = arm->getField("armsMaxPosition"); f != nullptr)
                hand.max_position = static_cast<float>(f->getSFFloat());
        }
        else
            std::cout << "No node DEF " << arm_def << "; using the KinovaGen3 proto's default gripper "
                      << "range [" << hand.min_position << ", " << hand.max_position << "] m."
                      << std::endl;

        if (hand.right_sensor) hand.right_sensor->enable(compute_period);
        if (hand.left_sensor)  hand.left_sensor->enable(compute_period);
        // Enough to hold an object, not enough to crush it.
        hand.right_motor->setAvailableForce(40.0);
        hand.left_motor->setAvailableForce(40.0);
    };
    open_hand("Right_", "KINOVA_ARM_R", right_hand);
    open_hand("Left_",  "KINOVA_ARM_L", left_hand);

    // ── Sensors ──────────────────────────────────────────────────────────────────────────────────
    camera360_1    = robot->getCamera("camera_360_1");
    camera360_2    = robot->getCamera("camera_360_2");
    zed            = robot->getCamera("zed");
    zedRangeFinder = robot->getRangeFinder("zed-ranger");
    // Two lidar devices share the same pose: "helios" (900 columns, 20 Hz profile) and "helios_hr"
    // (1800 columns, 10 Hz profile). Only the selected one is enabled, so the other costs nothing.
    heliosLidar    = robot->getLidar(lidar_profile_hz_ == 10 ? "helios_hr" : "helios");
    accelerometer  = robot->getAccelerometer("accelerometer");
    gyroscope      = robot->getGyro("gyro");

    // Heavy sensors run at sensor_period; the IMU is cheap and feeds the control loop, so it stays at
    // the step rate — that rate GAP is the whole reason for having two periods.
    if(camera360_1 and camera360_2)
    {
        camera360_1->enable(cam360_period);
        camera360_2->enable(cam360_period);
    }
    if(zed)            zed->enable(zed_period);
    if(zedRangeFinder) zedRangeFinder->enable(zed_period);
    if(heliosLidar)    heliosLidar->enable(lidar_period);
    if(accelerometer)  accelerometer->enable(compute_period);
    if(gyroscope)      gyroscope->enable(compute_period);
    else std::cout << "No 'gyro' device on this robot — IMU_getAngularVel() will report cov = -1 "
                      "(unknown). Add a Gyro{} to protos/P3Bot.proto's sensors field." << std::endl;

    std::cout << "Base: mecanum (" << wheels_found << " driven wheels, R=" << WHEEL_RADIUS
              << " m, LX=" << LX << " m, LY=" << LY << " m)" << std::endl;
    std::cout << "FullPose: " << velocity_source << "-derived velocity @ "
              << (1000.0 / fullpose_publish_period_ms) << " Hz; IMU @ step rate ("
              << (1000.0 / compute_period) << " Hz nominal)" << std::endl;

    // ── Home poses, applied once to the arms present ──
    // P3BOT_HOME_R/L: arms resting along the body. ⚠ Solved for the OLD tilted arm mounts; must be
    // re-solved (each side on its own, not sign-flipped) for the V2 upright mounts.
    // ARM_TABLE_HOME_R: single arm on a desk (arm_table.wbt), tool pointing down.
    const RoboCompKinovaArm::Angles P3BOT_HOME_R{-1.5985f, -1.1188f, -2.0034f,  0.7738f, -2.5660f,  2.2280f, -1.1933f};
    const RoboCompKinovaArm::Angles P3BOT_HOME_L{-1.5503f,  1.1393f,  1.9669f, -0.7492f,  2.6203f, -2.2280f, -1.9245f};
    const RoboCompKinovaArm::Angles ARM_TABLE_HOME_R{0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 1.4f, 0.0f};

    if (have_arm_r)
    {
        set_gripper_pos(right_hand, 0.f);
        // move_arm_with_angle(have_arm_l ? P3BOT_HOME_R : ARM_TABLE_HOME_R, kinovaArmRMotors, 0.8f);
    }
    if (have_arm_l)
    {
        set_gripper_pos(left_hand, 0.f);
        // move_arm_with_angle(P3BOT_HOME_L, kinovaArmLMotors, 0.8f);
    }

    std::cout << "Worker initialized OK" << std::endl;
}

void SpecificWorker::compute()
{
    // The wall clock every payload's `timestamp` is expressed in. The SIM clock is read separately
    // below; the two are not interchangeable and the difference is what sim_wall_rate_ tracks.
    const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch()).count();

    // Webots only refreshes a device's data once per its sampling period, so when the step rate is
    // faster than sensor_period the extra passes would reprocess the same point clouds and images.
    // Skip them.
    const double sim_now_ms = robot != nullptr ? robot->getTime() * 1000.0 : 0.0;
    const auto due = [sim_now_ms](double& next, int period)
    {
        if(sim_now_ms < next) return false;
        next = sim_now_ms + period;
        return true;
    };
    const bool new_lidar  = due(next_lidar_sample_ms, lidar_period);
    const bool new_cam360 = due(next_cam360_sample_ms, cam360_period);
    const bool new_zed    = due(next_zed_sample_ms, zed_period);

    // The robot pose/speed feeds the control loop, so it is refreshed every step regardless of the
    // sensor rate.
    if(robotNode) receiving_robotSpeed(robot, now);
    if(new_lidar  and heliosLidar)                  receiving_lidarData("helios", heliosLidar, double_buffer_helios, helios_delay_queue, now);
    if(new_cam360 and camera360_1 and camera360_2)  receiving_camera360Data(camera360_1, camera360_2, now);
    if(new_zed    and zed and zedRangeFinder)       receiving_cameraRGBD(zed, zedRangeFinder, now);

    // Refresh the snapshots served by the RPC servants, then drain any work they queued. Everything
    // that talks to Webots happens on this thread.
    update_imu_data(now);
    update_arm_state(now);
    update_object_poses();
    run_webots_tasks();

    // Push any speed command received over RPC since the last step.
    apply_pending_speed_command();

    robot->step(this->getPeriod("Compute"));

    if(pars.humans)
    {
        parseHumanObjects();
        humansMovement();
    }

    fps.print("FPS:");
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

#pragma endregion ROBOCOMP_METHODS

#pragma region DATA_CATCHING

// Webots refreshes a device on multiples of its sampling period, so its sample is up to one
// period older than the loop time.
double SpecificWorker::acquisition_sim_ms(double sim_now_ms, int device_period_ms)
{
    if (device_period_ms <= 0) return sim_now_ms;
    return std::floor(sim_now_ms / device_period_ms) * device_period_ms;
}

// The same instant on the wall clock. The sample is (sim_now - t_acq) of SIMULATION time old, and the
// simulation runs slower than the wall by sim_wall_rate_, so it is that much /sim_wall_rate_ of WALL
// time old.
long SpecificWorker::acquisition_wall_ms(long loop_wall_ms, double sim_now_ms, int device_period_ms) const
{
    const double age_sim_ms = sim_now_ms - acquisition_sim_ms(sim_now_ms, device_period_ms);
    const double rate = sim_wall_rate_ > 0.05 ? sim_wall_rate_ : 1.0;
    return loop_wall_ms - static_cast<long>(age_sim_ms / rate);
}

// Runs in the compute() thread only (from receiving_robotSpeed), so clock_pairs_ needs no lock.
void SpecificWorker::update_sim_wall_rate(double sim_seconds, long wall_ms)
{
    // Trailing 10 s window plus a light EMA: sim lag changes slowly with machine load.
    clock_pairs_.emplace_back(sim_seconds, wall_ms);
    while (clock_pairs_.size() > 2 and (wall_ms - clock_pairs_.front().second) > 10000)
        clock_pairs_.pop_front();

    const double dwall = (wall_ms - clock_pairs_.front().second) / 1000.0;
    const double dsim  = sim_seconds - clock_pairs_.front().first;

    // Until there is a real span to divide, leave the rate at its last value (1.0 at startup). A
    // paused simulation lands here too: dsim is 0, so the last good rate is held.
    if (dwall < 0.5 or dsim <= 0.0)
        return;

    const double r = dsim / dwall;
    // Sane band. Outside it something has gone wrong — a clock jump, a step that blocked for seconds,
    // a resumed pause — and a bogus scale is worse than none, so hold the last good value.
    if (r > 0.05 and r < 2.0)
    {
        constexpr double alpha = 0.05;
        sim_wall_rate_ = (1.0 - alpha) * sim_wall_rate_ + alpha * r;
    }
}

double SpecificWorker::generate_noise(double stddev)
{
    // Reuse the generator to avoid reseeding cost per call.
    static thread_local std::mt19937 gen(std::random_device{}());
    std::normal_distribution<> d(0.0, stddev);
    return d(gen);
}

// Compute thread only. Differences the wheel encoders over sim time across the publish interval
// and inverts the mecanum IK. Returns false (previous velocity held) when there is no usable sample.
bool SpecificWorker::update_wheel_odometry(double sim_seconds, bool publishing)
{
    double angle[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i)
    {
        if (ps[i] == nullptr) { wheel_odo_.valid = false; return false; }
        angle[i] = ps[i]->getValue();
        // NaN until the first robot->step(). compute() calls us BEFORE the step, so cycle 1 always
        // lands here: publish zeros for one cycle rather than a garbage difference.
        if (not std::isfinite(angle[i])) { wheel_odo_.seeded = false; wheel_odo_.valid = false; return false; }
    }

    // Inverse of the mecanum IK; w[1] and w[3] (left wheels) have inverted hinge axes.
    const auto twist_from = [this](const double *anchor_angle, const double *angle_now, double dt)
    {
        double w[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4; ++i)
            w[i] = (angle_now[i] - anchor_angle[i]) / dt;

        const double R = WHEEL_RADIUS, c = LX + LY;
        const double adv  = R * 0.25      * ( w[0] - w[1] + w[2] - w[3]);
        const double side = R * 0.25      * (-w[0] - w[1] + w[2] + w[3]);
        const double rot  = R / (4.0 * c) * ( w[0] + w[1] + w[2] + w[3]);

        // (0) = -side, (1) = forward: the convention receiving_robotSpeed's velocity_local already
        // uses, so nothing downstream has to know where the numbers came from.
        return std::pair{Eigen::Vector2f{static_cast<float>(-side), static_cast<float>(adv)}, rot};
    };

    // Seed the anchor from a real encoder reading (also after a reset), never from zero.
    if (not wheel_odo_.seeded)
    {
        std::copy(std::begin(angle), std::end(angle), std::begin(wheel_odo_.pub_angle));
        wheel_odo_.pub_sim_t  = sim_seconds;
        wheel_odo_.seeded = true;
        wheel_odo_.valid  = false;
        return false;
    }

    if (publishing)
    {
        // dt is not constant: use sim time and only reject dt == 0 (paused simulator).
        const double pub_dt = sim_seconds - wheel_odo_.pub_sim_t;
        if (pub_dt > 1e-4)
        {
            auto [v, rot] = twist_from(wheel_odo_.pub_angle, angle, pub_dt);
            wheel_odo_.v_local = v;
            wheel_odo_.rot     = rot;
            wheel_odo_.valid   = true;
        }
        // Re-anchor even when the sample was rejected, or the next publish differences across the gap.
        std::copy(std::begin(angle), std::end(angle), std::begin(wheel_odo_.pub_angle));
        wheel_odo_.pub_sim_t = sim_seconds;
    }

    return wheel_odo_.valid;
}

void SpecificWorker::receiving_robotSpeed(webots::Supervisor* _robot, long timestamp)
{
    const double* position    = robotNode->getPosition();
    const double* orientation_m = robotNode->getOrientation();
    const double* velocity    = robotNode->getVelocity();

    // getOrientation() gives atan2 = -theta. Forward is local +Y (the proto's +90 deg device wrapper),
    // so heading = theta + pi/2 and M_PI_2 is subtracted, as in the Shadow bridge.
    const float orientation = static_cast<float>(atan2(orientation_m[1], orientation_m[0]) - M_PI_2);

#if BRIDGE_YAWRATE_DIAG
    // See the note on BRIDGE_YAWRATE_DIAG in specificworker.h. yaw_raw is UNSHIFTED (no M_PI_2) so
    // it unwraps cleanly offline; gyro_z is the raw device read, the same value published on the IMU.
    {
        static std::ofstream diag_csv("yawrate_diag.csv");
        static bool hdr = false;
        if (not hdr)
        {
            diag_csv.imbue(std::locale::classic());   // es_ES would emit a comma decimal separator
            diag_csv << "sim_t,wall_ms,yaw_raw,wz_sup,gyro_z,vx_world,vy_world,sim_wall_rate\n";
            hdr = true;
        }
        const double gz = (gyroscope != nullptr and gyroscope->getValues() != nullptr)
                        ? gyroscope->getValues()[2] : std::numeric_limits<double>::quiet_NaN();
        diag_csv << _robot->getTime()
                 << ',' << timestamp
                 << ',' << atan2(orientation_m[1], orientation_m[0])
                 << ',' << velocity[5]
                 << ',' << gz
                 << ',' << velocity[0]
                 << ',' << velocity[1]
                 << ',' << sim_wall_rate_ << '\n';
    }
#endif

    // Velocities are per sim second; simTimestamp carries that clock.
    update_sim_wall_rate(_robot->getTime(), timestamp);
#if BRIDGE_WALL_TIME_VELOCITIES
    const double k_wall = sim_wall_rate_;
#else
    const double k_wall = 1.0;
#endif

    Eigen::Matrix2f rt_rotation_matrix;
    rt_rotation_matrix << cos(orientation), -sin(orientation),
                          sin(orientation),  cos(orientation);

    // Multiply the velocity vector by the inverse of the rotation matrix to get the velocity in the
    // robot reference system. The (y, x) swap pairs with the -θ above; together they are one rotation.
    const Eigen::Vector2f velocity_world(velocity[1], velocity[0]);
    const Eigen::Vector2f velocity_local_sup = static_cast<float>(k_wall)
                                             * (rt_rotation_matrix.inverse() * velocity_world);
    const double rot_velocity_sup = k_wall * velocity[5];

    // OmniRobot keeps the supervisor's values and the step rate: it is the ground-truth reference the
    // wheel channel is measured against, and getBaseState() callers expect a fresh snapshot every
    // cycle regardless of how often FullPose goes out. So refresh it BEFORE the publish gate below.
    update_base_state(position, velocity_local_sup, rot_velocity_sup);

    // FullPose is the WHEEL channel and goes out at its own, slower rate — the IMU keeps the step
    // rate, and that gap is what a consumer's high-rate injection propagates across. Sample the
    // encoders every step regardless, so the per-step anchor stays fresh.
    const double sim_now_ms = _robot->getTime() * 1000.0;
    const bool publishing = sim_now_ms >= next_fullpose_publish_ms;
    update_wheel_odometry(_robot->getTime(), publishing);

    if (not publishing) return;
    next_fullpose_publish_ms = sim_now_ms + fullpose_publish_period_ms;

    // Both channels are per sim second (odometry divides by sim-time deltas).
    Eigen::Vector2f velocity_local = velocity_local_sup;
    double          rot_velocity   = rot_velocity_sup;
    if (fullpose_velocity_source_ == VelocitySource::Wheels)
    {
        // No fallback to the supervisor when the wheel sample is not yet valid: the held value is 0,
        // which is the truth at t=0, and a silent per-cycle source switch would be worse than one
        // cycle of zeros.
        velocity_local = static_cast<float>(k_wall) * wheel_odo_.v_local;
        rot_velocity   = k_wall * wheel_odo_.rot;
    }

    // ── Synthetic wheel error, applied to the published values ──
    // Constant scale plus noise density (sigma/sqrt(dt), dt = publish period).
    if (sensor_noise_enabled_)
    {
        const double dt_pub = std::max(1e-3, fullpose_publish_period_ms * 1e-3);
        const double inv_sqrt_dt = 1.0 / std::sqrt(dt_pub);
        // Wheel noise grows with speed (slip, timing); a stopped wheel reports ~zero:
        //     sigma(v) = sqrt(sigma_floor^2 + (k_slip * |v|)^2). velCov below uses the same sigma.
        const auto sigma_at = [&](double sigma_floor, double k_slip, double v)
        { return std::hypot(sigma_floor, k_slip * v); };
        wheel_sigma_v_eff_ = sigma_at(wheel_sigma_v_floor_, wheel_sigma_v_,
                                      std::hypot(velocity_local.x(), velocity_local.y()));
        wheel_sigma_w_eff_ = sigma_at(wheel_sigma_w_floor_, wheel_sigma_w_, std::abs(rot_velocity));
        velocity_local.x() = static_cast<float>(velocity_local.x() * (1.0 + wheel_scale_v_)
                                                + generate_noise(wheel_sigma_v_eff_ * inv_sqrt_dt));
        velocity_local.y() = static_cast<float>(velocity_local.y() * (1.0 + wheel_scale_v_)
                                                + generate_noise(wheel_sigma_v_eff_ * inv_sqrt_dt));
        rot_velocity = rot_velocity * (1.0 + wheel_scale_w_)
                     + generate_noise(wheel_sigma_w_eff_ * inv_sqrt_dt);
    }

    RoboCompFullPoseEstimation::FullPoseEuler pose_data{};

    // Position, in METRES and in world axes — the units and axes webots-bridge publishes. The previous
    // version emitted (-y, x) in millimetres, which no other bridge in this stack does.
    pose_data.x = static_cast<float>(position[0]);
    pose_data.y = static_cast<float>(position[1]);
    pose_data.z = static_cast<float>(position[2]);

    // Orientation (Euler, radians), 2D.
    pose_data.rx = 0.0;
    pose_data.ry = 0.0;
    pose_data.rz = orientation;

    // Velocities in world frame.
    pose_data.vx = -velocity_local(0);
    pose_data.vy = -velocity_local(1);
    pose_data.vz = 0;
    pose_data.vrx = 0;
    pose_data.vry = 0;
    pose_data.vrz = static_cast<float>(rot_velocity);

    // Velocities in robot frame.
    pose_data.adv  = velocity_local(1);
    pose_data.side = -velocity_local(0);
    pose_data.rot  = static_cast<float>(rot_velocity);

    // timestamp is wall clock (latency); simTimestamp is the sim instant to integrate over.
    pose_data.source       = "p3bot-bridge";
    pose_data.timestamp    = timestamp;
    pose_data.simTimestamp = static_cast<long>(sim_now_ms);
    pose_data.simulated    = true;

    // Nominal config variances plus the synthetic noise actually applied (dt = publish period).
    const double dt_cov = std::max(1e-3, fullpose_publish_period_ms * 1e-3);
    // velCov follows the per-sample sigma(v).
    const double vel_extra = sensor_noise_enabled_
                           ? wheel_sigma_v_eff_ * wheel_sigma_v_eff_ / dt_cov : 0.0;
    const double rot_extra = sensor_noise_enabled_
                           ? wheel_sigma_w_eff_ * wheel_sigma_w_eff_ / dt_cov : 0.0;
    // Same no-zero contract as the IMU block: 0 reads as infinite confidence downstream.
    const auto safe_var_v = [](double v) -> double
    { return (v < 0.0) ? -1.0 : std::max(v, 1e-12); };
    pose_data.velCov.m00 = static_cast<float>(safe_var_v(nominal_vel_var_ + vel_extra));   // vx
    pose_data.velCov.m11 = static_cast<float>(safe_var_v(nominal_vel_var_ + vel_extra));   // vy
    pose_data.velCov.m55 = static_cast<float>(safe_var_v(nominal_rot_var_ + rot_extra));   // vrz
    pose_data.poseCov.m00 = -1.f;
    pose_data.accCov.m00  = -1.f;

    this->fullposeestimationpub_pubproxy->newFullPose(pose_data);
}

void SpecificWorker::update_base_state(const double *position, const Eigen::Vector2f &velocity_local,
                                       double rot_velocity)
{
    RoboCompGenericBase::TBaseState state{};

    state.x = static_cast<float>(position[0]);
    state.z = static_cast<float>(position[1]);
    // The world yaw of the robot's forward axis. Webots keeps the rotation field's angle positive and
    // flips the axis instead, so read the sign off the z component rather than trusting rot[3] alone —
    // otherwise every heading past π comes back mirrored.
    state.alpha = 0.f;
    if (robotRotationField != nullptr)
    {
        const double *rot = robotRotationField->getSFRotation();
        state.alpha = static_cast<float>(rot[2] >= 0 ? rot[3] : -rot[3]);
    }
    state.correctedX = state.x;
    state.correctedZ = state.z;
    state.correctedAlpha = state.alpha;
    state.advVz = velocity_local(1);
    state.advVx = -velocity_local(0);
    state.rotV  = static_cast<float>(rot_velocity);
    state.isMoving = std::hypot(state.advVx, state.advVz) > 0.001f or std::fabs(state.rotV) > 0.001f;

    const std::lock_guard<std::mutex> lock(base_state_mutex);
    base_state = state;
}

void SpecificWorker::receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2, long timestamp)
{
    RoboCompCamera360RGB::TImage newImage360{};

    if (_camera1->getWidth() != _camera2->getWidth() || _camera1->getHeight() != _camera2->getHeight())
    {
        std::cerr << "Error: Cameras with different resolutions." << std::endl;
        return;
    }

    // Acquisition instant, not loop time: both cameras were rendered at the last multiple of their
    // sampling period, so the frame can be up to a full sensor_period old.
    const double cam360_sim_now_ms = robot != nullptr ? robot->getTime() * 1000.0 : 0.0;
    newImage360.timestamp = acquisition_wall_ms(timestamp, cam360_sim_now_ms, cam360_period);

    newImage360.width  = 2 * _camera1->getWidth();
    newImage360.height = _camera1->getHeight();
    newImage360.depth  = 3;
    // The DEVICE's sampling period, which is what tells a consumer how old this frame may be. It used
    // to report fps.get_period() — the loop period, unrelated to when the cameras were rendered.
    newImage360.period = static_cast<float>(cam360_period);

    const unsigned char* webotsImageData1 = _camera1->getImage();
    const unsigned char* webotsImageData2 = _camera2->getImage();
    if (webotsImageData1 == nullptr or webotsImageData2 == nullptr) return;  // no sample yet

    cv::Mat img_1 = cv::Mat(cv::Size(_camera1->getWidth(), _camera1->getHeight()), CV_8UC4);
    cv::Mat img_2 = cv::Mat(cv::Size(_camera2->getWidth(), _camera2->getHeight()), CV_8UC4);
    img_1.data = (uchar *)webotsImageData1;
    cv::cvtColor(img_1, img_1, cv::COLOR_RGBA2RGB);
    img_2.data = (uchar *)webotsImageData2;
    cv::cvtColor(img_2, img_2, cv::COLOR_RGBA2RGB);

    // ── 360 composite: [cam2 tail | cam1 whole | cam2 head] ──
    // Rolled half a camera width so the front camera sits whole in the centre. The front camera
    // must see both arms. Projection must stay spherical: the retina assumes an equirect map.
    const int cw    = _camera1->getWidth();
    const int shift = cw / 2;
    cv::Mat img_final = cv::Mat(cv::Size(cw * 2, _camera1->getHeight()), CV_8UC3);
    img_2(cv::Rect(cw - shift, 0, shift, _camera2->getHeight()))
        .copyTo(img_final(cv::Rect(0, 0, shift, _camera1->getHeight())));
    img_1.copyTo(img_final(cv::Rect(shift, 0, cw, _camera1->getHeight())));
    img_2(cv::Rect(0, 0, cw - shift, _camera2->getHeight()))
        .copyTo(img_final(cv::Rect(shift + cw, 0, cw - shift, _camera1->getHeight())));

    newImage360.image.resize(img_final.total() * img_final.elemSize());
    memcpy(&newImage360.image[0], img_final.data, img_final.total() * img_final.elemSize());
    newImage360.compressed = false;

    if(pars.delay)
        camera_queue.push(newImage360);

    double_buffer_360.put(std::move(newImage360));
}

// Reads the lidar tiltAngle from the scene and logs it. Geometry ignores it (it rotates the
// sweep in azimuth), so a non-zero value means the proto was changed. Expected 0.
static double lidar_tilt_angle(webots::Supervisor* sup, webots::Lidar* dev, const std::string& name)
{
    static std::map<std::string, double> cache;
    if (const auto it = cache.find(name); it != cache.end())
        return it->second;

    double tilt = 0.0;
    const char* how = "absent -> 0";
    if (sup != nullptr and dev != nullptr)
        if (webots::Node* node = sup->getFromDevice(dev); node != nullptr)
        {
            webots::Field* f = node->getField("tiltAngle");
            if (f != nullptr) how = "getField";
            else { f = node->getBaseNodeField("tiltAngle"); if (f != nullptr) how = "getBaseNodeField"; }
            if (f != nullptr) tilt = f->getSFFloat();
        }
    // The MOUNT as well, because "is the cloud upside down?" cannot be answered from the beam
    // geometry alone, and reading it here beats inferring it from a proto file that may not be
    // the one the running world loaded.
    double rot[4] = {0, 0, 0, 0};
    if (sup != nullptr and dev != nullptr)
        if (webots::Node* node = sup->getFromDevice(dev); node != nullptr)
        {
            webots::Field* rf = node->getField("rotation");
            if (rf == nullptr) rf = node->getBaseNodeField("rotation");
            if (rf != nullptr)
                if (const double* v = rf->getSFRotation(); v != nullptr)
                    for (int k = 0; k < 4; ++k) rot[k] = v[k];
        }
    std::cout << "[Lidar] " << name << ": tiltAngle = " << tilt << " rad ("
              << tilt * 180.0 / M_PI << " deg, " << how << ")"
              << "  node rotation = [" << rot[0] << " " << rot[1] << " " << rot[2]
              << "] " << rot[3] << " rad" << std::endl;
    cache[name] = tilt;
    return tilt;
}

void SpecificWorker::receiving_lidarData(const std::string &name, webots::Lidar* _lidar,
                                         DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> &_lidar3dData,
                                         FixedSizeDeque<RoboCompLidar3D::TData>& delay_queue, long timestamp)
{
    if (!_lidar) { std::cout << "No lidar available." << std::endl; return; }

    const float *rangeImage = _lidar->getRangeImage();
    if (rangeImage == nullptr) return;   // no sample yet (first cycle, before any step)

    const int    horizontalResolution = _lidar->getHorizontalResolution();
    const int    verticalResolution   = _lidar->getNumberOfLayers();
    const double fov                  = _lidar->getFov();
    const double angleResolution      = fov / horizontalResolution;
    [[maybe_unused]] const double tiltAngle = lidar_tilt_angle(robot, _lidar, name);
    // ── Beam geometry from the device: fan symmetric about its horizon, mount in the RT edge ──
    const double verticalFov = _lidar->getVerticalFov();

    RoboCompLidar3D::TData newLidar3dData{};

    // Stamp with the lidar's acquisition instant and its own sampling period.
    const double lidar_sim_now_ms = robot != nullptr ? robot->getTime() * 1000.0 : 0.0;
    newLidar3dData.timestamp    = acquisition_wall_ms(timestamp, lidar_sim_now_ms, lidar_period);
    newLidar3dData.simTimestamp = static_cast<long>(acquisition_sim_ms(lidar_sim_now_ms, lidar_period));
    newLidar3dData.simulated    = true;
    newLidar3dData.period       = static_cast<float>(lidar_period);

    newLidar3dData.points.reserve(static_cast<size_t>(horizontalResolution) * verticalResolution);

    // Precompute the angles: sin/cos per point costs ~74k pairs per scan at this resolution.
    std::vector<float> h_angle(horizontalResolution), h_cos(horizontalResolution), h_sin(horizontalResolution);
    for (int i = 0; i < horizontalResolution; ++i)
    {
        const float a = static_cast<float>(M_PI - i * angleResolution - fov / 2.0);
        h_angle[i] = a; h_cos[i] = std::cos(a); h_sin[i] = std::sin(a);
    }
    // N-1 GAPS, not N: the Lidar reference defines verticalFieldOfView as "the angle between FIRST
    // and LAST layer", so the extreme layers sit exactly at +-vfov/2. j = 0 is the TOP beam
    // ("ordered ... from the TOP to the BOTTOM layer").
    std::vector<float> v_angle(verticalResolution), v_cos(verticalResolution), v_sin(verticalResolution);
    const double v_step = (verticalResolution > 1) ? verticalFov / (verticalResolution - 1) : 0.0;
    for (int j = 0; j < verticalResolution; ++j)
    {
        const float a = static_cast<float>(M_PI - verticalFov / 2.0 + j * v_step);
        v_angle[j] = a; v_cos[j] = std::cos(a); v_sin[j] = std::sin(a);
    }

    // Keep only the layers of the real H32F70 (-55..+15 deg) from the wider symmetric fan.
    // No config => all layers. 1e-6 slack keeps the window edges.
    static std::map<std::string, std::pair<double,double>> win_cache;
    if (not win_cache.contains(name))
    {
        double lo = -1e9, hi = 1e9; bool set = false;
        try { lo = configLoader.get<double>("LidarChannels." + name + ".ElevMinDeg"); set = true; } catch(...) {}
        try { hi = configLoader.get<double>("LidarChannels." + name + ".ElevMaxDeg"); set = true; } catch(...) {}
        win_cache[name] = {lo, hi};
        if (set)
            std::cout << "[Lidar] " << name << ": channel window [" << lo << ", " << hi
                      << "] deg — layers outside it are channels the real device does not have"
                      << std::endl;
    }
    const auto [win_lo, win_hi] = win_cache[name];
    std::vector<char> layer_kept(verticalResolution, 1);
    int kept_layers = 0;
    for (int j = 0; j < verticalResolution; ++j)
    {
        const double elev_deg = -(v_angle[j] - M_PI) * 180.0 / M_PI;   // + is UP, device frame
        layer_kept[j] = (elev_deg >= win_lo - 1e-6 and elev_deg <= win_hi + 1e-6) ? 1 : 0;
        kept_layers += layer_kept[j];
    }
    static std::set<std::string> announced;
    if (announced.insert(name).second)
        std::cout << "[Lidar] " << name << ": layers=" << verticalResolution
                  << " vfov=" << verticalFov * 180.0 / M_PI << " deg step=" << v_step * 180.0 / M_PI
                  << " deg tilt=" << tiltAngle * 180.0 / M_PI << " deg (declared, NOT applied)"
                  << " -> device-frame elevation span ["
                  << (-(v_angle[verticalResolution-1] - M_PI)) * 180.0 / M_PI << " .. "
                  << (-(v_angle[0] - M_PI)) * 180.0 / M_PI << "] deg (+ = up)  channels kept "
                  << kept_layers << "/" << verticalResolution << std::endl;

    // h_angle is strictly DECREASING in i, so walking i backwards emits the cloud already sorted by
    // ascending phi. That replaces the std::ranges::sort this used to run over ~74k points every scan.
    for (int i = horizontalResolution - 1; i >= 0; --i)
    {
        for (int j = 0; j < verticalResolution; ++j)
        {
            const float distance = rangeImage[j * horizontalResolution + i];   // metres

            const Eigen::Vector3f lidar_point(distance * h_cos[i] * v_cos[j],
                                              distance * h_sin[i] * v_cos[j],
                                              distance * v_sin[j]);
            // isfinite, not just !isinf: a NaN range would survive an isinf test and then sail
            // through the `r <= 0.2` gate below, since every comparison against NaN is false.
            if (not (std::isfinite(lidar_point.x()) and std::isfinite(lidar_point.y())
                     and std::isfinite(lidar_point.z())))
                continue;

            // The hardcoded helios band is gone — it was tuned against a fabricated fan and against
            // the belief that tiltAngle flips the sensor. The channel window above replaces it.
            if (not layer_kept[j])
                continue;

            RoboCompLidar3D::TPoint point;
            point.r = lidar_point.norm();
            if (point.r <= 0.2f) continue;   // inside this radius the beam is hitting the robot itself

            point.x = lidar_point.x();
            point.y = lidar_point.y();
            point.z = lidar_point.z();
            point.phi   = h_angle[i];
            point.theta = v_angle[j];
            point.distance2d = std::hypot(lidar_point.x(), lidar_point.y());
            newLidar3dData.points.push_back(point);
        }
    }

    if(pars.delay)
        delay_queue.push(newLidar3dData);

    _lidar3dData.put(std::move(newLidar3dData));
}

void SpecificWorker::receiving_cameraRGBD(webots::Camera* _camera, webots::RangeFinder* _rangeFinder,
                                          long timestamp)
{
    RoboCompCameraRGBDSimple::TRGBD new_zed_image{};

    // Acquisition instant and device sampling period (CameraRGBDSimple has no simTimestamp).
    const double zed_sim_now_ms = robot != nullptr ? robot->getTime() * 1000.0 : 0.0;
    const long   zed_acq_ms = acquisition_wall_ms(timestamp, zed_sim_now_ms, zed_period);
    new_zed_image.image.alivetime = new_zed_image.depth.alivetime = new_zed_image.points.alivetime = zed_acq_ms;
    new_zed_image.image.period = new_zed_image.depth.period = new_zed_image.points.period =
        static_cast<float>(zed_period);

    const int width  = _camera->getWidth();
    const int height = _camera->getHeight();
    const double cfov = _camera->getFov();
    const double cfx  = width / (2.0 * tan(cfov / 2.0));
    new_zed_image.image.focalx = cfx;
    new_zed_image.image.focaly = cfx;   // square-pixel assumption
    new_zed_image.image.width  = new_zed_image.depth.width  = width;
    new_zed_image.image.height = new_zed_image.depth.height = height;
    new_zed_image.image.compressed = new_zed_image.depth.compressed = new_zed_image.points.compressed = false;

    // -------------------- RGB --------------------
    const unsigned char *webotsImageData = _camera->getImage();
    if (webotsImageData == nullptr) return;   // no sample yet (first cycle, before any step)
    cv::Mat imageMatBGRA(height, width, CV_8UC4, (void*)webotsImageData);
    cv::Mat imageMatRGB;
    cv::cvtColor(imageMatBGRA, imageMatRGB, cv::COLOR_BGRA2RGB);
    new_zed_image.image.image.resize(imageMatRGB.total() * imageMatRGB.channels());
    new_zed_image.image.depth = imageMatRGB.channels();
    std::memcpy(new_zed_image.image.image.data(), imageMatRGB.data, new_zed_image.image.image.size());

    // -------------------- Depth --------------------
    const double fov = _rangeFinder->getFov();
    const double fx  = width / (2.0 * tan(fov / 2.0));
    new_zed_image.depth.focalx = fx;
    new_zed_image.depth.focaly = fx;
    new_zed_image.depth.width  = _rangeFinder->getWidth();
    new_zed_image.depth.height = _rangeFinder->getHeight();
    new_zed_image.depth.depthFactor = 1;   // metres

    const float* depthImage = _rangeFinder->getRangeImage();
    if (depthImage == nullptr) { qWarning() << "Error: Depth data is null."; return; }
    cv::Mat depthMat(height, width, CV_32FC1, (void*)depthImage);
    new_zed_image.depth.depth.resize(static_cast<size_t>(width) * height * sizeof(float));
    std::memcpy(new_zed_image.depth.depth.data(), depthMat.data, new_zed_image.depth.depth.size());

    // -------------------- Point cloud --------------------
    if(pars.points3D)
    {
        RoboCompCameraRGBDSimple::TPoints& points = new_zed_image.points;
        points.alivetime  = zed_acq_ms;
        points.period     = static_cast<float>(zed_period);
        points.compressed = false;
        constexpr int offset = 2;
        points.points.reserve(static_cast<size_t>(width / offset) * (height / offset));
        const float cx = width / 2.0f, cy = height / 2.0f;
        for (int v = 0; v < height; v += offset)
            for (int u = 0; u < width; u += offset)
            {
                const float Z = depthMat.at<float>(v, u);
                if (std::isnan(Z) || Z <= 0.0f || Z > 10000.0f) continue;
                points.points.emplace_back((u - cx) * Z / fx, (v - cy) * Z / fx, Z);
            }
    }

    double_buffer_zed.put(std::move(new_zed_image));
}

#pragma endregion DATA_CATCHING

#pragma region DEFERRED_WEBOTS_WORK

// Called from servant threads: queue a closure to be run in the Webots thread.
void SpecificWorker::post_webots_task(std::function<void()> task)
{
    const std::lock_guard<std::mutex> lock(webots_tasks_mutex);
    webots_tasks.push_back(std::move(task));
}

// Called from compute(): drain the queue outside the lock, so a task that takes a while cannot block
// servants trying to post new ones.
void SpecificWorker::run_webots_tasks()
{
    std::vector<std::function<void()>> tasks;
    {
        const std::lock_guard<std::mutex> lock(webots_tasks_mutex);
        if (webots_tasks.empty()) return;
        tasks.swap(webots_tasks);
    }
    for (auto &task : tasks)
        task();
}

#pragma endregion DEFERRED_WEBOTS_WORK

#pragma region CAMERA360RGB_INTERFACE

RoboCompCamera360RGB::TImage SpecificWorker::Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
{
    hibernation = true;
    if(pars.delay and camera_queue.full())
        return camera_queue.back();
    return double_buffer_360.get_idemp();
}

#pragma endregion CAMERA360RGB_INTERFACE

#pragma region CAMERARGBDSIMPLE_INTERFACE

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
    hibernation = true;
	return double_buffer_zed.get_idemp();
}

// The partial getters project out the one member they return: copying the whole TRGBD
// (image + depth + points) to hand back a third of it costs several MB per call, and at 1920x1080
// that is the difference between a getDepth() a controller can poll and one it cannot.
RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
    hibernation = true;
	return double_buffer_zed.get_idemp_with([](const auto &rgbd) { return rgbd.depth; });
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
    hibernation = true;
	return double_buffer_zed.get_idemp_with([](const auto &rgbd) { return rgbd.image; });
}

RoboCompCameraRGBDSimple::TPoints SpecificWorker::CameraRGBDSimple_getPoints(std::string camera)
{
    hibernation = true;
	return double_buffer_zed.get_idemp_with([](const auto &rgbd) { return rgbd.points; });
}

#pragma endregion CAMERARGBDSIMPLE_INTERFACE

#pragma region LIDAR3D_INTERFACE

RoboCompLidar3D::TColorCloudData SpecificWorker::Lidar3D_getColorCloudData()
{
	RoboCompLidar3D::TColorCloudData ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
	return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor)
{
    hibernation = true;
    // P3Bot carries one 3D lidar. Naming a different one is a caller-side mistake worth reporting:
    // silently returning the helios cloud would let it go unnoticed for a long time.
    if (not name.empty() and name != "helios")
    {
        std::cout << "Getting data from a not implemented lidar (" << name << "). Try 'helios'." << std::endl;
        return RoboCompLidar3D::TData();
    }
    if (pars.delay and helios_delay_queue.full())
        return helios_delay_queue.back();
    return double_buffer_helios.get_idemp();
}

RoboCompLidar3D::TDataImage SpecificWorker::Lidar3D_getLidarDataArrayProyectedInImage(std::string name)
{
    RoboCompLidar3D::TDataImage ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
    return ret;
}

RoboCompLidar3D::TDataCategory SpecificWorker::Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp)
{
    RoboCompLidar3D::TDataCategory ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
    return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataProyectedInImage(std::string name)
{
	RoboCompLidar3D::TData ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
	return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor)
{
	RoboCompLidar3D::TData ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
	return ret;
}

#pragma endregion LIDAR3D_INTERFACE

#pragma region OMNIROBOT_INTERFACE

void SpecificWorker::OmniRobot_correctOdometer(int x, int z, float alpha)
{
    printNotImplementedWarningMessage(__FUNCTION__);
}

void SpecificWorker::OmniRobot_getBasePose(int &x, int &z, float &alpha)
{
    hibernation = true;

    // Millimetres here (int fields), unlike getBaseState(). Served from the compute() snapshot.
    const std::lock_guard<std::mutex> lock(base_state_mutex);
    x = static_cast<int>(std::lround(base_state.x * 1000.f));
    z = static_cast<int>(std::lround(base_state.z * 1000.f));
    alpha = base_state.alpha;
}

void SpecificWorker::OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
    hibernation = true;

    const std::lock_guard<std::mutex> lock(base_state_mutex);
    state = base_state;
}

void SpecificWorker::OmniRobot_resetOdometer()
{
    printNotImplementedWarningMessage(__FUNCTION__);
}

void SpecificWorker::OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
    printNotImplementedWarningMessage(__FUNCTION__);
}

void SpecificWorker::OmniRobot_setOdometerPose(int x, int z, float alpha)
{
    printNotImplementedWarningMessage(__FUNCTION__);
}

void SpecificWorker::OmniRobot_setSpeedBase(float advx, float advz, float rot)
{
    hibernation = true;

    // Only queue the command. Webots buffers actuator requests until the next wb_robot_step() anyway,
    // so applying it in compute() costs no extra actuation latency and keeps this call at memory-copy
    // speed instead of blocking the servant thread until the simulator answers.
    const std::lock_guard<std::mutex> lock(speed_command_mutex);
    speed_command = {advx, advz, rot};
    speed_command_pending = true;
}

void SpecificWorker::OmniRobot_stopBase()
{
    hibernation = true;

    const std::lock_guard<std::mutex> lock(speed_command_mutex);
    speed_command = {0.f, 0.f, 0.f};
    speed_command_pending = true;
}

// Runs in the compute() thread, just before robot->step().
void SpecificWorker::apply_pending_speed_command()
{
    SpeedCommand cmd;
    {
        const std::lock_guard<std::mutex> lock(speed_command_mutex);
        if (not speed_command_pending) return;
        cmd = speed_command;
        speed_command_pending = false;
    }

    const double side = cmd.advx * 0.001;   // mm/s -> m/s, positive to the robot's RIGHT
    const double adv  = cmd.advz * 0.001;   // mm/s -> m/s, positive forward (local +X)
    const double c    = LX + LY;

    // Mecanum IK. Left wheels (wheel2, wheel4) have inverted hinges: (-adv) rows, all-plus rot.
    // update_wheel_odometry() is its exact inverse; change both together.
    const double speeds[4] = {
        ( adv - side + c * cmd.rot) / WHEEL_RADIUS,   // wheel1  front-right
        (-adv - side + c * cmd.rot) / WHEEL_RADIUS,   // wheel2  front-left   (axis inverted)
        ( adv + side + c * cmd.rot) / WHEEL_RADIUS,   // wheel3  rear-right
        (-adv + side + c * cmd.rot) / WHEEL_RADIUS    // wheel4  rear-left    (axis inverted)
    };

    for (int i = 0; i < 4; i++)
        if (motors[i] != nullptr)
            motors[i]->setVelocity(speeds[i]);
}

#pragma endregion OMNIROBOT_INTERFACE

#pragma region KINOVA_ARM_COMMON

// All four of these run in the Webots thread only, posted from the servants.

void SpecificWorker::move_arm_with_angle(const RoboCompKinovaArm::Angles &jointAngles,
                                         const std::vector<webots::Motor*> &armMotors, float max_velocity)
{
    const size_t loop_limit = std::min(jointAngles.size(), armMotors.size());
    for (size_t i = 0; i < loop_limit; ++i)
    {
        if (armMotors[i] == nullptr) { std::cerr << "Null motor at joint " << i << std::endl; continue; }
        // Position mode: setVelocity is the max speed (0.8 rad/s, below the proto's 0.8727).
        armMotors[i]->setPosition(jointAngles[i]);
        armMotors[i]->setVelocity(max_velocity);
    }
}

void SpecificWorker::move_arm_with_speed(const RoboCompKinovaArm::Speeds &jointSpeeds,
                                         const std::vector<webots::Motor*> &armMotors)
{
    const size_t loop_limit = std::min(jointSpeeds.size(), armMotors.size());
    for (size_t i = 0; i < loop_limit; ++i)
    {
        if (armMotors[i] == nullptr) { std::cerr << "Null motor at joint " << i << std::endl; continue; }
        armMotors[i]->setPosition(INFINITY);            // leave position control for velocity control
        armMotors[i]->setVelocity(jointSpeeds[i]);
    }
}

void SpecificWorker::stop_arm(const std::vector<webots::Motor*> &armMotors)
{
    // Zero velocity in VELOCITY mode, not "hold the current position": a position hold would need a
    // fresh encoder read per joint and would fight whatever the arm was already doing. This is the
    // same thing the base's stopBase does.
    for (auto *motor : armMotors)
        if (motor != nullptr)
        {
            motor->setPosition(INFINITY);
            motor->setVelocity(0.0);
        }
}

void SpecificWorker::set_gripper_pos(const Hand &hand, float pos)
{
    if (not hand.valid()) return;

    // NOTE THE INVERSION, which is the interface's convention and not a bug: the slider's minStop is
    // fingers together and maxStop is fingers apart, so pos = 0 opens the hand and pos = 1 closes it.
    // getGripperState() reports `opening` in these same units so a caller can round-trip it.
    pos = 1.f - std::clamp(pos, 0.0f, 1.0f);
    float target = hand.min_position + pos * (hand.max_position - hand.min_position);

    // Pull the commanded position 1 mm inside the slider's hard limits. Commanding exactly at minStop
    // or maxStop makes the motor integrate error against the joint constraint, producing
    // high-frequency shiver that no amount of damping fully removes.
    constexpr float margin = 0.001f;
    target = std::clamp(target, hand.min_position + margin, hand.max_position - margin);

    hand.right_motor->setPosition(target);
    hand.left_motor->setPosition(target);
}

// Runs in the compute() thread. One pass over both arms' encoders per cycle, so the servants below
// never touch Webots.
void SpecificWorker::sample_arm(const std::vector<webots::PositionSensor*> &sensors,
                                const std::vector<webots::Motor*> &arm_motors,
                                const Hand &hand, long timestamp, ArmSnapshot &out) const
{
    out.joints.timestamp = timestamp;
    out.joints.joints.resize(arm_motors.size());
    for (size_t i = 0; i < arm_motors.size(); ++i)
    {
        RoboCompKinovaArm::TJoint joint{};
        joint.id = static_cast<int>(i);
        if (i < sensors.size() and sensors[i] != nullptr and arm_motors[i] != nullptr)
        {
            joint.angle = static_cast<float>(sensors[i]->getValue());
            joint.velocity = static_cast<float>(arm_motors[i]->getVelocity());
        }
        // torque/current/voltage/temperatures stay zero: Webots exposes none of them for these motors,
        // and inventing a number would be worse than a visible zero.
        out.joints.joints[i] = joint;
    }

    if (not hand.valid()) return;
    const float span = hand.max_position - hand.min_position;
    // MEASURED, not commanded: the proto puts a PositionSensor on each finger slider. Each finger
    // travels on its own axis (the proto's two SliderJoints are axis 1 0 0 and −1 0 0), so the jaw
    // separation is the sum of the two readings.
    const float right = hand.right_sensor != nullptr ? static_cast<float>(hand.right_sensor->getValue())
                                                     : hand.min_position;
    const float left  = hand.left_sensor  != nullptr ? static_cast<float>(hand.left_sensor->getValue())
                                                     : hand.min_position;
    const float mean  = 0.5f * (right + left);
    // Reported in the SAME units setGripperPos() accepts (0 = open, 1 = closed), so the two round-trip.
    out.gripper.opening  = span > 1e-6f ? 1.f - std::clamp((mean - hand.min_position) / span, 0.f, 1.f)
                                        : 0.f;
    out.gripper.distance = right + left;    // jaw separation, metres
    // Forces stay zero: Webots reports no contact force on a LinearMotor without a TouchSensor, and
    // the proto has none.
}

void SpecificWorker::update_arm_state(long timestamp)
{
    ArmSnapshot r, l;
    if (not kinovaArmRMotors.empty()) sample_arm(kinovaArmRSensors, kinovaArmRMotors, right_hand, timestamp, r);
    if (not kinovaArmLMotors.empty()) sample_arm(kinovaArmLSensors, kinovaArmLMotors, left_hand,  timestamp, l);

    const std::lock_guard<std::mutex> lock(arm_state_mutex);
    if (not kinovaArmRMotors.empty()) arm_r_state = std::move(r);
    if (not kinovaArmLMotors.empty()) arm_l_state = std::move(l);
}

#pragma endregion KINOVA_ARM_COMMON

#pragma region KINOVA_ARM_R_INTERFACE

bool SpecificWorker::KinovaArm_closeGripper()
{
    return KinovaArm_setGripperPos(1.f);
}

RoboCompKinovaArm::TPose SpecificWorker::KinovaArm_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo)
{
	RoboCompKinovaArm::TPose ret{};
    // Deliberately not implemented: this needs the arm's forward kinematics, which lives in the
    // controller (Pinocchio) and not in a Webots bridge. Serving a wrong pose would be worse than none.
    printNotImplementedWarningMessage(__FUNCTION__);
	return ret;
}

RoboCompKinovaArm::TGripper SpecificWorker::KinovaArm_getGripperState()
{
    hibernation = true;
    const std::lock_guard<std::mutex> lock(arm_state_mutex);
	return arm_r_state.gripper;
}

RoboCompKinovaArm::TJoints SpecificWorker::KinovaArm_getJointsState()
{
    hibernation = true;
    const std::lock_guard<std::mutex> lock(arm_state_mutex);
    return arm_r_state.joints;
}

RoboCompKinovaArm::TToolInfo SpecificWorker::KinovaArm_getToolInfo()
{
	RoboCompKinovaArm::TToolInfo ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
	return ret;
}

void SpecificWorker::KinovaArm_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles)
{
    hibernation = true;
    post_webots_task([this, angles = std::move(angles.jointAngles)]
                     { move_arm_with_angle(angles, kinovaArmRMotors, 0.8f); });
}

void SpecificWorker::KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds)
{
    hibernation = true;
    post_webots_task([this, speeds = std::move(speeds.jointSpeeds)]
                     { move_arm_with_speed(speeds, kinovaArmRMotors); });
}

void SpecificWorker::KinovaArm_openGripper()
{
    KinovaArm_setGripperPos(0.f);
}

void SpecificWorker::KinovaArm_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo)
{
    // Cartesian servoing belongs in the controller, which owns the IK. See getCenterOfTool.
    printNotImplementedWarningMessage(__FUNCTION__);
}

bool SpecificWorker::KinovaArm_setGripperPos(float pos)
{
    hibernation = true;
    if (not right_hand.valid()) return false;
    post_webots_task([this, pos] { set_gripper_pos(right_hand, pos); });
    return true;
}

void SpecificWorker::KinovaArm_stopArm()
{
    hibernation = true;
    post_webots_task([this] { stop_arm(kinovaArmRMotors); });
}

#pragma endregion KINOVA_ARM_R_INTERFACE

#pragma region KINOVA_ARM_L_INTERFACE

bool SpecificWorker::KinovaArm1_closeGripper()
{
    return KinovaArm1_setGripperPos(1.f);
}

RoboCompKinovaArm::TPose SpecificWorker::KinovaArm1_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo)
{
	RoboCompKinovaArm::TPose ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
	return ret;
}

RoboCompKinovaArm::TGripper SpecificWorker::KinovaArm1_getGripperState()
{
    hibernation = true;
    const std::lock_guard<std::mutex> lock(arm_state_mutex);
	return arm_l_state.gripper;
}

RoboCompKinovaArm::TJoints SpecificWorker::KinovaArm1_getJointsState()
{
    hibernation = true;
    const std::lock_guard<std::mutex> lock(arm_state_mutex);
    return arm_l_state.joints;
}

RoboCompKinovaArm::TToolInfo SpecificWorker::KinovaArm1_getToolInfo()
{
	RoboCompKinovaArm::TToolInfo ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
	return ret;
}

void SpecificWorker::KinovaArm1_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles)
{
    hibernation = true;
    post_webots_task([this, angles = std::move(angles.jointAngles)]
                     { move_arm_with_angle(angles, kinovaArmLMotors, 0.8f); });
}

void SpecificWorker::KinovaArm1_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds)
{
    hibernation = true;
    post_webots_task([this, speeds = std::move(speeds.jointSpeeds)]
                     { move_arm_with_speed(speeds, kinovaArmLMotors); });
}

void SpecificWorker::KinovaArm1_openGripper()
{
    KinovaArm1_setGripperPos(0.f);
}

void SpecificWorker::KinovaArm1_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo)
{
    printNotImplementedWarningMessage(__FUNCTION__);
}

bool SpecificWorker::KinovaArm1_setGripperPos(float pos)
{
    hibernation = true;
    if (not left_hand.valid()) return false;
    post_webots_task([this, pos] { set_gripper_pos(left_hand, pos); });
    return true;
}

void SpecificWorker::KinovaArm1_stopArm()
{
    hibernation = true;
    post_webots_task([this] { stop_arm(kinovaArmLMotors); });
}

#pragma endregion KINOVA_ARM_L_INTERFACE

#pragma region JOYSTICKADAPTER_INTERFACE

//SUBSCRIPTION to sendData method from JoystickAdapter interface
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
    hibernation = true;

    float side = 0, adv = 0, rot = 0;
    for (const RoboCompJoystickAdapter::AxisParams &axis : data.axes)
    {
        if(axis.name == "rotate")
            rot = axis.value;
        else if (axis.name == "advance")
            adv = axis.value;
        else if (axis.name == "side")
            // NOT negated, unlike the version this replaces. `side` is positive to the robot's RIGHT
            // throughout this file (see apply_pending_speed_command), which is webots-bridge's
            // convention too; the old minus here paired with the old 90°-rotated velocity frame.
            side = axis.value;
        else
            std::cout << "[ JoystickAdapter ] Warning: Using a non-defined axes (" << axis.name << ")." << std::endl;
    }
    if(pars.do_joystick)
        OmniRobot_setSpeedBase(side, adv, rot);
}

#pragma endregion JOYSTICKADAPTER_INTERFACE

#pragma region IMU_INTERFACE

// Runs in the compute() thread. The accelerometer and gyro are sampled at the compute period, so a
// per-cycle snapshot loses no information.
void SpecificWorker::update_imu_data(long timestamp)
{
    RoboCompIMU::DataImu data{};

    // IMU sampled every step; gyro is per sim second, integrate over simTimestamp.
    const long sim_ts = robot != nullptr ? static_cast<long>(robot->getTime() * 1000.0) : 0;

    // Diagonal, isotropic, nominal. See the header for why measuring the simulator's own noise would
    // be worse than useless here. Off-diagonals stay zero: uncorrelated is the honest claim.
    const auto diag_cov = [](double var)
    {
        RoboCompIMU::Cov3 c{};
        c.m00 = c.m11 = c.m22 = static_cast<float>(var);
        return c;
    };
    const RoboCompIMU::Cov3 unknown_cov = []{ RoboCompIMU::Cov3 c{}; c.m00 = -1.f; return c; }();

    // ── A PUBLISHED VARIANCE IS NEVER ZERO ────────────────────────────────────────────────────────
    // Zero variance reads downstream as INFINITE confidence, and a consumer weighting by 1/var then
    // divides by zero or hands that channel unlimited authority over the estimate -- silently, with
    // no error anywhere. The contract is therefore: a published variance is either strictly positive
    // or exactly -1, the "producer does not know" sentinel that every consumer here already tests
    // for. It must never be 0, whatever a config file says: NominalGyroVar/NominalAccVar/
    // NominalVelVar are all editable, and "0" is the value someone writes when they mean "perfect".
    // The floor is one part in 10^12 of the quantity, i.e. numerically negligible but never
    // degenerate; anything genuinely unknown should be sent as -1 rather than as a tiny number.
    const auto safe_var = [](double v) -> double
    { return (v < 0.0) ? -1.0 : std::max(v, 1e-12); };

    // Anything this robot does not carry says "unknown" rather than a confident zero.
    data.acc.cov  = unknown_cov;
    data.gyro.cov = unknown_cov;
    data.acc.timestamp = data.gyro.timestamp = data.mag.timestamp = data.rot.timestamp = timestamp;
    data.acc.simTimestamp = data.gyro.simTimestamp = data.mag.simTimestamp = data.rot.simTimestamp = sim_ts;
    data.acc.simulated = data.gyro.simulated = data.mag.simulated = data.rot.simulated = true;

    // ── DEVICE FRAME -> BODY FRAME ────────────────────────────────────────────────────────────────
    // P3Bot.proto puts GROUP_SENSORS inside `Pose { rotation 0 0 1 1.5707963267948966 }`, the same
    // wrapper that makes the body +Y-forward. So every sensor's device axes are rotated +90 deg about
    // z from the body frame, and a Webots device reports in ITS OWN axes. Publishing those raw leaks
    // a construction detail to every consumer: measured on 2026-08-23, room_concept's accelerometer
    // integration found acc_x tracking forward motion and acc_y tracking centripetal (corr -0.796),
    // i.e. exactly this rotation, after the consumer had documented the opposite.
    //
    //   v_body = Rz(+90) * v_device   =>   body_x = -dev_y,  body_y = +dev_x,  body_z = dev_z
    //
    // Doing it HERE and not in each consumer is the point: the device-to-body relationship is a fact
    // about how this robot is built, which the bridge owns and no consumer can know. If the proto's
    // wrapper rotation ever changes, this constant moves with it -- the proto comment already lists
    // the three things that must move together, and this is now a fourth.
    const auto to_body = [](double dx, double dy) { return std::pair{-dy, dx}; };

    if (accelerometer != nullptr)
        if (const double *values = accelerometer->getValues(); values != nullptr)
        {
            // Proper acceleration, gravity included. No scale knob: real error is mount tilt.
            const double dt_step = std::max(1e-3, static_cast<double>(imu_step_period_ms_) * 1e-3);
            auto [ax, ay] = to_body(values[0], values[1]);
            double az = values[2];
            if (sensor_noise_enabled_)
            {
                const double s_step = acc_sigma_ / std::sqrt(dt_step);
                ax += generate_noise(s_step); ay += generate_noise(s_step); az += generate_noise(s_step);
            }
            // Same contract as the gyro: the published variance describes the data ACTUALLY SENT, so
            // a consumer weighting this sample is never handed a covariance that lies.
            const double acc_var = safe_var(nominal_acc_var_
                                 + (sensor_noise_enabled_ ? acc_sigma_ * acc_sigma_ / dt_step : 0.0));
            data.acc = {(float)ax, (float)ay, (float)az,
                        diag_cov(acc_var), timestamp, sim_ts, true};
        }

    if (gyroscope != nullptr)
        if (const double *values = gyroscope->getValues(); values != nullptr)
        {
            // ── Synthetic gyro error: noise and slow bias, no scale (dt = step period) ──
            double gz = values[2];
            const double dt_step = std::max(1e-3, static_cast<double>(imu_step_period_ms_) * 1e-3);
            if (sensor_noise_enabled_)
            {
                // Scale first, then bias and noise, so the injected scale is exactly the configured one.
                gz *= (1.0 + gyro_scale_);
                gz += gyro_bias_ + generate_noise(gyro_sigma_ / std::sqrt(dt_step));
            }
            // The published variance must describe the data actually sent: a consumer weighting this
            // by the nominal figure while the samples carry injected noise would be given a covariance
            // that lies, and every downstream precision would be wrong in the same direction.
            const double gyro_var = safe_var(nominal_gyro_var_
                                  + (sensor_noise_enabled_ ? gyro_sigma_ * gyro_sigma_ / dt_step : 0.0));
            // gz needs no rotation: a rotation about z leaves the z component invariant, which is why
            // heading has been correct all along while the linear axes were not.
            const auto [gx_b, gy_b] = to_body(values[0], values[1]);
            data.gyro = {(float)gx_b, (float)gy_b, (float)gz,
                         diag_cov(gyro_var), timestamp, sim_ts, true};
        }

    if (robotNode != nullptr)
    {
        // The supervisor's orientation, standing in for the internal fusion a real IMU runs. It is
        // exact, hence "unknown" rather than a nominal variance — claiming a number here would be
        // inventing one, and a zero would read as infinite confidence.
        const auto [roll, pitch, yaw] = rotationMatrixToEulerZYX(robotNode->getOrientation());
        data.rot = {roll, pitch, yaw, unknown_cov, timestamp, sim_ts, true};
    }

    // Never populated by this bridge.
    data.mag.cov = unknown_cov;
    data.temperature = 0.f;

    const std::lock_guard<std::mutex> lock(imu_mutex);
    imu_data = data;
}

RoboCompIMU::Acceleration SpecificWorker::IMU_getAcceleration()
{
    hibernation = true;
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data.acc;
}

RoboCompIMU::Gyroscope SpecificWorker::IMU_getAngularVel()
{
    hibernation = true;
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data.gyro;
}

RoboCompIMU::DataImu SpecificWorker::IMU_getDataImu()
{
    hibernation = true;
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data;
}

RoboCompIMU::Magnetic SpecificWorker::IMU_getMagneticFields()
{
    RoboCompIMU::Magnetic ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
    return ret;
}

RoboCompIMU::Orientation SpecificWorker::IMU_getOrientation()
{
    hibernation = true;
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data.rot;
}

void SpecificWorker::IMU_resetImu()
{
    printNotImplementedWarningMessage(__FUNCTION__);
}

// Note this returns +yaw, whereas the FullPose `rz` published by receiving_robotSpeed is -heading.
// The two conventions are different on purpose and both are long-standing; do not "unify" one into
// the other without checking every consumer.
std::tuple<float, float, float> SpecificWorker::rotationMatrixToEulerZYX(const double* R)
{
    float roll, pitch, yaw;

    if (R[6] < 1.0) {
        if (R[6] > -1.0) {
            pitch = std::asin(-R[6]);
            roll  = std::atan2(R[7], R[8]);
            yaw   = std::atan2(R[3], R[0]);
        } else {
            // R[6] == -1
            pitch = M_PI / 2.0;
            roll  = -std::atan2(-R[1], R[4]);
            yaw   = 0.0;
        }
    } else {
        // R[6] == +1
        pitch = -M_PI / 2.0;
        roll  = std::atan2(-R[1], R[4]);
        yaw   = 0.0;
    }

    return {roll, pitch, yaw};
}

#pragma endregion IMU_INTERFACE

#pragma region WEBOTS2ROBOCOMP_INTERFACE

static RoboCompWebots2Robocomp::Quaternion axisAngleToQuaternion(const webots::Field &rotation)
{
    RoboCompWebots2Robocomp::Quaternion q;
    double rx = rotation.getSFRotation()[0];
    double ry = rotation.getSFRotation()[1];
    double rz = rotation.getSFRotation()[2];
    const double angle = rotation.getSFRotation()[3];

    if (const double norm = std::sqrt(rx*rx + ry*ry + rz*rz); norm > 0) {
        rx /= norm; ry /= norm; rz /= norm;
    }
    const double halfAngle = angle * 0.5;
    const double sinHalf   = std::sin(halfAngle);
    q.w = std::cos(halfAngle);
    q.x = rx * sinHalf;
    q.y = ry * sinHalf;
    q.z = rz * sinHalf;
    return q;
}

// Webots thread only. DEF first, then a top-level node whose "name" field equals the key.
webots::Node* SpecificWorker::find_scene_node(const std::string& key)
{
    if (auto it = scene_node_cache_.find(key); it != scene_node_cache_.end())
        return it->second;

    webots::Node* node = robot->getFromDef(key);

    if (node == nullptr)
        if (webots::Field* children = robot->getRoot()->getField("children"); children != nullptr)
        {
            const int count = children->getCount();
            for (int i = 0; i < count; ++i)
            {
                webots::Node* child = children->getMFNode(i);
                if (child == nullptr) continue;
                webots::Field* name_field = child->getField("name");
                if (name_field == nullptr) continue;
                if (name_field->getSFString() == key) { node = child; break; }
            }
        }

    if (node != nullptr) scene_node_cache_[key] = node;  // cache hits only
    return node;
}

// Runs in the compute() thread: resolve any newly requested key, then refresh every tracked pose. The
// Webots calls happen with no lock held, so servants reading the cache are never blocked by them.
void SpecificWorker::update_object_poses()
{
    std::vector<std::string> defs;
    {
        const std::lock_guard<std::mutex> lock(object_poses_mutex);
        if (object_poses.empty()) return;
        defs.reserve(object_poses.size());
        for (const auto &[def, _] : object_poses)
            defs.push_back(def);
    }

    std::vector<TrackedObject> refreshed(defs.size());
    for (size_t i = 0; i < defs.size(); ++i)
    {
        const std::string &def = defs[i];
        refreshed[i].resolved = true;   // resolved means "compute() has tried", not "found"

        auto handles = object_handles.find(def);
        if (handles == object_handles.end())
        {
            TrackedObjectHandles h;
            if (webots::Node *node = find_scene_node(def); node != nullptr)
            {
                h.translation = node->getField("translation");
                h.rotation    = node->getField("rotation");
                if (h.translation == nullptr or h.rotation == nullptr)
                    std::cout << "Object '" << def << "' has no translation or rotation field" << std::endl;
            }
            else
                std::cout << "Object with DEF or name '" << def << "' not found" << std::endl;

            handles = object_handles.emplace(def, h).first;
        }

        if (handles->second.translation == nullptr or handles->second.rotation == nullptr)
            continue;   // leaves a zeroed pose

        const double *position = handles->second.translation->getSFVec3f();
        refreshed[i].pose.position = RoboCompWebots2Robocomp::Vector3{
            static_cast<float>(position[0] * 1000),
            static_cast<float>(position[1] * 1000),
            static_cast<float>(position[2] * 1000),
        };
        refreshed[i].pose.orientation = axisAngleToQuaternion(*handles->second.rotation);
    }

    {
        const std::lock_guard<std::mutex> lock(object_poses_mutex);
        for (size_t i = 0; i < defs.size(); ++i)
            object_poses[defs[i]] = refreshed[i];
    }
    object_poses_cv.notify_all();
}

RoboCompWebots2Robocomp::ObjectPose SpecificWorker::Webots2Robocomp_getObjectPose(std::string DEF)
{
    hibernation = true;

    std::unique_lock<std::mutex> lock(object_poses_mutex);
    TrackedObject &tracked = object_poses[DEF];   // registers the key on first use
    if (tracked.resolved)
        return tracked.pose;

    // First ever request for this key: nothing cached yet, so wait for one compute() cycle to resolve
    // it. Bounded, and only ever paid once per key — every later call is a map lookup.
    object_poses_cv.wait_for(lock, std::chrono::seconds(1), [&tracked] { return tracked.resolved; });
    if (not tracked.resolved)
        std::cout << "Timed out resolving object with DEF or name '" << DEF << "'" << std::endl;
    return tracked.pose;
}

void SpecificWorker::Webots2Robocomp_setObjectPose(std::string DEF, RoboCompWebots2Robocomp::ObjectPose pose)
{
    hibernation = true;
    // Deferred: this writes to the scene tree, which only the Webots thread may do.
    post_webots_task([this, DEF = std::move(DEF), pose]
    {
        webots::Node *node = find_scene_node(DEF);
        if (node == nullptr)
        {
            std::cerr << "[Error] setObjectPose: no node with DEF or name '" << DEF << "'.\n";
            return;
        }
        webots::Field *t = node->getField("translation");
        webots::Field *r = node->getField("rotation");
        if (t == nullptr or r == nullptr)
        {
            std::cerr << "[Error] setObjectPose: '" << DEF << "' has no translation/rotation field.\n";
            return;
        }
        // getObjectPose reports millimetres, so accept them here too — a caller must be able to feed
        // back what it just read.
        const double xyz[3] = {pose.position.x / 1000.0, pose.position.y / 1000.0, pose.position.z / 1000.0};
        t->setSFVec3f(xyz);

        // Quaternion -> Webots axis-angle. A near-identity quaternion has an ill-conditioned axis, so
        // fall back to +Z with angle 0 rather than dividing by a vanishing sin(θ/2).
        const double n = std::sqrt(pose.orientation.x * pose.orientation.x +
                                   pose.orientation.y * pose.orientation.y +
                                   pose.orientation.z * pose.orientation.z);
        double axis_angle[4] = {0.0, 0.0, 1.0, 0.0};
        if (n > 1e-9)
        {
            const double w = std::clamp<double>(pose.orientation.w, -1.0, 1.0);
            axis_angle[0] = pose.orientation.x / n;
            axis_angle[1] = pose.orientation.y / n;
            axis_angle[2] = pose.orientation.z / n;
            axis_angle[3] = 2.0 * std::atan2(n, w);
        }
        r->setSFRotation(axis_angle);
    });
}

void SpecificWorker::Webots2Robocomp_resetWebots()
{
    hibernation = true;
    // Deferred. simulationReset() keeps node handles valid but rewinds encoders and clock,
    // so the odometry anchors are dropped.
    post_webots_task([this]
    {
        std::cout << "[Info] resetWebots: restoring the world to its initial state." << std::endl;
        wheel_odo_ = WheelOdometry{};
        clock_pairs_.clear();
        sim_wall_rate_ = 1.0;
        next_zed_sample_ms = next_cam360_sample_ms = next_lidar_sample_ms = 0.0;
        next_fullpose_publish_ms = 0.0;
        robot->simulationReset();
    });
}

void SpecificWorker::Webots2Robocomp_setArmJointsInstant(RoboCompKinovaArm::TJointAngles angles)
{
    hibernation = true;
    // "Instant" = position command at the proto's maxVelocity (joints have no DEF to teleport).
    post_webots_task([this, angles = std::move(angles.jointAngles)]
                     { move_arm_with_angle(angles, kinovaArmRMotors, 0.8727f); });
}

void SpecificWorker::Webots2Robocomp_setDoorAngle(std::string DEF, float angle)
{
    hibernation = true;
    // The DEF is copied into the task: the caller's string is gone by the time the supervisor thread
    // runs this, and the lookup itself must happen there.
    post_webots_task([this, DEF = std::move(DEF), angle] { setDoorAperture(DEF, angle); });
}

// Open/close the hinged door named by its Webots DEF. Runs on the supervisor thread. An unknown DEF,
// or a node with no `position` field, is reported and ignored — a request to move a door that is not
// there must fail loudly and change nothing, never silently move a different one.
void SpecificWorker::setDoorAperture(const std::string &DEF, float _aperture)
{
    if (DEF.empty()) { std::cerr << "[Error] setDoorAngle called with an empty DEF.\n"; return; }

    // Resolved once per DEF and cached: getFromDef walks the scene tree, and this can be called every
    // control cycle while an affordance servos the door open. A null result is cached too — it means
    // "this scene has no such node", which does not change while the world is loaded.
    auto it = doorNodes.find(DEF);
    if (it == doorNodes.end())
        it = doorNodes.emplace(DEF, robot->getFromDef(DEF)).first;
    webots::Node *door = it->second;
    if (door == nullptr)
    {
        std::cerr << "[Error] no node with DEF '" << DEF << "' in this world — door not moved.\n";
        return;
    }

    constexpr float minAperture = -static_cast<float>(M_PI);
    constexpr float maxAperture = 0.0f;
    if (_aperture < minAperture || _aperture > maxAperture)
        std::cout << "[Warning] The aperture value " << _aperture << " is out of the allowed range ["
                  << minAperture << ", " << maxAperture << "]. It will be clamped automatically.\n";
    const float aperture = std::clamp(_aperture, minAperture, maxAperture);

    // `position` lives on a HingeJoint's jointParameters — a DEF that names the door's Solid instead of
    // its joint resolves above and fails here, so the message names the DEF: that is the difference
    // between "wrong door" and "wrong node in the door".
    webots::Field *positionField = door->getField("position");
    if (positionField == nullptr)
    {
        std::cerr << "[Error] node '" << DEF << "' has no 'position' field — it is not a hinge joint "
                     "(is this the door's Solid rather than its HingeJoint?).\n";
        return;
    }
    positionField->setSFFloat(aperture);
    std::cout << "[Info] Door '" << DEF << "' aperture set to " << aperture << " rad.\n";
}

void SpecificWorker::Webots2Robocomp_setPathToHuman(int humanId, RoboCompGridder::TPath path)
{
    hibernation = true;
    // Deferred: this both reads the supervisor and mutates humanObjects, which compute() rewrites.
    // Running it there makes the call return immediately and removes the race on the map.
    post_webots_task([this, humanId, path = std::move(path)] { set_path_to_human(humanId, path); });
}

void SpecificWorker::set_path_to_human(int humanId, const RoboCompGridder::TPath &path)
{
    auto it = humanObjects.find(humanId);
    if (it == humanObjects.end() or it->second.node == nullptr)
    {
        qInfo() << "Human" << humanId << "not found (is Humans=true and does the world have a CROWD node?)";
        return;
    }
    if (not it->second.path.empty() or path.size() <= 3) return;
    if (robotTranslationField == nullptr or robotRotationField == nullptr) return;

    // The path arrives in the ROBOT's frame, in mm; the humans live in world coordinates, in metres.
    const auto x = robotTranslationField->getSFVec3f()[0] * 1000;
    const auto y = robotTranslationField->getSFVec3f()[1] * 1000;
    const auto rot = robotRotationField->getSFRotation();

    const Eigen::Vector3d axis(rot[0], rot[1], rot[2]);
    const Eigen::AngleAxisd angleAxis(rot[3], axis.normalized());
    const Eigen::Vector3d eulerAngles = Eigen::Quaterniond(angleAxis).toRotationMatrix().eulerAngles(0, 1, 2);
    const auto tf = create_affine_matrix(eulerAngles.x(), eulerAngles.y(), eulerAngles.z(),
                                         Eigen::Vector3d{x, y, 0});

    RoboCompGridder::TPath transformed_path;
    transformed_path.reserve(path.size());
    for(const auto &p : path)
    {
        const Eigen::Vector2f tf_point = (tf * Eigen::Vector4d(p.y, -p.x, 0.0, 1.0)).head(2).cast<float>();
        transformed_path.emplace_back(RoboCompGridder::TPoint(tf_point.x() / 1000, tf_point.y() / 1000, 100.0));
    }
    it->second.path = std::move(transformed_path);
}

// Webots thread. Finds the DEF HUMAN_* children of a DEF CROWD node, if this world has one.
void SpecificWorker::parseHumanObjects()
{
    webots::Node* crowdNode = robot->getFromDef("CROWD");
    if(crowdNode == nullptr)
    {
        static bool reported = false;
        if(not reported) { qInfo() << "CROWD node not found; Humans is on but this world has no crowd."; reported = true; }
        return;
    }

    webots::Field* childrenField = crowdNode->getFieldByIndex(0);
    if (childrenField == nullptr) return;
    for (int i = 0; i < childrenField->getCount(); ++i)
    {
        webots::Node *child = childrenField->getMFNode(i);
        if (child == nullptr) continue;
        if (child->getDef().find("HUMAN_") != std::string::npos)
            humanObjects[i].node = child;
    }
}

// Webots thread. Drives each human one step along its path by setting its velocity directly — these
// are kinematic scene nodes, not robots with controllers of their own.
void SpecificWorker::humansMovement()
{
    for (auto &[id, human] : humanObjects)
    {
        if (human.node == nullptr) continue;

        double velocity[6] = {0, 0, 0, 0, 0, 0};
        if (not human.path.empty())
        {
            const double *position = human.node->getPosition();
            Eigen::Vector3d target{human.path.front().x - position[0], human.path.front().y - position[1], 0};

            // Within 300 mm the waypoint counts as reached.
            if (target.norm() < 0.3)
                human.path.erase(human.path.begin());
            else
            {
                target.normalize();
                velocity[0] = target.x();
                velocity[1] = target.y();
            }
        }
        human.node->setVelocity(velocity);
    }
}

Matrix4d SpecificWorker::create_affine_matrix(double a, double b, double c, Vector3d trans)
{
    Transform<double, 3, Eigen::Affine> t;
    t = Translation<double, 3>(trans);
    t.rotate(AngleAxis<double>(a, Vector3d::UnitX()));
    t.rotate(AngleAxis<double>(b, Vector3d::UnitY()));
    t.rotate(AngleAxis<double>(c, Vector3d::UnitZ()));
    return t.matrix();
}

#pragma endregion WEBOTS2ROBOCOMP_INTERFACE

void SpecificWorker::printNotImplementedWarningMessage(const std::string &functionName)
{
    std::cout << "Function not implemented used: [" << functionName << "]" << std::endl;
}

/**************************************/
// From the RoboCompFullPoseEstimationPub you can publish calling this methods:
// RoboCompFullPoseEstimationPub::void this->fullposeestimationpub_pubproxy->newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::Point3D
// RoboCompCameraRGBDSimple::TPoints
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompIMU you can use this types:
// RoboCompIMU::Acceleration
// RoboCompIMU::Gyroscope
// RoboCompIMU::Magnetic
// RoboCompIMU::Orientation
// RoboCompIMU::DataImu

/**************************************/
// From the RoboCompKinovaArm you can use this types:
// RoboCompKinovaArm::TPose
// RoboCompKinovaArm::TAxis
// RoboCompKinovaArm::TToolInfo
// RoboCompKinovaArm::TGripper
// RoboCompKinovaArm::TJoint
// RoboCompKinovaArm::TJoints
// RoboCompKinovaArm::TJointSpeeds
// RoboCompKinovaArm::TJointAngles

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompWebots2Robocomp you can use this types:
// RoboCompWebots2Robocomp::Vector3
// RoboCompWebots2Robocomp::Quaternion
// RoboCompWebots2Robocomp::ObjectPose

/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData
