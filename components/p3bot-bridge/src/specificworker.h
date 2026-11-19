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

/**
	\brief Bridge component that connects Webots with Robocomp for the P3Bot robot
	@author Robolab Group | Sergio Eslava & Alejandro Torrejón & Jorge Castellón
*/
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// Hibernation stays off: this bridge drives robot->step(), so hibernating stalls the whole world.
//#define HIBERNATION_ENABLED

//##################################
//#########   INCLUDES   ###########
//##################################
#include <genericworker.h>
#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Lidar.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Device.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "fixedsizedeque.h"
#include <doublebuffer/DoubleBuffer.h>
#include <fps/fps.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <random>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

//##################################
//#####   OTHER DEFINITIONS   ######
//##################################
using namespace std;
using namespace Eigen;

// ── Base geometry: must match protos/P3Bot.proto (device frame, +X forward) ─────────────────────
// LX/LY are half-distances between wheel centres; WHEEL_RADIUS is the roller envelope.
#define WHEEL_RADIUS 0.0765
#define LX 0.126
#define LY 0.25577
// Motors wheel1..wheel4 = front-right, front-left, rear-right, rear-left (not the URDF numbering).
// The left wheels have an inverted hinge axis, hence the (+, -, +, -) forward row in the kinematics.

// Publish velocities per wall-second instead of per sim-second (old contract). Off: payloads carry simTimestamp.
#ifndef BRIDGE_WALL_TIME_VELOCITIES          // -DBRIDGE_WALL_TIME_VELOCITIES=n overrides
#define BRIDGE_WALL_TIME_VELOCITIES 0
#endif

// Set to 1 to write yawrate_diag.csv (supervisor yaw rate vs gyro) to locate rotation scale errors.
// is exactly 1440 deg. At that baseline a 3.3% error is 47 deg and cannot be confused with noise.
#ifndef BRIDGE_YAWRATE_DIAG                  // -DBRIDGE_YAWRATE_DIAG=1 turns it on
#define BRIDGE_YAWRATE_DIAG 1
#endif

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();

    // ##########################
    // # Camera360RGB interface #
    // ##########################
	RoboCompCamera360RGB::TImage Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight);

    // ##############################
    // # CameraRGBDSimple interface #
    // ##############################
	RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera);
	RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera);
	RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera);
	RoboCompCameraRGBDSimple::TPoints CameraRGBDSimple_getPoints(std::string camera);

    // #################
    // # IMU interface #
    // #################
	RoboCompIMU::Acceleration IMU_getAcceleration();
	RoboCompIMU::Gyroscope IMU_getAngularVel();
	RoboCompIMU::DataImu IMU_getDataImu();
	RoboCompIMU::Magnetic IMU_getMagneticFields();
	RoboCompIMU::Orientation IMU_getOrientation();
	void IMU_resetImu();

    // ##########################
    // # KinovaArm_R interface  #
    // ##########################
	bool KinovaArm_closeGripper();
	RoboCompKinovaArm::TPose KinovaArm_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo);
	RoboCompKinovaArm::TGripper KinovaArm_getGripperState();
	RoboCompKinovaArm::TJoints KinovaArm_getJointsState();
	RoboCompKinovaArm::TToolInfo KinovaArm_getToolInfo();
	void KinovaArm_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles);
	void KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds);
	void KinovaArm_openGripper();
	void KinovaArm_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo);
	bool KinovaArm_setGripperPos(float pos);
	void KinovaArm_stopArm();

    // ##########################
    // # KinovaArm_L interface  #
    // ##########################
	bool KinovaArm1_closeGripper();
	RoboCompKinovaArm::TPose KinovaArm1_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo);
	RoboCompKinovaArm::TGripper KinovaArm1_getGripperState();
	RoboCompKinovaArm::TJoints KinovaArm1_getJointsState();
	RoboCompKinovaArm::TToolInfo KinovaArm1_getToolInfo();
	void KinovaArm1_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles);
	void KinovaArm1_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds);
	void KinovaArm1_openGripper();
	void KinovaArm1_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo);
	bool KinovaArm1_setGripperPos(float pos);
	void KinovaArm1_stopArm();

    // #####################
    // # LIDAR3D interface #
    // #####################
	RoboCompLidar3D::TColorCloudData Lidar3D_getColorCloudData();
	RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor);
	RoboCompLidar3D::TDataImage Lidar3D_getLidarDataArrayProyectedInImage(std::string name);
	RoboCompLidar3D::TDataCategory Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp);
	RoboCompLidar3D::TData Lidar3D_getLidarDataProyectedInImage(std::string name);
	RoboCompLidar3D::TData Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor);

    // #######################
    // # OMNIROBOT interface #
    // #######################
	void OmniRobot_correctOdometer(int x, int z, float alpha);
	void OmniRobot_getBasePose(int &x, int &z, float &alpha);
	void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
	void OmniRobot_resetOdometer();
	void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state);
	void OmniRobot_setOdometerPose(int x, int z, float alpha);
	void OmniRobot_setSpeedBase(float advx, float advz, float rot);
	void OmniRobot_stopBase();

    // ##############################
    // # JoystickAdapter interface  #
    // ##############################
	void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

    // ###############################
    // # Webots2Robocomp interface   #
    // ###############################
	RoboCompWebots2Robocomp::ObjectPose Webots2Robocomp_getObjectPose(std::string DEF);
	void Webots2Robocomp_resetWebots();
	void Webots2Robocomp_setArmJointsInstant(RoboCompKinovaArm::TJointAngles angles);
	void Webots2Robocomp_setDoorAngle(std::string DEF, float angle);
	void Webots2Robocomp_setObjectPose(std::string DEF, RoboCompWebots2Robocomp::ObjectPose pose);
	void Webots2Robocomp_setPathToHuman(int humanId, RoboCompGridder::TPath path);

public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

private:
    /**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

    // ── Webots devices ───────────────────────────────────────────────────────────────────────────
    // Every one of these may be nullptr: arm_table.wbt loads a bare Robot named "P3Bot" carrying only
    // the right arm, and the bridge is expected to run against it unchanged. Guard, never assume.
    webots::Supervisor* robot = nullptr;
    webots::Node*       robotNode = nullptr;
    webots::Field*      robotTranslationField = nullptr;
    webots::Field*      robotRotationField = nullptr;

    webots::Motor*          motors[4]{nullptr, nullptr, nullptr, nullptr};
    webots::PositionSensor* ps[4]{nullptr, nullptr, nullptr, nullptr};

    std::vector<webots::Motor*>          kinovaArmRMotors;
    std::vector<webots::PositionSensor*> kinovaArmRSensors;
    std::vector<webots::Motor*>          kinovaArmLMotors;
    std::vector<webots::PositionSensor*> kinovaArmLSensors;

    webots::Camera*      camera360_1 = nullptr;
    webots::Camera*      camera360_2 = nullptr;
    webots::Camera*      zed = nullptr;
    webots::RangeFinder* zedRangeFinder = nullptr;
    webots::Lidar*       heliosLidar = nullptr;
    webots::Accelerometer* accelerometer = nullptr;
    webots::Gyro*          gyroscope = nullptr;

    // Gripper: a pair of opposed SliderJoints per hand, no DEF needed to reach the motors. The
    // PositionSensors are the proto's Hand_PositionSensor_Right/_Left, which is what lets
    // getGripperState() report a MEASURED opening rather than the last thing that was commanded.
    struct Hand
    {
        webots::Motor*          right_motor = nullptr;
        webots::Motor*          left_motor  = nullptr;
        webots::PositionSensor* right_sensor = nullptr;
        webots::PositionSensor* left_sensor  = nullptr;
        float min_position = 0.f;       // slider minStop, from the arm's armsMinPosition field
        float max_position = 0.0425f;   //        maxStop,             armsMaxPosition
        bool  valid() const { return right_motor != nullptr and left_motor != nullptr; }
    };
    Hand right_hand;
    Hand left_hand;

    // Exact names for the actuators and sensors of KinovaGen3 in Webots. The proto prefixes both with
    // its `prefix` field: "Right_" for kinova_arm_r, "Left_" for kinova_arm_l.
    const std::vector<std::string> kinovaMotorNames = {
            "Actuator1", "Actuator2", "Actuator3",
            "Actuator4", "Actuator5", "Actuator6", "Actuator7"
    };
    const std::vector<std::string> kinovaSensorNames = {
            "Actuator1_sensor", "Actuator2_sensor", "Actuator3_sensor",
            "Actuator4_sensor", "Actuator5_sensor", "Actuator6_sensor", "Actuator7_sensor"
    };

    // ── OmniRobot / KinovaArm RPC decoupling ──
    // Webots API calls only happen in compute(); servants read/write these snapshots.
    struct SpeedCommand { float advx = 0.f, advz = 0.f, rot = 0.f; };
    std::mutex   speed_command_mutex;
    SpeedCommand speed_command;
    bool         speed_command_pending = false;
    void apply_pending_speed_command();

    std::mutex base_state_mutex;
    RoboCompGenericBase::TBaseState base_state{};
    void update_base_state(const double *position, const Eigen::Vector2f &velocity_local, double rot_velocity);

    // Servants that need to *write* to the simulation queue a closure here; compute() drains the queue
    // in the Webots thread just before stepping. This is how every arm command, door move, object
    // teleport and world reset reaches the simulator.
    std::mutex webots_tasks_mutex;
    std::vector<std::function<void()>> webots_tasks;
    void post_webots_task(std::function<void()> task);
    void run_webots_tasks();

    // ── Arm snapshots: sampled once per cycle by compute() ──
    struct ArmSnapshot
    {
        RoboCompKinovaArm::TJoints  joints{};
        RoboCompKinovaArm::TGripper gripper{};
    };
    std::mutex  arm_state_mutex;
    ArmSnapshot arm_r_state;
    ArmSnapshot arm_l_state;
    void update_arm_state(long timestamp);
    void sample_arm(const std::vector<webots::PositionSensor*> &sensors,
                    const std::vector<webots::Motor*> &arm_motors,
                    const Hand &hand, long timestamp, ArmSnapshot &out) const;

    // Applied in the Webots thread; the servants only post them.
    void move_arm_with_angle(const RoboCompKinovaArm::Angles &jointAngles,
                             const std::vector<webots::Motor*> &armMotors, float max_velocity);
    void move_arm_with_speed(const RoboCompKinovaArm::Speeds &jointSpeeds,
                             const std::vector<webots::Motor*> &armMotors);
    void stop_arm(const std::vector<webots::Motor*> &armMotors);
    void set_gripper_pos(const Hand &hand, float pos);

    // ── Object pose cache: keys registered on first use, refreshed every cycle ──
    struct TrackedObject
    {
        RoboCompWebots2Robocomp::ObjectPose pose{};
        bool resolved = false;      // compute() has looked this key up at least once
    };
    struct TrackedObjectHandles     // compute()-thread only, never locked
    {
        webots::Field *translation = nullptr;
        webots::Field *rotation = nullptr;
    };
    std::mutex object_poses_mutex;
    std::condition_variable object_poses_cv;
    std::map<std::string, TrackedObject> object_poses;
    std::map<std::string, TrackedObjectHandles> object_handles;
    void update_object_poses();

    // Find a scene node by DEF, falling back to its "name" field. Webots thread only.
    webots::Node* find_scene_node(const std::string& key);
    std::unordered_map<std::string, webots::Node*> scene_node_cache_;

    // Doors, resolved lazily by DEF and cached (a null is cached too: "no such node in this world"
    // does not change while the world is loaded).
    std::map<std::string, webots::Node*> doorNodes;
    void setDoorAperture(const std::string &DEF, float aperture);

    // ── Humans ───────────────────────────────────────────────────────────────────────────────────
    // Inert unless `Humans = true` in config AND the world has a CROWD node whose children are
    // DEF HUMAN_*. play_room_human.wbt is the world this exists for.
    struct WebotsHuman
    {
        webots::Node *node = nullptr;
        RoboCompGridder::TPath path;
    };
    std::map<int, WebotsHuman> humanObjects;
    void parseHumanObjects();
    void humansMovement();
    void set_path_to_human(int humanId, const RoboCompGridder::TPath &path);
    Matrix4d create_affine_matrix(double a, double b, double c, Vector3d trans);

    // ── IMU snapshot ─────────────────────────────────────────────────────────────────────────────
    std::mutex imu_mutex;
    RoboCompIMU::DataImu imu_data{};
    void update_imu_data(long timestamp);
    static std::tuple<float, float, float> rotationMatrixToEulerZYX(const double* R);

    // ── Sim-to-wall rate, used only by acquisition_wall_ms() ──
    std::deque<std::pair<double, long>> clock_pairs_;   // (sim seconds, wall ms)
    double sim_wall_rate_ = 1.0;
    void   update_sim_wall_rate(double sim_seconds, long wall_ms);

    // Sim time (ms) of the device's current sample: the last multiple of its sampling period.
    static double acquisition_sim_ms(double sim_now_ms, int device_period_ms);
    // ... and its wall-clock equivalent, for the existing `timestamp` field.
    long acquisition_wall_ms(long loop_wall_ms, double sim_now_ms, int device_period_ms) const;

    // ── Wheel odometry published on FullPoseEstimationPub ──
    // Forward kinematics, exact inverse of apply_pending_speed_command() (c = LX + LY):
    //     adv  = R/4     * ( w0 - w1 + w2 - w3)
    //     side = R/4     * (-w0 - w1 + w2 + w3)
    //     rot  = R/(4*c) * ( w0 + w1 + w2 + w3)
    // Differenced over the publish interval; the gyro is NOT fused here (separate IMU interface).
    enum class VelocitySource { Wheels, Supervisor };
    VelocitySource fullpose_velocity_source_ = VelocitySource::Wheels;

    struct WheelOdometry
    {
        double pub_angle[4]{};              // rad, the anchor the published twist is differenced from
        double pub_sim_t  = 0.0;
        bool   seeded     = false;          // the anchor holds a real sample
        Eigen::Vector2f v_local{0.f, 0.f};  // per SIM second. (0) = -side, (1) = forward — the
                                            // convention receiving_robotSpeed already uses
        double rot        = 0.0;            // rad/sim-s about local +z, encoders only
        bool   valid      = false;          // v_local/rot hold a usable measurement
    };
    WheelOdometry wheel_odo_;               // compute()-thread only; no lock, like clock_pairs_
    bool update_wheel_odometry(double sim_seconds, bool publishing);

    // Publish period for the wheel channel. The IMU stays at the step rate; that rate GAP is what a
    // consumer's high-rate injection propagates across.
    int    fullpose_publish_period_ms = 100;
    double next_fullpose_publish_ms   = 0.0;

    // ── Synthetic sensor error (all zero = ground truth) ──
    // Wheels get scale + speed-dependent noise; the gyro gets its own noise and bias, no scale.
    // sigma_* are densities: the rate is perturbed by sigma/sqrt(dt).
    bool   sensor_noise_enabled_   = false;
    // Wheel noise: sigma(v) = hypot(floor, k_slip * |v|).
    double wheel_sigma_v_          = 0.0;   // slip coefficient, m/sqrt(s) per (m/s)
    double wheel_sigma_w_          = 0.0;   // slip coefficient, rad/sqrt(s) per (rad/s)
    double wheel_sigma_v_floor_    = 0.0;   // m/sqrt(s)   — sub-count residual at standstill
    double wheel_sigma_w_floor_    = 0.0;   // rad/sqrt(s) — same, yaw
    double wheel_sigma_v_eff_      = 0.0;   // what this sample actually used, injected AND advertised
    double wheel_sigma_w_eff_      = 0.0;
    double wheel_scale_v_          = 0.0;   // fraction: reported = true*(1+s), constant for the run
    double wheel_scale_w_          = 0.0;   // fraction — the scrubbing error
    double gyro_sigma_             = 0.0;   // rad/sqrt(s) — gyro noise density, INDEPENDENT of wheels
    double gyro_bias_              = 0.0;   // rad/s       — constant gyro offset
    // Test only: known gyro scale error to check downstream estimators. Keep 0.
    double gyro_scale_             = 0.0;   // fraction: reported = true*(1+s)
    // Accelerometer noise density, m/s^2/sqrt(Hz). The linear channel had no synthetic error at all
    // while the gyro and wheels both did, so it was the one sensor whose published covariance could
    // not be wrong -- and also the one no consumer could weight honestly.
    double acc_sigma_              = 0.0;
    static double generate_noise(double stddev);

    // Nominal variances published with each sample (sim sensors are exact; zero would mean infinite confidence).
    double nominal_gyro_var_ = 1e-5;        // (rad/s)^2
    double nominal_acc_var_  = 1e-3;        // (m/s^2)^2
    double nominal_vel_var_  = 1e-4;        // (m/s)^2
    double nominal_rot_var_  = 1e-4;        // (rad/s)^2

    // ── Timers ──
    // Sampling period (ms) of the heavy sensors (lidar, 360, zed), independent of Period.Compute.
    int    sensor_period = 0;
    // Per-sensor sampling periods (ms), each a multiple of Period.Compute. Derived from [Sensors] Hz.
    int    zed_period = 0;
    int    cam360_period = 0;
    int    lidar_period = 0;
    int    lidar_profile_hz_ = 20;   // 20 -> "helios" (900 cols), 10 -> "helios_hr" (1800 cols)
    // The STEP period, kept because the IMU samples every step while `sensor_period` is the (possibly
    // slower) heavy-sensor rate. Used to turn a gyro noise DENSITY into a per-sample perturbation.
    int    imu_step_period_ms_ = 0;
    double next_sensor_sample_ms = 0.0;   // next simulation time a sample is due (legacy, unused)
    double next_zed_sample_ms = 0.0;
    double next_cam360_sample_ms = 0.0;
    double next_lidar_sample_ms = 0.0;

    struct PARAMS
    {
        bool delay = false;
        bool do_joystick = true;
        bool points3D = false;
        bool humans = false;
    };
    PARAMS pars;

    // ── Buffers ──────────────────────────────────────────────────────────────────────────────────
    FixedSizeDeque<RoboCompCamera360RGB::TImage> camera_queue{10};
    FixedSizeDeque<RoboCompLidar3D::TData>       helios_delay_queue{10};
    DoubleBuffer<RoboCompCamera360RGB::TImage, RoboCompCamera360RGB::TImage> double_buffer_360;
    DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData>             double_buffer_helios;
    DoubleBuffer<RoboCompCameraRGBDSimple::TRGBD, RoboCompCameraRGBDSimple::TRGBD> double_buffer_zed;

    // ── Data-catching methods (Webots thread only) ───────────────────────────────────────────────
    void receiving_robotSpeed(webots::Supervisor* _robot, long timestamp);
    void receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2, long timestamp);
    void receiving_lidarData(const std::string &name, webots::Lidar* _lidar,
                             DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData>& lidar_doubleBuffer,
                             FixedSizeDeque<RoboCompLidar3D::TData>& delay_queue, long timestamp);
    void receiving_cameraRGBD(webots::Camera* _camera, webots::RangeFinder* _rangeFinder, long timestamp);

    static void printNotImplementedWarningMessage(const std::string &functionName);

signals:
        //void customSignal();
};

#endif
