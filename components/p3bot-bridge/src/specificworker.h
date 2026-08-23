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

// If you want to reduce the period automatically due to lack of use, uncomment the following line.
// Left OFF for P3Bot: hibernation drops the loop to 500 ms, and this bridge is the only thing calling
// robot->step(), so hibernating stalls the whole simulation for every other client of the world —
// including the arm controller, which does not poll the bridge while it is planning. The servants
// still set `hibernation` (a GenericWorker member that exists either way), so enabling this is a
// one-line change and everything below already tolerates the resulting variable step (see
// WheelOdometry: dt comes from the sim clock, never from a configured period).
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

// ── Base geometry: these MUST match protos/P3Bot.proto ───────────────────────────────────────────
// The proto anchors the four wheels at (x = ±0.125, y = ±0.24) in the robot's local frame and gives
// each wheel a bounding Cylinder of radius 0.08. Unlike webots-shadow's Shadow.proto, P3Bot's FORWARD
// axis is local +X: every wheel hinge runs along ±Y, and the zed camera (which looks down its own +X)
// is mounted with identity rotation. So x is longitudinal and y lateral, and LX/LY below are
// HALF-distances, centre to centre / 2 — the form (LX + LY) is used in.
#define WHEEL_RADIUS 0.08   // proto: wheel boundingObject Cylinder `radius 0.08`
#define LX 0.125            // longitudinal half-distance [m]  (proto anchors: |±0.125|)
#define LY 0.240            // lateral half-distance [m]       (proto anchors: |±0.24|)
//
// THESE REPLACE LX 0.270 / LY 0.475 AND ROTATION_INCREMENT_COEFFICIENT 6.3, which together were a
// fitted stand-in for the one constant that actually appears in the kinematics, c/R:
//     old   (LX + LY) * 6.3      = 0.745 * 6.3 = 4.6935
//     true  (LX + LY) / R        = 0.365 / 0.08 = 4.5625
// i.e. the base over-rotated by 2.9% against every commanded omega, and the old constants could not
// be checked against the proto because they were not distances the proto contains. There is no fudge
// factor now: apply_pending_speed_command() and update_wheel_odometry() are exact algebraic inverses
// built from the same three numbers, so they cannot drift apart.
//
// ── Wheel indexing and the SIGN of each motor ────────────────────────────────────────────────────
// Indices 0..3 are motors "wheel1".."wheel4" throughout, which is also the proto's order:
//     wheel1 (+0.125, -0.24) front-right   axis  0  1 0   contactMaterial InteriorWheelMat
//     wheel2 (+0.125, +0.24) front-left    axis  0 -1 0   contactMaterial ExteriorWheelMat
//     wheel3 (-0.125, -0.24) rear-right    axis  0  1 0   contactMaterial ExteriorWheelMat
//     wheel4 (-0.125, +0.24) rear-left     axis  0 -1 0   contactMaterial InteriorWheelMat
// The two LEFT wheels declare an INVERTED hinge axis, so a positive motor velocity drives them
// backwards. That is why the forward-motion row of the kinematics below reads (+, -, +, -) rather
// than (+, +, +, +), and it is a property of the proto, not a sign error — do not "tidy" it. The
// Interior/Exterior contact materials alternate on the diagonals, which is the standard mecanum
// X-configuration the roller equations assume.

// Publish velocities in per-WALL-second units instead of the true per-SIM-second ones. OFF, as in
// webots-bridge: FullPoseEuler and the IMU structs carry `simTimestamp`, so a consumer integrates the
// true rate over the clock that rate is measured in and needs no correction. sim_wall_rate_ is still
// estimated, but only for acquisition_wall_ms(), where converting a sample's age in sim-ms to an age
// in wall-ms genuinely needs it. Turning this back on (=1) restores the old contract for a consumer
// that integrates on `timestamp` and cannot be updated; do not run mixed.
#ifndef BRIDGE_WALL_TIME_VELOCITIES          // -DBRIDGE_WALL_TIME_VELOCITIES=n overrides
#define BRIDGE_WALL_TIME_VELOCITIES 0
#endif

// Set to 1 to emit yawrate_diag.csv from receiving_robotSpeed. Off = zero cost.
// Ported from webots-bridge 2026-08-22 to localise a MEASURED 3.3% rotation SCALE error: over a
// driven lap, room_concept's published heading turned 0.9674x as far as the Webots supervisor
// (ground truth via robot_gt_*), and wg_ratio independently read 1.032 wheel/gyro — the same 3.3%
// from a completely separate instrument.
//
// This log removes the localiser AND the transport from the question: every column is a supervisor
// or device read from the SAME cycle, so
//     d(yaw)/d(sim_t)   vs   wz_sup   vs   gyro_z
// is self-contained. If d(yaw)/d(sim_t) matches wz_sup but gyro_z is 3.3% short, the error is the
// SENSOR; if all three agree, it is born downstream (transport stamps or room_concept's
// integration). Logging sim AND wall time separates a pacing problem from a wrong value.
//
// PROCEDURE: pivot four full turns in place, closing on the starting heading, so the true rotation
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

    // ── OmniRobot / KinovaArm RPC decoupling ─────────────────────────────────────────────────────
    // The Webots libController API must only be touched from the thread that calls robot->step().
    // Supervisor and device getters block until the simulator answers, so an Ice servant thread
    // calling them lands anywhere inside the current step and the RPC latency swings between ~0 and
    // one full period — and two servants calling at once is simply a data race. Servants therefore
    // only touch the snapshots below; compute() is the single point that talks to Webots.
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

    // ── Arm snapshots ────────────────────────────────────────────────────────────────────────────
    // getJointsState() used to read the PositionSensors straight from the servant thread, one Webots
    // round-trip per joint per call. Now compute() samples both arms once per cycle and the servants
    // copy a struct.
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

    // ── Object pose cache ────────────────────────────────────────────────────────────────────────
    // getObjectPose() takes an arbitrary DEF, so it cannot be snapshotted up front. Instead each key
    // is registered on first use and refreshed every cycle from then on: only the very first call for
    // a given key waits for the Webots thread.
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

    // Resolve a scene node by DEF first, falling back to a top-level "name" field scan so world
    // authors don't need to remember to add a DEF. Webots PROTOs (Desk, WaterBottleOpaque, …) expose
    // a "name" but get no automatic DEF, and P3Bot's worlds are written that way.
    // Webots-thread only, like every other node lookup here.
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

    // ── Sim-to-wall rate ─────────────────────────────────────────────────────────────────────────
    // d(sim)/d(wall) over a short trailing window. No longer scales any published velocity (see
    // BRIDGE_WALL_TIME_VELOCITIES); acquisition_wall_ms() needs it to turn a sample's age in sim-ms
    // into an age in wall-ms, and that is its only remaining use.
    std::deque<std::pair<double, long>> clock_pairs_;   // (sim seconds, wall ms)
    double sim_wall_rate_ = 1.0;
    void   update_sim_wall_rate(double sim_seconds, long wall_ms);

    // Acquisition instant of a device's current sample, on the sim clock. Webots refreshes a device on
    // steps that are multiples of its sampling period, so the sample in hand was taken at the last
    // such multiple — NOT at the loop time, which is what every payload used to be stamped with even
    // though a lidar frame can be a full sensor period old. Returns ms.
    static double acquisition_sim_ms(double sim_now_ms, int device_period_ms);
    // ... and its wall-clock equivalent, for the existing `timestamp` field.
    long acquisition_wall_ms(long loop_wall_ms, double sim_now_ms, int device_period_ms) const;

    // ── Wheel odometry: the velocity this bridge PUBLISHES on FullPoseEstimationPub ───────────────
    // Previously that channel carried robotNode->getVelocity() — the supervisor's ground truth, which
    // has no wheel slip and exercises none of the path the real robot uses. The four PositionSensors
    // ("wheel1sensor".."wheel4sensor") were already in the proto and simply never read.
    //
    // This is the FORWARD kinematics: the exact algebraic inverse of the IK in
    // apply_pending_speed_command(), built from the same WHEEL_RADIUS/LX/LY so the two cannot drift
    // apart. Writing w_i for the RAW motor rates (indices = wheel1..wheel4, left wheels inverted as
    // described at the top of this file) and c = LX + LY:
    //     adv  = R/4     * ( w0 - w1 + w2 - w3)
    //     side = R/4     * (-w0 - w1 + w2 + w3)
    //     rot  = R/(4*c) * ( w0 + w1 + w2 + w3)
    // The all-plus row for rotation is not a typo: with the left hinges inverted, a pure spin drives
    // all four motors the same way. The IK matrix is column-orthogonal (norms 4, 4, 4c^2), so this
    // least-squares inverse is exact.
    //
    // THE GYRO IS DELIBERATELY NOT FUSED IN HERE. FullPoseEstimationPub is the *wheel* channel; the
    // IMU is a separate Ice interface the consumer reads and integrates itself. Pre-fusing them in the
    // bridge would leave that consumer no way to weigh the two sources against each other, which is
    // the entire job of the filter component this arrangement stands in for.
    //
    // THE ANCHOR IS THE PUBLISH INSTANT, NOT THE STEP. Differencing over the whole publish interval
    // (100 ms) makes the published velocity the exact MEAN over the interval a consumer will integrate
    // it across. Sampling an 8 ms snapshot every 100 ms instead would alias the roller-contact
    // transient and would NOT integrate correctly. (webots-bridge also keeps a second, per-step anchor;
    // that one exists purely to populate its wheel_odom_diag.csv columns and is dropped here.)
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

    // ── Synthetic sensor error (SIMULATION FIDELITY, not a test hook) ────────────────────────────
    // This bridge publishes the supervisor's GROUND TRUTH, whose velocity noise is some five orders
    // below what any consumer's motion model is configured for — so in simulation the motion prior
    // does no work, and anything tuned against it is tuned against a robot that does not exist.
    //
    // ★ THE WHEELS AND THE GYRO GET SEPARATE ERROR MODELS, AND THAT IS THE POINT. A gyro does NOT
    // share the wheels' rotation error: a base over-reports rotation because it turns partly by
    // SCRUBBING. Applying one error to both channels would simulate a gyro that scrubs, make the gyro
    // look useless, and argue against a feature that is genuinely valuable on hardware. So: wheels
    // carry a SCALE plus encoder noise; the gyro carries its own small noise and a slow BIAS, and no
    // scale at all.
    //
    // sigma_* are DENSITIES (m/sqrt(s), rad/sqrt(s)): the rate is perturbed by sigma/sqrt(dt) so the
    // accumulated increment has variance sigma^2*dt. That is what makes one constant mean the same
    // thing on the 10 Hz wheel channel and the ~125 Hz IMU channel.
    // ⚠ All zero (the default for P3Bot) is inert and the bridge publishes ground truth exactly.
    bool   sensor_noise_enabled_   = false;
    double wheel_sigma_v_          = 0.0;   // m/sqrt(s)   — encoder velocity noise density
    double wheel_sigma_w_          = 0.0;   // rad/sqrt(s) — wheel-derived yaw-rate noise density
    double wheel_scale_v_          = 0.0;   // fraction: reported = true*(1+s), constant for the run
    double wheel_scale_w_          = 0.0;   // fraction — the scrubbing error
    double gyro_sigma_             = 0.0;   // rad/sqrt(s) — gyro noise density, INDEPENDENT of wheels
    double gyro_bias_              = 0.0;   // rad/s       — constant gyro offset
    // TEST INSTRUMENT, not a realism knob. The bias and sigma above model what a rate gyro really
    // does; a scale error is not one of those things, which is why this channel deliberately had
    // none. It exists solely to inject a KNOWN error so an estimator downstream can be checked
    // against a truth it cannot see: if room_concept's learned gyro scale converges to 1/(1+s), the
    // channel is observable and a null result from it means something; if it does not move, the
    // channel was never identifiable and any null result from it meant nothing. Leave at 0.0 for
    // any run that is meant to represent a real robot.
    double gyro_scale_             = 0.0;   // fraction: reported = true*(1+s)
    // Accelerometer noise density, m/s^2/sqrt(Hz). The linear channel had no synthetic error at all
    // while the gyro and wheels both did, so it was the one sensor whose published covariance could
    // not be wrong -- and also the one no consumer could weight honestly.
    double acc_sigma_              = 0.0;
    static double generate_noise(double stddev);

    // Nominal per-sample variances, SI units squared, from config. The simulated gyro and encoders are
    // EXACT and the accelerometer only quantises at 0.001, so a measured sim covariance would be ~0 —
    // and a zero variance reads downstream as infinite confidence, which wrecks any filter. So publish
    // plausible real-robot figures and let the real IMU and base controller overwrite them once
    // calibrated.
    double nominal_gyro_var_ = 1e-5;        // (rad/s)^2
    double nominal_acc_var_  = 1e-3;        // (m/s^2)^2
    double nominal_vel_var_  = 1e-4;        // (m/s)^2
    double nominal_rot_var_  = 1e-4;        // (rad/s)^2

    // ── Timers ───────────────────────────────────────────────────────────────────────────────────
    // Sampling period (ms) for the heavy sensors: lidar, 360 camera, zed. Decoupled from
    // Period.Compute so the control/actuation loop can step fast without reprocessing point clouds and
    // images at the same rate.
    int    sensor_period = 0;
    // The STEP period, kept because the IMU samples every step while `sensor_period` is the (possibly
    // slower) heavy-sensor rate. Used to turn a gyro noise DENSITY into a per-sample perturbation.
    int    imu_step_period_ms_ = 0;
    double next_sensor_sample_ms = 0.0;   // next simulation time a sample is due

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
