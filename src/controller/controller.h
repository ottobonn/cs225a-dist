#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "trajectory.h"

// cs225a includes
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

// external includes
#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <model/ModelInterface.h>

// std includes
#include <string>
#include <thread>

#include <iostream>

typedef struct {
	Eigen::Vector3d bottom_left;
	Eigen::Vector3d top_left;
	Eigen::Vector3d bottom_right;
	Eigen::Vector3d top_right;
} Bounds3d;

class Controller {

public:

	Controller(std::shared_ptr<Model::ModelInterface> robot,
		        const std::string &robot_name,
            const std::string &trajectory_file_name) :
		robot(robot),
		dof(robot->dof()),
		JOINT_TORQUES_COMMANDED_KEY(kRedisKeyPrefix + robot_name + "::actuators::fgc"),
		EE_POSITION_KEY            (kRedisKeyPrefix + robot_name + "::tasks::ee_pos"),
		EE_POSITION_DESIRED_KEY    (kRedisKeyPrefix + robot_name + "::tasks::ee_pos_des"),
		JOINT_ANGLES_KEY           (kRedisKeyPrefix + robot_name + "::sensors::q"),
		JOINT_VELOCITIES_KEY       (kRedisKeyPrefix + robot_name + "::sensors::dq"),
		TIMESTAMP_KEY              (kRedisKeyPrefix + robot_name + "::timestamp"),
		KP_POSITION_KEY            (kRedisKeyPrefix + robot_name + "::tasks::kp_pos"),
		KV_POSITION_KEY            (kRedisKeyPrefix + robot_name + "::tasks::kv_pos"),
		KP_ORIENTATION_KEY         (kRedisKeyPrefix + robot_name + "::tasks::kp_ori"),
		KV_ORIENTATION_KEY         (kRedisKeyPrefix + robot_name + "::tasks::kv_ori"),
		KP_JOINT_KEY               (kRedisKeyPrefix + robot_name + "::tasks::kp_joint"),
		KV_JOINT_KEY               (kRedisKeyPrefix + robot_name + "::tasks::kv_joint"),
		command_torques_(dof),
		Jv_(3, dof),
		J_(6, dof),
		g_(dof),
		q_des_(dof),
		dq_des_(dof),
		controller_state_(REDIS_SYNCHRONIZATION),
    trajectory_(Trajectory::fromFile(trajectory_file_name))
	{
		command_torques_.setZero();

		// Home configuration for joints
		q_des_ << 0, -45, 0, 90, 0, -45, 0;
		q_des_ *= M_PI / 180.0;
		dq_des_.setZero();

		// Desired end-effector rotation
		// (Set in operational-space torque control law functions)
		R_des_.setIdentity();

		// Desired marker carousel rotation
		tool_angle_des_ = 0;

		// Ignore gravity because Sawyer compensates for it automatically
		g_.setZero();

    // Set up trajectory iteration
    currentToolpath_ = trajectory_.sequence.begin();
    currentToolpathPoint_ = currentToolpath_->points.begin();

		// Set up image bounds
		image_corners.bottom_left = Eigen::Vector3d(0.4, 0.2, 0.0025);
		image_corners.bottom_right = Eigen::Vector3d(0.4, -0.2, 0.0025);
		image_corners.top_left = Eigen::Vector3d(0.8, 0.2, -0.0001);
		image_corners.top_right = Eigen::Vector3d(0.8, -0.2, -0.005);
	}

	/***** Public functions *****/

	void initialize();
	void runLoop();

protected:

	/***** Enums *****/

	// State enum for controller state machine inside runloop()
	enum ControllerState {
		REDIS_SYNCHRONIZATION,
		JOINT_SPACE_INITIALIZATION,
		OP_SPACE_POSITION_CONTROL,
    MOVING_TO_TOOL_CHANGE,
    CHANGING_TOOL,
    MOVING_FROM_TOOL_CHANGE,
    TRAJECTORY_COMPLETE
	};

	// Return values from computeControlTorques() methods
	enum ControllerStatus {
		RUNNING,  // Not yet converged to goal position
		FINISHED  // Converged to goal position
	};

	/***** Constants *****/

	const int dof;  // Initialized with robot model
	const double kToleranceQ  = 0.1;  // Joint space initialization tolerance
	const double kToleranceDq = 0.1;  // Joint space initialization tolerance
	const double kMaxVelocity = 0.1;  // Maximum end effector velocity

  const double kToleranceTrajectoryX = 0.01;
  const double kToleranceTrajectoryDx = 0.01;
	const double kToleranceTrajectoryR = 0.001;

	const int kControlFreq = 1000;         // 1 kHz control loop
	const int kInitializationPause = 1e6;  // 1ms pause before starting control loop

	const string kEELinkName = "right_l6";

	const HiredisServerInfo kRedisServerInfo = {
		"127.0.0.1",  // hostname
		6379,         // port
		{ 1, 500000 } // timeout = 1.5 seconds
	};

	// Redis keys:
	const std::string kRedisKeyPrefix = "cs225a::robot::";
	// - write:
	const std::string JOINT_TORQUES_COMMANDED_KEY;
	const std::string EE_POSITION_KEY;
	const std::string EE_POSITION_DESIRED_KEY;
	// - read:
	const std::string JOINT_ANGLES_KEY;
	const std::string JOINT_VELOCITIES_KEY;
	const std::string TIMESTAMP_KEY;
	const std::string KP_POSITION_KEY;
	const std::string KV_POSITION_KEY;
	const std::string KP_ORIENTATION_KEY;
	const std::string KV_ORIENTATION_KEY;
	const std::string KP_JOINT_KEY;
	const std::string KV_JOINT_KEY;
	const std::string KP_JOINT_INIT_KEY;
	const std::string KV_JOINT_INIT_KEY;

	/***** Member functions *****/

	void readRedisValues();
	void updateModel();
	void writeRedisValues();
	ControllerStatus computeJointSpaceControlTorques();
	ControllerStatus computeOperationalSpaceControlTorques();
	Eigen::Matrix3d R_des();
	ControllerStatus OSControlLawZSpin();
	ControllerStatus OSControlLaw6DOF();
  Eigen::Vector3d ImagePointToOperationalPoint(Eigen::Vector2d image_point);
	Eigen::Vector3d ToolChangePosition();

	/***** Member variables *****/

	// Robot
	const std::shared_ptr<Model::ModelInterface> robot;

	// Redis
	RedisClient redis_client_;
	std::string redis_buf_;

	// Timer
	LoopTimer timer_;
	double t_curr_;
	uint64_t controller_counter_ = 0;

	// State machine
	ControllerState controller_state_;

	// Controller variables
	Eigen::VectorXd command_torques_;
	Eigen::MatrixXd Jv_;
	Eigen::MatrixXd J_;
	Eigen::MatrixXd N_;
	Eigen::VectorXd g_;
	Eigen::Vector3d x_, dx_;
	Eigen::VectorXd q_des_, dq_des_;
	Eigen::Vector3d x_des_, dx_des_;
	Eigen::Matrix3d R_;
	Eigen::Matrix3d R_des_;
	Eigen::Vector3d omega_;
	float tool_angle_des_; // radians!

	// Default gains (used only when keys are nonexistent in Redis)
	double kp_pos_ = 40;
	double kv_pos_ = 10;
	double kp_ori_ = 100;
	double kv_ori_ = 50;
	double kp_joint_ = 40;
	double kv_joint_ = 30;

  // Trajectory
  Trajectory trajectory_;
  TrajectorySequenceIterator currentToolpath_;
  TrajectoryToolPathPointIterator currentToolpathPoint_;
  const double kToolChangeOffset = 0.01;

	// The robot faces forward in the x direction, with y increasing to its left
  const Eigen::Vector2d kImageBoundsMin = Eigen::Vector2d(0.7, -0.2);
  const Eigen::Vector2d kImageBoundsMax = kImageBoundsMin + Eigen::Vector2d(0.4, 0.4);

	Bounds3d image_corners;

	const double kToolIntervalRadians = 2 * M_PI / 8;

	// The offset from the end-effector frame origin to the end-effector mount plate
	const Eigen::Vector3d kEEMountOffset = Eigen::Vector3d(0, 0, 0.03);
	// The offset from the end-effector origin to the tool tip
	// TODO measure actual tool length
	const Eigen::Vector3d kToolTipOffset = kEEMountOffset + Eigen::Vector3d(0, 0, -0.2);
};

#endif //CONTROLLER_H
