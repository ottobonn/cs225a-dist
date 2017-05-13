#include "controller.h"

#include <iostream>
#include <fstream>

#include <signal.h>
static volatile bool g_runloop = true;
void stop(int) { g_runloop = false; }

// Return true if any elements in the Eigen::MatrixXd are NaN
template<typename Derived>
static inline bool isnan(const Eigen::MatrixBase<Derived>& x) {
	return (x.array() != x.array()).any();
}

using namespace std;

/**
 * Controller::readRedisValues()
 * ------------------------------
 * Retrieve all read keys from Redis.
 */
void Controller::readRedisValues() {
	// read from Redis current sensor values
	redis_client_.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
	redis_client_.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

	// Get current simulation timestamp from Redis
	redis_client_.getDouble(TIMESTAMP_KEY, &t_curr_);

	// Read in KP and KV from Redis (can be changed on the fly in Redis)
	redis_client_.getInt(KP_POSITION_KEY, &kp_pos_);
  redis_client_.getInt(KV_POSITION_KEY, &kv_pos_);
  redis_client_.getInt(KP_ORIENTATION_KEY, &kp_ori_);
  redis_client_.getInt(KV_ORIENTATION_KEY, &kv_ori_);
	redis_client_.getInt(KP_JOINT_KEY, &kp_joint_);
  redis_client_.getInt(KV_JOINT_KEY, &kv_joint_);
}

/**
 * Controller::writeRedisValues()
 * -------------------------------
 * Send all write keys to Redis.
 */
void Controller::writeRedisValues() {
	// Send end effector position and desired position
	redis_client_.setEigenMatrixDerivedString(EE_POSITION_KEY, x_);
	redis_client_.setEigenMatrixDerivedString(EE_POSITION_DESIRED_KEY, x_des_);

	// Send torques
	redis_client_.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques_);
}

/**
 * Controller::updateModel()
 * --------------------------
 * Update the robot model and all the relevant member variables.
 */
void Controller::updateModel() {
	// Update the model
	robot->updateModel();

	// Forward kinematics
	robot->position(x_, kWristLinkName, kWristCenterOffset);
	robot->linearVelocity(dx_, kWristLinkName, kWristCenterOffset);
	robot->rotation(R_wrist_, kWristLinkName);
	robot->angularVelocity(omega_wrist_, kWristLinkName);

	// Jacobians
	robot->Jv(Jv_, kWristLinkName, kWristCenterOffset);
	robot->J(J_, kWristLinkName, kWristCenterOffset);
	J5_ = J_.block(1, 0, 5, dof);
	robot->nullspaceMatrix(N5_, J5_);

	// Dynamics
	robot->taskInertiaMatrixWithPseudoInv(Lambda0_, J5_);
	robot->gravityVector(g_);
}

/**
 * Controller::computeJointSpaceControlTorques()
 * ----------------------------------------------
 * Controller to initialize robot to desired joint position.
 */
Controller::ControllerStatus Controller::computeJointSpaceControlTorques() {
	// Finish if the robot has converged to q_initial
	Eigen::VectorXd q_err = robot->_q - q_des_;
	Eigen::VectorXd dq_err = robot->_dq - dq_des_;
	if (q_err.norm() < kToleranceQ && dq_err.norm() < kToleranceDq) {
		return FINISHED;
	}

	// Compute torques
	Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;
	command_torques_ = robot->_M * ddq + g_;
	return RUNNING;
}

/**
 * Controller::computeOperationalSpaceControlTorques()
 * ----------------------------------------------------
 * Controller to move end effector to desired position.
 */
Controller::ControllerStatus Controller::computeOperationalSpaceControlTorques() {
	// PD position control with velocity saturation
	Eigen::Vector3d x_err = x_ - x_des_;
	dx_des_ = -(kp_pos_ / kv_pos_) * x_err;
	double v = kMaxVelocity / dx_des_.norm();
	if (v > 1) v = 1;
	Eigen::Vector3d dx_err = dx_ - v * dx_des_;
	Eigen::Vector3d ddx = -kv_pos_ * dx_err;

	// Posture control and damping
	Eigen::VectorXd q_err = robot->_q - q_des_;
	q_err.setZero();
	Eigen::VectorXd dq_err = robot->_dq - dq_des_;
	Eigen::VectorXd ddq = -kp_joint_ * q_err - kv_joint_ * dq_err;

	// Wrist orientation control
	Eigen::Vector3d dPhi;
  robot->orientationError(dPhi, R_wrist_des_, R_wrist_);
	Eigen::Vector2d dPhi_ignoring_x = dPhi.tail(2);
	Eigen::Vector2d omega_wrist_ignoring_x = omega_wrist_.tail(2);
	Eigen::Vector2d orientation_accel = (kp_ori_ * -dPhi_ignoring_x) - (kv_ori_ * omega_wrist_ignoring_x);
	Eigen::VectorXd accel(5);
	accel << orientation_accel, ddx;

	// Control torques with posture projected into the nullspace of the 5-dof task
	Eigen::VectorXd F = Lambda0_ * accel;
	Eigen::VectorXd F_posture = robot->_M * ddq;
	command_torques_ = J5_.transpose() * F + N5_.transpose() * F_posture + g_;

  if (x_err.norm() < kToleranceTrajectoryX
    && dx_err.norm() < kToleranceTrajectoryDx) {
    return FINISHED;
  } else {
    return RUNNING;
  }
}

Controller::ControllerStatus Controller::computeToolChangeControlTorques() {
	// Hold position for tool change
	q_des_ = robot->_q;
	// Rotate the tool carousel
	q_des_(dof - 1) = marker_q_des_;
	return computeJointSpaceControlTorques();
}

/**
 * public Controller::initialize()
 * --------------------------------
 * Initialize timer and Redis client
 */
void Controller::initialize() {
	// Create a loop timer
	timer_.setLoopFrequency(kControlFreq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer_.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer_.initializeTimer(kInitializationPause); // 1 ms pause before starting loop

	// Start redis client
	// Make sure redis-server is running at localhost with default port 6379
	redis_client_.serverIs(kRedisServerInfo);

	// Set gains in Redis if not initialized
	redis_client_.setValueIfUninitialized(KP_POSITION_KEY, kp_pos_);
	redis_client_.setValueIfUninitialized(KV_POSITION_KEY, kv_pos_);
	redis_client_.setValueIfUninitialized(KP_ORIENTATION_KEY, kp_ori_);
	redis_client_.setValueIfUninitialized(KV_ORIENTATION_KEY, kv_ori_);
	redis_client_.setValueIfUninitialized(KP_JOINT_KEY, kp_joint_);
	redis_client_.setValueIfUninitialized(KV_JOINT_KEY, kv_joint_);
}

/**
 * Convert a point in the image plane (with coordinates on the interval [0, 1])
 * to a point in 3D operational space.
 */
Eigen::Vector3d Controller::ImagePointToOperationalPoint(Eigen::Vector2d image_point) {
  Eigen::Vector3d op_point;
  float x_size = kImageBoundsMax(0) - kImageBoundsMin(0);
  float y_size = kImageBoundsMax(1) - kImageBoundsMin(1);
  float x = (image_point(0) - 0.5) * x_size + kImageBoundsMin(0);
  float y = image_point(1) * y_size + kImageBoundsMin(1);
  op_point << x, y, 0;
  return op_point;
}

/**
 * public Controller::runLoop()
 * -----------------------------
 * Controller state machine
 */
void Controller::runLoop() {
	while (g_runloop) {
		// Wait for next scheduled loop (controller must run at precise rate)
		timer_.waitForNextLoop();
		++controller_counter_;

		// Get latest sensor values from Redis and update robot model
		readRedisValues();
		updateModel();

		switch (controller_state_) {
			// Wait until valid sensor values have been published to Redis
			case REDIS_SYNCHRONIZATION:
				if (isnan(robot->_q)) continue;
				cout << "Redis synchronized. Switching to joint space controller." << endl;
				controller_state_ = JOINT_SPACE_INITIALIZATION;
				break;

			// Initialize robot to default joint configuration
			case JOINT_SPACE_INITIALIZATION:
				if (computeJointSpaceControlTorques() == FINISHED) {
					cout << "Joint position initialized. Initializing tool position." << endl;
					x_des_ = kToolChangePosition;
					controller_state_ = Controller::MOVING_TO_TOOL_CHANGE;
					break;
				}
				break;

			// Control end effector to desired position
			case OP_SPACE_POSITION_CONTROL:
				if (computeOperationalSpaceControlTorques() == FINISHED) {
          cout << "Reached trajectory point." << endl;
          // Get next trajectory point
          currentToolpathPoint_++;
          if (currentToolpathPoint_ == currentToolpath_->points.end()) {
            currentToolpath_++;
            if (currentToolpath_ == trajectory_.sequence.end()) {
              cout << "Trajectory complete. Holding position." << endl;
							x_des_ = kToolChangePosition;
							controller_state_ = Controller::TRAJECTORY_COMPLETE;
              break;
            }
    				// Switch tools
						cout << "Moving to tool change position." << endl;
						x_des_ = kToolChangePosition;
						controller_state_ = MOVING_TO_TOOL_CHANGE;
						break;
          }
          x_des_ = ImagePointToOperationalPoint(*currentToolpathPoint_);
        }
				break;

			case MOVING_TO_TOOL_CHANGE:
				if (computeOperationalSpaceControlTorques() == FINISHED) {
					// Arrived at tool change position
					cout << "Beginning tool change." << endl;
					// Rotate the tool carousel
					marker_q_des_ = kToolIntervalRadians * currentToolpath_->tool;
					controller_state_ = Controller::CHANGING_TOOL;
					break;
				}
			  break;

			case CHANGING_TOOL:
				if (computeToolChangeControlTorques() == FINISHED) {
					// Tool changed. Start new tool path.
					cout << "Tool change complete." << endl;
					currentToolpathPoint_ = currentToolpath_->points.begin();
					x_des_ = ImagePointToOperationalPoint(*currentToolpathPoint_);
					controller_state_ = Controller::OP_SPACE_POSITION_CONTROL;
					break;
				}
				break;

			case TRAJECTORY_COMPLETE:
			  // Hold position
				computeOperationalSpaceControlTorques();
				break;

			// Invalid state. Zero torques and exit program.
			default:
				cout << "Invalid controller state. Stopping controller." << endl;
				g_runloop = false;
				command_torques_.setZero();
				break;
		}

		// Check command torques before sending them
		if (isnan(command_torques_)) {
			cout << "NaN command torques. Sending zero torques to robot." << endl;
			command_torques_.setZero();
		}

		// Send command torques
		writeRedisValues();
	}

	// Zero out torques before quitting
	command_torques_.setZero();
	redis_client_.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques_);
}

int main(int argc, char** argv) {

	// Parse command line
	if (argc != 5) {
		cout << "Usage: demo_app <path-to-world.urdf> <path-to-robot.urdf> <robot-name> <trajectory_file_name>" << endl;
		exit(0);
	}
	// argument 0: executable name
	// argument 1: <path-to-world.urdf>
	string world_file(argv[1]);
	// argument 2: <path-to-robot.urdf>
	string robot_file(argv[2]);
	// argument 3: <robot-name>
	string robot_name(argv[3]);
  // argument 4: <trajectory file name>
  string trajectory_file_name(argv[4]);

	// Load robot
	cout << "Loading robot: " << robot_file << endl;
	auto robot = make_shared<Model::ModelInterface>(robot_file, Model::rbdl, Model::urdf, false);
	robot->updateModel();

	// Start controller app
	cout << "Initializing app with " << robot_name << endl;
	Controller app(move(robot), robot_name, trajectory_file_name);
	app.initialize();
	cout << "App initialized. Waiting for Redis synchronization." << endl;
	app.runLoop();

	return 0;
}
