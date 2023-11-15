#include "robotActionController.hpp"


unitree_legged_msgs::LowState RobotActionController::last_known_state = unitree_legged_msgs::LowState();

void RobotActionController::actionHalted() {
    duration_counter = 0;
    handleKeyPressed(false);
}

BT::NodeStatus RobotActionController::actionStart() {
    last_known_state = ros_manager.getRobotState();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RobotActionController::actionRunning(const std::vector<double>& targetPos) {
    duration_counter += 1;

    if(duration_counter > MOVEMENT_DURATION_MS) {
        actionHalted();
        unitree_legged_msgs::LowState robot_state = ros_manager.getRobotState();
        ROS_INFO("JOINT_POSITION: %f", robot_state.motorState[0].q);
        return BT::NodeStatus::SUCCESS;
    }

    interpolateJoints(targetPos);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RobotActionController::actionRunning(std::vector<double>& targetPos) {
    return actionRunning(const_cast<const std::vector<double>&>(targetPos));
}


std::vector<double> RobotActionController::getCOGJointPositions(int liftedLeg) {

    std::vector<std::string> hipNames = {"FR_hip", "FL_hip", "RR_hip", "RL_hip"};
    std::vector<std::string> feetNames = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};

    // Get positions of the robot's feet at standing pose (wrt trunk).
    std::vector<tf2::Vector3> initialfeetPositions;
    for (int i = 0; i < feetNames.size(); i++) {
        geometry_msgs::TransformStamped transform;
        try {
            transform = buffer.lookupTransform("trunk", feetNames[i], ros::Time(0));

            tf2::Vector3 translation;
            tf2::fromMsg(transform.transform.translation, translation);
            initialfeetPositions.push_back(translation);
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
        }
    }


    // Using the robot's initial feet position, get the robot's best point for stability
    tf2::Vector3 cog = calculateCoGPosition(initialfeetPositions, liftedLeg);
    
    // Having gotten what CoG is, create a transformation matrix of each feet to the trunk. 
    // With this trunk being the new calculated CoG position. We only move by x & y
    std::vector<Eigen::Matrix4d> T_foot_trunk_list;
    for (int i = 0; i < initialfeetPositions.size(); i++) {
        Eigen::Vector3d new_pos;
        new_pos.x() = initialfeetPositions.at(i).x() - cog.x();
        new_pos.y() = initialfeetPositions.at(i).y() - cog.y();
        new_pos.z() = initialfeetPositions.at(i).z();

        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
        transformation.block<3, 1>(0, 3) = new_pos;
        T_foot_trunk_list.push_back(transformation);
    }

    // Get transformations matrix of each feet relative to hip.
    // Kinematics equations are based on this :)
    std::vector<Eigen::Matrix4d> T_foot_hip_list;

    for (int i = 0; i < hipNames.size(); i++) {
        geometry_msgs::TransformStamped transform;
        try {
            transform = buffer.lookupTransform("trunk", hipNames[i], ros::Time(0));

            tf2::Vector3 translation;
            tf2::fromMsg(transform.transform.translation, translation);

            tf2::Quaternion rotation;
            tf2::fromMsg(transform.transform.rotation, rotation);

            Eigen::Quaterniond eigenRotation(rotation.w(), rotation.x(), rotation.y(), rotation.z());
            Eigen::Vector3d eigenTranslation(translation.x(), translation.y(), translation.z());

            Eigen::Matrix4d T_hip_trunk;
            T_hip_trunk.setIdentity();
            T_hip_trunk.block<3, 3>(0, 0) = eigenRotation.toRotationMatrix();
            T_hip_trunk.block<3, 1>(0, 3) = eigenTranslation;

            Eigen::Matrix4d T_trunk_hip = T_hip_trunk.inverse();
            Eigen::Matrix4d T_foot_hip = T_trunk_hip * T_foot_trunk_list[i];

            T_foot_hip_list.push_back(T_foot_hip);
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
        }
    }

    std::vector<double> joint_angles;
    for (int i = 0; i < T_foot_hip_list.size(); i++) {   
        std::vector<double> solverRes;

        // Because 1 and 3 are left legs according to order:
        // FR, FL, RR, RL.
        Eigen::Vector3d footPosition = T_foot_hip_list[i].block<3, 1>(0, 3);
        if (i == 1 || i == 3) {
            solverRes = ikSolver(footPosition, false);
        } else {
            solverRes = ikSolver(footPosition, true);
        }

        joint_angles.insert(joint_angles.end(), solverRes.begin(), solverRes.end());
    }

    return joint_angles;
}

Eigen::Vector3d RobotActionController::getCurrentFootPosition(const std::string& legName) {
    std::string hipName = legName + "_hip";
    std::string footName = legName + "_foot";

    try {
        geometry_msgs::TransformStamped trunkToFootTransform = buffer.lookupTransform("trunk", footName, ros::Time(0));
        geometry_msgs::TransformStamped trunkToHipTransform = buffer.lookupTransform("trunk", hipName, ros::Time(0));

        Eigen::Vector3d footPosition;
        footPosition.x() = trunkToFootTransform.transform.translation.x;
        footPosition.y() = trunkToFootTransform.transform.translation.y;
        footPosition.z() = trunkToFootTransform.transform.translation.z;

        Eigen::Vector3d hipPosition;
        hipPosition.x() = trunkToHipTransform.transform.translation.x;
        hipPosition.y() = trunkToHipTransform.transform.translation.y;
        hipPosition.z() = trunkToHipTransform.transform.translation.z;

        Eigen::Quaterniond rotation(
            trunkToHipTransform.transform.rotation.w,
            trunkToHipTransform.transform.rotation.x,
            trunkToHipTransform.transform.rotation.y,
            trunkToHipTransform.transform.rotation.z
        );

        Eigen::Matrix4d T_hip_trunk = Eigen::Matrix4d::Identity();
        T_hip_trunk.block<3, 3>(0, 0) = rotation.toRotationMatrix();
        T_hip_trunk.block<3, 1>(0, 3) = hipPosition;

        Eigen::Matrix4d T_foot_trunk = Eigen::Matrix4d::Identity();
        T_foot_trunk.block<3, 1>(0, 3) = footPosition;

        Eigen::Matrix4d T_trunk_hip = T_hip_trunk.inverse();
        Eigen::Matrix4d T_foot_hip = T_trunk_hip * T_foot_trunk;

        Eigen::Vector3d correctedFootPosition = T_foot_hip.block<3, 1>(0, 3);
        return correctedFootPosition;
    } catch (const tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return Eigen::Vector3d(1.0, 2.0, 3.0); //TODO: fix this thingy
    }
}

void RobotActionController::configurePID(ros::Time& current_time) {
    prev_time = current_time;
    error_cumulative = 0;
    prev_command = 0;
    prev_error = Config::FORCE_CMD_SETPOINT;
}


tf2::Vector3 RobotActionController::calculateCoGPosition(const std::vector<tf2::Vector3>& feet, int liftedLeg) {
    tf2::Vector3 cog;
    double totalArea = 0.0;

    int size = feet.size();
    liftedLeg = (liftedLeg + size) % size;

    for (int i = 0; i < size; i++) {

        if (liftedLeg != -1) {
            if (i == liftedLeg || (i + 1) % size == liftedLeg || (i + 2) % size == liftedLeg) {
                continue;
            }
        }

        double area = calculateFeetArea(feet[i], feet[(i + 1) % size], feet[(i + 2) % size]);
        totalArea += area;

        tf2::Vector3 centroid(0.0, 0.0, 0.0);

        centroid.setX((feet[i].x() + feet[(i + 1) % size].x() + feet[(i + 2) % size].x()) / 3.0);
        centroid.setY((feet[i].y() + feet[(i + 1) % size].y() + feet[(i + 2) % size].y()) / 3.0);
        centroid.setZ((feet[i].z() + feet[(i + 1) % size].z() + feet[(i + 2) % size].z()) / 3.0);

        cog.setX(cog.x() + centroid.x() * area);
        cog.setY(cog.y() + centroid.y() * area);
        cog.setZ(cog.z() + centroid.z() * area);
    }

    if (totalArea > 0.0) {
        cog.setX(cog.x() / totalArea);
        cog.setY(cog.y() / totalArea);
        cog.setZ(cog.z() / totalArea);
    } else {
        cog.setZero();
    }

    return cog;
}

double RobotActionController::calculateFeetArea(const tf2::Vector3& p1, const tf2::Vector3& p2, const tf2::Vector3& p3) {
    double a = sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) + pow(p1.z() - p2.z(), 2));
    double b = sqrt(pow(p2.x() - p3.x(), 2) + pow(p2.y() - p3.y(), 2) + pow(p2.z() - p3.z(), 2));
    double c = sqrt(pow(p3.x() - p1.x(), 2) + pow(p3.y() - p1.y(), 2) + pow(p3.z() - p1.z(), 2));

    double s = (a + b + c) / 2.0;
    return sqrt(s * (s - a) * (s - b) * (s - c));
}

double RobotActionController::calculatePIDControlOutput(double feedbackForce, ros::Time& currentTime) {
    // Calculate time elapsed since the previous update
    ros::Duration time_elapsed = currentTime - prev_time;
    double delta_time = time_elapsed.toSec() * 1000.0;

    // Check for zero delta_time to prevent division by zero
    if (delta_time == 0) {
        return prev_command;
    }

    // Calculate the error between the setpoint and feedback
    double error = Config::FORCE_CMD_SETPOINT - feedbackForce;

    // Calculate the error rate (derivative term)
    double error_rate = (error - prev_error) / delta_time;

    // Debugging information
    ROS_INFO("ERROR: %f", error);

    // Check for a sign change in the error, reset integral term if sign flips
    if ((error * prev_error) < 0) {
        error_cumulative = 0;
    }

    // ROS_INFO("ERROR_CUMULATIVE: %f", error_cumulative);
    // Update the integral term (anti-windup mechanism)
    error_cumulative += error * delta_time;

    // Initialize the control command
    double command = 0;

    // Determine the control action based on the feedback
    if (std::abs(feedbackForce) > Config::FORCE_CMD_SETPOINT) {
        // Calculate the control command for pulling action
        command = (P_PULL * error) + (D_PULL * error_rate) + (I_PULL * error_cumulative);
        // command = (P_PULL * error) + (D_PULL * error_rate) + (I_PULL * error_cumulative);
    } else {
        // Calculate the control command for pushing action
        command = (P_PUSH * error) + (D_PUSH * error_rate) + (I_PUSH * error_cumulative);
        // command = (P_PUSH * error) + (D_PUSH * error_rate) + (I_PUSH * error_cumulative);
    }

    // Store the current command, error, and time for the next iteration
    prev_command = command;
    prev_error = error;
    prev_time = currentTime;

    // Return the computed control command
    return command;
}

std::vector<double> RobotActionController::ikSolver(const Eigen::Vector3d& footPosition, bool isRight) {
    double footPositionX_ = footPosition.x();
    double footPositionY_ = footPosition.y();
    double footPositionZ_ = footPosition.z();

    double xPos_squared = footPositionX_ * footPositionX_;
    double yPos_squared = footPositionY_ * footPositionY_;
    double zPos_squared = footPositionZ_ * footPositionZ_;

    double hipLength_squared = Config::HIP_LENGTH * Config::HIP_LENGTH;
    double thighLength_squared = Config::THIGH_LENGTH * Config::THIGH_LENGTH;
    double calfLength_squared = Config::CALF_LENGTH * Config::CALF_LENGTH;

    std::vector<double> jointAngles{0, 0, 0};

    // Theta One    
    double alpha = std::acos(Config::HIP_LENGTH / std::sqrt(yPos_squared + zPos_squared));
    double beta = std::atan2(footPositionZ_, footPositionY_);
    if (isRight) {
        jointAngles.at(0) = M_PI - alpha + beta;
    } else {
        jointAngles.at(0) = alpha + beta;
    }

    if (std::isnan(jointAngles.at(0))) {
        return std::vector<double>();
    }

    // Theta Three
    double gamma =
        std::acos((hipLength_squared + thighLength_squared + calfLength_squared -
                    xPos_squared - yPos_squared - zPos_squared) /
                    (2 * Config::THIGH_LENGTH * Config::CALF_LENGTH));
    jointAngles.at(2) = gamma - M_PI;

    if (std::isnan(jointAngles.at(2))) {
        return std::vector<double>();
    }

    // Theta Two
    double z_prime = -std::sqrt(yPos_squared + zPos_squared - hipLength_squared);
    double psi = std::atan2(Config::CALF_LENGTH * std::sin(jointAngles.at(2)),
                            Config::THIGH_LENGTH + Config::CALF_LENGTH * std::cos(jointAngles.at(2)));
    jointAngles.at(1) = M_PI - psi + std::atan2(footPositionX_, z_prime);

    if (std::isnan(jointAngles.at(1))) {
        return std::vector<double>();
    }

    for (int i = 0; i < 3; i++) {
        if (jointAngles.at(i) >= M_PI) {
            jointAngles.at(i) -= 2 * M_PI;
        }
        if (jointAngles.at(i) <= -M_PI) {
            jointAngles.at(i) += 2 * M_PI;
        }
    }

    return jointAngles;
}

void RobotActionController::interpolateJoints(const std::vector<double>& targetPos) {
    double percent = static_cast<double>(duration_counter) / static_cast<double>(MOVEMENT_DURATION_MS);

    ROS_INFO("Initial Position: %f", last_known_state.motorState[0].q);
    ROS_INFO("Target Position: %f", targetPos[0]);
    ROS_INFO("Percent: %f", percent);
    ROS_INFO("Duration Counter: %f", static_cast<double>(duration_counter));
    ROS_INFO("Movement duration: %f", static_cast<double>(MOVEMENT_DURATION_MS));
    for (int j = 0; j < Config::NUM_OF_JOINTS; j++) {
        ros_manager.setRobotCmd(j, (last_known_state.motorState[j].q * (1 - percent)) + (targetPos[j] * percent));
    }
}

void RobotActionController::interpolateJoints(std::vector<double>& targetPos) {
    return interpolateJoints(const_cast<const std::vector<double>&>(targetPos));
}

bool RobotActionController::isJointsCloseToTarget(const unitree_legged_msgs::LowState& currentState, const std::vector<double>& targetPos) {

    double threshold = 0.005;

    for (size_t i = 0; i < Config::NUM_OF_JOINTS; i++) {
        double currentJointPosition = currentState.motorState[i].q;
        double targetJointPosition = targetPos[i];
        double positionDifference = std::abs(currentJointPosition - targetJointPosition);

        if (positionDifference > threshold) {
            return false;
        }
    }

    return true;
}

