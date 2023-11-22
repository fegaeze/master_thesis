#include "robotActions.hpp"


std::vector<double> RobotGoToCogAction::cog_position = std::vector<double>();
std::vector<double> RobotFrRaiseAction::fr_foot_target_position = std::vector<double>();

const std::vector<double> RobotStandAction::STAND_JOINT_POSITIONS = {
    0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
    0.0, 0.67, -1.3, -0.0, 0.67, -1.3
};
const std::vector<double> RobotLieDownAction::LIE_DOWN_JOINT_POSITIONS = {
    -0.5, 1.15, -2.7, 0.5, 1.15, -2.7,
    -0.5, 1.15, -2.7, 0.5, 1.15, -2.7
};


/** INITIALIZATION ACTION */
BT::NodeStatus RobotInitializationAction::robotStateReceived() {
    unitree_legged_msgs::LowState low_state = ros_manager.getRobotState();

    double FR_Calf = low_state.motorState[2].q;
    double FL_Calf = low_state.motorState[5].q;
    double RR_Calf = low_state.motorState[8].q;
    double RL_Calf = low_state.motorState[11].q;

    if(FR_Calf != 0.0 && FL_Calf != 0.0 && RR_Calf != 0.0 && RL_Calf != 0.0) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotInitializationAction::robotInitialized() {
    if(robot_initialized) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotInitializationAction::robotInitialization() {
    ROS_INFO("GO1_INITIAL_STATE");
    ros_manager.setRobotParams();
    robot_initialized = true;
    return BT::NodeStatus::SUCCESS;
}

void RobotInitializationAction::registerNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerSimpleCondition(
        "RobotInitialized", std::bind(&RobotInitializationAction::robotInitialized, this));

    factory.registerSimpleCondition(
        "RobotStateReceived", std::bind(&RobotInitializationAction::robotStateReceived, this));

    factory.registerSimpleAction(
        "RobotInitialization", std::bind(&RobotInitializationAction::robotInitialization, this));
}


/** LIE DOWN ACTION */
void RobotLieDownAction::handleKeyPressed(bool pressed) {
    action_service_manager.setLieDownKeyPressed(pressed);
}

BT::NodeStatus RobotLieDownAction::onStart() {
    ROS_INFO("GO1_LIE_DOWN_ACTION");
    return actionStart();
}

BT::NodeStatus RobotLieDownAction::onRunning() {
    return actionRunning(LIE_DOWN_JOINT_POSITIONS);
}

void RobotLieDownAction::onHalted() {
    actionHalted();
}


/** DROP_FOOT ACTION */
void RobotDropFootAction::handleKeyPressed(bool pressed) {
    action_service_manager.setDropFootKeyPressed(pressed);
}

BT::NodeStatus RobotDropFootAction::onStart() {
    ROS_INFO("GO1_DROP_FOOT_ACTION");
    ros::Time current_time = ros::Time::now();
    configurePID(current_time);
    return actionStart();
}

BT::NodeStatus RobotDropFootAction::onRunning() {

    double force_feedback = std::abs(ros_manager.getCurrentForce());

    if(contact_initiated == false && force_feedback > 2.0) {
        contact_initiated = true;
    }

    ROS_INFO("================================================");
    ROS_INFO("Force Feedback (f): %f", force_feedback);

    double footZChange = 0.1 / 50.0;

    if(contact_initiated) {
        ros::Time current_time = ros::Time::now();
        double control_cmd = calculatePIDControlOutput(force_feedback, current_time);
        ROS_INFO("The control command is: %f", control_cmd);
        footZChange = control_cmd / 100;
    }

    std::vector<Eigen::Vector3d> footPositions;
    std::vector<std::string> footNames = {"FR", "FL", "RR", "RL"};

    ROS_INFO("Change Position by: %f", footZChange);
    for (const auto& footName : footNames) {
        Eigen::Vector3d position = getCurrentFootPosition(footName);
        if(footName == "FR") {
            // ROS_INFO("The position x is: %f", position.x());
            // ROS_INFO("The position y is: %f", position.y());
            ROS_INFO("The position z before change is: %f", position.z());
            position.z() -= footZChange;
            ROS_INFO("The position z after change is: %f", position.z());
        }

        if(footName == "RR") {
            ROS_INFO("The position x before change is: %f", position.x());
            ROS_INFO("The position y before change is: %f", position.y());
            ROS_INFO("The position z before change is: %f", position.z());
        }

        // TODO: MAP BACK TO HOME POSITION

        position.x() -= (footZChange / 4);
        position.y() += (footZChange / 4);

        if(footName == "RR") {
            ROS_INFO("The position x after change is is: %f", position.x());
            ROS_INFO("The position y after change is is: %f", position.y());
            ROS_INFO("The position z after change is is: %f", position.z
            ());
        }
        footPositions.push_back(position);
    }

    Eigen::Vector3d frFootPosition = footPositions.at(0);
    if(frFootPosition.z() <= -0.33) {
        ROS_INFO("Full length reached");
        actionHalted();
        return BT::NodeStatus::SUCCESS;
    }

    ROS_INFO("================================================");
    ROS_INFO("                                                  ");

    std::vector<double> flattenedLegJoints;
    for (size_t i = 0; i < footPositions.size(); ++i) {
        const auto& position = footPositions.at(i % 4);

        // Solve inverse kinematics and get leg joints
        std::vector<double> legJoints;
        std::string footName = footNames.at(i);

        if (footName == "FR" || footName == "RR") {
            legJoints = ikSolver(position, true);
        } else {
            legJoints = ikSolver(position, false);
        }

        // Check if leg joints are empty
        if (legJoints.empty()) {
            ROS_INFO("Inverse kinematics failed");
            return BT::NodeStatus::RUNNING;
        }

        flattenedLegJoints.insert(flattenedLegJoints.end(), legJoints.begin(), legJoints.end());
    }

    for (int j = 0; j < flattenedLegJoints.size(); j++) {
        ros_manager.setRobotCmd(j, flattenedLegJoints.at(j));
    }

    footPositions.clear();
    flattenedLegJoints.clear();
    return BT::NodeStatus::RUNNING;
}

void RobotDropFootAction::onHalted() {
    actionHalted();
}


/** STAND ACTION */
void RobotStandAction::handleKeyPressed(bool pressed) {
    action_service_manager.setStandKeyPressed(pressed);
}

BT::NodeStatus RobotStandAction::onStart() {
    ROS_INFO("GO1_STAND_STATE_ACTION");
    return actionStart();
}

BT::NodeStatus RobotStandAction::onRunning() {
    return actionRunning(STAND_JOINT_POSITIONS);
}

void RobotStandAction::onHalted() {
    actionHalted();
}


/** RAISE FR LEG ACTION */
void RobotFrRaiseAction::handleKeyPressed(bool pressed) {
    action_service_manager.setFrRaiseKeyPressed(pressed);
}

BT::NodeStatus RobotFrRaiseAction::onStart() {
    ROS_INFO("GO1_RAISE_FR_ACTION");

    Eigen::Vector3d footPosition = getCurrentFootPosition("FR");

    double reductionPercentage = 0.7;
    footPosition.z() -= reductionPercentage * footPosition.z();

    std::vector<double> leg_joints = ikSolver(footPosition, true);
    unitree_legged_msgs::LowState robot_state = ros_manager.getRobotState();

    fr_foot_target_position.clear();
    for (size_t j = 0; j < robot_state.motorState.size(); j++) {
        if (j < leg_joints.size()) {
            fr_foot_target_position.push_back(leg_joints.at(j));
        } else {
            fr_foot_target_position.push_back(robot_state.motorState[j].q);
        }
    }

    return actionStart();
}

BT::NodeStatus RobotFrRaiseAction::onRunning() {
    return actionRunning(fr_foot_target_position);
}

void RobotFrRaiseAction::onHalted() {
    actionHalted();
}


/** GO TO COG ACTION */
void RobotGoToCogAction::handleKeyPressed(bool pressed) {}

BT::NodeStatus RobotGoToCogAction::onStart() {
    ROS_INFO("GO1_GO_TO_COG_ACTION");
    int footPosition = action_service_manager.getRobotFootIndex();
    cog_position = getCOGJointPositions(footPosition);
    return actionStart();
}

BT::NodeStatus RobotGoToCogAction::onRunning() {
    if(cog_position.empty()) {
        ROS_INFO("Cannot go to COG");
        return BT::NodeStatus::SUCCESS;
    }
    return actionRunning(cog_position);
}

void RobotGoToCogAction::onHalted() {
    actionHalted();
}