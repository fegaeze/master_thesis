#include "robotActions.hpp"


std::vector<double> RobotGoToCogAction::cog_joint_positions = std::vector<double>();
const std::vector<double> RobotStandAction::STAND_JOINT_POSITIONS = {
    0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
    0.0, 0.67, -1.3, -0.0, 0.67, -1.3
};
const std::vector<double> RobotLieDownAction::LIE_DOWN_JOINT_POSITIONS = {
    -0.5, 1.15, -2.7, 0.5, 1.15, -2.7,
    -0.5, 1.15, -2.7, 0.5, 1.15, -2.7
};
const std::vector<double> RobotFrRaiseAction::FR_RAISE_JOINT_POSITIONS = {
    0.0, 0.67, -2.5, -0.0, 0.67, -1.3,
    0.0, 0.67, -1.3, -0.0, 0.67, -1.3
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
    return actionStart();
}

BT::NodeStatus RobotFrRaiseAction::onRunning() {
    return actionRunning(FR_RAISE_JOINT_POSITIONS);
}

void RobotFrRaiseAction::onHalted() {
    actionHalted();
}


/** GO TO COG ACTION */
void RobotGoToCogAction::handleKeyPressed(bool pressed) {}

BT::NodeStatus RobotGoToCogAction::onStart() {
    ROS_INFO("GO1_GO_TO_COG_ACTION");
    int footPosition = action_service_manager.getRobotFootIndex();
    cog_joint_positions = getCOGJointPositions(footPosition);
    // ROS_INFO("FR: %f, %f, %f", cog_joint_positions[0], cog_joint_positions[1], cog_joint_positions[2]);
    // ROS_INFO("FL: %f, %f, %f", cog_joint_positions[3], cog_joint_positions[4], cog_joint_positions[5]);
    // ROS_INFO("RR: %f, %f, %f", cog_joint_positions[6], cog_joint_positions[7], cog_joint_positions[8]);
    // ROS_INFO("RL: %f, %f, %f", cog_joint_positions[9], cog_joint_positions[10], cog_joint_positions[11]);
    return actionStart();
}

BT::NodeStatus RobotGoToCogAction::onRunning() {
    return actionRunning(cog_joint_positions);
}

void RobotGoToCogAction::onHalted() {
    actionHalted();
}