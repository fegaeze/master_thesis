#include "robotActions.hpp"


std::vector<double> RobotGoToCogAction::cog_position = std::vector<double>();
std::vector<double> RobotFrRaiseAction::fr_raise_target_position = std::vector<double>();

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

    double reductionPercentage = 0.5;
    footPosition.z() -= reductionPercentage * footPosition.z();

    std::vector<double> leg_joints = ikSolver(footPosition, true);

    unitree_legged_msgs::LowState robot_state = ros_manager.getRobotState();
    for (size_t i = 0; i < robot_state.motorState.size(); i++) {
        fr_raise_target_position.push_back(robot_state.motorState[i].q);
    }

    for (size_t i = 0; i < leg_joints.size(); i++) {
        fr_raise_target_position.at(i) = leg_joints.at(i);
    }

    return actionStart();
}

BT::NodeStatus RobotFrRaiseAction::onRunning() {
    return actionRunning(fr_raise_target_position);
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
    return actionRunning(cog_position);
}

void RobotGoToCogAction::onHalted() {
    actionHalted();
}