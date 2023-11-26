#include "robotActions.hpp"


Eigen::Vector3d RobotDropFootAction::initial_foot_position = Eigen::Vector3d();
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

    Eigen::Vector3d footPosition = getCurrentFootPosition("FR");
    initial_foot_position = footPosition;

    configureFIS();
    configurePID(current_time);
    return actionStart();
}

BT::NodeStatus RobotDropFootAction::onRunning() {
    double force_feedback = -ros_manager.getCurrentForce();
    Eigen::Vector3d footPosition = getCurrentFootPosition("FR");

    if(contact_initiated == false && force_feedback > 3.0) {
        initial_foot_position = footPosition;
        contact_initiated = true;
    }
    
    ROS_INFO("================================================");
    ROS_INFO("The current foot position: %lf", footPosition.z());
    ROS_INFO("Force Feedback (f): %lf", force_feedback);

    double percent = 0.0;
    if(contact_initiated) {
        double control_cmd = runControlMethod(force_feedback, initial_foot_position.z(), footPosition.z());

        ROS_INFO("The control command is: %lf", control_cmd);

        if(control_cmd == -1) {
            ROS_INFO("Controller returned bad values");
            return BT::NodeStatus::SUCCESS;
        }

        percent = (std::abs(footPosition.z()) - std::abs(initial_foot_position.z())) / (std::abs(Config::LEG_Z_POS_LIMIT) - std::abs(initial_foot_position.z()));
        footPosition.x() = (initial_foot_position.x() * (1 - percent)) + (Config::LEG_X_POS_LIMIT * percent);
        footPosition.y() = (initial_foot_position.y() * (1 - percent)) + (Config::LEG_Y_POS_LIMIT * percent);
        footPosition.z() -= control_cmd;
    } else {
        footPosition.x() = initial_foot_position.x();
        footPosition.y() = initial_foot_position.y();
        footPosition.z() -= (0.1 / 50);
        ROS_INFO("The X position is: %lf", footPosition.x());
        ROS_INFO("The Y position is: %lf", footPosition.y());
    }

    ROS_INFO("The proposed foot position: %lf", footPosition.z());
    ROS_INFO("================================================");
    ROS_INFO("                                                ");

    if(footPosition.z() <= Config::LEG_Z_POS_LIMIT) {
        actionHalted();
        return BT::NodeStatus::SUCCESS;
    }

    std::vector<double> leg_joints = ikSolver(footPosition, true);
    if(leg_joints.empty()) {
        return BT::NodeStatus::RUNNING;
    }
    
    for (int j = 0; j < 3; j++) {
        ros_manager.setRobotCmd(j, leg_joints.at(j));
    }

    if(contact_initiated) {
        for (int j = 3; j < Config::NUM_OF_JOINTS; j++) {
            ros_manager.setRobotCmd(j, (RobotActionController::last_known_state.motorState[j].q * (1 - percent)) + (RobotStandAction::STAND_JOINT_POSITIONS[j] * percent));
        }
    }

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