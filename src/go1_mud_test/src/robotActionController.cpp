#include "robotActionController.hpp"


unitree_legged_msgs::LowState RobotActionController::last_known_state{};

void RobotActionController::interpolateJoints(const double *targetPos) {
    double percent = static_cast<double>(duration_counter) / static_cast<double>(MOVEMENT_DURATION_MS);
    for (int j = 0; j < Config::NUM_OF_JOINTS; j++) {
        ros_manager.setRobotCmd(j, (last_known_state.motorState[j].q * (1 - percent)) + (targetPos[j] * percent));
    }
}

BT::NodeStatus RobotActionController::actionStart() {
    last_known_state = ros_manager.getRobotState();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RobotActionController::actionRunning(const double *targetPos) {
    interpolateJoints(targetPos);
    duration_counter += 1;
    if(duration_counter >= MOVEMENT_DURATION_MS) {
        duration_counter = 0;
        handleKeyPressed(false);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void RobotActionController::actionHalted() {
    duration_counter = 0;
    handleKeyPressed(false);
}