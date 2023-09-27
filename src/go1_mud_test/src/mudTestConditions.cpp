#include "mudTestConditions.hpp"


namespace Go1LieDownCond {

    BT::NodeStatus lieDownKeyPressed() {
        ROS_INFO("GO1_LIE_DOWN_KEY_PRESSED");
        return BT::NodeStatus::SUCCESS;
    }

}

namespace Go1StandCond {

    BT::NodeStatus standKeyPressed() {
        HardwareController& controller = HardwareController::getInstance();
        ROS_INFO("GO1_STAND_KEY_PRESSED");

        bool pressed = controller.getKeyPressed();
        if(pressed) {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

}