#include "mudTestConditions.hpp"


namespace Go1LieDownCond {

    BT::NodeStatus lieDownKeyPressed() {
        ROS_INFO("GO1_LIE_DOWN_KEY_PRESSED");
        return BT::NodeStatus::SUCCESS;
    }

}

namespace Go1StandCond {

    BT::NodeStatus standKeyPressed() {
        ROS_INFO("GO1_STAND_KEY_PRESSED");
        return BT::NodeStatus::SUCCESS;
    }

}