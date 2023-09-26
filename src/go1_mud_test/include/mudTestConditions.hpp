#pragma once

#include "controller.hpp"
#include "behaviortree_cpp/action_node.h"


namespace Go1LieDownCond {
    BT::NodeStatus lieDownKeyPressed();
}

namespace Go1StandCond {
    BT::NodeStatus standKeyPressed();
}