#include "actionServiceManager.hpp"
#include "config.hpp"


ActionServiceManager& ActionServiceManager::getInstance() {
  static ActionServiceManager instance;
  return instance;
}

ActionServiceManager& ActionServiceManager::(ros::NodeHandle& nh, std::string robot_name) {
    if (!class_initialized) {
        static ActionServiceManager instance;
        nh_ = nh;
        class_initialized = true;
        robot_name = robot_name;
        action_service_server = nh_.advertiseService(robot_name_ + "/action", &ActionServiceManager::actionCallback, this);
        return instance;
    }
}

bool ActionServiceManager::actionCallback(
    go1_mud_test::ActionService::Request& req,
    go1_mud_test::ActionService::Response& res) {
    std::string action = req.action;

    res.success = true;  
    if (req.action == Config::RobotAction::STAND) {
        setStandKeyPressed(true);      
    } else if (req.action == Config::RobotAction::LIE_DOWN) {
        setLieDownKeyPressed(true);
    } else if (req.action == Config::RobotAction::RAISE_FR_FOOT) {
        setFrRaiseKeyPressed(true);
    } else if (req.action == Config::RobotAction::RAISE_FL_FOOT) {
        setFlRaiseKeyPressed(true);
    } else if (req.action == Config::RobotAction::RAISE_RR_FOOT) {
        setRrRaiseKeyPressed(true);
    } else if (req.action == Config::RobotAction::RAISE_RL_FOOT) {
        setRlRaiseKeyPressed(true);
    } else {
        res.success = false;
    }

    return res.success; 
}


void ActionServiceManager::setStandKeyPressed(bool pressed) {
    stand_key_pressed = pressed;
}
void ActionServiceManager::setLieDownKeyPressed(bool pressed) {
    lie_down_key_pressed = pressed;
}
void ActionServiceManager::setFrRaiseKeyPressed(bool pressed) {
    fr_raise_key_pressed = pressed;
}
void ActionServiceManager::setFlRaiseKeyPressed(bool pressed) {
    fl_raise_key_pressed = pressed;
}
void ActionServiceManager::setRrRaiseKeyPressed(bool pressed) {
    rr_raise_key_pressed = pressed;
}
void ActionServiceManager::setRlRaiseKeyPressed(bool pressed) {
    rl_raise_key_pressed = pressed;
}


BT::NodeStatus ActionServiceManager::lieDownKeyPressed() {
    if(lie_down_key_pressed) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServiceManager::standKeyPressed() {
    if(stand_key_pressed) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServiceManager::frRaiseKeyPressed() {
    if(fr_raise_key_pressed) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServiceManager::flRaiseKeyPressed() {
    if(fl_raise_key_pressed) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServiceManager::rrRaiseKeyPressed() {
    if(rr_raise_key_pressed) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServiceManager::rlRaiseKeyPressed() {
    if(rl_raise_key_pressed) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void ActionServiceManager::registerNodes(BT::BehaviorTreeFactory &factory) {
    factory.registerSimpleAction("LieDownKeyPressed", std::bind(&ActionServiceManager::lieDownKeyPressed, this));
    factory.registerSimpleAction("StandKeyPressed", std::bind(&ActionServiceManager::standKeyPressed, this));
    factory.registerSimpleAction("FrRaiseKeyPressed", std::bind(&ActionServiceManager::frRaiseKeyPressed, this));
    factory.registerSimpleAction("FlRaiseKeyPressed", std::bind(&ActionServiceManager::flRaiseKeyPressed, this));
    factory.registerSimpleAction("RrRaiseKeyPressed", std::bind(&ActionServiceManager::rrRaiseKeyPressed, this));
    factory.registerSimpleAction("RlRaiseKeyPressed", std::bind(&ActionServiceManager::rlRaiseKeyPressed, this));
}
