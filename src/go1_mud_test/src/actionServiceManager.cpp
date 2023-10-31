#include "actionServiceManager.hpp"
#include "config.hpp"


bool ActionServiceManager::class_initialized = false;
bool ActionServiceManager::lie_down_key_pressed = false;
bool ActionServiceManager::stand_key_pressed = false;
bool ActionServiceManager::fr_raise_key_pressed = false;
bool ActionServiceManager::fl_raise_key_pressed = false;
bool ActionServiceManager::rr_raise_key_pressed = false;
bool ActionServiceManager::rl_raise_key_pressed = false;

int ActionServiceManager::robot_foot_idx = UNITREE_LEGGED_SDK::FR_;

ActionServiceManager& ActionServiceManager::getInstance() {
  static ActionServiceManager instance;
  return instance;
}

ActionServiceManager& ActionServiceManager::getInstance(ros::NodeHandle& nh, std::string rname) {
    static ActionServiceManager instance;
    if (!class_initialized) {
        instance.initialize(nh, rname);
        class_initialized = true;
    }
    return instance;
}

void ActionServiceManager::initialize(ros::NodeHandle& nh, std::string rname) {
    nh_ = nh;
    robot_name = rname;
    action_service_server = nh_.advertiseService(robot_name + "/action", &ActionServiceManager::actionCallback, this);
}

bool ActionServiceManager::actionCallback(
    go1_mud_test::ActionService::Request& req,
    go1_mud_test::ActionService::Response& res) {
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

int ActionServiceManager::getRobotFootIndex() {
    return robot_foot_idx;
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
        robot_foot_idx = UNITREE_LEGGED_SDK::FR_;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServiceManager::flRaiseKeyPressed() {
    if(fl_raise_key_pressed) {
        robot_foot_idx = UNITREE_LEGGED_SDK::FL_;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServiceManager::rrRaiseKeyPressed() {
    if(rr_raise_key_pressed) {
        robot_foot_idx =UNITREE_LEGGED_SDK::RR_;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServiceManager::rlRaiseKeyPressed() {
    if(rl_raise_key_pressed) {
        robot_foot_idx = UNITREE_LEGGED_SDK::RL_;
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
