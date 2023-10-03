#include "actionServer.hpp"

ActionServer& ActionServer::getInstance() {
  static ActionServer instance;
  return instance;
}

void ActionServer::initialize(ros::NodeHandle& nh, std::string robot_name) {
    if (!initialized_) {
        nh_ = nh;
        initialized_ = true;
        robot_name_ = robot_name;
        robotActionServer_ = nh_.advertiseService(robot_name_ + "/action", &ActionServer::actionCallback, this);
    }
}

bool ActionServer::actionCallback(
    go1_mud_test::ActionService::Request& req,
    go1_mud_test::ActionService::Response& res) {
    std::string action = req.action;
    res.success = true;  
    if (action == "STAND") {
        setStandKeyPressed(true);      
    } else if (action == "LIE_DOWN") {
        setLieDownKeyPressed(true);
    } else if (action == "RAISE_FR_FOOT") {
        setFrRaiseKeyPressed(true);
    } else if (action == "RAISE_FL_FOOT") {
        setFlRaiseKeyPressed(true);
    } else if (action == "RAISE_RR_FOOT") {
        setRrRaiseKeyPressed(true);
    } else if (action == "RAISE_RL_FOOT") {
        setRlRaiseKeyPressed(true);
    } else {
        res.success = false;
    }

    return res.success; 
}

void ActionServer::setStandKeyPressed(bool pressed) {
    standKeyPressed_ = pressed;
}
void ActionServer::setLieDownKeyPressed(bool pressed) {
    lieDownKeyPressed_ = pressed;
}
void ActionServer::setFrRaiseKeyPressed(bool pressed) {
    frRaiseKeyPressed_ = pressed;
}
void ActionServer::setFlRaiseKeyPressed(bool pressed) {
    flRaiseKeyPressed_ = pressed;
}
void ActionServer::setRrRaiseKeyPressed(bool pressed) {
    rrRaiseKeyPressed_ = pressed;
}
void ActionServer::setRlRaiseKeyPressed(bool pressed) {
    rlRaiseKeyPressed_ = pressed;
}


BT::NodeStatus ActionServer::lieDownKeyPressed() {
    if(lieDownKeyPressed_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServer::standKeyPressed() {
    if(standKeyPressed_) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServer::frRaiseKeyPressed() {
    if(frRaiseKeyPressed_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServer::flRaiseKeyPressed() {
    if(flRaiseKeyPressed_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServer::rrRaiseKeyPressed() {
    if(rrRaiseKeyPressed_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionServer::rlRaiseKeyPressed() {
    if(rlRaiseKeyPressed_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void ActionServer::registerNodes(BT::BehaviorTreeFactory &factory) {
    factory.registerSimpleAction("LieDownKeyPressed", std::bind(&ActionServer::lieDownKeyPressed, this));
    factory.registerSimpleAction("StandKeyPressed", std::bind(&ActionServer::standKeyPressed, this));
    factory.registerSimpleAction("FrRaiseKeyPressed", std::bind(&ActionServer::frRaiseKeyPressed, this));
    factory.registerSimpleAction("FlRaiseKeyPressed", std::bind(&ActionServer::flRaiseKeyPressed, this));
    factory.registerSimpleAction("RrRaiseKeyPressed", std::bind(&ActionServer::rrRaiseKeyPressed, this));
    factory.registerSimpleAction("RlRaiseKeyPressed", std::bind(&ActionServer::rlRaiseKeyPressed, this));
}
