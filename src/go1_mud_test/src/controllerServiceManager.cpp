#include "controllerServiceManager.hpp"
#include "config.hpp"


bool ControllerServiceManager::class_initialized = false;
std::string ControllerServiceManager::control_method = Config::RobotController::PID;

ControllerServiceManager& ControllerServiceManager::getInstance() {
  static ControllerServiceManager instance;
  return instance;
}

ControllerServiceManager& ControllerServiceManager::getInstance(ros::NodeHandle& nh, std::string rname) {
    static ControllerServiceManager instance;
    if (!class_initialized) {
        instance.initialize(nh, rname);
        class_initialized = true;
    }
    return instance;
}

void ControllerServiceManager::initialize(ros::NodeHandle& nh, std::string rname) {
    nh_ = nh;
    robot_name = rname;
    controller_service_server = nh_.advertiseService(robot_name + "/controller", &ControllerServiceManager::controllerCallback, this);
}

bool ControllerServiceManager::controllerCallback(
    go1_mud_test::ControllerService::Request& req,
    go1_mud_test::ControllerService::Response& res) {
    res.success = true;  
    if (req.controller == Config::RobotController::PID) {
        setControlMethod(Config::RobotController::PID);      
    } else if (req.controller == Config::RobotController::FIS) {
        setControlMethod(Config::RobotController::FIS);
    } else {
        res.success = false;
    }

    return res.success; 
}


std::string ControllerServiceManager::getControlMethod() {
   return control_method;
}

void ControllerServiceManager::setControlMethod(std::string method) {
   control_method = method;
}