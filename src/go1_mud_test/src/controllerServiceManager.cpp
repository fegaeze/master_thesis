#include "controllerServiceManager.hpp"
#include "config.hpp"


bool ControllerServiceManager::class_initialized = false;
std::map<std::string, double> ControllerServiceManager::pid_gains = Config::RobotController::PID::gains;
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

    controller_type_service_server = nh_.advertiseService(robot_name + "/controller/type", &ControllerServiceManager::controllerTypeServiceCallback, this);
    pid_tuning_service_server = nh_.advertiseService(robot_name + "/controller/pid/gains", &ControllerServiceManager::pidTuningServiceCallback, this);
}

bool ControllerServiceManager::controllerTypeServiceCallback(
    go1_mud_test::ControllerTypeService::Request& req,
    go1_mud_test::ControllerTypeService::Response& res) {
    res.success = true;  
    if (req.type == Config::RobotController::TYPE::PID) {
        setControlMethod(Config::RobotController::TYPE::PID);      
    } else if (req.type == Config::RobotController::TYPE::FIS) {
        setControlMethod(Config::RobotController::TYPE::FIS);
    } else {
        res.success = false;
    }
    return res.success; 
}

bool ControllerServiceManager::pidTuningServiceCallback(
    go1_mud_test::PIDTuningService::Request& req,
    go1_mud_test::PIDTuningService::Response& res) {

    pid_gains["kp_push"] = req.kp_push;
    pid_gains["ki_push"] = req.ki_push;
    pid_gains["kd_push"] = req.kd_push;
    pid_gains["kp_pull"] = req.kp_pull;
    pid_gains["ki_pull"] = req.ki_pull;
    pid_gains["kd_pull"] = req.kd_pull;

    res.success = true;
    return res.success; 
}


std::map<std::string, double> ControllerServiceManager::getPIDGains() {
    return pid_gains;
}

std::string ControllerServiceManager::getControlMethod() {
   return control_method;
}

void ControllerServiceManager::setControlMethod(std::string method) {
   control_method = method;
}