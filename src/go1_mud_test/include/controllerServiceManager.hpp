#pragma once

#include <ros/ros.h>

#include "behaviortree_cpp/bt_factory.h"

#include "config.hpp"
#include "go1_mud_test/ControllerTypeService.h"
#include "go1_mud_test/PIDTuningService.h"

class ControllerServiceManager {
    public:
        static ControllerServiceManager& getInstance();
        static ControllerServiceManager& getInstance(ros::NodeHandle& nh, std::string robot_name);

        void initialize(ros::NodeHandle& nh, std::string rname);

        std::map<std::string, double> getPIDGains();

        std::string getControlMethod();  
        void setControlMethod(std::string control_method);  

    private:
        ControllerServiceManager() {};
        ControllerServiceManager(const ControllerServiceManager&) = delete;
        ControllerServiceManager& operator=(const ControllerServiceManager&) = delete;

        static bool class_initialized;
        static std::map<std::string, double> pid_gains;
        
        std::string control_method = Config::RobotController::TYPE::PID;
        std::string robot_name;

        ros::ServiceServer controller_type_service_server;
        ros::ServiceServer pid_tuning_service_server;

        ros::NodeHandle nh_;

        bool controllerTypeServiceCallback(
            go1_mud_test::ControllerTypeService::Request& req,
            go1_mud_test::ControllerTypeService::Response& res
        );
        bool pidTuningServiceCallback(
            go1_mud_test::PIDTuningService::Request& req,
            go1_mud_test::PIDTuningService::Response& res
        );

};