#pragma once

#include <ros/ros.h>

#include "behaviortree_cpp/bt_factory.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "go1_mud_test/ControllerService.h"

class ControllerServiceManager {
    public:
        static ControllerServiceManager& getInstance();
        static ControllerServiceManager& getInstance(ros::NodeHandle& nh, std::string robot_name);

        void initialize(ros::NodeHandle& nh, std::string rname);

        std::string getControlMethod();  
        void setControlMethod(std::string control_method);  

    private:
        ControllerServiceManager() {};
        ControllerServiceManager(const ControllerServiceManager&) = delete;
        ControllerServiceManager& operator=(const ControllerServiceManager&) = delete;

        static bool class_initialized;
        static std::string control_method;
        std::string robot_name;

        ros::NodeHandle nh_;
        ros::ServiceServer controller_service_server;

        bool controllerCallback(
            go1_mud_test::ControllerService::Request& req,
            go1_mud_test::ControllerService::Response& res
        );

};