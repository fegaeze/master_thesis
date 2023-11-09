#pragma once

#include <ros/ros.h>

#include "behaviortree_cpp/bt_factory.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "go1_mud_test/ActionService.h"

class ActionServiceManager {
    public:
        static ActionServiceManager& getInstance();
        static ActionServiceManager& getInstance(ros::NodeHandle& nh, std::string robot_name);

        void initialize(ros::NodeHandle& nh, std::string rname);
        void registerNodes(BT::BehaviorTreeFactory &factory);

        int getRobotFootIndex();
        void setDropFootKeyPressed(bool pressed);
        void setFrRaiseKeyPressed(bool pressed);
        void setFlRaiseKeyPressed(bool pressed);
        void setLieDownKeyPressed(bool pressed);
        void setRrRaiseKeyPressed(bool pressed);
        void setRlRaiseKeyPressed(bool pressed);
        void setStandKeyPressed(bool pressed);

    private:
        ActionServiceManager() {};
        ActionServiceManager(const ActionServiceManager&) = delete;
        ActionServiceManager& operator=(const ActionServiceManager&) = delete;

        static bool class_initialized;
        static bool drop_foot_key_pressed;
        static bool fr_raise_key_pressed;
        static bool fl_raise_key_pressed;
        static bool lie_down_key_pressed;
        static bool rr_raise_key_pressed;
        static bool rl_raise_key_pressed;
        static bool stand_key_pressed;
        static int robot_foot_idx;
        std::string robot_name;

        ros::NodeHandle nh_;
        ros::ServiceServer action_service_server;

        bool actionCallback(
            go1_mud_test::ActionService::Request& req,
            go1_mud_test::ActionService::Response& res
        );

        BT::NodeStatus dropFootKeyPressed();
        BT::NodeStatus flRaiseKeyPressed();
        BT::NodeStatus rrRaiseKeyPressed();
        BT::NodeStatus lieDownKeyPressed();
        BT::NodeStatus frRaiseKeyPressed();
        BT::NodeStatus rlRaiseKeyPressed();
        BT::NodeStatus standKeyPressed();

};