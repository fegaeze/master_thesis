#pragma once

#include <ros/ros.h>
#include "behaviortree_cpp/bt_factory.h"
#include "go1_mud_test/ActionService.h"

class ActionServiceManager {
    public:
        static ActionServiceManager& getInstance();
        static ActionServiceManager& getInstance(ros::NodeHandle& nh, std::string robot_name);

        void registerNodes(BT::BehaviorTreeFactory &factory);

        void setStandKeyPressed(bool pressed);
        void setLieDownKeyPressed(bool pressed);
        void setFrRaiseKeyPressed(bool pressed);
        void setFlRaiseKeyPressed(bool pressed);
        void setRrRaiseKeyPressed(bool pressed);
        void setRlRaiseKeyPressed(bool pressed);

    private:
        ActionServiceManager() {};
        ActionServiceManager(const ActionServiceManager&) = delete;
        ActionServiceManager& operator=(const ActionServiceManager&) = delete;

        bool class_initialized;
        std::string robot_name;
        bool lie_down_key_pressed = false;
        bool stand_key_pressed = false;
        bool fr_raise_key_pressed = false;
        bool fl_raise_key_pressed = false;
        bool rr_raise_key_pressed = false;
        bool rl_raise_key_pressed = false;

        ros::NodeHandle nh_;
        ros::ServiceServer action_service_server;

        bool actionCallback(
            go1_mud_test::ActionService::Request& req,
            go1_mud_test::ActionService::Response& res
        );

        BT::NodeStatus lieDownKeyPressed();
        BT::NodeStatus standKeyPressed();
        BT::NodeStatus frRaiseKeyPressed();
        BT::NodeStatus flRaiseKeyPressed();
        BT::NodeStatus rrRaiseKeyPressed();
        BT::NodeStatus rlRaiseKeyPressed();

};