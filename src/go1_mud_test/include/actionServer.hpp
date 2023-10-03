#pragma once

#include <ros/ros.h>
#include "behaviortree_cpp/bt_factory.h"
#include "go1_mud_test/ActionService.h"

class ActionServer {
    public:
        static ActionServer& getInstance();
    
        void initialize(ros::NodeHandle& nh, std::string robot_name);
        void registerNodes(BT::BehaviorTreeFactory &factory);
        void setStandKeyPressed(bool pressed);
        void setLieDownKeyPressed(bool pressed);
        void setFrRaiseKeyPressed(bool pressed);
        void setFlRaiseKeyPressed(bool pressed);
        void setRrRaiseKeyPressed(bool pressed);
        void setRlRaiseKeyPressed(bool pressed);

    private:
        ActionServer() : initialized_(false) {};
        ActionServer(const ActionServer&) = delete;
        ActionServer& operator=(const ActionServer&) = delete;

        bool initialized_;
        std::string robot_name_;
        bool lieDownKeyPressed_ = false;
        bool standKeyPressed_ = false;
        bool frRaiseKeyPressed_ = false;
        bool flRaiseKeyPressed_ = false;
        bool rrRaiseKeyPressed_ = false;
        bool rlRaiseKeyPressed_ = false;

        ros::NodeHandle nh_;
        ros::ServiceServer robotActionServer_;

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