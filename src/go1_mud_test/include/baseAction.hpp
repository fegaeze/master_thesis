#pragma once

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "unitree_legged_msgs/LowState.h"

#include "actionServer.hpp"
#include "rosInterfaceManager.hpp"


class Go1Base : public BT::StatefulActionNode {
    public:
        Go1Base(const std::string& name, const BT::NodeConfig& config) 
            : StatefulActionNode(name, config),
            rosManager_(ROSInterfaceManager::getInstance()),
            actionServer_(ActionServer::getInstance()) {}

        static BT::PortsList providedPorts() {
            return {};
        }

        unitree_legged_msgs::LowState getLastKnownState();
        void setLastKnownState(unitree_legged_msgs::LowState state);

        virtual BT::NodeStatus onStart() override = 0;
        virtual BT::NodeStatus onRunning() override = 0;
        virtual void onHalted() override = 0;

    protected:
        static unitree_legged_msgs::LowState lastKnownState_;

        int durationCounter_ = 0;
        ActionServer& actionServer_;
        ROSInterfaceManager& rosManager_;
};