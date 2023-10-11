#pragma once

#include "behaviortree_cpp/action_node.h"
#include "unitree_legged_msgs/LowState.h"

#include "actionServiceManager.hpp"
#include "config.hpp"
#include "rosInterfaceManager.hpp"


class RobotActionController : public BT::StatefulActionNode {
    public:
        
        RobotActionController(const std::string& name, const BT::NodeConfig& config) 
            : StatefulActionNode(name, config),
            ros_manager(ROSInterfaceManager::getInstance()),
            action_service_manager(ActionServiceManager::getInstance()) {}

        static BT::PortsList providedPorts() {
            return {};
        }

        virtual BT::NodeStatus onStart() override = 0;
        virtual BT::NodeStatus onRunning() override = 0;
        virtual void onHalted() override = 0;

    protected:
    
        ActionServiceManager& action_service_manager;
        virtual void handleKeyPressed(bool pressed) = 0;

        void actionHalted();
        BT::NodeStatus actionStart();
        BT::NodeStatus actionRunning(const double *targetPos);

    private:
        static constexpr int MOVEMENT_DURATION_MS = 5 * Config::LOOP_RATE_HZ;
        static unitree_legged_msgs::LowState last_known_state;

        int duration_counter = 0;
        ROSInterfaceManager& ros_manager;

        void interpolateJoints(const double *targetPos);
};