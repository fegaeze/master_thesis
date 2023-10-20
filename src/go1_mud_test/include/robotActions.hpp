#pragma once

#include "behaviortree_cpp/bt_factory.h"

#include "robotActionController.hpp"

namespace chr = std::chrono;

class RobotInitializationAction  {
    public:
        RobotInitializationAction() : 
            ros_manager(ROSInterfaceManager::getInstance()) {}

        void registerNodes(BT::BehaviorTreeFactory& factory);

    private:
        bool robot_initialized = false;
        ROSInterfaceManager& ros_manager;
        BT::NodeStatus robotInitialization();
        BT::NodeStatus robotInitialized();
        BT::NodeStatus robotStateReceived();
};

class RobotStandAction : public RobotActionController {
    public:
        RobotStandAction(const std::string& name, const BT::NodeConfig& config) 
            : RobotActionController(name, config) {}

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        static const std::vector<double> STAND_JOINT_POSITIONS;
        void handleKeyPressed(bool pressed) override;
};

class RobotLieDownAction : public RobotActionController {
    public:
        RobotLieDownAction(const std::string& name, const BT::NodeConfig& config) 
            : RobotActionController(name, config) {}

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        static const std::vector<double> LIE_DOWN_JOINT_POSITIONS;
        void handleKeyPressed(bool pressed) override;
};

class RobotFrRaiseAction : public RobotActionController {
    public:
        RobotFrRaiseAction(const std::string& name, const BT::NodeConfig& config) 
            : RobotActionController(name, config) {}

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        static const std::vector<double> FR_RAISE_JOINT_POSITIONS;
        void handleKeyPressed(bool pressed) override;
};

class RobotGoToCogAction : public RobotActionController {
    public:
        RobotGoToCogAction(const std::string& name, const BT::NodeConfig& config) 
            : RobotActionController(name, config) {}

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        static std::vector<double> cog_joint_positions;
        void handleKeyPressed(bool pressed) override;
};