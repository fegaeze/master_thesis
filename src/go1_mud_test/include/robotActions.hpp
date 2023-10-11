#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tf/transform_listener.h>
#include <vector>

#include "behaviortree_cpp/action_node.h"
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
        static const double STAND_JOINT_POSITIONS[Config::NUM_OF_JOINTS];
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
        static const double LIE_DOWN_JOINT_POSITIONS[Config::NUM_OF_JOINTS];
        void handleKeyPressed(bool pressed) override;
};


class MudTest {
    private:
        double hipLength_, thighLength_, calfLength_;
        tf::Vector3 calculateCoGPosition(const std::vector<tf::Vector3>& feet, int liftedLeg);
        std::vector<double> ikSolver(const Eigen::Matrix4d& footPose, bool isRight=true);
        tf::TransformListener listener;
        double triangleArea(const tf::Vector3& p1, const tf::Vector3& p2, const tf::Vector3& p3);

    public:
        MudTest();
        std::vector<double> getCOGJointPositions(int liftedLeg=0);
};
