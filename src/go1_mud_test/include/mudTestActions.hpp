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

#include "baseAction.hpp"

namespace chr = std::chrono;

class Go1Initialized  {
    public:
        Go1Initialized() : 
            rosManager_(ROSInterfaceManager::getInstance()) {}

        void registerNodes(BT::BehaviorTreeFactory& factory);

    private:
        bool paramInitialized_ = false;
        ROSInterfaceManager& rosManager_;
        BT::NodeStatus robotInitialization();
        BT::NodeStatus robotParamInitialized();
        BT::NodeStatus robotStateReceived();
};

class Go1Stand : public Go1Base {
public:
    Go1Stand(const std::string& name, const BT::NodeConfig& config) 
        : Go1Base(name, config) {}

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    const double STAND_JOINT_POSITIONS[ROSInterfaceManager::NUM_OF_JOINTS] = {
        0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
        0.0, 0.67, -1.3, -0.0, 0.67, -1.3
    };
};

class Go1LieDown : public Go1Base {
public:
    Go1LieDown(const std::string& name, const BT::NodeConfig& config) 
        : Go1Base(name, config) {}

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    const double LIE_DOWN_JOINT_POSITIONS[ROSInterfaceManager::NUM_OF_JOINTS] = {
        -0.5, 1.15, -2.7, -0.5, 1.15, -2.7,
        -0.5, 1.15, -2.7, -0.5, 1.15, -2.7
    };
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
