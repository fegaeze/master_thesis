#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tf/transform_listener.h>
#include <vector>

#include "controller.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace chr = std::chrono;


class Go1Initialized  {
    public:
        Go1Initialized() : controller_(HardwareController::getInstance()) {}

        void registerNodes(BT::BehaviorTreeFactory& factory);

    private:
        bool paramInitialized_ = false;
        HardwareController& controller_;
        BT::NodeStatus robotInitialization();
        BT::NodeStatus robotParamInitialized();
        BT::NodeStatus robotStateReceived();
};

class Go1Stand : public BT::StatefulActionNode {
    public:
        Go1Stand(const std::string& name, const BT::NodeConfig& config) 
            : StatefulActionNode(name, config),
              controller_(HardwareController::getInstance()) {}

        static BT::PortsList providedPorts() {
            return {};
        }

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;

    private:
        int durationCounter_ = 0;
        const double STAND_JOINT_POSITIONS[HardwareController::NUM_OF_JOINTS] = {
            0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
            0.0, 0.67, -1.3, -0.0, 0.67, -1.3
        };

        HardwareController& controller_;
        unitree_legged_msgs::LowState initialState_;
};

class Go1LieDown : public BT::StatefulActionNode {
    public:
        Go1LieDown(const std::string& name, const BT::NodeConfig& config) 
            : StatefulActionNode(name, config),
              controller_(HardwareController::getInstance()) {}

        static BT::PortsList providedPorts() {
            return {};
        }

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;

    private:
        int durationCounter_ = 0;
        const double LIE_DOWN_JOINT_POSITIONS[HardwareController::NUM_OF_JOINTS] = {
            0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
            0.0, 0.67, -1.3, -0.0, 0.67, -1.3
        };

        HardwareController& controller_;
        unitree_legged_msgs::LowState initialState_;
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
