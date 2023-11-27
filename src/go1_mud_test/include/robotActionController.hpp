#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "evaluatefis.h"
#include "behaviortree_cpp/action_node.h"
#include "unitree_legged_msgs/LowState.h"

#include "actionServiceManager.hpp"
#include "controllerServiceManager.hpp"
#include "config.hpp"
#include "rosInterfaceManager.hpp"


class RobotActionController : public BT::StatefulActionNode {
    public:
        
        RobotActionController(const std::string& name, const BT::NodeConfig& config) 
            : StatefulActionNode(name, config),
            ros_manager(ROSInterfaceManager::getInstance()),
            action_service_manager(ActionServiceManager::getInstance()),
            controller_service_manager(ControllerServiceManager::getInstance()),
            buffer(),
            tfl(buffer) {}

        static BT::PortsList providedPorts() {
            return {};
        }

        virtual BT::NodeStatus onStart() override = 0;
        virtual BT::NodeStatus onRunning() override = 0;
        virtual void onHalted() override = 0;

    protected:
        ActionServiceManager& action_service_manager;
        ControllerServiceManager& controller_service_manager;
        ROSInterfaceManager& ros_manager;
        virtual void handleKeyPressed(bool pressed) = 0;

        void actionHalted();
        BT::NodeStatus actionStart();
        BT::NodeStatus actionRunning(std::vector<double>& targetPos);
        BT::NodeStatus actionRunning(const std::vector<double>& targetPos);

        bool contact_initiated = false;
        bool controller_initiated = false;
        double controller_initial_output = 0.0;
        
        std::vector<double> getCOGJointPositions(int liftedLeg);
        Eigen::Vector3d getCurrentFootPosition(const std::string& legName);
        void configurePID(ros::Time& current_time);
        void configureFIS();
        void updateStiffness(double foot_displacement, double current_force);
        double filter(double input);

        double runControlMethod(double feedbackForce, double initial_position, double current_position);
        double calculatePIDControlOutput(double feedbackForce, double error, ros::Time& currentTime);
        double calculateFISControlOutput(double error, double foot_displacement, double current_force);
        std::vector<double> ikSolver(const Eigen::Vector3d& footPosition, bool isRight);
        void interpolateJoints(std::vector<double>& targetPos);
        void interpolateJoints(const std::vector<double>& targetPos);

        static unitree_legged_msgs::LowState last_known_state;
    private:
        static constexpr int MOVEMENT_DURATION_MS = 5 * Config::LOOP_RATE_HZ;

        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener tfl;

        int duration_counter = 0;
        int rate_command_counter = 0;

        double error_cumulative = 0;
        double prev_command = 0;
        double prev_error = Config::FORCE_CMD_SETPOINT;
        ros::Time prev_time = ros::Time::now();

        double mean_displacement = 0.0;
        double mean_force = 0.0;
        int num_data_points = 0;
        double numerator = 0.0;
        double denominator = 0.0;

        tf2::Vector3 calculateCoGPosition(const std::vector<tf2::Vector3>& feet, int liftedLeg);
        double calculateFeetArea(const tf2::Vector3& p1, const tf2::Vector3& p2, const tf2::Vector3& p3);
        bool isJointsCloseToTarget(const unitree_legged_msgs::LowState& currentState, const std::vector<double>& targetPos);
};