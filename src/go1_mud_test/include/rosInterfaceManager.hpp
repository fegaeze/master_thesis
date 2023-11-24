#pragma once

#include <iostream>
#include <ros/ros.h>

#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "go1_mud_test/ControllerData.h"

#include "config.hpp"

class ROSInterfaceManager {

    public:
        static ROSInterfaceManager& getInstance();
        static ROSInterfaceManager& getInstance(ros::NodeHandle& nh, std::string rname);
        
        double getCurrentForce();
        unitree_legged_msgs::LowState getRobotState();

        void initialize(ros::NodeHandle& nh, std::string rname);
        void publishControllerData(double force_error, double current_force, double foot_displacement, double ctrl_output, double initial_position, double current_position);
        void publishRobotCmd();
        void setRobotCmd(int joint, double pos);
        void setRobotParams();

    private:

        ROSInterfaceManager() {};
        ROSInterfaceManager(const ROSInterfaceManager&) = delete;
        ROSInterfaceManager& operator=(const ROSInterfaceManager&) = delete;

        static bool class_initialized;
        static unitree_legged_msgs::LowCmd robot_cmd;
        static unitree_legged_msgs::LowState robot_state;
        static geometry_msgs::WrenchStamped robot_force_state;
        static sensor_msgs::JointState joint_state;
        
        ros::NodeHandle nh_;
        ros::Publisher joint_state_pub, real_robot_cmd_pub, sim_robot_cmd_pub[Config::NUM_OF_JOINTS], controller_data_analysis_pub;
        ros::Subscriber imu_sub, force_sub, real_robot_state_sub, sim_robot_state_sub[Config::NUM_OF_JOINTS];

        std::string robot_name;
        
        void setPublishers();
        void setSubscriptions();

        void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &msg);
        void imuCallback(const sensor_msgs::Imu &msg);
        void forceCallback(const geometry_msgs::WrenchStamped &msg);

        void FRhipCallback(const unitree_legged_msgs::MotorState &msg);
        void FRthighCallback(const unitree_legged_msgs::MotorState &msg);
        void FRcalfCallback(const unitree_legged_msgs::MotorState &msg);

        void FLhipCallback(const unitree_legged_msgs::MotorState &msg);
        void FLthighCallback(const unitree_legged_msgs::MotorState &msg);
        void FLcalfCallback(const unitree_legged_msgs::MotorState &msg);

        void RRhipCallback(const unitree_legged_msgs::MotorState &msg);
        void RRthighCallback(const unitree_legged_msgs::MotorState &msg);
        void RRcalfCallback(const unitree_legged_msgs::MotorState &msg);

        void RLhipCallback(const unitree_legged_msgs::MotorState &msg);
        void RLthighCallback(const unitree_legged_msgs::MotorState &msg);
        void RLcalfCallback(const unitree_legged_msgs::MotorState &msg);
};
