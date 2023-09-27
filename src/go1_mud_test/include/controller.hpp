#pragma once

#include <iostream>
#include <ros/ros.h>

#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"


class HardwareController {

    public:
        static HardwareController& getInstance();
        
        // Constants
        static constexpr int LOOP_RATE_HZ = 100;
        static constexpr int MOVEMENT_DURATION_MS = 5 * LOOP_RATE_HZ; // 5 seconds
        static constexpr int WAIT_DURATION_MS = 10 * LOOP_RATE_HZ;    // 10 seconds

        // Methods
        unitree_legged_msgs::LowState getLowState();
        void initialize(ros::NodeHandle& nh);
        void interpolateJoints(unitree_legged_msgs::LowState initialState, 
            const double *targetPos, int duration, int durationCounter
        );
        void publishLowCmd();
        void setRobotParams();

        bool getKeyPressed();
        void setKeyPressed(bool pressed);

    private:

        HardwareController() : initialized_(false) {};
        HardwareController(const HardwareController&) = delete;
        HardwareController& operator=(const HardwareController&) = delete;

        // Private members
        bool initialized_;
        bool keyPressed_ = true;
        
        ros::NodeHandle nh_;
        ros::Publisher jointState_pub_;
        ros::Publisher lowCmd_pub_;
        ros::Subscriber lowState_sub_;

        sensor_msgs::JointState jointState_;
        unitree_legged_msgs::LowCmd lowCmd_;
        unitree_legged_msgs::LowState lowState_;
        
        // Private methods
        void setPublishers();
        void setSubscriptions();

        void imuCallback(const sensor_msgs::Imu &msg);
        void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &msg);
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
        void FRfootCallback(const geometry_msgs::WrenchStamped &msg);
        void FLfootCallback(const geometry_msgs::WrenchStamped &msg);
        void RRfootCallback(const geometry_msgs::WrenchStamped &msg);
        void RLfootCallback(const geometry_msgs::WrenchStamped &msg);
};
