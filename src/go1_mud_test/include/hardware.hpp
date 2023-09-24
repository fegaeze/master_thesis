#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include <iostream>
#include <ros/ros.h>

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/WrenchStamped.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

class HardwareController {

    public:
        HardwareController(ros::NodeHandle nh);

        // Constants
        static constexpr int LOOP_RATE_HZ = 100;
        static constexpr double MOVEMENT_DURATION_MS = 5.0 * double(LOOP_RATE_HZ); // 5 seconds
        static constexpr double WAIT_DURATION_MS = 10.0 * double(LOOP_RATE_HZ);    // 10 seconds

        // Methods
        void publishLowCmd();
        void stand(double percentCounter);

    private:
    
        // Constants
        const double STAND_JOINT_POSITIONS[12] = {
            0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
            0.0, 0.67, -1.3, -0.0, 0.67, -1.3
        };

        // Private members
        ros::NodeHandle nh_;
        ros::Publisher lowCmd_pub_;
        ros::Subscriber lowState_sub_;
        unitree_legged_msgs::LowCmd lowCmd_;
        unitree_legged_msgs::LowState lowState_;
        
        // Private methods
        void initializeRobotParams();
        void interpolateJoints(const double *targetPos, double duration, double percentCounter);
        
        void setPublishers();
        void setRobotGains(double kp, double kd);
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

#endif
