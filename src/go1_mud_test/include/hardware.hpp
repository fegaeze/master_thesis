
#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <stdexcept>
#include <tf/tf.h>
#include <vector>

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/WrenchStamped.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"

#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

class HardwareController {
    private:
        bool start_up;
        ros::NodeHandle nm_, sharednm_;
        ros::Publisher servo_pub[12], lowState_pub;
        ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub;
        std::string robot_name;
        unitree_legged_msgs::LowCmd lowCmd;
        unitree_legged_msgs::LowState lowState;

        void setupSubscriptions();
        void setupPublishers();

        void imuCallback(const sensor_msgs::Imu &msg);

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

    public:
        HardwareController(std::string rname, ros::NodeHandle sharednm);
        void motion_init();
        void publish_low_state();
        void moveAllPosition(double *targetPos, double duration);
        void sendServoCmd();
        void stand();
};

#endif
