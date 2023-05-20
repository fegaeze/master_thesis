
#ifndef __UTILS_H__
#define __UTILS_H__

#include <ros/ros.h>
#include <tf/tf.h>

#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"

#define FEET_SIZE 4
#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace unitree_model {
    extern ros::Publisher servo_pub[12];
    extern unitree_legged_msgs::LowCmd lowCmd;
    extern unitree_legged_msgs::LowState lowState;

    void stand();
    void motion_init();
    void sendServoCmd();
    void moveAllPosition(double* jointPositions, double duration);
}

#endif
