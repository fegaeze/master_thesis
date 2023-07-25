/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "hardware.hpp"


HardwareController::HardwareController(std::string rname, ros::NodeHandle sharednm): sharednm_{sharednm}, robot_name{rname}, start_up{true}
{
    setupSubscriptions();
    setupPublishers();
}

void HardwareController::publish_low_state()
{
    stand();
    lowState_pub.publish(lowState);
}

void HardwareController::sendServoCmd()
{
    for (int m = 0; m < 12; m++)
    {
        servo_pub[m].publish(lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
    usleep(1000);
}

void HardwareController::stand()
{
    double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                    0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos, 1000);
}

void HardwareController::motion_init()
{
    for (int i = 0; i < 4; i++)
    {
        lowCmd.motorCmd[i * 3 + 0].mode = 0x0A;
        lowCmd.motorCmd[i * 3 + 0].Kp = 70;
        lowCmd.motorCmd[i * 3 + 0].dq = 0;
        lowCmd.motorCmd[i * 3 + 0].Kd = 3;
        lowCmd.motorCmd[i * 3 + 0].tau = 0;
        lowCmd.motorCmd[i * 3 + 1].mode = 0x0A;
        lowCmd.motorCmd[i * 3 + 1].Kp = 180;
        lowCmd.motorCmd[i * 3 + 1].dq = 0;
        lowCmd.motorCmd[i * 3 + 1].Kd = 8;
        lowCmd.motorCmd[i * 3 + 1].tau = 0;
        lowCmd.motorCmd[i * 3 + 2].mode = 0x0A;
        lowCmd.motorCmd[i * 3 + 2].Kp = 300;
        lowCmd.motorCmd[i * 3 + 2].dq = 0;
        lowCmd.motorCmd[i * 3 + 2].Kd = 15;
        lowCmd.motorCmd[i * 3 + 2].tau = 0;
    }
    
    for (int i = 0; i < 12; i++)
    {
        lowCmd.motorCmd[i].q = lowState.motorState[i].q;
    }

    stand();
}

void HardwareController::moveAllPosition(double* targetPos, double duration)
{
    double lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
        }
        sendServoCmd();
    }
}

void HardwareController::setupSubscriptions()
{
    imu_sub = nm_.subscribe("/trunk_imu", 1, &HardwareController::imuCallback, this);
    footForce_sub[0] = nm_.subscribe("/visual/FR_foot_contact/the_force", 1, &HardwareController::FRfootCallback, this);
    footForce_sub[1] = nm_.subscribe("/visual/FL_foot_contact/the_force", 1, &HardwareController::FLfootCallback, this);
    footForce_sub[2] = nm_.subscribe("/visual/RR_foot_contact/the_force", 1, &HardwareController::RRfootCallback, this);
    footForce_sub[3] = nm_.subscribe("/visual/RL_foot_contact/the_force", 1, &HardwareController::RLfootCallback, this);
    servo_sub[0] = nm_.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &HardwareController::FRhipCallback, this);
    servo_sub[1] = nm_.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &HardwareController::FRthighCallback, this);
    servo_sub[2] = nm_.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &HardwareController::FRcalfCallback, this);
    servo_sub[3] = nm_.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &HardwareController::FLhipCallback, this);
    servo_sub[4] = nm_.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &HardwareController::FLthighCallback, this);
    servo_sub[5] = nm_.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &HardwareController::FLcalfCallback, this);
    servo_sub[6] = nm_.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &HardwareController::RRhipCallback, this);
    servo_sub[7] = nm_.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &HardwareController::RRthighCallback, this);
    servo_sub[8] = nm_.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &HardwareController::RRcalfCallback, this);
    servo_sub[9] = nm_.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &HardwareController::RLhipCallback, this);
    servo_sub[10] = nm_.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &HardwareController::RLthighCallback, this);
    servo_sub[11] = nm_.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &HardwareController::RLcalfCallback, this);
}

void HardwareController::setupPublishers()
{
    lowState_pub = sharednm_.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    servo_pub[0] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = sharednm_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);
}

void HardwareController::imuCallback(const sensor_msgs::Imu &msg)
{
    lowState.imu.quaternion[0] = msg.orientation.w;
    lowState.imu.quaternion[1] = msg.orientation.x;
    lowState.imu.quaternion[2] = msg.orientation.y;
    lowState.imu.quaternion[3] = msg.orientation.z;

    lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    lowState.imu.gyroscope[2] = msg.angular_velocity.z;

    lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void HardwareController::FRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    start_up = false;
    lowState.motorState[0].mode = msg.mode;
    lowState.motorState[0].q = msg.q;
    lowState.motorState[0].dq = msg.dq;
    lowState.motorState[0].tauEst = msg.tauEst;
}

void HardwareController::FRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[1].mode = msg.mode;
    lowState.motorState[1].q = msg.q;
    lowState.motorState[1].dq = msg.dq;
    lowState.motorState[1].tauEst = msg.tauEst;
}

void HardwareController::FRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[2].mode = msg.mode;
    lowState.motorState[2].q = msg.q;
    lowState.motorState[2].dq = msg.dq;
    lowState.motorState[2].tauEst = msg.tauEst;
}

void HardwareController::FLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    start_up = false;
    lowState.motorState[3].mode = msg.mode;
    lowState.motorState[3].q = msg.q;
    lowState.motorState[3].dq = msg.dq;
    lowState.motorState[3].tauEst = msg.tauEst;
}

void HardwareController::FLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[4].mode = msg.mode;
    lowState.motorState[4].q = msg.q;
    lowState.motorState[4].dq = msg.dq;
    lowState.motorState[4].tauEst = msg.tauEst;
}

void HardwareController::FLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[5].mode = msg.mode;
    lowState.motorState[5].q = msg.q;
    lowState.motorState[5].dq = msg.dq;
    lowState.motorState[5].tauEst = msg.tauEst;
}

void HardwareController::RRhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    start_up = false;
    lowState.motorState[6].mode = msg.mode;
    lowState.motorState[6].q = msg.q;
    lowState.motorState[6].dq = msg.dq;
    lowState.motorState[6].tauEst = msg.tauEst;
}

void HardwareController::RRthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[7].mode = msg.mode;
    lowState.motorState[7].q = msg.q;
    lowState.motorState[7].dq = msg.dq;
    lowState.motorState[7].tauEst = msg.tauEst;
}

void HardwareController::RRcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[8].mode = msg.mode;
    lowState.motorState[8].q = msg.q;
    lowState.motorState[8].dq = msg.dq;
    lowState.motorState[8].tauEst = msg.tauEst;
}

void HardwareController::RLhipCallback(const unitree_legged_msgs::MotorState &msg)
{
    start_up = false;
    lowState.motorState[9].mode = msg.mode;
    lowState.motorState[9].q = msg.q;
    lowState.motorState[9].dq = msg.dq;
    lowState.motorState[9].tauEst = msg.tauEst;
}

void HardwareController::RLthighCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[10].mode = msg.mode;
    lowState.motorState[10].q = msg.q;
    lowState.motorState[10].dq = msg.dq;
    lowState.motorState[10].tauEst = msg.tauEst;
}

void HardwareController::RLcalfCallback(const unitree_legged_msgs::MotorState &msg)
{
    lowState.motorState[11].mode = msg.mode;
    lowState.motorState[11].q = msg.q;
    lowState.motorState[11].dq = msg.dq;
    lowState.motorState[11].tauEst = msg.tauEst;
}

void HardwareController::FRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[0].x = msg.wrench.force.x;
    lowState.eeForce[0].y = msg.wrench.force.y;
    lowState.eeForce[0].z = msg.wrench.force.z;
    lowState.footForce[0] = msg.wrench.force.z;
}

void HardwareController::FLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[1].x = msg.wrench.force.x;
    lowState.eeForce[1].y = msg.wrench.force.y;
    lowState.eeForce[1].z = msg.wrench.force.z;
    lowState.footForce[1] = msg.wrench.force.z;
}

void HardwareController::RRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[2].x = msg.wrench.force.x;
    lowState.eeForce[2].y = msg.wrench.force.y;
    lowState.eeForce[2].z = msg.wrench.force.z;
    lowState.footForce[2] = msg.wrench.force.z;
}

void HardwareController::RLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
    lowState.eeForce[3].x = msg.wrench.force.x;
    lowState.eeForce[3].y = msg.wrench.force.y;
    lowState.eeForce[3].z = msg.wrench.force.z;
    lowState.footForce[3] = msg.wrench.force.z;
}