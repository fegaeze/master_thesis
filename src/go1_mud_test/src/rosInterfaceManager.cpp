/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "rosInterfaceManager.hpp"
#include "config.hpp"

using namespace std::chrono_literals;


ROSInterfaceManager& ROSInterfaceManager::getInstance() {
  static ROSInterfaceManager instance;
  return instance;
}

ROSInterfaceManager& ROSInterfaceManager::getInstance(ros::NodeHandle& nh, std::string rname) {
  if (!class_initialized) {
    static ROSInterfaceManager instance;
    
    nh_ = nh;
    robot_name = rname;
    class_initialized = true;

    // jointState_.name = {
    //   "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
    //   "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
    //   "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", 
    //   "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
    // };

    setPublishers();
    setSubscriptions();

    robot_cmd.head[0] = 0xFE;
    robot_cmd.head[1] = 0xEF;
    robot_cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;

    for (int i = 0; i < 4; i++) {
      robot_cmd.motorCmd[i*3+0].mode = 0x0A;
      robot_cmd.motorCmd[i*3+0].Kp = 0;
      robot_cmd.motorCmd[i*3+0].dq = 0;
      robot_cmd.motorCmd[i*3+0].Kd = 0;
      robot_cmd.motorCmd[i*3+0].tau = 0;

      robot_cmd.motorCmd[i*3+1].mode = 0x0A;
      robot_cmd.motorCmd[i*3+1].Kp = 0;
      robot_cmd.motorCmd[i*3+1].dq = 0;
      robot_cmd.motorCmd[i*3+1].Kd = 0;
      robot_cmd.motorCmd[i*3+1].tau = 0;

      robot_cmd.motorCmd[i*3+2].mode = 0x0A;
      robot_cmd.motorCmd[i*3+2].Kp = 0;
      robot_cmd.motorCmd[i*3+2].dq = 0;
      robot_cmd.motorCmd[i*3+2].Kd = 0;
      robot_cmd.motorCmd[i*3+2].tau = 0;
    }

    for(int i=0; i<Config::NUM_OF_JOINTS; i++){
      robot_cmd.motorCmd[i].q = robot_state.motorState[i].q;
    }

    return instance;
  }
}


unitree_legged_msgs::LowState ROSInterfaceManager::getRobotState() {
  return robot_state;
}

void ROSInterfaceManager::setPublishers() {
  joint_state_pub = nh_.advertise<sensor_msgs::JointState>("/" + robot_name + "/joint_states", 1);
  real_robot_cmd_pub = nh_.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);
  sim_low_cmd_pub[0] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
  sim_low_cmd_pub[1] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
  sim_low_cmd_pub[2] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
  sim_low_cmd_pub[3] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
  sim_low_cmd_pub[4] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
  sim_low_cmd_pub[5] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
  sim_low_cmd_pub[6] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
  sim_low_cmd_pub[7] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
  sim_low_cmd_pub[8] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
  sim_low_cmd_pub[9] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
  sim_low_cmd_pub[10] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
  sim_low_cmd_pub[11] = nh_.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);
}

void ROSInterfaceManager::setRobotParams() {
  for(int i=0; i<Config::NUM_OF_JOINTS; i++){
    robot_cmd.motorCmd[i].q = robot_state.motorState[i].q;
  }

  for (int i = 0; i < 4; i++) {
    robot_cmd.motorCmd[i * 3 + 0].Kp = 70 * 0.05;
    robot_cmd.motorCmd[i * 3 + 0].Kd = 3 * 0.05;

    robot_cmd.motorCmd[i * 3 + 1].Kp = 180 * 0.05;
    robot_cmd.motorCmd[i * 3 + 1].Kd = 8 * 0.05;

    robot_cmd.motorCmd[i * 3 + 2].Kp = 300 * 0.05;
    robot_cmd.motorCmd[i * 3 + 2].Kd = 15 * 0.05;
  }
}

void ROSInterfaceManager::setSubscriptions() {
  imu_sub = nh_.subscribe("/trunk_imu", 1, &ROSInterfaceManager::imuCallback, this);
  real_low_state_sub = nh_.subscribe("/low_state", 1, &ROSInterfaceManager::lowStateCallback, this);
  sim_low_state_sub[0] = nh_.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &ROSInterfaceManager::FRhipCallback, this);
  sim_low_state_sub[1] = nh_.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &ROSInterfaceManager::FRthighCallback, this);
  sim_low_state_sub[2] = nh_.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &ROSInterfaceManager::FRcalfCallback, this);
  sim_low_state_sub[3] = nh_.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &ROSInterfaceManager::FLhipCallback, this);
  sim_low_state_sub[4] = nh_.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &ROSInterfaceManager::FLthighCallback, this);
  sim_low_state_sub[5] = nh_.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &ROSInterfaceManager::FLcalfCallback, this);
  sim_low_state_sub[6] = nh_.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &ROSInterfaceManager::RRhipCallback, this);
  sim_low_state_sub[7] = nh_.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &ROSInterfaceManager::RRthighCallback, this);
  sim_low_state_sub[8] = nh_.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &ROSInterfaceManager::RRcalfCallback, this);
  sim_low_state_sub[9] = nh_.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &ROSInterfaceManager::RLhipCallback, this);
  sim_low_state_sub[10] = nh_.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &ROSInterfaceManager::RLthighCallback, this);
  sim_low_state_sub[11] = nh_.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &ROSInterfaceManager::RLcalfCallback, this);
}

void ROSInterfaceManager::publishRobotCmd() {
  real_low_cmd_pub.publish(robot_cmd);
  for (int m = 0; m < Config::NUM_OF_JOINTS; m++) {
    sim_low_cmd_pub[m].publish(robot_cmd.motorCmd[m]);
  }
}


/** CALLBACK METHODS */

void ROSInterfaceManager::lowStateCallback(
  const unitree_legged_msgs::LowState::ConstPtr &msg) {
  robot_state.imu.quaternion[0] = msg->imu.quaternion[0];
  robot_state.imu.quaternion[1] = msg->imu.quaternion[1];
  robot_state.imu.quaternion[2] = msg->imu.quaternion[2];
  robot_state.imu.quaternion[3] = msg->imu.quaternion[3];

  robot_state.imu.gyroscope[0] = msg->imu.gyroscope[0];
  robot_state.imu.gyroscope[1] = msg->imu.gyroscope[1];
  robot_state.imu.gyroscope[2] = msg->imu.gyroscope[2];

  robot_state.imu.accelerometer[0] = msg->imu.accelerometer[0];
  robot_state.imu.accelerometer[1] = msg->imu.accelerometer[1];
  robot_state.imu.accelerometer[2] = msg->imu.accelerometer[2];

  FRhipCallback(msg->motorState[UNITREE_LEGGED_SDK::FR_0]);
  FRthighCallback(msg->motorState[UNITREE_LEGGED_SDK::FR_1]);
  FRcalfCallback(msg->motorState[UNITREE_LEGGED_SDK::FR_2]);
  FLhipCallback(msg->motorState[UNITREE_LEGGED_SDK::FL_0]);
  FLthighCallback(msg->motorState[UNITREE_LEGGED_SDK::FL_1]);
  FLcalfCallback(msg->motorState[UNITREE_LEGGED_SDK::FL_2]);
  RRhipCallback(msg->motorState[UNITREE_LEGGED_SDK::RR_0]);
  RRthighCallback(msg->motorState[UNITREE_LEGGED_SDK::RR_1]);
  RRcalfCallback(msg->motorState[UNITREE_LEGGED_SDK::RR_2]);
  RLhipCallback(msg->motorState[UNITREE_LEGGED_SDK::RL_0]);
  RLthighCallback(msg->motorState[UNITREE_LEGGED_SDK::RL_1]);
  RLcalfCallback(msg->motorState[UNITREE_LEGGED_SDK::RL_2]);

  // jointState_.header.stamp = ros::Time::now();
  // for (int i(0); i < Config::NUM_OF_JOINTS; ++i) {
  //   jointState_.position.push_back(robot_state.motorState[i].q);
  //   jointState_.velocity.push_back(robot_state.motorState[i].dq);
  //   jointState_.effort.push_back(robot_state.motorState[i].tauEst);
  // }
  // jointState_pub_.publish(jointState_);
}

void ROSInterfaceManager::imuCallback(const sensor_msgs::Imu &msg)
{
  robot_state.imu.quaternion[0] = msg.orientation.w;
  robot_state.imu.quaternion[1] = msg.orientation.x;
  robot_state.imu.quaternion[2] = msg.orientation.y;
  robot_state.imu.quaternion[3] = msg.orientation.z;

  robot_state.imu.gyroscope[0] = msg.angular_velocity.x;
  robot_state.imu.gyroscope[1] = msg.angular_velocity.y;
  robot_state.imu.gyroscope[2] = msg.angular_velocity.z;

  robot_state.imu.accelerometer[0] = msg.linear_acceleration.x;
  robot_state.imu.accelerometer[1] = msg.linear_acceleration.y;
  robot_state.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void ROSInterfaceManager::FRhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[0].mode = msg.mode;
  robot_state.motorState[0].q = msg.q;
  robot_state.motorState[0].dq = msg.dq;
  robot_state.motorState[0].tauEst = msg.tauEst;
}

void ROSInterfaceManager::FRthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[1].mode = msg.mode;
  robot_state.motorState[1].q = msg.q;
  robot_state.motorState[1].dq = msg.dq;
  robot_state.motorState[1].tauEst = msg.tauEst;
}

void ROSInterfaceManager::FRcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[2].mode = msg.mode;
  robot_state.motorState[2].q = msg.q;
  robot_state.motorState[2].dq = msg.dq;
  robot_state.motorState[2].tauEst = msg.tauEst;
}

void ROSInterfaceManager::FLhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[3].mode = msg.mode;
  robot_state.motorState[3].q = msg.q;
  robot_state.motorState[3].dq = msg.dq;
  robot_state.motorState[3].tauEst = msg.tauEst;
}

void ROSInterfaceManager::FLthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[4].mode = msg.mode;
  robot_state.motorState[4].q = msg.q;
  robot_state.motorState[4].dq = msg.dq;
  robot_state.motorState[4].tauEst = msg.tauEst;
}

void ROSInterfaceManager::FLcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[5].mode = msg.mode;
  robot_state.motorState[5].q = msg.q;
  robot_state.motorState[5].dq = msg.dq;
  robot_state.motorState[5].tauEst = msg.tauEst;
}

void ROSInterfaceManager::RRhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[6].mode = msg.mode;
  robot_state.motorState[6].q = msg.q;
  robot_state.motorState[6].dq = msg.dq;
  robot_state.motorState[6].tauEst = msg.tauEst;
}

void ROSInterfaceManager::RRthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[7].mode = msg.mode;
  robot_state.motorState[7].q = msg.q;
  robot_state.motorState[7].dq = msg.dq;
  robot_state.motorState[7].tauEst = msg.tauEst;
}

void ROSInterfaceManager::RRcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[8].mode = msg.mode;
  robot_state.motorState[8].q = msg.q;
  robot_state.motorState[8].dq = msg.dq;
  robot_state.motorState[8].tauEst = msg.tauEst;
}

void ROSInterfaceManager::RLhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[9].mode = msg.mode;
  robot_state.motorState[9].q = msg.q;
  robot_state.motorState[9].dq = msg.dq;
  robot_state.motorState[9].tauEst = msg.tauEst;
}

void ROSInterfaceManager::RLthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[10].mode = msg.mode;
  robot_state.motorState[10].q = msg.q;
  robot_state.motorState[10].dq = msg.dq;
  robot_state.motorState[10].tauEst = msg.tauEst;
}

void ROSInterfaceManager::RLcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  robot_state.motorState[11].mode = msg.mode;
  robot_state.motorState[11].q = msg.q;
  robot_state.motorState[11].dq = msg.dq;
  robot_state.motorState[11].tauEst = msg.tauEst;
}