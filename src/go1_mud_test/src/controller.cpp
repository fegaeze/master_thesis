/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "controller.hpp"

using namespace std::chrono_literals;


/** INITIALIZATION METHODS */

HardwareController& HardwareController::getInstance() {
  static HardwareController instance;
  return instance;
}

void HardwareController::initialize(ros::NodeHandle& nh) {
  if (!initialized_) {
    nh_ = nh;
    initialized_ = true;

    setPublishers();
    setSubscriptions();

    this->lowCmd_.head[0] = 0xFE;
    this->lowCmd_.head[1] = 0xEF;
    this->lowCmd_.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;

    for (int i = 0; i < 4; i++) {
      this->lowCmd_.motorCmd[i*3+0].mode = 0x0A;
      this->lowCmd_.motorCmd[i*3+0].Kp = 0;
      this->lowCmd_.motorCmd[i*3+0].dq = 0;
      this->lowCmd_.motorCmd[i*3+0].Kd = 0;
      this->lowCmd_.motorCmd[i*3+0].tau = 0;

      this->lowCmd_.motorCmd[i*3+1].mode = 0x0A;
      this->lowCmd_.motorCmd[i*3+1].Kp = 0;
      this->lowCmd_.motorCmd[i*3+1].dq = 0;
      this->lowCmd_.motorCmd[i*3+1].Kd = 0;
      this->lowCmd_.motorCmd[i*3+1].tau = 0;

      this->lowCmd_.motorCmd[i*3+2].mode = 0x0A;
      this->lowCmd_.motorCmd[i*3+2].Kp = 0;
      this->lowCmd_.motorCmd[i*3+2].dq = 0;
      this->lowCmd_.motorCmd[i*3+2].Kd = 0;
      this->lowCmd_.motorCmd[i*3+2].tau = 0;
    }

    for(int i=0; i<12; i++){
      this->lowCmd_.motorCmd[i].q = this->lowState_.motorState[i].q;
    }

    publishLowCmd();
  }
}


/** GETTER & SETTER */
unitree_legged_msgs::LowState HardwareController::getLowState() {
  return this->lowState_;
}

void HardwareController::setPublishers() {
  this->jointState_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  this->lowCmd_pub_ = nh_.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);
}

void HardwareController::setRobotParams() {
  for(int i=0; i<12; i++){
    this->lowCmd_.motorCmd[i].q = this->lowState_.motorState[i].q;
  }

  for (int i = 0; i < 4; i++) {
    this->lowCmd_.motorCmd[i * 3 + 0].Kp = 70*0.5;
    this->lowCmd_.motorCmd[i * 3 + 0].Kd = 3*0.5;

    this->lowCmd_.motorCmd[i * 3 + 1].Kp = 180*0.5;
    this->lowCmd_.motorCmd[i * 3 + 1].Kd = 8*0.5;

    this->lowCmd_.motorCmd[i * 3 + 2].Kp = 300*0.5;
    this->lowCmd_.motorCmd[i * 3 + 2].Kd = 15*0.5;
  }
}

void HardwareController::setSubscriptions() {
  this->lowState_sub_ = nh_.subscribe("/low_state", 1, &HardwareController::lowStateCallback, this);
}


/** ACTION METHODS */

void HardwareController::interpolateJoints(
    unitree_legged_msgs::LowState initialState, 
    const double *targetPos, int duration, int durationCounter
  ) {
  double initialPos[12];
  double percent = static_cast<double>(durationCounter) / static_cast<double>(duration);

  for (int j = 0; j < 12; j++) {
    initialPos[j] = initialState.motorState[j].q;
    this->lowCmd_.motorCmd[j].q = (initialPos[j] * (1 - percent)) + (targetPos[j] * percent);
  }
}

void HardwareController::publishLowCmd() {
  this->lowCmd_pub_.publish(this->lowCmd_);
}


/** CALLBACK METHODS */

void HardwareController::lowStateCallback(
  const unitree_legged_msgs::LowState::ConstPtr &msg) {
  this->lowState_.imu.quaternion[0] = msg->imu.quaternion[0];
  this->lowState_.imu.quaternion[1] = msg->imu.quaternion[1];
  this->lowState_.imu.quaternion[2] = msg->imu.quaternion[2];
  this->lowState_.imu.quaternion[3] = msg->imu.quaternion[3];

  this->lowState_.imu.gyroscope[0] = msg->imu.gyroscope[0];
  this->lowState_.imu.gyroscope[1] = msg->imu.gyroscope[1];
  this->lowState_.imu.gyroscope[2] = msg->imu.gyroscope[2];

  this->lowState_.imu.accelerometer[0] = msg->imu.accelerometer[0];
  this->lowState_.imu.accelerometer[1] = msg->imu.accelerometer[1];
  this->lowState_.imu.accelerometer[2] = msg->imu.accelerometer[2];

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

  this->jointState_.header.stamp = ros::Time::now();
  for (int i(0); i < 12; ++i) {
    this->jointState_.position[i] = msg->motorState[i].q;
    this->jointState_.velocity[i] = msg->motorState[i].dq;
    this->jointState_.effort[i] = msg->motorState[i].tauEst;
  }

  this->jointState_pub_.publish(this->jointState_);
}

void HardwareController::imuCallback(const sensor_msgs::Imu &msg)
{
    this->lowState_.imu.quaternion[0] = msg.orientation.w;
    this->lowState_.imu.quaternion[1] = msg.orientation.x;
    this->lowState_.imu.quaternion[2] = msg.orientation.y;
    this->lowState_.imu.quaternion[3] = msg.orientation.z;

    this->lowState_.imu.gyroscope[0] = msg.angular_velocity.x;
    this->lowState_.imu.gyroscope[1] = msg.angular_velocity.y;
    this->lowState_.imu.gyroscope[2] = msg.angular_velocity.z;

    this->lowState_.imu.accelerometer[0] = msg.linear_acceleration.x;
    this->lowState_.imu.accelerometer[1] = msg.linear_acceleration.y;
    this->lowState_.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void HardwareController::FRhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[0].mode = msg.mode;
  this->lowState_.motorState[0].q = msg.q;
  this->lowState_.motorState[0].dq = msg.dq;
  this->lowState_.motorState[0].tauEst = msg.tauEst;
}

void HardwareController::FRthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[1].mode = msg.mode;
  this->lowState_.motorState[1].q = msg.q;
  this->lowState_.motorState[1].dq = msg.dq;
  this->lowState_.motorState[1].tauEst = msg.tauEst;
}

void HardwareController::FRcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[2].mode = msg.mode;
  this->lowState_.motorState[2].q = msg.q;
  this->lowState_.motorState[2].dq = msg.dq;
  this->lowState_.motorState[2].tauEst = msg.tauEst;
}

void HardwareController::FLhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[3].mode = msg.mode;
  this->lowState_.motorState[3].q = msg.q;
  this->lowState_.motorState[3].dq = msg.dq;
  this->lowState_.motorState[3].tauEst = msg.tauEst;
}

void HardwareController::FLthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[4].mode = msg.mode;
  this->lowState_.motorState[4].q = msg.q;
  this->lowState_.motorState[4].dq = msg.dq;
  this->lowState_.motorState[4].tauEst = msg.tauEst;
}

void HardwareController::FLcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[5].mode = msg.mode;
  this->lowState_.motorState[5].q = msg.q;
  this->lowState_.motorState[5].dq = msg.dq;
  this->lowState_.motorState[5].tauEst = msg.tauEst;
}

void HardwareController::RRhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[6].mode = msg.mode;
  this->lowState_.motorState[6].q = msg.q;
  this->lowState_.motorState[6].dq = msg.dq;
  this->lowState_.motorState[6].tauEst = msg.tauEst;
}

void HardwareController::RRthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[7].mode = msg.mode;
  this->lowState_.motorState[7].q = msg.q;
  this->lowState_.motorState[7].dq = msg.dq;
  this->lowState_.motorState[7].tauEst = msg.tauEst;
}

void HardwareController::RRcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[8].mode = msg.mode;
  this->lowState_.motorState[8].q = msg.q;
  this->lowState_.motorState[8].dq = msg.dq;
  this->lowState_.motorState[8].tauEst = msg.tauEst;
}

void HardwareController::RLhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[9].mode = msg.mode;
  this->lowState_.motorState[9].q = msg.q;
  this->lowState_.motorState[9].dq = msg.dq;
  this->lowState_.motorState[9].tauEst = msg.tauEst;
}

void HardwareController::RLthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[10].mode = msg.mode;
  this->lowState_.motorState[10].q = msg.q;
  this->lowState_.motorState[10].dq = msg.dq;
  this->lowState_.motorState[10].tauEst = msg.tauEst;
}

void HardwareController::RLcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  this->lowState_.motorState[11].mode = msg.mode;
  this->lowState_.motorState[11].q = msg.q;
  this->lowState_.motorState[11].dq = msg.dq;
  this->lowState_.motorState[11].tauEst = msg.tauEst;
}

void HardwareController::FRfootCallback(
    const geometry_msgs::WrenchStamped &msg) {
  this->lowState_.eeForce[0].x = msg.wrench.force.x;
  this->lowState_.eeForce[0].y = msg.wrench.force.y;
  this->lowState_.eeForce[0].z = msg.wrench.force.z;
  this->lowState_.footForce[0] = msg.wrench.force.z;
}

void HardwareController::FLfootCallback(
    const geometry_msgs::WrenchStamped &msg) {
  this->lowState_.eeForce[1].x = msg.wrench.force.x;
  this->lowState_.eeForce[1].y = msg.wrench.force.y;
  this->lowState_.eeForce[1].z = msg.wrench.force.z;
  this->lowState_.footForce[1] = msg.wrench.force.z;
}

void HardwareController::RRfootCallback(
    const geometry_msgs::WrenchStamped &msg) {
  this->lowState_.eeForce[2].x = msg.wrench.force.x;
  this->lowState_.eeForce[2].y = msg.wrench.force.y;
  this->lowState_.eeForce[2].z = msg.wrench.force.z;
  this->lowState_.footForce[2] = msg.wrench.force.z;
}

void HardwareController::RLfootCallback(
    const geometry_msgs::WrenchStamped &msg) {
  this->lowState_.eeForce[3].x = msg.wrench.force.x;
  this->lowState_.eeForce[3].y = msg.wrench.force.y;
  this->lowState_.eeForce[3].z = msg.wrench.force.z;
  this->lowState_.footForce[3] = msg.wrench.force.z;
}