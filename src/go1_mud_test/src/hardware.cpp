/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "hardware.hpp"

/** CONSTRUCTOR */

HardwareController::HardwareController(ros::NodeHandle nh) : nh_{nh} {
  initializeRobotParams();
  setPublishers();
  setSubscriptions();
  publishLowCmd();
}


/** INITIALIZATION METHODS */

void HardwareController::initializeRobotParams() {
  lowCmd_.head[0] = 0xFE;
  lowCmd_.head[1] = 0xEF;
  lowCmd_.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;

  for (int i = 0; i < 4; i++) {
    lowCmd_.motorCmd[i*3+0].mode = 0x0A;
    lowCmd_.motorCmd[i*3+0].Kp = 0;
    lowCmd_.motorCmd[i*3+0].dq = 0;
    lowCmd_.motorCmd[i*3+0].Kd = 0;
    lowCmd_.motorCmd[i*3+0].tau = 0;

    lowCmd_.motorCmd[i*3+1].mode = 0x0A;
    lowCmd_.motorCmd[i*3+1].Kp = 0;
    lowCmd_.motorCmd[i*3+1].dq = 0;
    lowCmd_.motorCmd[i*3+1].Kd = 0;
    lowCmd_.motorCmd[i*3+1].tau = 0;

    lowCmd_.motorCmd[i*3+2].mode = 0x0A;
    lowCmd_.motorCmd[i*3+2].Kp = 0;
    lowCmd_.motorCmd[i*3+2].dq = 0;
    lowCmd_.motorCmd[i*3+2].Kd = 0;
    lowCmd_.motorCmd[i*3+2].tau = 0;
  }

  for(int i=0; i<12; i++){
    lowCmd_.motorCmd[i].q = lowState_.motorState[i].q;
  }
}

void HardwareController::setPublishers() {
  lowCmd_pub_ = nh_.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);
}

void HardwareController::setSubscriptions() {
  lowState_sub_ = nh_.subscribe("/low_state", 1, &HardwareController::lowStateCallback, this);
}


/** ACTION METHODS */

void HardwareController::interpolateJoints(const double *targetPos, double duration, double counter) {
  double lastPos[12];
  double percent = counter / duration;

  for (int i = 0; i < 4; i++) {
    lowCmd_.motorCmd[i * 3 + 0].Kp = 70*0.5;
    lowCmd_.motorCmd[i * 3 + 0].Kd = 3*0.5;

    lowCmd_.motorCmd[i * 3 + 1].Kp = 180*0.5;
    lowCmd_.motorCmd[i * 3 + 1].Kd = 8*0.5;

    lowCmd_.motorCmd[i * 3 + 2].Kp = 300*0.5;
    lowCmd_.motorCmd[i * 3 + 2].Kd = 15*0.5;
  }

  for (int j = 0; j < 12; j++) {
    if(lastPos[2] != 0.0 && lastPos[5] != 0.0 && lastPos[8] != 0.0 && lastPos[11] != 0.0) {
      break;
    }

    lastPos[j] = lowState_.motorState[j].q;
  }

  for (int j = 0; j < 12; j++) {
    lowCmd_.motorCmd[j].q = (lastPos[j] * (1 - percent)) + (targetPos[j] * percent);
  }
}

void HardwareController::publishLowCmd() {
  lowCmd_pub_.publish(lowCmd_);
}

void HardwareController::setRobotGains(double kp, double kd) {
  for (int i = 0; i < 4; i++) {
    lowCmd_.motorCmd[i * 3 + 0].Kp = kp;
    lowCmd_.motorCmd[i * 3 + 0].Kd = kd;

    lowCmd_.motorCmd[i * 3 + 1].Kp = kp;
    lowCmd_.motorCmd[i * 3 + 1].Kd = kd;

    lowCmd_.motorCmd[i * 3 + 2].Kp = kp;
    lowCmd_.motorCmd[i * 3 + 2].Kd = kd;
  }
}

void HardwareController::stand(double progress) {
  interpolateJoints(STAND_JOINT_POSITIONS, MOVEMENT_DURATION_MS, progress);
}


/** CALLBACK METHODS */

void HardwareController::lowStateCallback(
  const unitree_legged_msgs::LowState::ConstPtr &msg) {
  lowState_.imu.quaternion[0] = msg->imu.quaternion[0];
  lowState_.imu.quaternion[1] = msg->imu.quaternion[1];
  lowState_.imu.quaternion[2] = msg->imu.quaternion[2];
  lowState_.imu.quaternion[3] = msg->imu.quaternion[3];

  lowState_.imu.gyroscope[0] = msg->imu.gyroscope[0];
  lowState_.imu.gyroscope[1] = msg->imu.gyroscope[1];
  lowState_.imu.gyroscope[2] = msg->imu.gyroscope[2];

  lowState_.imu.accelerometer[0] = msg->imu.accelerometer[0];
  lowState_.imu.accelerometer[1] = msg->imu.accelerometer[1];
  lowState_.imu.accelerometer[2] = msg->imu.accelerometer[2];

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
}

void HardwareController::imuCallback(const sensor_msgs::Imu &msg)
{
    lowState_.imu.quaternion[0] = msg.orientation.w;
    lowState_.imu.quaternion[1] = msg.orientation.x;
    lowState_.imu.quaternion[2] = msg.orientation.y;
    lowState_.imu.quaternion[3] = msg.orientation.z;

    lowState_.imu.gyroscope[0] = msg.angular_velocity.x;
    lowState_.imu.gyroscope[1] = msg.angular_velocity.y;
    lowState_.imu.gyroscope[2] = msg.angular_velocity.z;

    lowState_.imu.accelerometer[0] = msg.linear_acceleration.x;
    lowState_.imu.accelerometer[1] = msg.linear_acceleration.y;
    lowState_.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void HardwareController::FRhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[0].mode = msg.mode;
  lowState_.motorState[0].q = msg.q;
  lowState_.motorState[0].dq = msg.dq;
  lowState_.motorState[0].tauEst = msg.tauEst;
}

void HardwareController::FRthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[1].mode = msg.mode;
  lowState_.motorState[1].q = msg.q;
  lowState_.motorState[1].dq = msg.dq;
  lowState_.motorState[1].tauEst = msg.tauEst;
}

void HardwareController::FRcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[2].mode = msg.mode;
  lowState_.motorState[2].q = msg.q;
  lowState_.motorState[2].dq = msg.dq;
  lowState_.motorState[2].tauEst = msg.tauEst;
}

void HardwareController::FLhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[3].mode = msg.mode;
  lowState_.motorState[3].q = msg.q;
  lowState_.motorState[3].dq = msg.dq;
  lowState_.motorState[3].tauEst = msg.tauEst;
}

void HardwareController::FLthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[4].mode = msg.mode;
  lowState_.motorState[4].q = msg.q;
  lowState_.motorState[4].dq = msg.dq;
  lowState_.motorState[4].tauEst = msg.tauEst;
}

void HardwareController::FLcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[5].mode = msg.mode;
  lowState_.motorState[5].q = msg.q;
  lowState_.motorState[5].dq = msg.dq;
  lowState_.motorState[5].tauEst = msg.tauEst;
}

void HardwareController::RRhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[6].mode = msg.mode;
  lowState_.motorState[6].q = msg.q;
  lowState_.motorState[6].dq = msg.dq;
  lowState_.motorState[6].tauEst = msg.tauEst;
}

void HardwareController::RRthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[7].mode = msg.mode;
  lowState_.motorState[7].q = msg.q;
  lowState_.motorState[7].dq = msg.dq;
  lowState_.motorState[7].tauEst = msg.tauEst;
}

void HardwareController::RRcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[8].mode = msg.mode;
  lowState_.motorState[8].q = msg.q;
  lowState_.motorState[8].dq = msg.dq;
  lowState_.motorState[8].tauEst = msg.tauEst;
}

void HardwareController::RLhipCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[9].mode = msg.mode;
  lowState_.motorState[9].q = msg.q;
  lowState_.motorState[9].dq = msg.dq;
  lowState_.motorState[9].tauEst = msg.tauEst;
}

void HardwareController::RLthighCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[10].mode = msg.mode;
  lowState_.motorState[10].q = msg.q;
  lowState_.motorState[10].dq = msg.dq;
  lowState_.motorState[10].tauEst = msg.tauEst;
}

void HardwareController::RLcalfCallback(
    const unitree_legged_msgs::MotorState &msg) {
  lowState_.motorState[11].mode = msg.mode;
  lowState_.motorState[11].q = msg.q;
  lowState_.motorState[11].dq = msg.dq;
  lowState_.motorState[11].tauEst = msg.tauEst;
}

void HardwareController::FRfootCallback(
    const geometry_msgs::WrenchStamped &msg) {
  lowState_.eeForce[0].x = msg.wrench.force.x;
  lowState_.eeForce[0].y = msg.wrench.force.y;
  lowState_.eeForce[0].z = msg.wrench.force.z;
  lowState_.footForce[0] = msg.wrench.force.z;
}

void HardwareController::FLfootCallback(
    const geometry_msgs::WrenchStamped &msg) {
  lowState_.eeForce[1].x = msg.wrench.force.x;
  lowState_.eeForce[1].y = msg.wrench.force.y;
  lowState_.eeForce[1].z = msg.wrench.force.z;
  lowState_.footForce[1] = msg.wrench.force.z;
}

void HardwareController::RRfootCallback(
    const geometry_msgs::WrenchStamped &msg) {
  lowState_.eeForce[2].x = msg.wrench.force.x;
  lowState_.eeForce[2].y = msg.wrench.force.y;
  lowState_.eeForce[2].z = msg.wrench.force.z;
  lowState_.footForce[2] = msg.wrench.force.z;
}

void HardwareController::RLfootCallback(
    const geometry_msgs::WrenchStamped &msg) {
  lowState_.eeForce[3].x = msg.wrench.force.x;
  lowState_.eeForce[3].y = msg.wrench.force.y;
  lowState_.eeForce[3].z = msg.wrench.force.z;
  lowState_.footForce[3] = msg.wrench.force.z;
}