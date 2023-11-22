#pragma once

namespace Config {
    constexpr int LOOP_RATE_HZ = 500;
    constexpr int NUM_OF_JOINTS = 12;
    constexpr double LEG_X_POS_LIMIT = -0.003;
    constexpr double LEG_Y_POS_LIMIT = -0.08;
    constexpr double LEG_Z_POS_LIMIT = -0.38;

    constexpr double FORCE_CMD_SETPOINT = 30.0;
    constexpr double CALF_LENGTH = 0.213;
    constexpr double HIP_LENGTH = 0.08;
    constexpr double THIGH_LENGTH = 0.213;
    
    const std::string JOINT_NAMES[NUM_OF_JOINTS] = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", 
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
    };

    namespace RobotAction {
        const std::string STAND = "STAND";
        const std::string LIE_DOWN = "LIE_DOWN";
        const std::string DROP_FOOT = "DROP_FOOT";
        const std::string RAISE_FR_FOOT = "RAISE_FR_FOOT";
        const std::string RAISE_FL_FOOT = "RAISE_FL_FOOT";
        const std::string RAISE_RR_FOOT = "RAISE_RR_FOOT";
        const std::string RAISE_RL_FOOT = "RAISE_RL_FOOT";
    }

    namespace RobotController {
        const std::string FIS = "FIS";
        const std::string PID = "PID";
    }
}