#pragma once

namespace Config {
    constexpr int NUM_OF_JOINTS = 12;
    constexpr int LOOP_RATE_HZ = 500;

    namespace RobotAction {
        const std::string STAND = "STAND";
        const std::string LIE_DOWN = "LIE_DOWN";
        const std::string RAISE_FR_FOOT = "RAISE_FR_FOOT";
        const std::string RAISE_FL_FOOT = "RAISE_FL_FOOT";
        const std::string RAISE_RR_FOOT = "RAISE_RR_FOOT";
        const std::string RAISE_RL_FOOT = "RAISE_RL_FOOT";
    }
}