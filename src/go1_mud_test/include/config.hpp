#pragma once

namespace Config {
    constexpr int NUM_OF_JOINTS = 12;
    constexpr int LOOP_RATE_HZ = 500;

    enum class RobotAction {
        STAND,
        LIE_DOWN,
        RAISE_FR_FOOT,
        RAISE_FL_FOOT,
        RAISE_RR_FOOT,
        RAISE_RL_FOOT
    };
}