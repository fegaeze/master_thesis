#include <chrono>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>

#include "hardware.hpp"
#include "mudTest.hpp"
#include "ros/init.h"

enum class State { 
    GO1_INITIAL_STATE,
    GO1_STAND_SEQUENCE, 
    GO1_CALCULATE_JOINT_POSITIONS, 
    GO1_GO_TO_COG_POSITION,
    GO1_RAISE_LEG 
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "go1_mud_test_node");

    ros::NodeHandle nm;
    std::string robotName;
    ros::param::get("/robot_name", robotName);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    usleep(30000);

    ros::Rate loop_rate(2000);

    MudTest mudtest;
    HardwareController hardwareController(robotName, nm);

    State currentState = State::GO1_STAND_SEQUENCE;

    std::chrono::seconds duration(10);
    auto startTime = std::chrono::steady_clock::now();

    double joint_positions[12];
    hardwareController.motion_init();

    while (ros::ok()) {
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::chrono::seconds elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);

        switch (currentState) {
            case State::GO1_INITIAL_STATE:
                std::cout << "GO1_INITIAL_STATE" << std::endl;
                if (elapsedTime >= duration) {
                    currentState = State::GO1_STAND_SEQUENCE;
                    startTime = std::chrono::steady_clock::now();
                }
                break;
            case State::GO1_STAND_SEQUENCE:
                std::cout << "GO1_STAND_SEQUENCE" << std::endl;
                hardwareController.publish_low_state();
                if (elapsedTime >= duration) {
                    currentState = State::GO1_CALCULATE_JOINT_POSITIONS;
                    startTime = std::chrono::steady_clock::now();
                }
                break;
            case State::GO1_CALCULATE_JOINT_POSITIONS:
                std::cout << "GO1_CALCULATE_JOINT_POSITIONS" << std::endl;
                try 
                {
                    std::vector<double> targetPos = mudtest.getCOGJointPositions();
                    if (targetPos.size() == 12) 
                    {
                        for (size_t i = 0; i < 12; ++i) {
                            joint_positions[i] = targetPos[i];
                        }
                    } else 
                    {
                        std::cout << "Something has gone wrong with joint_angles. It is less than 12" << std::endl;
                    }
                } catch (tf::TransformException ex)
                {
                    return 1;
                }
                if (elapsedTime >= duration) {
                    currentState = State::GO1_GO_TO_COG_POSITION;
                    startTime = std::chrono::steady_clock::now();
                }
                break;

            case State::GO1_GO_TO_COG_POSITION:
                std::cout << "GO1_GO_TO_COG_POSITION" << std::endl;
                hardwareController.moveAllPosition(joint_positions, 3000);
                if (elapsedTime >= duration) {
                    currentState = State::GO1_RAISE_LEG;
                    startTime = std::chrono::steady_clock::now();
                }
                break;
            case State::GO1_RAISE_LEG:
                std::cout << "GO1_RAISE_LEG" << std::endl;
                joint_positions[0] = 0.0;
                joint_positions[1] = -0.0;
                joint_positions[2] = -1.85;
                hardwareController.moveAllPosition(joint_positions, 3000);
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}