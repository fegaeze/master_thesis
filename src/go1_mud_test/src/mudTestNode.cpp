#include <chrono>

#include "hardware.hpp"
#include "mudTest.hpp"

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

    ros::AsyncSpinner spinner(1);
    spinner.start();

    MudTest mudtest;
    HardwareController hardwareController(nm);
    ros::Rate loop_rate(hardwareController.LOOP_RATE_HZ);

    State currentState = State::GO1_INITIAL_STATE;
    auto startTime = std::chrono::steady_clock::now();

    std::chrono::milliseconds wait_duration(static_cast<long int>(hardwareController.WAIT_DURATION_MS));
    double progressCounter = 0.0;

    while (ros::ok()) {

        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::chrono::milliseconds elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime);

        switch (currentState) {
            case State::GO1_INITIAL_STATE:
                ROS_INFO("GO1_INITIAL_STATE");
                if (elapsedTime >= wait_duration) {
                    currentState = State::GO1_STAND_SEQUENCE;
                    startTime = currentTime;
                }
                break;
  
            case State::GO1_STAND_SEQUENCE:
                ROS_INFO("GO1_STAND_SEQUENCE");
                hardwareController.stand(progressCounter);
                progressCounter += 1.0;

                if (progressCounter > HardwareController::MOVEMENT_DURATION_MS) {
                    progressCounter = 0.0;
                    currentState = State::GO1_CALCULATE_JOINT_POSITIONS;
                    startTime = currentTime;
                }
                break;

            case State::GO1_CALCULATE_JOINT_POSITIONS:
                ROS_INFO("GO1_CALCULATE_JOINT_POSITIONS");
                // try 
                // {
                //     int liftedLeg = 0;
                //     std::vector<double> targetPos = mudtest.getCOGJointPositions(liftedLeg);
                //     if (targetPos.size() == 12) {
                //         ROS_INFO("Calculating joint positions...");

                //         for (size_t i = 0; i < 12; ++i) {
                //             joint_positions[i] = targetPos[i];
                //         }
                        
                //         currentState = State::GO1_GO_TO_COG_POSITION;
                //         startTime = currentTime;
                //     } else {
                //         ROS_ERROR("Error: Something has gone wrong with joint angles. It is not of size 12.");
                //     }
                // } catch (const std::exception& ex) {
                //     ROS_ERROR("Exception occurred: %s", ex.what());
                // }
                // break;

            // case State::GO1_GO_TO_COG_POSITION:
            //     ROS_INFO("GO1_GO_TO_COG_POSITION");

            //     if (!moveAllPositionCalled) {
            //         hardwareController.moveAllPosition(joint_positions, 10000);
            //         moveAllPositionCalled = true;
            //     }

            //     if (elapsedTime >= hardwareController.wait_duration) {
            //         currentState = State::GO1_RAISE_LEG;
            //         startTime = currentTime;
            //         moveAllPositionCalled = false;
            //     }
            //     break;

            // case State::GO1_RAISE_LEG:
            //     std::cout << "GO1_RAISE_LEG" << std::endl;
            //     if (!moveAllPositionCalled) {
            //         joint_positions[0] = 0.0;
            //         joint_positions[1] = -0.0;
            //         joint_positions[2] = -1.85;
            //         hardwareController.moveAllPosition(joint_positions, 10000);
            //         moveAllPositionCalled = true;
            //     }
            //     break;
        
        }
        
        ros::spinOnce();
        hardwareController.publishLowCmd();
        loop_rate.sleep();
    }

    return 0;
}