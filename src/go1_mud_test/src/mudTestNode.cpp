#include <ros/package.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "actionServiceManager.hpp"
#include "controllerServiceManager.hpp"
#include "rosInterfaceManager.hpp"
#include "robotActions.hpp"
#include "config.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "go1_mud_test_node");
    ros::NodeHandle nh;

    std::string rname;
    ros::param::get("/robot_name", rname);
    ros::AsyncSpinner spinner(1);
    ros::Rate rate(Config::LOOP_RATE_HZ);
    spinner.start();

    ActionServiceManager& action_service_manager = ActionServiceManager::getInstance(nh, rname);
    ControllerServiceManager& controller_service_manager = ControllerServiceManager::getInstance(nh, rname);
    ROSInterfaceManager& ros_manager = ROSInterfaceManager::getInstance(nh, rname);

    BT::BehaviorTreeFactory factory;
    RobotInitializationAction robot_init_action;

    factory.registerNodeType<RobotFrRaiseAction>("RobotFrRaiseAction");
    factory.registerNodeType<RobotGoToCogAction>("RobotGoToCogAction");
    factory.registerNodeType<RobotLieDownAction>("RobotLieDownAction");
    factory.registerNodeType<RobotDropFootAction>("RobotDropFootAction");
    factory.registerNodeType<RobotStandAction>("RobotStandAction");
    action_service_manager.registerNodes(factory);
    robot_init_action.registerNodes(factory);

    std::string package_name = "go1_mud_test";
    std::string relative_file_path = "/behavior_trees/mud_test.xml";

    BT::Tree tree;
    std::string package_path = ros::package::getPath(package_name);

    if (package_path != "") {
        std::string absolute_file_path = package_path + relative_file_path;
        tree = factory.createTreeFromFile(absolute_file_path);
    } else {
        ROS_ERROR("Failed to retrieve path for package '%s'", package_name.c_str());
    }

    UNITREE_LEGGED_SDK::Safety safe_mode(UNITREE_LEGGED_SDK::LeggedType::Go1);

    while(ros::ok()) {
        tree.tickOnce();

        std::tuple<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState> safe_mode_params = ros_manager.getSafeModeParams();

        UNITREE_LEGGED_SDK::LowCmd robot_cmd = std::get<0>(safe_mode_params);
        UNITREE_LEGGED_SDK::LowState robot_state = std::get<1>(safe_mode_params);

        safe_mode.PositionLimit(robot_cmd);
        int power_safe_mode = safe_mode.PowerProtect(robot_cmd, robot_state, 1);
        int position_safe_mode = safe_mode.PositionProtect(robot_cmd, robot_state, 0.0025);

        if(power_safe_mode < 0 || position_safe_mode < 0) {
            ROS_INFO("Safety Mode Triggered");
            break;
        }

        ros_manager.publishRobotCmd();
        rate.sleep();
        ros::spinOnce();
    }


    ros::shutdown();
    return 0;
}