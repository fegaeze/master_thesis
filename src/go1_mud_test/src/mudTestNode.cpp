#include <ros/package.h>

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

    while(ros::ok()) {
        tree.tickOnce();
        ros_manager.publishRobotCmd();
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}