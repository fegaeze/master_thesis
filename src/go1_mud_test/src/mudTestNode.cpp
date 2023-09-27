#include <ros/package.h>

#include "controller.hpp"
#include "mudTestActions.hpp"
#include "mudTestConditions.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "go1_mud_test_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    HardwareController& controller = HardwareController::getInstance();
    
    controller.initialize(nh);
    ros::Rate loop_rate(controller.LOOP_RATE_HZ);

    BT::BehaviorTreeFactory factory;

    Go1Initialized go1_initialized;
    go1_initialized.registerNodes(factory);

    factory.registerNodeType<Go1LieDown>("Go1LieDown");
    factory.registerSimpleCondition("LieDownKeyPressed", std::bind(Go1LieDownCond::lieDownKeyPressed));

    factory.registerNodeType<Go1Stand>("Go1Stand");
    factory.registerSimpleCondition("StandKeyPressed", std::bind(Go1StandCond::standKeyPressed));

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

    while (ros::ok()) {
        tree.tickOnce();
        ros::spinOnce();
        controller.publishLowCmd();
        loop_rate.sleep();
    }

    return 0;
}