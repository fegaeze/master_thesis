#include <ros/package.h>

#include "actionServer.hpp"
#include "rosInterfaceManager.hpp"
#include "mudTestActions.hpp"


int main(int argc, char **argv) {
    ros::init(argc, argv, "go1_mud_test_node");
    ros::NodeHandle nh;

    std::string rname;
    ros::param::get("/robot_name", rname);

    ActionServer& actionServer = ActionServer::getInstance();
    ROSInterfaceManager& rosManager = ROSInterfaceManager::getInstance();

    ros::Rate rate(rosManager.LOOP_RATE_HZ);

    rosManager.initialize(nh, rname);
    actionServer.initialize(nh, rname);

    BT::BehaviorTreeFactory factory;
    actionServer.registerNodes(factory);

    Go1Initialized go1_initialized;
    go1_initialized.registerNodes(factory);

    factory.registerNodeType<Go1LieDown>("Go1LieDown");
    factory.registerNodeType<Go1Stand>("Go1Stand");

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
        rosManager.publishLowCmd();
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}