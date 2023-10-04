#include "robotActions.hpp"


constexpr double RobotStandAction::STAND_JOINT_POSITIONS[Config::NUM_OF_JOINTS] = {
    0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
    0.0, 0.67, -1.3, -0.0, 0.67, -1.3
}

constexpr double RobotLieDownAction::LIE_DOWN_JOINT_POSITIONS[Config::NUM_OF_JOINTS] = {
    -0.5, 1.15, -2.7, -0.5, 1.15, -2.7,
    -0.5, 1.15, -2.7, -0.5, 1.15, -2.7
};


/** INITIALIZATION ACTION */
BT::NodeStatus RobotInitializationAction::robotStateReceived() {
    unitree_legged_msgs::LowState low_state = ros_manager.getLowState();

    double FR_Calf = low_state.motorState[2].q;
    double FL_Calf = low_state.motorState[5].q;
    double RR_Calf = low_state.motorState[8].q;
    double RL_Calf = low_state.motorState[11].q;

    if(FR_Calf != 0.0 && FL_Calf != 0.0 && RR_Calf != 0.0 && RL_Calf != 0.0) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotInitializationAction::robotInitialized() {
    if(robot_initialized) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotInitializationAction::robotInitialization() {
    ROS_INFO("GO1_INITIAL_STATE");
    ros_manager.setRobotParams();
    robot_initialized = true;
    return BT::NodeStatus::SUCCESS;
}

void RobotInitializationAction::registerNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerSimpleCondition(
        "RobotInitialized", std::bind(&RobotInitializationAction::robotInitialized, this));

    factory.registerSimpleCondition(
        "RobotStateReceived", std::bind(&RobotInitializationAction::robotStateReceived, this));

    factory.registerSimpleAction(
        "RobotInitialization", std::bind(&RobotInitializationAction::robotInitialization, this));
}


/** LIE DOWN ACTION */

void RobotLieDownAction::handleKeyPressed(bool pressed) override {
    action_service_manager.setLieDownKeyPressed(pressed);
}

BT::NodeStatus RobotLieDownAction::onStart() {
    ROS_INFO("GO1_LIE_DOWN_ACTION");
    return actionStart();
}

BT::NodeStatus RobotLieDownAction::onRunning() {
    return actionRunning(LIE_DOWN_JOINT_POSITIONS);
}

void RobotLieDownAction::onHalted() {
    actionHalted();
}


/** STAND ACTION */
void RobotStandAction::handleKeyPressed(bool pressed) override {
    action_service_manager.setStandKeyPressed(pressed);
}

BT::NodeStatus RobotStandAction::onStart() {
    ROS_INFO("GO1_STAND_STATE_ACTION");
    return actionStart();
}

BT::NodeStatus RobotStandAction::onRunning() {
    return actionRunning(STAND_JOINT_POSITIONS);
}

void RobotStandAction::onHalted() {
    actionHalted();
}


MudTest::MudTest(): hipLength_{0.08}, thighLength_{0.213}, calfLength_{0.213} {}

double MudTest::triangleArea(const tf::Vector3& p1, const tf::Vector3& p2, const tf::Vector3& p3)
{
    double a = sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) + pow(p1.z() - p2.z(), 2));
    double b = sqrt(pow(p2.x() - p3.x(), 2) + pow(p2.y() - p3.y(), 2) + pow(p2.z() - p3.z(), 2));
    double c = sqrt(pow(p3.x() - p1.x(), 2) + pow(p3.y() - p1.y(), 2) + pow(p3.z() - p1.z(), 2));

    double s = (a + b + c) / 2.0;
    return sqrt(s * (s - a) * (s - b) * (s - c));
}

tf::Vector3 MudTest::calculateCoGPosition(const std::vector<tf::Vector3>& feet, int liftedLeg) 
{
    tf::Vector3 cog;
    double totalArea = 0.0;

    int size = feet.size();
    liftedLeg = (liftedLeg + size) % size;

    for (int i = 0; i < size; i++)
    {

        if (liftedLeg != -1)
        {
            if (i == liftedLeg || (i + 1) % size == liftedLeg || (i + 2) % size == liftedLeg)
            {
                continue;
            }
        }

        double area = MudTest::triangleArea(feet[i], feet[(i + 1) % size], feet[(i + 2) % size]);
        totalArea += area;

        tf::Vector3 centroid(0.0, 0.0, 0.0);

        centroid.setX((feet[i].x() + feet[(i + 1) % size].x() + feet[(i + 2) % size].x()) / 3.0);
        centroid.setY((feet[i].y() + feet[(i + 1) % size].y() + feet[(i + 2) % size].y()) / 3.0);
        centroid.setZ((feet[i].z() + feet[(i + 1) % size].z() + feet[(i + 2) % size].z()) / 3.0);

        cog.setX(cog.x() + centroid.x() * area);
        cog.setY(cog.y() + centroid.y() * area);
        cog.setZ(cog.z() + centroid.z() * area);
    }

    if (totalArea > 0.0)
    {
        cog.setX(cog.x() / totalArea);
        cog.setY(cog.y() / totalArea);
        cog.setZ(cog.z() / totalArea);
    }
    else
    {
        cog.setZero();
    }
    return cog;
}

std::vector<double> MudTest::ikSolver(const Eigen::Matrix4d& footPose, bool isRight) 
{

    Eigen::Vector3d footPosition = footPose.block<3, 1>(0, 3);

    double footPositionX_ = footPosition.x();
    double footPositionY_ = footPosition.y();
    double footPositionZ_ = footPosition.z();

    double xPos_squared = footPositionX_ * footPositionX_;
    double yPos_squared = footPositionY_ * footPositionY_;
    double zPos_squared = footPositionZ_ * footPositionZ_;

    double hipLength_squared = hipLength_ * hipLength_;
    double thighLength_squared = thighLength_ * thighLength_;
    double calfLength_squared = calfLength_ * calfLength_;

    std::vector<double> jointAngles{0, 0, 0};

    // Theta One    
    double alpha = std::acos(hipLength_ / std::sqrt(yPos_squared + zPos_squared));
    double beta = std::atan2(footPositionZ_, footPositionY_);
    if (isRight) {
        jointAngles.at(0) = M_PI - alpha + beta;
    } else {
        jointAngles.at(0) = alpha + beta;
    }

    // Theta Three
    double gamma =
        std::acos((hipLength_squared + thighLength_squared + calfLength_squared -
                    xPos_squared - yPos_squared - zPos_squared) /
                    (2 * thighLength_ * calfLength_));
    jointAngles.at(2) = gamma - M_PI;

    // Theta Two
    double z_prime = -std::sqrt(yPos_squared + zPos_squared - hipLength_squared);
    double psi = std::atan2(calfLength_ * std::sin(jointAngles.at(2)),
                            thighLength_ + calfLength_ * std::cos(jointAngles.at(2)));
    jointAngles.at(1) = M_PI - psi + std::atan2(footPositionX_, z_prime);


    for (int i = 0; i < 3; i++) {
        if (jointAngles.at(i) >= M_PI) {
        jointAngles.at(i) -= 2 * M_PI;
        }
        if (jointAngles.at(i) <= -M_PI) {
        jointAngles.at(i) += 2 * M_PI;
        }
    }

    return jointAngles;
}

std::vector<double> MudTest::getCOGJointPositions(int liftedLeg)
{
    ROS_INFO("Start get COG");

    ROS_INFO("declared listener");
    std::vector<std::string> hipNames = {"/FR_hip", "/FL_hip", "/RR_hip", "/RL_hip"};
    std::vector<std::string> feetNames = {"/FR_foot", "/FL_foot", "/RR_foot", "/RL_foot"};

    ROS_INFO("initialized vec strings");
    // Get positions of the robot's feet at standing pose (wrt trunk).
    std::vector<tf::Vector3> initialfeetPositions;
    for (int i = 0; i < feetNames.size(); i++)
    {
        tf::StampedTransform transform;
        try
        {
            // listener.waitForTransform("trunk", feetNames[i], ros::Time::now(), ros::Duration(3.0));
            listener.lookupTransform("/trunk", feetNames[i], ros::Time(0), transform);
            initialfeetPositions.push_back(transform.getOrigin());
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }

    ROS_INFO("after loop");
    // Using the robot's initial feet position, get the robot's best point for stability
    tf::Vector3 cog = MudTest::calculateCoGPosition(initialfeetPositions, liftedLeg);
    std::cout << "COG Position: " << cog.x() << " " << cog.y() << " " << cog.z() << " " << std::endl;
    
    // Having gotten what CoG is, create a transformation matrix of each feet to the trunk. 
    // With this trunk being the new calculated CoG position. We only move by x & y
    std::vector<Eigen::Matrix4d> T_foot_trunk_list;
    for (int i = 0; i < initialfeetPositions.size(); i++)
    {
        Eigen::Vector3d new_pos;
        new_pos.x() = initialfeetPositions.at(i).x() - cog.x();
        new_pos.y() = initialfeetPositions.at(i).y() - cog.y();
        new_pos.z() = cog.z();

        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
        transformation.block<3, 1>(0, 3) = new_pos;
        T_foot_trunk_list.push_back(transformation);
    }

    // Get transformations matrix of each feet relative to hip.
    // Kinematics equations are based on this :)
    std::vector<Eigen::Matrix4d> T_foot_hip_list;
    for (int i = 0; i < hipNames.size(); i++)
    {
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("trunk", hipNames[i], ros::Time::now(), ros::Duration(3.0));
            listener.lookupTransform("trunk", hipNames[i], ros::Time(0), transform);

            tf::Vector3 translation = transform.getOrigin();
            tf::Quaternion rotation = transform.getRotation();

            Eigen::Quaterniond eigenRotation(rotation.w(), rotation.x(), rotation.y(), rotation.z());
            Eigen::Vector3d eigenTranslation(translation.x(), translation.y(), translation.z());

            Eigen::Matrix4d T_hip_trunk;
            T_hip_trunk.setIdentity();
            T_hip_trunk.block<3, 3>(0, 0) = eigenRotation.toRotationMatrix();
            T_hip_trunk.block<3, 1>(0, 3) = eigenTranslation;

            Eigen::Matrix4d T_trunk_hip = T_hip_trunk.inverse();
            Eigen::Matrix4d T_foot_hip = T_trunk_hip * T_foot_trunk_list[i];

            T_foot_hip_list.push_back(T_foot_hip);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }

    std::vector<double> joint_angles;
    for (int i = 0; i < T_foot_hip_list.size(); i++)
    {   
        std::vector<double> solverRes;

        // Because 1 and 3 are left legs according to order:
        // FR, FL, RR, RL.
        if (i == 1 || i == 3) {
            solverRes = MudTest::ikSolver(T_foot_hip_list[i], false);
        } else {
            solverRes = MudTest::ikSolver(T_foot_hip_list[i]);
        }

        joint_angles.insert(joint_angles.end(), solverRes.begin(), solverRes.end());
    }

    return joint_angles;
}