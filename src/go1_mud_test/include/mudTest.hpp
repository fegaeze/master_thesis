
#ifndef __MUDTEST_H__
#define __MUDTEST_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <stdexcept>
#include <string>
#include <tf/transform_listener.h>
#include <vector>


class MudTest {
    private:
        double hipLength_, thighLength_, calfLength_;
        tf::Vector3 calculateCoGPosition(const std::vector<tf::Vector3>& feet, int liftedLeg=-1);
        std::vector<double> ikSolver(const Eigen::Matrix4d& footPose, bool isRight=true);
        double triangleArea(const tf::Vector3& p1, const tf::Vector3& p2, const tf::Vector3& p3);

    public:
        MudTest();
        std::vector<double> getCOGJointPositions();
};

#endif
