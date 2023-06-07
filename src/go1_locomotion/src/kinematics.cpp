#include "kinematics.h"


Kinematics::Kinematics(
    const std::vector<double> &footPosition, 
    const std::string &name
) {
    if (footPosition.size() != 3) {
        throw std::invalid_argument("End-effector position vector must have a size of 3");
    }

    name_ = name;
    hipLength_ = 0.08;
    thighLength_ = 0.213;
    calfLength_ = 0.213;
    footPositionX_ = footPosition[0];
    footPositionY_ = footPosition[1];
    footPositionZ_ = footPosition[2];
}

std::vector<double> Kinematics::ikSolver(bool isRight) {
    double xPos_squared = footPositionX_ * footPositionX_;
    double yPos_squared = footPositionY_ * footPositionY_;
    double zPos_squared = footPositionZ_ * footPositionZ_;

    double hipLength_squared = hipLength_ * hipLength_;
    double thighLength_squared = thighLength_ * thighLength_;
    double calfLength_squared = calfLength_ * calfLength_;

    double theta1;
    if(isRight) {
        theta1 = M_PI - std::acos(hipLength_ / std::sqrt(yPos_squared + zPos_squared)) + std::atan2(footPositionZ_, footPositionY_);
    } else {
        theta1 = std::acos(hipLength_ / std::sqrt(yPos_squared + zPos_squared)) + std::atan2(footPositionZ_, footPositionY_);
    }

    double alpha = std::acos((hipLength_squared + thighLength_squared + calfLength_squared - xPos_squared - yPos_squared - zPos_squared) / 2 * thighLength_ * calfLength_);
    double theta3 = alpha - M_PI;

    double z_prime = std::sqrt(yPos_squared + zPos_squared - hipLength_squared);
    double psi = std::atan2(calfLength_ * std::sin(theta3), (thighLength_ + calfLength_) * std::cos(theta3));
    double theta2 = M_PI - psi + std::atan2(footPositionX_, z_prime);

    if(theta1 >= M_PI)
    {
        theta1 -= 2 * M_PI;
    } else if (theta1 <= -M_PI) {
        theta1 += 2 * M_PI;
    }

    if(theta2 >= M_PI)
    {
        theta2 -= 2 * M_PI;
    } else if (theta2 <= -M_PI) {
        theta2 += 2 * M_PI;
    }

    std::vector<double> jointAngles = { theta1, theta2, theta3 };
    return jointAngles;
}