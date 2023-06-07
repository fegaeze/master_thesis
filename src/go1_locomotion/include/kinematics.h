#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

class Kinematics {
public:
    Kinematics(
        const std::vector<double>& footPosition,
        const std::string& name
    );

    std::vector<double> ikSolver(bool isRight = true);

private:
    std::string name_;
    double hipLength_, thighLength_, calfLength_;
    double footPositionX_, footPositionY_, footPositionZ_;
};

#endif // KINEMATICS_H