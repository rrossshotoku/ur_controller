/// @file dh_parameters.cpp
/// @brief DH parameters for Universal Robots models

#include "ur_controller/kinematics/dh_parameters.hpp"
#include <cmath>
#include <stdexcept>

namespace ur_controller {
namespace kinematics {

namespace {

// Pi constant
constexpr double kPi = 3.14159265358979323846;

// DH parameters from Universal Robots official documentation
// https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

DHParameters createUR3eParameters() {
    DHParameters params;
    // UR3e DH parameters (standard convention)
    // Joint 1
    params.joints[0] = {0.0, 0.15185, kPi / 2.0, 0.0};
    // Joint 2
    params.joints[1] = {-0.24355, 0.0, 0.0, 0.0};
    // Joint 3
    params.joints[2] = {-0.2132, 0.0, 0.0, 0.0};
    // Joint 4
    params.joints[3] = {0.0, 0.13105, kPi / 2.0, 0.0};
    // Joint 5
    params.joints[4] = {0.0, 0.08535, -kPi / 2.0, 0.0};
    // Joint 6
    params.joints[5] = {0.0, 0.0921, 0.0, 0.0};
    return params;
}

DHParameters createUR5eParameters() {
    DHParameters params;
    // UR5e DH parameters (standard convention)
    // Joint 1
    params.joints[0] = {0.0, 0.1625, kPi / 2.0, 0.0};
    // Joint 2
    params.joints[1] = {-0.425, 0.0, 0.0, 0.0};
    // Joint 3
    params.joints[2] = {-0.3922, 0.0, 0.0, 0.0};
    // Joint 4
    params.joints[3] = {0.0, 0.1333, kPi / 2.0, 0.0};
    // Joint 5
    params.joints[4] = {0.0, 0.0997, -kPi / 2.0, 0.0};
    // Joint 6
    params.joints[5] = {0.0, 0.0996, 0.0, 0.0};
    return params;
}

DHParameters createUR10eParameters() {
    DHParameters params;
    // UR10e DH parameters (standard convention)
    // Joint 1
    params.joints[0] = {0.0, 0.1807, kPi / 2.0, 0.0};
    // Joint 2
    params.joints[1] = {-0.6127, 0.0, 0.0, 0.0};
    // Joint 3
    params.joints[2] = {-0.57155, 0.0, 0.0, 0.0};
    // Joint 4
    params.joints[3] = {0.0, 0.17415, kPi / 2.0, 0.0};
    // Joint 5
    params.joints[4] = {0.0, 0.11985, -kPi / 2.0, 0.0};
    // Joint 6
    params.joints[5] = {0.0, 0.11655, 0.0, 0.0};
    return params;
}

DHParameters createUR16eParameters() {
    DHParameters params;
    // UR16e DH parameters (standard convention)
    // Joint 1
    params.joints[0] = {0.0, 0.1807, kPi / 2.0, 0.0};
    // Joint 2
    params.joints[1] = {-0.4784, 0.0, 0.0, 0.0};
    // Joint 3
    params.joints[2] = {-0.36, 0.0, 0.0, 0.0};
    // Joint 4
    params.joints[3] = {0.0, 0.17415, kPi / 2.0, 0.0};
    // Joint 5
    params.joints[4] = {0.0, 0.11985, -kPi / 2.0, 0.0};
    // Joint 6
    params.joints[5] = {0.0, 0.11655, 0.0, 0.0};
    return params;
}

DHParameters createUR20Parameters() {
    DHParameters params;
    // UR20 DH parameters (standard convention)
    // Joint 1
    params.joints[0] = {0.0, 0.2363, kPi / 2.0, 0.0};
    // Joint 2
    params.joints[1] = {-0.8620, 0.0, 0.0, 0.0};
    // Joint 3
    params.joints[2] = {-0.7287, 0.0, 0.0, 0.0};
    // Joint 4
    params.joints[3] = {0.0, 0.1784, kPi / 2.0, 0.0};
    // Joint 5
    params.joints[4] = {0.0, 0.1784, -kPi / 2.0, 0.0};
    // Joint 6
    params.joints[5] = {0.0, 0.1744, 0.0, 0.0};
    return params;
}

}  // namespace

DHParameters DHParameters::forModel(URModel model) {
    switch (model) {
        case URModel::UR3e:
            return createUR3eParameters();
        case URModel::UR5e:
            return createUR5eParameters();
        case URModel::UR10e:
            return createUR10eParameters();
        case URModel::UR16e:
            return createUR16eParameters();
        case URModel::UR20:
            return createUR20Parameters();
        default:
            throw std::invalid_argument("Unknown UR model");
    }
}

JointLimits defaultJointLimits(URModel model) {
    JointLimits limits;

    // Common limits for e-Series (joints 1-3 have different limits than 4-6)
    double vel_large = 3.14159;   // rad/s for joints 1-3
    double vel_small = 3.14159;   // rad/s for joints 4-6 (UR5e)
    double accel = 40.0;          // rad/s^2

    switch (model) {
        case URModel::UR3e:
            vel_large = 3.14159;
            vel_small = 6.28318;  // Wrist joints faster on smaller robots
            break;
        case URModel::UR5e:
            vel_large = 3.14159;
            vel_small = 3.14159;
            break;
        case URModel::UR10e:
            vel_large = 2.094395;  // Slower for larger robot
            vel_small = 3.14159;
            break;
        case URModel::UR16e:
            vel_large = 2.094395;
            vel_small = 3.14159;
            break;
        case URModel::UR20:
            vel_large = 1.57079;   // Slowest for largest robot
            vel_small = 2.094395;
            break;
    }

    // All UR e-Series have +/- 2*pi position limits
    constexpr double pos_limit = 2.0 * kPi;

    for (size_t i = 0; i < 3; ++i) {
        limits.joints[i] = {-pos_limit, pos_limit, vel_large, accel};
    }
    for (size_t i = 3; i < 6; ++i) {
        limits.joints[i] = {-pos_limit, pos_limit, vel_small, accel};
    }

    return limits;
}

}  // namespace kinematics
}  // namespace ur_controller
