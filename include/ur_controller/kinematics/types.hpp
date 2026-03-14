#pragma once

/// @file types.hpp
/// @brief Type definitions for the kinematics module

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>

namespace ur_controller {
namespace kinematics {

/// @brief Fixed-size vector for 6 joint angles (radians)
using JointVector = Eigen::Matrix<double, 6, 1>;

/// @brief 6x6 Jacobian matrix
using Jacobian = Eigen::Matrix<double, 6, 6>;

/// @brief Supported Universal Robots models
enum class URModel : uint8_t {
    UR3e,
    UR5e,
    UR10e,
    UR16e,
    UR20
};

/// @brief Joint limits for a single joint
struct JointLimit {
    double min;  ///< Minimum position (radians)
    double max;  ///< Maximum position (radians)
    double velocity;  ///< Maximum velocity (rad/s)
    double acceleration;  ///< Maximum acceleration (rad/s^2)
};

/// @brief Joint limits for all 6 joints
struct JointLimits {
    std::array<JointLimit, 6> joints;

    /// @brief Check if joint positions are within limits
    /// @param q Joint positions
    /// @return true if all joints within limits
    [[nodiscard]] bool withinLimits(const JointVector& q) const {
        for (int i = 0; i < 6; ++i) {
            if (q[i] < joints[static_cast<size_t>(i)].min ||
                q[i] > joints[static_cast<size_t>(i)].max) {
                return false;
            }
        }
        return true;
    }
};

/// @brief Singularity type detected
enum class SingularityType {
    None,
    Shoulder,  ///< Wrist center on J1 axis
    Elbow,     ///< Arm fully extended or folded
    Wrist      ///< J4 and J6 axes aligned (J5 near 0 or pi)
};

/// @brief Result of singularity analysis
struct SingularityInfo {
    SingularityType type = SingularityType::None;
    double condition_number = 0.0;  ///< Jacobian condition number
    bool near_singularity = false;
};

/// @brief Convert URModel enum to string
[[nodiscard]] inline const char* modelToString(URModel model) {
    switch (model) {
        case URModel::UR3e: return "UR3e";
        case URModel::UR5e: return "UR5e";
        case URModel::UR10e: return "UR10e";
        case URModel::UR16e: return "UR16e";
        case URModel::UR20: return "UR20";
        default: return "Unknown";
    }
}

}  // namespace kinematics
}  // namespace ur_controller
