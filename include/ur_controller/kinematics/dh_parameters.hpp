#pragma once

/// @file dh_parameters.hpp
/// @brief Denavit-Hartenberg parameters for Universal Robots

#include "ur_controller/kinematics/types.hpp"
#include <array>

namespace ur_controller {
namespace kinematics {

/// @brief DH parameters for a single joint
///
/// Using standard DH convention:
/// - a: Link length (distance along x_{i} from z_{i-1} to z_i)
/// - d: Link offset (distance along z_{i-1} from x_{i-1} to x_i)
/// - alpha: Link twist (angle from z_{i-1} to z_i about x_i)
/// - theta_offset: Joint angle offset (added to joint variable)
struct DHJoint {
    double a;            ///< Link length (meters)
    double d;            ///< Link offset (meters)
    double alpha;        ///< Link twist (radians)
    double theta_offset; ///< Joint angle offset (radians)
};

/// @brief Complete DH parameter set for a UR robot
struct DHParameters {
    std::array<DHJoint, 6> joints;

    /// @brief Get DH parameters for a specific UR model
    /// @param model The UR robot model
    /// @return DH parameters for the model
    [[nodiscard]] static DHParameters forModel(URModel model);
};

/// @brief Get default joint limits for a UR model
/// @param model The UR robot model
/// @return Joint limits for the model
[[nodiscard]] JointLimits defaultJointLimits(URModel model);

}  // namespace kinematics
}  // namespace ur_controller
