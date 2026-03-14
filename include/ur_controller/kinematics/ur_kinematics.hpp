#pragma once

/// @file ur_kinematics.hpp
/// @brief Analytical kinematics for Universal Robots
///
/// Provides forward and inverse kinematics using analytical solutions.
/// The IK implementation is based on the closed-form solution for 6-DOF
/// robots with a spherical wrist (joints 4, 5, 6 axes intersecting).

#include "ur_controller/kinematics/types.hpp"
#include "ur_controller/kinematics/dh_parameters.hpp"

#include <Eigen/Geometry>
#include <array>
#include <optional>
#include <vector>

namespace ur_controller {
namespace kinematics {

/// @brief Analytical kinematics engine for Universal Robots
///
/// This class provides high-performance forward and inverse kinematics
/// using closed-form analytical solutions. The IK typically completes
/// in ~5 microseconds, compared to ~100 microseconds for numerical methods.
///
/// Thread safety: All methods are const and can be called concurrently.
/// No internal state is modified after construction.
///
/// @note All angles are in radians, all distances in meters.
class URKinematics {
public:
    /// @brief Construct kinematics for a specific UR model
    /// @param model The UR robot model (UR3e, UR5e, UR10e, UR16e, UR20)
    explicit URKinematics(URModel model);

    /// @brief Get the robot model
    [[nodiscard]] URModel model() const { return model_; }

    /// @brief Get the DH parameters
    [[nodiscard]] const DHParameters& dhParameters() const { return dh_; }

    /// @brief Get default joint limits for this model
    [[nodiscard]] const JointLimits& jointLimits() const { return limits_; }

    // -------------------------------------------------------------------------
    // Forward Kinematics
    // -------------------------------------------------------------------------

    /// @brief Compute TCP pose from joint angles
    ///
    /// @param q Joint angles (6 values, radians)
    /// @return Homogeneous transformation from base to TCP
    [[nodiscard]] Eigen::Isometry3d forward(const JointVector& q) const;

    /// @brief Compute all link transforms
    ///
    /// Returns transforms from base frame to each link frame.
    /// Useful for collision checking and visualization.
    ///
    /// @param q Joint angles (6 values, radians)
    /// @return Array of 7 transforms: [base, link1, link2, link3, link4, link5, link6/TCP]
    [[nodiscard]] std::array<Eigen::Isometry3d, 7> allLinkTransforms(
        const JointVector& q) const;

    // -------------------------------------------------------------------------
    // Inverse Kinematics
    // -------------------------------------------------------------------------

    /// @brief Compute all IK solutions for a target pose
    ///
    /// Returns up to 8 joint configurations that reach the target TCP pose.
    /// Solutions are not filtered by joint limits - use selectBestSolution()
    /// for limit-aware selection.
    ///
    /// @param target Target TCP pose (homogeneous transform, base frame)
    /// @return Vector of joint configurations (may be empty if unreachable)
    [[nodiscard]] std::vector<JointVector> inverse(
        const Eigen::Isometry3d& target) const;

    /// @brief Select the best IK solution from candidates
    ///
    /// Filters solutions by joint limits and selects the one closest
    /// to the current configuration, avoiding large joint jumps.
    ///
    /// @param solutions Candidate IK solutions
    /// @param current_q Current joint configuration
    /// @param max_joint_jump Maximum allowed change per joint (radians)
    /// @return Best solution, or nullopt if none valid
    [[nodiscard]] std::optional<JointVector> selectBestSolution(
        const std::vector<JointVector>& solutions,
        const JointVector& current_q,
        double max_joint_jump = 0.5) const;

    /// @brief Select best solution with custom limits
    [[nodiscard]] std::optional<JointVector> selectBestSolution(
        const std::vector<JointVector>& solutions,
        const JointVector& current_q,
        const JointLimits& limits,
        double max_joint_jump = 0.5) const;

    // -------------------------------------------------------------------------
    // Jacobian and Singularities
    // -------------------------------------------------------------------------

    /// @brief Compute the geometric Jacobian
    ///
    /// The Jacobian maps joint velocities to TCP velocities:
    /// [v; omega] = J * q_dot
    ///
    /// @param q Joint angles (radians)
    /// @return 6x6 Jacobian matrix
    [[nodiscard]] Jacobian jacobian(const JointVector& q) const;

    /// @brief Check if configuration is near a singularity
    ///
    /// @param q Joint angles
    /// @param threshold Condition number threshold (default: 100)
    /// @return true if near singularity
    [[nodiscard]] bool nearSingularity(
        const JointVector& q,
        double threshold = 100.0) const;

    /// @brief Analyze singularity in detail
    ///
    /// @param q Joint angles
    /// @param threshold Condition number threshold
    /// @return Detailed singularity information
    [[nodiscard]] SingularityInfo analyzeSingularity(
        const JointVector& q,
        double threshold = 100.0) const;

    // -------------------------------------------------------------------------
    // Utility
    // -------------------------------------------------------------------------

    /// @brief Wrap angle to [-pi, pi]
    [[nodiscard]] static double wrapAngle(double angle);

    /// @brief Wrap all joint angles to [-pi, pi]
    [[nodiscard]] static JointVector wrapAngles(const JointVector& q);

private:
    /// @brief Compute single DH transformation matrix
    [[nodiscard]] Eigen::Isometry3d dhTransform(
        double a, double d, double alpha, double theta) const;

    /// @brief Compute wrist center position from TCP pose
    [[nodiscard]] Eigen::Vector3d wristCenter(
        const Eigen::Isometry3d& tcp_pose) const;

    URModel model_;
    DHParameters dh_;
    JointLimits limits_;

    // Cached geometric parameters for IK
    double d1_, a2_, a3_, d4_, d5_, d6_;
};

}  // namespace kinematics
}  // namespace ur_controller
