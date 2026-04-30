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

/// @brief Discrete arm configuration (one of 8 possible)
///
/// UR robots have 3 binary configuration choices creating 8 discrete configurations.
/// When planning trajectories, the robot should stay in the same configuration
/// to avoid discontinuous joint jumps.
struct ArmConfiguration {
    int8_t shoulder{0};  ///< +1 = front (q1 near 0), -1 = back (q1 near ±π)
    int8_t elbow{0};     ///< +1 = up (q3 < 0), -1 = down (q3 > 0)
    int8_t wrist{0};     ///< +1 = up (q5 in [0,π]), -1 = down (q5 in [-π,0])

    bool operator==(const ArmConfiguration& other) const {
        return shoulder == other.shoulder &&
               elbow == other.elbow &&
               wrist == other.wrist;
    }

    bool operator!=(const ArmConfiguration& other) const {
        return !(*this == other);
    }
};

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

    /// @brief Compute IK solution using numerical iteration (damped least squares)
    ///
    /// Iteratively converges to a solution starting from the seed configuration.
    /// More robust near singularities than analytical IK, and naturally follows
    /// the closest solution to the seed (no configuration jumps).
    ///
    /// Uses damped least squares (DLS) Jacobian pseudo-inverse:
    ///   J+ = J^T * (J*J^T + λ²I)^(-1)
    ///
    /// @param target Target TCP pose (homogeneous transform, base frame)
    /// @param seed Starting joint configuration (iteration seed)
    /// @param max_iterations Maximum iterations before giving up (default 50)
    /// @param tolerance Convergence tolerance in meters/radians (default 1e-6)
    /// @return Converged joint solution, or nullopt if failed to converge
    ///
    /// @note Typical execution time: 1-10 microseconds (1-5 iterations typical)
    /// @note Thread-safe: no internal state modified
    [[nodiscard]] std::optional<JointVector> inverseNumerical(
        const Eigen::Isometry3d& target,
        const JointVector& seed,
        int max_iterations = 50,
        double tolerance = 1e-6) const;

    // -------------------------------------------------------------------------
    // Configuration Tracking
    // -------------------------------------------------------------------------

    /// @brief Determine the arm configuration from joint angles
    ///
    /// Returns which of the 8 discrete configurations the robot is in.
    /// This is based on:
    /// - Shoulder (q1): front (near 0) or back (near ±π)
    /// - Elbow (q3): up (negative) or down (positive)
    /// - Wrist (q5): up (in [0,π]) or down (in [-π,0])
    ///
    /// @param q Joint angles
    /// @return Configuration indices
    [[nodiscard]] ArmConfiguration getConfiguration(const JointVector& q) const;

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
