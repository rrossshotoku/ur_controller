/// @file velocity_first/types.hpp
/// @brief Type definitions for velocity-first trajectory planning

#pragma once

#include "ur_controller/kinematics/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <vector>

namespace ur_controller {
namespace trajectory {
namespace velocity_first {

using kinematics::JointVector;

// =============================================================================
// Core Types
// =============================================================================

/// @brief Target velocity for a segment between waypoints
struct SegmentVelocity {
    Eigen::Vector3d direction{0, 0, 0};  ///< Unit direction vector
    double speed{0.0};                    ///< Magnitude (m/s)
    double duration{0.0};                 ///< Segment time (seconds)

    /// @brief Get velocity vector (direction * speed)
    [[nodiscard]] Eigen::Vector3d vector() const {
        return direction * speed;
    }

    /// @brief Check if this is a rest state (zero speed)
    [[nodiscard]] bool isRest() const {
        return speed < 1e-6;
    }
};

/// @brief Velocity transition at a waypoint
///
/// Represents the jerk-limited transition from one velocity to another,
/// centered around a waypoint time. The transition is symmetric.
struct VelocityTransition {
    double t_start{0.0};      ///< Transition start time
    double t_center{0.0};     ///< Waypoint time (center of transition)
    double t_end{0.0};        ///< Transition end time

    SegmentVelocity v_before; ///< Incoming velocity (before transition)
    SegmentVelocity v_after;  ///< Outgoing velocity (after transition)

    /// @brief Get transition duration
    [[nodiscard]] double duration() const {
        return t_end - t_start;
    }

    /// @brief Check if time t is within this transition
    [[nodiscard]] bool contains(double t) const {
        return t >= t_start && t <= t_end;
    }
};

/// @brief A waypoint with timing information
struct TimedWaypoint {
    Eigen::Vector3d position{0, 0, 0};
    Eigen::Quaterniond orientation{1, 0, 0, 0};
    double time{0.0};  ///< Absolute arrival time (seconds from start)

    /// @brief Saved joint positions when waypoint was taught (optional)
    std::optional<JointVector> joints;

    /// @brief Convert to Isometry3d pose
    [[nodiscard]] Eigen::Isometry3d toPose() const {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = position;
        pose.linear() = orientation.toRotationMatrix();
        return pose;
    }
};

/// @brief Configuration for velocity-first planning
struct VelocityFirstConfig {
    double max_speed{0.5};                ///< Maximum TCP speed (m/s)
    double max_acceleration{1.0};         ///< Maximum acceleration (m/s^2)
    double max_jerk{5.0};                 ///< Maximum jerk (m/s^3)
    double sample_rate{500.0};            ///< Samples per second (Hz)

    // Orientation limits
    double max_angular_velocity{1.0};     ///< rad/s
    double max_angular_acceleration{2.0}; ///< rad/s^2

    // Joint limits (for validation)
    double max_joint_velocity{1.5};       ///< rad/s per joint
};

// =============================================================================
// Helper Functions
// =============================================================================

/// @brief Spherical linear interpolation between unit vectors
///
/// Smoothly interpolates direction between two unit vectors.
/// Handles near-parallel and opposite direction cases.
///
/// @param d1 Start direction (unit vector)
/// @param d2 End direction (unit vector)
/// @param u Interpolation parameter [0, 1]
/// @return Interpolated unit direction
[[nodiscard]] inline Eigen::Vector3d slerpDirection(
    const Eigen::Vector3d& d1,
    const Eigen::Vector3d& d2,
    double u) {

    double dot = d1.dot(d2);

    // Nearly parallel - linear interpolation is fine
    if (dot > 0.9999) {
        return d1;
    }

    // Nearly opposite - interpolate through perpendicular
    if (dot < -0.9999) {
        // Find a perpendicular vector
        Eigen::Vector3d perp = d1.cross(Eigen::Vector3d::UnitZ());
        if (perp.norm() < 0.001) {
            perp = d1.cross(Eigen::Vector3d::UnitY());
        }
        perp.normalize();

        // Interpolate through the perpendicular
        double angle = M_PI * u;
        return (d1 * std::cos(angle) + perp * std::sin(angle)).normalized();
    }

    // Standard SLERP
    double angle = std::acos(std::clamp(dot, -1.0, 1.0));
    double sin_angle = std::sin(angle);
    if (sin_angle < 1e-6) {
        return d1;
    }

    double a = std::sin((1.0 - u) * angle) / sin_angle;
    double b = std::sin(u * angle) / sin_angle;
    return (d1 * a + d2 * b).normalized();
}

/// @brief Smooth step function (cubic Hermite)
///
/// Maps [0, 1] -> [0, 1] with zero derivative at endpoints.
/// Used for smooth velocity transitions.
///
/// @param u Input value in [0, 1]
/// @return Smoothly interpolated value in [0, 1]
[[nodiscard]] inline double smoothStep(double u) {
    u = std::clamp(u, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

/// @brief Smoother step function (quintic)
///
/// Maps [0, 1] -> [0, 1] with zero first and second derivatives at endpoints.
/// Even smoother than smoothStep for jerk-limited profiles.
///
/// @param u Input value in [0, 1]
/// @return Smoothly interpolated value in [0, 1]
[[nodiscard]] inline double smootherStep(double u) {
    u = std::clamp(u, 0.0, 1.0);
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0);
}

}  // namespace velocity_first
}  // namespace trajectory
}  // namespace ur_controller
