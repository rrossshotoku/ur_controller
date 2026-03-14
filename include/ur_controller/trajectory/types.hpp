/// @file types.hpp
/// @brief Type definitions for the trajectory system

#pragma once

#include "ur_controller/kinematics/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <optional>

namespace ur_controller {
namespace trajectory {

using kinematics::JointVector;

// =============================================================================
// Waypoint Definition (from UI)
// =============================================================================

/// @brief A single waypoint in a trajectory
struct Waypoint {
    Eigen::Vector3d position{0, 0, 0};      ///< XYZ position in meters (base frame)
    Eigen::Quaterniond orientation{1, 0, 0, 0}; ///< Orientation as quaternion

    double blend_radius{0.0};   ///< Blend radius in meters (0 = stop at waypoint)
    double segment_time{0.0};   ///< Time to reach this waypoint from previous (0 = auto)
    double pause_time{0.0};     ///< Time to pause at this waypoint

    /// @brief Convert to Eigen::Isometry3d pose
    [[nodiscard]] Eigen::Isometry3d toPose() const {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = position;
        pose.linear() = orientation.toRotationMatrix();
        return pose;
    }

    /// @brief Create waypoint from Isometry3d pose
    static Waypoint fromPose(const Eigen::Isometry3d& pose) {
        Waypoint wp;
        wp.position = pose.translation();
        wp.orientation = Eigen::Quaterniond(pose.rotation());
        return wp;
    }
};

// =============================================================================
// Trajectory Configuration
// =============================================================================

/// @brief Configuration for trajectory planning
struct TrajectoryConfig {
    // Cartesian limits
    double max_linear_velocity{0.5};        ///< m/s
    double max_linear_acceleration{1.0};    ///< m/s²
    double max_angular_velocity{1.0};       ///< rad/s
    double max_angular_acceleration{2.0};   ///< rad/s²

    // Joint limits (used for validation)
    double max_joint_velocity{1.5};         ///< rad/s (per joint)
    double max_joint_acceleration{3.0};     ///< rad/s² (per joint)
    double max_joint_jerk{10.0};            ///< rad/s³ (per joint)

    // Planning parameters
    double sample_rate{500.0};              ///< Hz (samples per second)
    double path_tolerance{0.001};           ///< meters (for blend arc computation)
};

// =============================================================================
// Pre-computed Trajectory Samples
// =============================================================================

/// @brief A single sample point in a pre-computed trajectory
struct TrajectorySample {
    double time{0.0};                       ///< Seconds from trajectory start
    JointVector joints;                     ///< Joint positions (radians)
    Eigen::Isometry3d pose;                 ///< TCP pose (for visualization)
    Eigen::Vector3d linear_velocity{0, 0, 0};   ///< TCP linear velocity (m/s)
    double speed{0.0};                      ///< TCP speed magnitude (m/s)
};

// =============================================================================
// Validation
// =============================================================================

/// @brief Severity level for validation warnings
enum class ValidationSeverity {
    Info,       ///< Informational message
    Warning,    ///< Warning (trajectory will execute but may have issues)
    Error       ///< Error (trajectory cannot be executed)
};

/// @brief A validation warning or error
struct ValidationMessage {
    ValidationSeverity severity{ValidationSeverity::Info};
    std::string message;
    std::optional<size_t> waypoint_index;   ///< Which waypoint (if applicable)
    std::optional<double> time;             ///< When in trajectory (if applicable)
};

// =============================================================================
// Planned Trajectory
// =============================================================================

/// @brief A fully pre-computed trajectory ready for execution
struct PlannedTrajectory {
    std::vector<TrajectorySample> samples;  ///< Pre-computed samples at sample_rate
    double total_duration{0.0};             ///< Total trajectory time in seconds
    double sample_period{0.002};            ///< 1/sample_rate in seconds

    std::vector<ValidationMessage> messages;    ///< Validation messages
    bool valid{false};                          ///< True if trajectory can be executed

    /// @brief Get sample at time t (with interpolation if needed)
    [[nodiscard]] TrajectorySample sampleAt(double t) const;

    /// @brief Get sample by index
    [[nodiscard]] const TrajectorySample& operator[](size_t index) const {
        return samples[index];
    }

    /// @brief Number of samples
    [[nodiscard]] size_t size() const { return samples.size(); }

    /// @brief Check if trajectory has any errors
    [[nodiscard]] bool hasErrors() const;

    /// @brief Check if trajectory has any warnings
    [[nodiscard]] bool hasWarnings() const;
};

// =============================================================================
// Visualization Data (for UI)
// =============================================================================

/// @brief Waypoint marker for UI display
struct WaypointMarker {
    size_t index;                   ///< Waypoint index
    Eigen::Vector3d position;       ///< XYZ position
    double time;                    ///< Time when reached
    double blend_radius;            ///< Blend radius
    bool has_pause;                 ///< True if pause at this waypoint
};

/// @brief Visualization data to send to the UI
struct TrajectoryVisualization {
    // Path visualization
    std::vector<Eigen::Vector3d> path_points;   ///< Dense path for 3D line
    std::vector<double> path_times;             ///< Time at each path point

    // Waypoint markers
    std::vector<WaypointMarker> waypoints;

    // Timing graphs (indexed by time)
    std::vector<double> times;          ///< Time values for graphs
    std::vector<double> speeds;         ///< TCP speed (m/s)
    std::vector<double> accelerations;  ///< TCP acceleration (m/s²)

    // Joint data (for optional joint-space visualization)
    std::vector<std::array<double, 6>> joint_positions;  ///< Joint angles over time
    std::vector<std::array<double, 6>> joint_velocities; ///< Joint velocities over time

    // Summary
    double total_duration{0.0};
    double total_distance{0.0};         ///< Total path length in meters
    double max_speed{0.0};              ///< Maximum TCP speed reached
};

// =============================================================================
// Executor State
// =============================================================================

/// @brief State of the trajectory executor
enum class ExecutorState {
    Idle,       ///< No trajectory loaded or execution complete
    Ready,      ///< Trajectory loaded, ready to execute
    Running,    ///< Currently executing trajectory
    Paused,     ///< Execution paused
    Error       ///< Error occurred during execution
};

/// @brief Convert ExecutorState to string
[[nodiscard]] inline const char* executorStateToString(ExecutorState state) {
    switch (state) {
        case ExecutorState::Idle:    return "idle";
        case ExecutorState::Ready:   return "ready";
        case ExecutorState::Running: return "running";
        case ExecutorState::Paused:  return "paused";
        case ExecutorState::Error:   return "error";
        default:                     return "unknown";
    }
}

}  // namespace trajectory
}  // namespace ur_controller
