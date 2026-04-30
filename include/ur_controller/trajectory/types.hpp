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
// Setup Pose - Joint-space target (executed as joint move)
// =============================================================================

/// @brief A setup pose defined by joint positions
/// @details Executed as a joint-space move (MoveJ style) via servoJ streaming.
/// Used to get the robot into a known configuration before a linear sequence.
struct SetupPose {
    JointVector joints;             ///< Target joint positions (radians)
    std::string name;               ///< Optional name/label for this pose
    double move_time{0.0};          ///< Time to reach pose (0 = auto based on limits)

    /// @brief Create SetupPose from joint array
    static SetupPose fromJoints(const JointVector& q, const std::string& label = "") {
        SetupPose sp;
        sp.joints = q;
        sp.name = label;
        return sp;
    }

    /// @brief Create SetupPose from raw array
    static SetupPose fromArray(const std::array<double, 6>& q, const std::string& label = "") {
        SetupPose sp;
        for (size_t i = 0; i < 6; ++i) {
            sp.joints[static_cast<Eigen::Index>(i)] = q[i];
        }
        sp.name = label;
        return sp;
    }
};

// =============================================================================
// Sequence Waypoint - Cartesian target (executed as linear move)
// =============================================================================

/// @brief A waypoint in a linear sequence (Cartesian space)
/// @details Part of a Sequence that is executed as linear moves (MoveL style).
/// The arm configuration is locked at the start of the sequence.
struct SequenceWaypoint {
    Eigen::Vector3d position{0, 0, 0};          ///< XYZ position in meters (base frame)
    Eigen::Quaterniond orientation{1, 0, 0, 0}; ///< Orientation as quaternion

    double blend_factor{0.0};   ///< Blend factor: distance from vertex to arc apex (0 = stop at waypoint)
    double segment_time{0.0};   ///< Time to reach this waypoint from previous (0 = auto)
    double pause_time{0.0};     ///< Time to pause at this waypoint

    /// @brief Joint positions when this waypoint was taught (optional)
    /// @details If set, these joints define the intended arm configuration for this pose
    std::optional<Eigen::Matrix<double, 6, 1>> joints;

    /// @brief Convert to Eigen::Isometry3d pose
    [[nodiscard]] Eigen::Isometry3d toPose() const {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = position;
        pose.linear() = orientation.toRotationMatrix();
        return pose;
    }

    /// @brief Create waypoint from Isometry3d pose
    static SequenceWaypoint fromPose(const Eigen::Isometry3d& pose) {
        SequenceWaypoint wp;
        wp.position = pose.translation();
        wp.orientation = Eigen::Quaterniond(pose.rotation());
        return wp;
    }
};

// =============================================================================
// Sequence - Linear motion through Cartesian waypoints
// =============================================================================

/// @brief A sequence of Cartesian waypoints executed as linear moves
/// @details The arm configuration is determined by the robot's joint positions
/// at the start of the sequence and remains locked throughout.
struct Sequence {
    std::vector<SequenceWaypoint> waypoints;    ///< Waypoints to traverse
    std::string name;                            ///< Optional name/label
    std::optional<JointVector> entry_joints;     ///< Joint config when first waypoint was captured

    /// @brief Check if sequence is empty
    [[nodiscard]] bool empty() const { return waypoints.empty(); }

    /// @brief Number of waypoints
    [[nodiscard]] size_t size() const { return waypoints.size(); }

    /// @brief Check if entry joints are specified
    [[nodiscard]] bool hasEntryJoints() const { return entry_joints.has_value(); }
};

// =============================================================================
// Trajectory Element - Either a SetupPose or a Sequence
// =============================================================================

/// @brief Type of trajectory element
enum class TrajectoryElementType {
    SetupPose,  ///< Joint-space move to a setup pose
    Sequence    ///< Linear moves through Cartesian waypoints
};

/// @brief A single element in a trajectory (either SetupPose or Sequence)
struct TrajectoryElement {
    TrajectoryElementType type;
    SetupPose setup_pose;       ///< Valid if type == SetupPose
    Sequence sequence;          ///< Valid if type == Sequence

    /// @brief Create a SetupPose element
    static TrajectoryElement makeSetupPose(const SetupPose& pose) {
        TrajectoryElement elem;
        elem.type = TrajectoryElementType::SetupPose;
        elem.setup_pose = pose;
        return elem;
    }

    /// @brief Create a Sequence element
    static TrajectoryElement makeSequence(const Sequence& seq) {
        TrajectoryElement elem;
        elem.type = TrajectoryElementType::Sequence;
        elem.sequence = seq;
        return elem;
    }
};

/// @brief A complete trajectory consisting of setup poses and sequences
struct Trajectory {
    std::vector<TrajectoryElement> elements;    ///< Ordered list of elements
    std::string name;                            ///< Optional trajectory name

    /// @brief Add a setup pose to the trajectory
    void addSetupPose(const SetupPose& pose) {
        elements.push_back(TrajectoryElement::makeSetupPose(pose));
    }

    /// @brief Add a sequence to the trajectory
    void addSequence(const Sequence& seq) {
        elements.push_back(TrajectoryElement::makeSequence(seq));
    }

    /// @brief Check if trajectory is empty
    [[nodiscard]] bool empty() const { return elements.empty(); }

    /// @brief Number of elements
    [[nodiscard]] size_t size() const { return elements.size(); }
};

// =============================================================================
// Legacy Waypoint (for backward compatibility)
// =============================================================================

/// @brief A single waypoint in a trajectory (legacy - use SequenceWaypoint)
/// @deprecated Use SequenceWaypoint for new code
struct Waypoint {
    Eigen::Vector3d position{0, 0, 0};      ///< XYZ position in meters (base frame)
    Eigen::Quaterniond orientation{1, 0, 0, 0}; ///< Orientation as quaternion

    double blend_factor{0.0};   ///< Blend factor: distance from vertex to arc apex (0 = stop at waypoint)
    double segment_time{0.0};   ///< Time to reach this waypoint from previous (0 = auto)
    double pause_time{0.0};     ///< Time to pause at this waypoint

    /// @brief Joint positions when this waypoint was taught (optional)
    std::optional<Eigen::Matrix<double, 6, 1>> joints;

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

    /// @brief Convert to SequenceWaypoint
    [[nodiscard]] SequenceWaypoint toSequenceWaypoint() const {
        SequenceWaypoint sw;
        sw.position = position;
        sw.orientation = orientation;
        sw.blend_factor = blend_factor;
        sw.segment_time = segment_time;
        sw.pause_time = pause_time;
        sw.joints = joints;
        return sw;
    }
};

// =============================================================================
// Trajectory Configuration
// =============================================================================

/// @brief Method used to plan a trajectory
/// @details Selectable from the GUI; controls which planner backend is used
/// when /api/trajectory/plan2 dispatches.
enum class PlanningMethod {
    Geometric,   ///< Inscribed-arc blends + S-curve along path length (legacy/default)
    AxisBlend    ///< Per-axis polyline-with-blends in the time domain
};

/// @brief Convert PlanningMethod to string
[[nodiscard]] inline const char* planningMethodToString(PlanningMethod m) {
    switch (m) {
        case PlanningMethod::Geometric:  return "geometric";
        case PlanningMethod::AxisBlend:  return "axis_blend";
        default:                          return "geometric";
    }
}

/// @brief Parse PlanningMethod from string
/// @return Geometric if string is unrecognized
[[nodiscard]] inline PlanningMethod planningMethodFromString(const std::string& s) {
    if (s == "axis_blend") return PlanningMethod::AxisBlend;
    return PlanningMethod::Geometric;
}

/// @brief Configuration for trajectory planning
struct TrajectoryConfig {
    // Cartesian limits
    double max_linear_velocity{0.5};        ///< m/s
    double max_linear_acceleration{1.0};    ///< m/s²
    double max_linear_jerk{5.0};            ///< m/s³
    double max_angular_velocity{1.0};       ///< rad/s
    double max_angular_acceleration{2.0};   ///< rad/s²

    // Joint limits (used for validation)
    double max_joint_velocity{1.5};         ///< rad/s (per joint)
    double max_joint_acceleration{3.0};     ///< rad/s² (per joint)
    double max_joint_jerk{10.0};            ///< rad/s³ (per joint)

    // Planning parameters
    double sample_rate{500.0};              ///< Hz (samples per second)
    double path_tolerance{0.001};           ///< meters (for blend arc computation)

    /// @brief Selected planner backend (default: Geometric)
    PlanningMethod planning_method{PlanningMethod::Geometric};
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
    double blend_factor;            ///< Blend factor (distance from vertex to arc apex)
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
    std::vector<double> jerks;          ///< TCP jerk (m/s³)

    // Segment boundaries for waypoint markers on graph
    std::vector<double> segment_times;  ///< Time at each waypoint/segment boundary

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
