/// @file validator.hpp
/// @brief Trajectory validation for reachability, singularities, and limits

#pragma once

#include "ur_controller/trajectory/types.hpp"
#include "ur_controller/kinematics/ur_kinematics.hpp"

#include <vector>

namespace ur_controller {
namespace trajectory {

/// @brief Result of validating a single waypoint
struct WaypointValidation {
    size_t index{0};                        ///< Waypoint index
    bool reachable{false};                  ///< IK solution exists
    bool within_limits{false};              ///< Solution within joint limits
    size_t num_solutions{0};                ///< Number of IK solutions found
    std::optional<JointVector> best_solution;  ///< Best IK solution if found
    std::vector<ValidationMessage> messages;   ///< Validation messages
};

/// @brief Result of validating path between waypoints
struct PathValidation {
    size_t segment_index{0};                ///< Segment index (from waypoint i to i+1)
    bool singularity_free{true};            ///< No singularities along segment
    bool velocity_feasible{true};           ///< Joint velocities within limits
    double max_velocity_ratio{0.0};         ///< Max ratio of velocity to limit (>1 = violation)
    std::vector<ValidationMessage> messages;
};

/// @brief Complete validation result for a trajectory
struct TrajectoryValidation {
    std::vector<WaypointValidation> waypoints;
    std::vector<PathValidation> segments;
    std::vector<ValidationMessage> all_messages;  ///< All messages combined
    bool valid{false};                            ///< True if executable

    /// @brief Check if any errors exist
    [[nodiscard]] bool hasErrors() const;

    /// @brief Check if any warnings exist
    [[nodiscard]] bool hasWarnings() const;

    /// @brief Get error count
    [[nodiscard]] size_t errorCount() const;

    /// @brief Get warning count
    [[nodiscard]] size_t warningCount() const;
};

/// @brief Configuration for trajectory validation
struct ValidationConfig {
    double singularity_threshold{100.0};    ///< Jacobian condition number threshold
    double max_joint_jump{3.14159};         ///< Max joint change between waypoints (rad) - detects config flips
    double path_sample_distance{0.01};      ///< Distance between path samples (m)
    bool check_singularities{true};         ///< Enable singularity checking
    bool check_velocities{true};            ///< Enable velocity limit checking
};

/// @brief Validates trajectories for reachability, singularities, and limits
///
/// The validator checks:
/// 1. All waypoints are reachable (valid IK solutions exist)
/// 2. Path between waypoints doesn't pass through singularities
/// 3. Required joint velocities don't exceed limits
/// 4. IK solutions have continuity (no large jumps)
///
/// Thread safety: Thread-safe after construction.
class TrajectoryValidator {
public:
    /// @brief Construct validator with kinematics engine
    /// @param kinematics Kinematics engine (must outlive validator)
    explicit TrajectoryValidator(const kinematics::URKinematics& kinematics);

    /// @brief Construct with custom configuration
    TrajectoryValidator(const kinematics::URKinematics& kinematics,
                        const ValidationConfig& config);

    /// @brief Validate a list of waypoints
    ///
    /// Performs full validation including:
    /// - IK reachability for each waypoint
    /// - Singularity detection along path
    /// - Velocity limit checking
    /// - IK solution continuity
    ///
    /// @param waypoints Waypoints to validate
    /// @param trajectory_config Trajectory parameters (velocity limits, etc.)
    /// @return Complete validation result
    [[nodiscard]] TrajectoryValidation validate(
        const std::vector<Waypoint>& waypoints,
        const TrajectoryConfig& trajectory_config) const;

    /// @brief Validate a single waypoint for reachability
    ///
    /// @param waypoint Waypoint to validate
    /// @param current_joints Optional current joint position for solution selection
    /// @return Validation result for this waypoint
    [[nodiscard]] WaypointValidation validateWaypoint(
        const Waypoint& waypoint,
        const std::optional<JointVector>& current_joints = std::nullopt) const;

    /// @brief Validate path segment between two waypoints
    ///
    /// @param start Start waypoint
    /// @param end End waypoint
    /// @param start_joints Joint solution at start
    /// @param segment_time Time for this segment
    /// @param config Trajectory configuration
    /// @return Validation result for this segment
    [[nodiscard]] PathValidation validateSegment(
        const Waypoint& start,
        const Waypoint& end,
        const JointVector& start_joints,
        double segment_time,
        const TrajectoryConfig& config) const;

    /// @brief Check if a Cartesian pose is reachable
    [[nodiscard]] bool isReachable(const Eigen::Isometry3d& pose) const;

    /// @brief Check if path between two poses is singularity-free
    [[nodiscard]] bool isSingularityFree(
        const Eigen::Isometry3d& start,
        const Eigen::Isometry3d& end,
        const JointVector& start_joints) const;

    /// @brief Get the validation configuration
    [[nodiscard]] const ValidationConfig& config() const { return config_; }

    /// @brief Set validation configuration
    void setConfig(const ValidationConfig& config) { config_ = config; }

private:
    /// @brief Interpolate between two poses
    [[nodiscard]] Eigen::Isometry3d interpolatePose(
        const Eigen::Isometry3d& start,
        const Eigen::Isometry3d& end,
        double t) const;

    /// @brief Estimate required joint velocities for a segment
    [[nodiscard]] JointVector estimateJointVelocities(
        const JointVector& start,
        const JointVector& end,
        double time) const;

    /// @brief Check if joint velocities exceed limits
    [[nodiscard]] bool velocitiesWithinLimits(
        const JointVector& velocities,
        const TrajectoryConfig& config) const;

    const kinematics::URKinematics& kinematics_;
    ValidationConfig config_;
};

}  // namespace trajectory
}  // namespace ur_controller
