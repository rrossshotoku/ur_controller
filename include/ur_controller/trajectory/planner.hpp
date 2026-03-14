/// @file planner.hpp
/// @brief Pre-computed trajectory planning from waypoints

#pragma once

#include "ur_controller/trajectory/types.hpp"
#include "ur_controller/trajectory/validator.hpp"
#include "ur_controller/kinematics/ur_kinematics.hpp"

#include <ruckig/ruckig.hpp>

#include <memory>
#include <vector>

namespace ur_controller {
namespace trajectory {

/// @brief Plans and pre-computes trajectories from waypoint lists
///
/// The planner takes a list of waypoints and generates a fully pre-computed
/// trajectory that can be played back at 500 Hz. The trajectory includes:
/// - Smooth joint-space motion using Ruckig
/// - Blend arcs at waypoint corners (if blend_radius > 0)
/// - Pause times at waypoints
/// - Validation of all waypoints and path segments
///
/// Thread safety: Thread-safe after construction.
class TrajectoryPlanner {
public:
    /// @brief Construct planner with kinematics engine
    /// @param kinematics Kinematics engine (must outlive planner)
    explicit TrajectoryPlanner(const kinematics::URKinematics& kinematics);

    /// @brief Construct with custom trajectory configuration
    TrajectoryPlanner(const kinematics::URKinematics& kinematics,
                      const TrajectoryConfig& config);

    /// @brief Plan a trajectory from waypoints
    ///
    /// Validates all waypoints, computes IK solutions, generates smooth
    /// motion profiles, and pre-samples the entire trajectory.
    ///
    /// @param waypoints List of waypoints to visit
    /// @return Planned trajectory with validation messages
    [[nodiscard]] PlannedTrajectory plan(
        const std::vector<Waypoint>& waypoints);

    /// @brief Plan a trajectory with custom start joints
    ///
    /// Use this when the robot is already at a known configuration.
    ///
    /// @param waypoints List of waypoints to visit
    /// @param start_joints Current joint configuration
    /// @return Planned trajectory with validation messages
    [[nodiscard]] PlannedTrajectory plan(
        const std::vector<Waypoint>& waypoints,
        const JointVector& start_joints);

    /// @brief Generate visualization data from a planned trajectory
    ///
    /// Creates data suitable for UI rendering including:
    /// - Dense path points for 3D line visualization
    /// - Velocity/acceleration graphs
    /// - Waypoint markers
    ///
    /// @param trajectory Planned trajectory
    /// @param waypoints Original waypoints
    /// @return Visualization data
    [[nodiscard]] TrajectoryVisualization generateVisualization(
        const PlannedTrajectory& trajectory,
        const std::vector<Waypoint>& waypoints) const;

    /// @brief Get the trajectory configuration
    [[nodiscard]] const TrajectoryConfig& config() const { return config_; }

    /// @brief Set trajectory configuration
    void setConfig(const TrajectoryConfig& config) { config_ = config; }

    /// @brief Get the validator
    [[nodiscard]] const TrajectoryValidator& validator() const { return validator_; }

private:
    /// @brief Segment between two waypoints
    struct Segment {
        size_t start_index{0};
        size_t end_index{0};
        JointVector start_joints;
        JointVector end_joints;
        double duration{0.0};
        double start_time{0.0};
        bool has_blend_in{false};
        bool has_blend_out{false};
    };

    /// @brief Plan joint-space motion using Ruckig
    /// @param segments Segments to plan
    /// @param waypoints Original waypoints
    /// @return Trajectory samples at sample_rate
    [[nodiscard]] std::vector<TrajectorySample> planJointMotion(
        const std::vector<Segment>& segments,
        const std::vector<Waypoint>& waypoints);

    /// @brief Create a single trajectory sample
    [[nodiscard]] TrajectorySample createSample(
        double time,
        const JointVector& joints,
        const JointVector& velocities) const;

    /// @brief Compute blend arc samples for a waypoint
    [[nodiscard]] std::vector<TrajectorySample> computeBlendArc(
        const Waypoint& waypoint,
        const JointVector& approach_joints,
        const JointVector& departure_joints,
        double blend_time,
        double start_time);

    /// @brief Calculate segment duration based on limits
    [[nodiscard]] double calculateSegmentDuration(
        const JointVector& start,
        const JointVector& end) const;

    /// @brief Plan a segment using Ruckig for optimal motion
    [[nodiscard]] std::vector<TrajectorySample> planSegmentWithRuckig(
        const JointVector& start_joints,
        const JointVector& end_joints,
        const JointVector& start_velocity,
        const JointVector& start_acceleration,
        double start_time);

    /// @brief Fallback smooth interpolation for a segment
    [[nodiscard]] std::vector<TrajectorySample> planSegmentSmooth(
        const Segment& seg,
        double start_time);

    const kinematics::URKinematics& kinematics_;
    TrajectoryConfig config_;
    TrajectoryValidator validator_;
};

}  // namespace trajectory
}  // namespace ur_controller
