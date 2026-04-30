/// @file velocity_first/planner.hpp
/// @brief Velocity-first trajectory planning
///
/// Plans trajectories by computing target velocities between waypoints,
/// then applying jerk-limited transitions. The position path emerges from
/// integrating velocity, naturally creating smooth blends around waypoints.

#pragma once

#include "ur_controller/trajectory/velocity_first/types.hpp"
#include "ur_controller/trajectory/types.hpp"
#include "ur_controller/kinematics/ur_kinematics.hpp"

#include <vector>

namespace ur_controller {
namespace trajectory {
namespace velocity_first {

/// @brief Velocity-first trajectory planner
///
/// Plans trajectories using the velocity-first approach:
/// 1. Compute target velocity vectors for each segment
/// 2. Plan jerk-limited transitions between velocities
/// 3. Integrate velocity to derive positions
/// 4. Solve IK at each sample point
///
/// Key characteristics:
/// - Path curves naturally during velocity transitions
/// - TCP blends around interior waypoints
/// - First/last waypoints are reached exactly (stop at rest)
/// - Jerk limit applies to velocity magnitude (scalar)
/// - Direction interpolates smoothly (SLERP)
///
/// Thread safety: Not thread-safe. Create separate instances for concurrent use.
class VelocityFirstPlanner {
public:
    /// @brief Construct planner with kinematics engine
    /// @param kinematics Kinematics engine (must outlive planner)
    explicit VelocityFirstPlanner(const kinematics::URKinematics& kinematics);

    /// @brief Construct with custom configuration
    VelocityFirstPlanner(
        const kinematics::URKinematics& kinematics,
        const VelocityFirstConfig& config);

    /// @brief Plan trajectory from timed waypoints
    ///
    /// Plans a trajectory through the given waypoints. The first waypoint
    /// starts from rest, the last waypoint ends at rest. Interior waypoints
    /// are blended (TCP passes near but may not hit exactly).
    ///
    /// @param waypoints Ordered list of waypoints with timing
    /// @param start_joints Current joint configuration
    /// @return Planned trajectory with pre-computed samples
    [[nodiscard]] PlannedTrajectory plan(
        const std::vector<TimedWaypoint>& waypoints,
        const JointVector& start_joints);

    /// @brief Plan trajectory from legacy Waypoint format
    ///
    /// Converts segment_time in each waypoint to absolute times,
    /// then plans using the velocity-first approach.
    ///
    /// @param waypoints Legacy waypoints with segment_time
    /// @param start_joints Current joint configuration
    /// @return Planned trajectory
    [[nodiscard]] PlannedTrajectory planFromLegacy(
        const std::vector<Waypoint>& waypoints,
        const JointVector& start_joints);

    /// @brief Generate visualization data from a planned trajectory
    ///
    /// Creates data suitable for UI rendering including path points,
    /// velocity graphs, and waypoint markers.
    ///
    /// @param trajectory Planned trajectory
    /// @param waypoints Original waypoints
    /// @return Visualization data for UI
    [[nodiscard]] TrajectoryVisualization generateVisualization(
        const PlannedTrajectory& trajectory,
        const std::vector<TimedWaypoint>& waypoints) const;

    /// @brief Get configuration
    [[nodiscard]] const VelocityFirstConfig& config() const { return config_; }

    /// @brief Set configuration
    void setConfig(const VelocityFirstConfig& config) { config_ = config; }

private:
    /// @brief Compute target velocities for all segments
    ///
    /// For each segment from waypoint i to i+1:
    ///   V_target = (P[i+1] - P[i]) / (t[i+1] - t[i])
    ///
    /// @param waypoints Waypoints with positions and times
    /// @return Target velocity for each segment
    [[nodiscard]] std::vector<SegmentVelocity> computeTargetVelocities(
        const std::vector<TimedWaypoint>& waypoints) const;

    /// @brief Compute velocity transitions at waypoints
    ///
    /// Creates symmetric jerk-limited transitions between segment velocities.
    /// Includes transitions at first waypoint (rest to V1) and last (V_n to rest).
    ///
    /// @param waypoints Waypoints with timing
    /// @param velocities Target velocities for each segment
    /// @return Velocity transitions
    [[nodiscard]] std::vector<VelocityTransition> computeTransitions(
        const std::vector<TimedWaypoint>& waypoints,
        const std::vector<SegmentVelocity>& velocities) const;

    /// @brief Compute minimum transition duration
    ///
    /// Based on jerk limit and velocity change:
    ///   T_min = 2 * sqrt(|dV| / J_max)  for triangular profile
    ///
    /// @param v1_speed Speed before transition
    /// @param v2_speed Speed after transition
    /// @return Minimum transition duration (seconds)
    [[nodiscard]] double computeMinTransitionDuration(
        double v1_speed,
        double v2_speed) const;

    /// @brief Validate velocities against limits
    ///
    /// Checks that all segment velocities are within max_speed.
    ///
    /// @param velocities Segment velocities to check
    /// @param messages Output for validation messages
    /// @return True if all velocities are valid
    [[nodiscard]] bool validateVelocities(
        const std::vector<SegmentVelocity>& velocities,
        std::vector<ValidationMessage>& messages) const;

    /// @brief Check for transition overlaps
    ///
    /// Ensures transitions don't overlap (segment not too short).
    ///
    /// @param transitions Computed transitions
    /// @param messages Output for validation messages
    /// @return True if no overlaps
    [[nodiscard]] bool validateTransitions(
        const std::vector<VelocityTransition>& transitions,
        std::vector<ValidationMessage>& messages) const;

    /// @brief Evaluate velocity at time t
    ///
    /// Considers active transitions and cruise phases.
    ///
    /// @param t Time in seconds
    /// @param velocities Segment velocities
    /// @param transitions Velocity transitions
    /// @return Velocity vector at time t
    [[nodiscard]] Eigen::Vector3d velocityAt(
        double t,
        const std::vector<SegmentVelocity>& velocities,
        const std::vector<VelocityTransition>& transitions) const;

    /// @brief Evaluate orientation at time t
    ///
    /// Interpolates between waypoint orientations based on time.
    ///
    /// @param t Time in seconds
    /// @param waypoints Waypoints with orientations
    /// @return Orientation at time t
    [[nodiscard]] Eigen::Quaterniond orientationAt(
        double t,
        const std::vector<TimedWaypoint>& waypoints) const;

    /// @brief Evaluate velocity within a transition
    ///
    /// Computes blended velocity using S-curve speed profile and SLERP direction.
    ///
    /// @param t Time within transition
    /// @param trans The velocity transition
    /// @return Blended velocity vector
    [[nodiscard]] Eigen::Vector3d evaluateTransition(
        double t,
        const VelocityTransition& trans) const;

    /// @brief Sample trajectory at sample_rate
    ///
    /// Integrates velocity to get positions, solves IK at each point.
    ///
    /// @param waypoints Waypoints with timing
    /// @param velocities Segment velocities
    /// @param transitions Velocity transitions
    /// @param start_joints Initial joint configuration
    /// @return Trajectory samples at sample_rate Hz
    [[nodiscard]] std::vector<TrajectorySample> sampleTrajectory(
        const std::vector<TimedWaypoint>& waypoints,
        const std::vector<SegmentVelocity>& velocities,
        const std::vector<VelocityTransition>& transitions,
        const JointVector& start_joints) const;

    /// @brief Apply final position correction
    ///
    /// Corrects accumulated integration error in final approach.
    ///
    /// @param samples Trajectory samples to correct
    /// @param target_position Final waypoint position
    /// @param total_duration Total trajectory time
    void applyFinalCorrection(
        std::vector<TrajectorySample>& samples,
        const Eigen::Vector3d& target_position,
        double total_duration) const;

    /// @brief Find which segment contains time t
    ///
    /// @param t Time in seconds
    /// @param waypoints Waypoints with timing
    /// @return Segment index (0 to N-2, or -1 if before first)
    [[nodiscard]] int findSegmentAt(
        double t,
        const std::vector<TimedWaypoint>& waypoints) const;

    /// @brief Find which transition contains time t
    ///
    /// @param t Time in seconds
    /// @param transitions Velocity transitions
    /// @return Transition index, or -1 if not in any transition
    [[nodiscard]] int findTransitionAt(
        double t,
        const std::vector<VelocityTransition>& transitions) const;

    const kinematics::URKinematics& kinematics_;
    VelocityFirstConfig config_;
};

}  // namespace velocity_first
}  // namespace trajectory
}  // namespace ur_controller
