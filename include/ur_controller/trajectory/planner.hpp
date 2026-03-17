/// @file planner.hpp
/// @brief Pre-computed trajectory planning from waypoints

#pragma once

#include "ur_controller/trajectory/types.hpp"
#include "ur_controller/trajectory/validator.hpp"
#include "ur_controller/kinematics/ur_kinematics.hpp"

#include <ruckig/ruckig.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace ur_controller {
namespace trajectory {

/// @brief Geometric information for a blend arc at a waypoint
struct BlendArcInfo {
    Eigen::Vector3d center;           ///< Arc center in Cartesian space
    double radius{0.0};               ///< Arc radius (geometric radius, not blend_radius)
    Eigen::Vector3d arc_start;        ///< Cartesian start point of arc
    Eigen::Vector3d arc_end;          ///< Cartesian end point of arc
    Eigen::Vector3d start_tangent;    ///< Direction at arc start (incoming)
    Eigen::Vector3d end_tangent;      ///< Direction at arc end (outgoing)
    Eigen::Vector3d normal;           ///< Normal to the arc plane
    double arc_angle{0.0};            ///< Total arc angle (radians)
    double arc_length{0.0};           ///< Distance along arc
    JointVector start_joints;         ///< IK solution at arc start
    JointVector end_joints;           ///< IK solution at arc end
    Eigen::Quaterniond start_orientation;  ///< Orientation at arc start
    Eigen::Quaterniond end_orientation;    ///< Orientation at arc end
};

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

    // =========================================================================
    // New API: SetupPose and Sequence planning
    // =========================================================================

    /// @brief Plan a SetupPose (joint-space move)
    ///
    /// Plans a smooth joint-space trajectory from current position to target.
    /// Uses Ruckig for jerk-limited motion profiles.
    ///
    /// @param setup_pose Target setup pose with joint positions
    /// @param start_joints Current joint configuration
    /// @return Planned trajectory (joint-space interpolation)
    [[nodiscard]] PlannedTrajectory planSetupPose(
        const SetupPose& setup_pose,
        const JointVector& start_joints);

    /// @brief Plan a Sequence (linear Cartesian moves)
    ///
    /// Plans linear moves through waypoints with the arm configuration
    /// locked based on the entry joints. If any waypoint cannot be reached
    /// in the locked configuration, planning fails.
    ///
    /// @param sequence Sequence of Cartesian waypoints
    /// @param entry_joints Joint configuration at sequence entry
    /// @return Planned trajectory with linear TCP paths
    [[nodiscard]] PlannedTrajectory planSequence(
        const Sequence& sequence,
        const JointVector& entry_joints);

    /// @brief Plan a complete Trajectory (setup poses + sequences)
    ///
    /// Plans each element in order, chaining them together. SetupPoses use
    /// joint-space moves, Sequences use linear Cartesian moves with locked
    /// configuration.
    ///
    /// @param trajectory Complete trajectory with elements
    /// @param start_joints Current joint configuration
    /// @return Planned trajectory ready for execution
    [[nodiscard]] PlannedTrajectory planTrajectory(
        const Trajectory& trajectory,
        const JointVector& start_joints);

    // =========================================================================
    // Legacy API (deprecated - use planSequence instead)
    // =========================================================================

    /// @brief Plan a trajectory from waypoints (legacy)
    /// @deprecated Use planSequence() for new code
    ///
    /// Validates all waypoints, computes IK solutions, generates smooth
    /// motion profiles, and pre-samples the entire trajectory.
    ///
    /// @param waypoints List of waypoints to visit
    /// @return Planned trajectory with validation messages
    [[nodiscard]] PlannedTrajectory plan(
        const std::vector<Waypoint>& waypoints);

    /// @brief Plan a trajectory with custom start joints (legacy)
    /// @deprecated Use planSequence() for new code
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

    /// @brief Get current IK method
    [[nodiscard]] IKMethod getIKMethod() const { return config_.ik_method; }

    /// @brief Set IK method
    void setIKMethod(IKMethod method) { config_.ik_method = method; }

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
        std::optional<BlendArcInfo> blend_arc;  ///< Blend arc at end of segment
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

    /// @brief Compute blend arc geometry for a waypoint
    /// @param prev_wp Previous waypoint (for incoming direction)
    /// @param blend_wp Waypoint where blending occurs
    /// @param next_wp Next waypoint (for outgoing direction)
    /// @param prev_joints Joint configuration at previous waypoint
    /// @return BlendArcInfo with arc geometry, or nullopt if blend not possible
    [[nodiscard]] std::optional<BlendArcInfo> computeBlendGeometry(
        const Waypoint& prev_wp,
        const Waypoint& blend_wp,
        const Waypoint& next_wp,
        const JointVector& prev_joints);

    /// @brief Plan samples along a blend arc in Cartesian space
    /// @param arc_info Blend arc geometry
    /// @param start_time Trajectory time at arc start
    /// @param arc_duration Duration for the arc (calculated by caller for timing consistency)
    /// @return Trajectory samples along the arc
    [[nodiscard]] std::vector<TrajectorySample> planBlendArcCartesian(
        const BlendArcInfo& arc_info,
        double start_time,
        double arc_duration);

    /// @brief Compute blend arc samples for a waypoint (legacy, unused)
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
    /// @param desired_duration User-specified duration (0 = time-optimal)
    /// @param target_velocity Target velocity at end (zero = stop, non-zero = blend)
    [[nodiscard]] std::vector<TrajectorySample> planSegmentWithRuckig(
        const JointVector& start_joints,
        const JointVector& end_joints,
        const JointVector& start_velocity,
        const JointVector& start_acceleration,
        double start_time,
        double desired_duration = 0.0,
        const JointVector& target_velocity = JointVector::Zero());

    /// @brief Fallback smooth interpolation for a segment
    [[nodiscard]] std::vector<TrajectorySample> planSegmentSmooth(
        const Segment& seg,
        double start_time);

    /// @brief Plan a linear Cartesian segment (MoveL)
    ///
    /// Interpolates TCP position linearly and orientation via SLERP.
    /// Solves IK at each sample point to get joint positions.
    /// Each IK solution is unwrapped relative to the previous sample,
    /// so joint angles accumulate naturally without discontinuities.
    ///
    /// @param start_pose Starting TCP pose
    /// @param end_pose Ending TCP pose
    /// @param start_joints Joint configuration at start (angles may be outside ±π)
    /// @param start_time Trajectory time at segment start
    /// @param duration Segment duration in seconds
    /// @param constant_velocity If true, use constant velocity (for blending).
    ///                          If false, use S-curve with accel/decel (for stopping).
    /// @return Trajectory samples with linear TCP path
    [[nodiscard]] std::vector<TrajectorySample> planSegmentLinearCartesian(
        const Eigen::Isometry3d& start_pose,
        const Eigen::Isometry3d& end_pose,
        const JointVector& start_joints,
        double start_time,
        double duration,
        bool constant_velocity = false);

    /// @brief Plan a segment with linear portion + half blend arc (unified sampling)
    ///
    /// Samples the linear portion and first/second half of the arc as one
    /// continuous path with evenly spaced samples at constant velocity.
    ///
    /// @param start_pose Starting TCP pose
    /// @param start_joints Joint configuration at start
    /// @param arc_info Blend arc geometry
    /// @param start_time Trajectory time at segment start
    /// @param duration Total segment duration
    /// @param first_half If true, plan linear + first half of arc.
    ///                   If false, plan second half of arc + linear to end.
    /// @param end_pose End pose (used when first_half=false)
    /// @return Trajectory samples with evenly spaced points
    [[nodiscard]] std::vector<TrajectorySample> planSegmentWithHalfArc(
        const Eigen::Isometry3d& start_pose,
        const JointVector& start_joints,
        const BlendArcInfo& arc_info,
        double start_time,
        double duration,
        bool first_half,
        const Eigen::Isometry3d& end_pose = Eigen::Isometry3d::Identity());

    /// @brief Solve IK using the configured method
    ///
    /// Uses analytical IK with filtering (IKMethod::Analytical) or
    /// numerical IK iteration (IKMethod::Numerical) based on config.
    ///
    /// @param pose Target TCP pose
    /// @param seed Current/seed joint configuration
    /// @return Joint solution, or nullopt if no solution found
    [[nodiscard]] std::optional<JointVector> solveIK(
        const Eigen::Isometry3d& pose,
        const JointVector& seed) const;

    const kinematics::URKinematics& kinematics_;
    TrajectoryConfig config_;
    TrajectoryValidator validator_;
};

}  // namespace trajectory
}  // namespace ur_controller
