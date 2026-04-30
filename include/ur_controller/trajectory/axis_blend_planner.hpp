/// @file axis_blend_planner.hpp
/// @brief Per-axis polyline-with-blends trajectory planner
///
/// Plans a trajectory by treating each translational axis (x, y, z) and the
/// angular velocity vector (ω) as independent polylines in the time domain.
/// Each segment between waypoints is a constant-velocity cruise; smooth
/// transitions are inserted symmetrically around each waypoint time.
///
/// This is an alternative to the geometric `TrajectoryPlanner`. The two are
/// selected from the GUI via `TrajectoryConfig::planning_method`.
///
/// Semantic differences from the geometric planner:
///   - `blend_factor` is interpreted in **seconds** (half-blend window τ),
///     not metres.
///   - The waypoint position is generally not visited exactly: the corner
///     is rounded by an offset proportional to (Δv · τ / 4).
///   - First and last segments include half-blends from/to rest.

#pragma once

#include "ur_controller/kinematics/ur_kinematics.hpp"
#include "ur_controller/trajectory/types.hpp"
#include "ur_controller/trajectory/validator.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <vector>

namespace ur_controller {
namespace trajectory {

/// @brief Per-axis polyline-with-blends planner
///
/// Builds a velocity profile from a knot list:
///   - Cruise phases are pairs of knots with the same velocity.
///   - Blend phases linearly interpolate velocity between two knots.
/// The trajectory is then sampled at `config.sample_rate` by integrating
/// linear velocity (trapezoidal) and angular velocity (quaternion exponential).
class AxisBlendPlanner {
public:
    /// @brief Construct planner with kinematics engine
    explicit AxisBlendPlanner(const kinematics::URKinematics& kinematics);

    /// @brief Construct with custom configuration
    AxisBlendPlanner(const kinematics::URKinematics& kinematics,
                     const TrajectoryConfig& config);

    /// @brief Plan with explicit start joints (preferred entry point)
    ///
    /// @param waypoints Cartesian waypoints with positions, orientations,
    ///                  segment_time and blend_factor (seconds).
    /// @param start_joints Joint configuration at the first waypoint.
    /// @return Pre-computed trajectory at config_.sample_rate
    [[nodiscard]] PlannedTrajectory plan(
        const std::vector<Waypoint>& waypoints,
        const JointVector& start_joints);

    /// @brief Plan when start joints are unknown
    ///
    /// Derives start joints from waypoints[0].joints if present, otherwise
    /// from numerical IK seeded with joint-limit midpoints.
    [[nodiscard]] PlannedTrajectory plan(const std::vector<Waypoint>& waypoints);

    /// @brief Get current configuration
    [[nodiscard]] const TrajectoryConfig& config() const { return config_; }

    /// @brief Set configuration
    void setConfig(const TrajectoryConfig& cfg) { config_ = cfg; }

private:
    /// @brief A knot in the (t → v, ω) velocity profile.
    /// Between consecutive knots, v and ω are linearly interpolated.
    struct Knot {
        double t{0.0};                                ///< Time (s)
        Eigen::Vector3d v{Eigen::Vector3d::Zero()};   ///< Linear vel (m/s, world frame)
        Eigen::Vector3d w{Eigen::Vector3d::Zero()};   ///< Angular vel (rad/s, world frame)
    };

    /// @brief Pre-segmenting result: per-segment cruise vectors + per-waypoint blend τ
    struct ProfilePlan {
        std::vector<double> seg_T;               ///< Segment durations (size N-1)
        std::vector<Eigen::Vector3d> seg_v;      ///< Cruise linear velocity per segment
        std::vector<Eigen::Vector3d> seg_w;      ///< Cruise angular velocity per segment
        std::vector<double> tau;                 ///< Blend half-width per waypoint (size N)
        std::vector<double> wp_time;             ///< Time at each waypoint (size N)
        double total_duration{0.0};
    };

    /// @brief Build per-segment velocities and per-waypoint blend windows
    [[nodiscard]] ProfilePlan buildProfile(
        const std::vector<Waypoint>& waypoints,
        std::vector<ValidationMessage>& messages) const;

    /// @brief Build the knot list from a ProfilePlan
    [[nodiscard]] std::vector<Knot> buildKnots(const ProfilePlan& plan) const;

    /// @brief Sample the trajectory.
    ///
    /// Position is integrated from a piecewise-linear v(t) (linear-velocity
    /// blends → parabolic position blends in ℝ³).
    ///
    /// Orientation is evaluated via SQUAD (Shoemake's spherical-quadrangle
    /// interpolation) per segment, parameterised by u = (t-t_i)/T_i.
    /// Intermediate control quaternions are chosen so the SQUAD curve is
    /// C¹ continuous through every interior waypoint and has zero angular
    /// velocity at trajectory start/end. SQUAD passes exactly through
    /// each waypoint orientation, so there's no orientation "corner cut"
    /// — orientation tracks the q_i's exactly while remaining smooth.
    [[nodiscard]] PlannedTrajectory sample(
        const std::vector<Knot>& knots,
        const ProfilePlan& profile,
        const std::vector<Waypoint>& waypoints,
        const JointVector& start_joints,
        std::vector<ValidationMessage>& messages) const;

    /// @brief Quaternion logarithm (returns v with exp_quat(v) = q)
    /// |v| equals half the rotation angle.
    [[nodiscard]] static Eigen::Vector3d quatLog(const Eigen::Quaterniond& q);

    /// @brief Quaternion exponential (input v, |v| is half the rotation angle)
    [[nodiscard]] static Eigen::Quaterniond quatExp(const Eigen::Vector3d& v);

    const kinematics::URKinematics& kinematics_;
    TrajectoryConfig config_;
    TrajectoryValidator validator_;
};

}  // namespace trajectory
}  // namespace ur_controller
