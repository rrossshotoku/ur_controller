/// @file planner.cpp
/// @brief Trajectory planner implementation with Ruckig motion profiles

#include "ur_controller/trajectory/planner.hpp"
#include "ur_controller/trajectory/s_curve.hpp"

#include <ruckig/ruckig.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include <algorithm>
#include <array>
#include <cmath>

namespace ur_controller {
namespace trajectory {

namespace {

constexpr double kPi = 3.14159265358979323846;

/// @brief Wrap angle to [-pi, pi]
double wrapAngle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

/// @brief Compute wrapped joint difference
JointVector jointDifference(const JointVector& end, const JointVector& start) {
    JointVector diff;
    for (size_t i = 0; i < 6; ++i) {
        diff[static_cast<Eigen::Index>(i)] = wrapAngle(end[static_cast<Eigen::Index>(i)] - start[static_cast<Eigen::Index>(i)]);
    }
    return diff;
}

/// @brief Smooth step function (cubic Hermite) - fallback for simple segments
double smoothStep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

/// @brief Result of velocity computation
struct VelocityComputeResult {
    bool success{false};
    double entry_velocity{0.0};
    double actual_duration{0.0};
    std::string error_message;

    // For failed cases, provide actionable alternatives
    double min_duration_this_seg{0.0};    // Min duration for this segment to achieve required V_exit
    double max_achievable_exit_vel{0.0};  // Max V_exit this segment can achieve with current duration
};

/// @brief Compute required entry velocity using exact analytical solution
///
/// For symmetric jerk-limited motion (S-curve profiles), the distance covered
/// equals exactly (V_entry + V_exit) * T / 2. This is NOT an approximation -
/// the curved jerk phases at the start and end of the velocity profile add
/// equal and opposite corrections that cancel out in the distance integral.
///
/// Mathematical proof (triangular profile, T < 2*A/J):
///   Phase 1: d1 = V_entry*T/2 - J*T³/48
///   Phase 2: d2 = V_entry*T/2 - 5*J*T³/48
///   Total:   D  = V_entry*T - J*T³/8
///   With ΔV = J*T²/4, trapezoidal area = V_entry*T - J*T³/8 = D ✓
///
/// Therefore: V_entry = 2*D/T - V_exit (exact solution)
///
/// Then validates that the velocity change is achievable:
/// - V_entry is within [0, V_max]
/// - For triangular profile (T < 2*A/J): |ΔV| ≤ J*T²/4
/// - For full profile (T >= 2*A/J): |ΔV| ≤ A*T - A²/J
///
/// @param distance Path distance in meters
/// @param duration Segment duration in seconds
/// @param exit_velocity Required exit velocity in m/s
/// @param max_velocity Maximum allowed velocity in m/s
/// @param max_acceleration Maximum allowed acceleration in m/s²
/// @param max_jerk Maximum allowed jerk in m/s³
/// @return VelocityComputeResult with entry velocity or error message
VelocityComputeResult computeEntryVelocityAnalytical(
    double distance,
    double duration,
    double exit_velocity,
    double max_velocity,
    double max_acceleration,
    double max_jerk) {

    VelocityComputeResult result;

    // Edge case: zero or very small distance
    if (distance < 0.001) {
        if (std::abs(exit_velocity) < 0.001) {
            result.success = true;
            result.entry_velocity = 0.0;
            result.actual_duration = duration;
            return result;
        }
        result.error_message = "Zero distance with non-zero exit velocity";
        return result;
    }

    // Edge case: zero duration
    if (duration <= 0.0) {
        result.error_message = "Zero or negative duration";
        return result;
    }

    // Core formula from distance/time constraint:
    // D = (V_entry + V_exit) * T / 2
    // V_entry = 2*D/T - V_exit
    double V_entry = 2.0 * distance / duration - exit_velocity;

    // Check for negative entry velocity (infeasible)
    if (V_entry < -0.001) {
        // This means exit_velocity > 2*D/T (can't cover the distance in time while exiting that fast)
        //
        // Compute actionable alternatives:
        // 1. Min duration for THIS segment (with V_entry=0): T_min = 2*D / V_exit
        // 2. Max achievable exit velocity (with V_entry=0): V_exit_max = 2*D / T
        //
        result.min_duration_this_seg = 2.0 * distance / exit_velocity;
        result.max_achievable_exit_vel = 2.0 * distance / duration;

        result.error_message = fmt::format(
            "Required entry velocity is negative ({:.3f} m/s). "
            "This segment (D={:.3f}m, T={:.2f}s) cannot exit at {:.3f} m/s.",
            V_entry, distance, duration, exit_velocity);
        return result;
    }

    // Clamp small negative values to zero
    if (V_entry < 0.0) {
        V_entry = 0.0;
    }

    // Check if entry velocity exceeds maximum
    if (V_entry > max_velocity) {
        // Compute minimum duration for V_entry = V_max
        // From: V_max = 2*D/T_min - V_exit
        // T_min = 2*D / (V_max + V_exit)
        result.min_duration_this_seg = 2.0 * distance / (max_velocity + exit_velocity);

        // Max achievable exit velocity with V_entry = V_max
        result.max_achievable_exit_vel = 2.0 * distance / duration - max_velocity;
        if (result.max_achievable_exit_vel < 0.0) {
            result.max_achievable_exit_vel = 0.0;
        }

        result.error_message = fmt::format(
            "Required entry velocity {:.3f} m/s exceeds limit {:.3f} m/s. "
            "This segment (D={:.3f}m, T={:.2f}s) needs a higher entry velocity than allowed.",
            V_entry, max_velocity, distance, duration);
        return result;
    }

    // Check if required velocity change is achievable with jerk-limited motion
    //
    // For jerk-limited deceleration/acceleration from V_entry to V_exit over time T:
    //
    // Triangular profile (T < 2*A/J): Peak accel = J*T/2, never reaches A_max
    //   Max achievable |ΔV| = J*T²/4
    //   If violated: T_min = 2*sqrt(|ΔV|/J)
    //
    // Full profile (T >= 2*A/J): Peak accel = A_max (the limit)
    //   Max achievable |ΔV| = A*T - A²/J
    //   If violated: T_min = |ΔV|/A + A/J
    //
    double delta_v = std::abs(exit_velocity - V_entry);
    double t_j = max_acceleration / max_jerk;  // Jerk phase duration

    if (duration < 2.0 * t_j) {
        // Triangular profile applies
        double max_delta_v = max_jerk * duration * duration / 4.0;

        if (delta_v > max_delta_v) {
            // Need more time - compute exact minimum duration for triangular
            double T_min = 2.0 * std::sqrt(delta_v / max_jerk);

            // Check if full profile would be faster
            double T_min_full = delta_v / max_acceleration + max_acceleration / max_jerk;
            if (T_min_full < T_min && T_min_full >= 2.0 * t_j) {
                T_min = T_min_full;
            }

            result.error_message = fmt::format(
                "Segment duration {:.2f}s too short for velocity change {:.3f} m/s. "
                "Jerk limit {:.1f} m/s³ allows max ΔV of {:.3f} m/s in this time. "
                "Minimum duration: {:.2f}s.",
                duration, delta_v, max_jerk, max_delta_v, T_min);
            return result;
        }
    } else {
        // Full profile applies (constant acceleration phase exists)
        double max_delta_v = max_acceleration * duration
                           - max_acceleration * max_acceleration / max_jerk;

        if (delta_v > max_delta_v) {
            // Need more time - compute exact minimum duration for full profile
            double T_min = delta_v / max_acceleration + max_acceleration / max_jerk;

            result.error_message = fmt::format(
                "Segment duration {:.2f}s too short for velocity change {:.3f} m/s. "
                "Acceleration limit {:.1f} m/s² allows max ΔV of {:.3f} m/s in this time. "
                "Minimum duration: {:.2f}s.",
                duration, delta_v, max_acceleration, max_delta_v, T_min);
            return result;
        }
    }

    result.success = true;
    result.entry_velocity = V_entry;
    result.actual_duration = duration;

    spdlog::debug("Analytical velocity: D={:.3f}m, T={:.2f}s, V_exit={:.3f}, V_entry={:.3f} m/s",
        distance, duration, exit_velocity, V_entry);

    return result;
}

}  // namespace

// =============================================================================
// TrajectoryPlanner
// =============================================================================

TrajectoryPlanner::TrajectoryPlanner(const kinematics::URKinematics& kinematics)
    : kinematics_(kinematics),
      config_{},
      validator_(kinematics) {}

TrajectoryPlanner::TrajectoryPlanner(const kinematics::URKinematics& kinematics,
                                     const TrajectoryConfig& config)
    : kinematics_(kinematics),
      config_(config),
      validator_(kinematics) {}

// =============================================================================
// IK Solving
// =============================================================================

std::optional<JointVector> TrajectoryPlanner::solveIK(
    const Eigen::Isometry3d& pose,
    const JointVector& seed) const {

    // Numerical IK using damped least squares, seeded from previous joints.
    // This guarantees continuous joint solutions (no branch flips).
    auto result = kinematics_.inverseNumerical(pose, seed);
    if (!result.has_value()) {
        spdlog::warn("solveIK: numerical IK did not converge");
        return std::nullopt;
    }

    // Verify with FK that the solution actually reaches the pose
    auto fk_check = kinematics_.forward(*result);

    // Check position error
    double pos_error = (fk_check.translation() - pose.translation()).norm();
    if (pos_error > 0.005) {  // 5mm tolerance
        spdlog::warn("solveIK: numerical IK position error {:.4f}m > 5mm, rejecting", pos_error);
        return std::nullopt;
    }

    // Check orientation error
    Eigen::Quaterniond target_quat(pose.rotation());
    Eigen::Quaterniond result_quat(fk_check.rotation());
    double dot = std::abs(target_quat.dot(result_quat));
    double orient_error = 2.0 * std::acos(std::min(dot, 1.0));  // Angle in radians
    if (orient_error > 0.01) {  // ~0.6 degree tolerance
        spdlog::warn("solveIK: numerical IK orientation error {:.2f}° > 0.6°, rejecting",
            orient_error * 180.0 / 3.14159);
        return std::nullopt;
    }

    return result;
}

// =============================================================================
// Planning
// =============================================================================

PlannedTrajectory TrajectoryPlanner::plan(const std::vector<Waypoint>& waypoints) {
    // Resolve start joints for first waypoint via numerical IK.
    if (waypoints.empty()) {
        PlannedTrajectory result;
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error, "No waypoints provided", std::nullopt, std::nullopt});
        return result;
    }

    // Prefer the saved joints on the first waypoint as the seed; otherwise use
    // the joint-limit midpoint as a neutral seed.
    JointVector seed;
    if (waypoints[0].joints.has_value()) {
        seed = *waypoints[0].joints;
    } else {
        const auto& limits = kinematics_.jointLimits();
        for (int i = 0; i < 6; ++i) {
            const auto& jl = limits.joints[static_cast<size_t>(i)];
            seed[i] = 0.5 * (jl.min + jl.max);
        }
    }

    auto pose = waypoints[0].toPose();
    auto start_joints_opt = kinematics_.inverseNumerical(pose, seed);
    if (!start_joints_opt.has_value()) {
        PlannedTrajectory result;
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error, "First waypoint is unreachable", 0, std::nullopt});
        return result;
    }

    JointVector start_joints = *start_joints_opt;

    auto start_config = kinematics_.getConfiguration(start_joints);
    spdlog::info("Starting trajectory planning with config: shoulder={} elbow={} wrist={}",
        start_config.shoulder, start_config.elbow, start_config.wrist);

    return plan(waypoints, start_joints);
}

PlannedTrajectory TrajectoryPlanner::plan(
    const std::vector<Waypoint>& waypoints,
    const JointVector& start_joints) {

    PlannedTrajectory result;
    result.sample_period = 1.0 / config_.sample_rate;

    // Validate waypoints
    auto validation = validator_.validate(waypoints, config_);
    result.messages = validation.all_messages;

    if (validation.hasErrors()) {
        result.valid = false;
        return result;
    }

    if (waypoints.size() < 2) {
        // Single waypoint - no motion needed
        result.valid = true;
        result.total_duration = 0.0;
        TrajectorySample sample = createSample(0.0, start_joints, JointVector::Zero());
        result.samples.push_back(sample);
        return result;
    }

    // Build segments
    std::vector<Segment> segments;
    JointVector current_joints = start_joints;
    double current_time = 0.0;

    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        Segment seg;
        seg.start_index = i;
        seg.end_index = i + 1;
        seg.start_joints = current_joints;
        seg.start_time = current_time;

        // Handle pause at current waypoint
        if (waypoints[i].pause_time > 0.0) {
            current_time += waypoints[i].pause_time;
        }
        seg.start_time = current_time;

        // Get IK solution for end waypoint via numerical IK seeded from current joints
        auto end_pose = waypoints[i + 1].toPose();
        auto best_sol = kinematics_.inverseNumerical(end_pose, current_joints);
        if (!best_sol.has_value()) {
            result.valid = false;
            result.messages.push_back({ValidationSeverity::Error,
                "Waypoint " + std::to_string(i + 2) + " is unreachable - numerical IK failed",
                i + 1, std::nullopt});
            return result;
        }

        seg.end_joints = *best_sol;

        // Warn if arm configuration changed (informational only)
        auto current_config = kinematics_.getConfiguration(current_joints);
        auto end_config = kinematics_.getConfiguration(seg.end_joints);
        if (current_config != end_config) {
            spdlog::warn("Segment {} crosses configuration boundary (elbow or wrist flip). "
                "Path may pass near singularity.", i);
            result.messages.push_back({ValidationSeverity::Warning,
                "Segment " + std::to_string(i + 1) + " to waypoint " + std::to_string(i + 2) +
                " crosses configuration boundary. Path may pass near singularity.",
                i + 1, std::nullopt});
        }

        // Determine segment duration (will be refined by Ruckig)
        if (waypoints[i + 1].segment_time > 0.0) {
            seg.duration = waypoints[i + 1].segment_time;
        } else {
            // Auto-compute duration based on joint velocity limits
            seg.duration = calculateSegmentDuration(seg.start_joints, seg.end_joints);
        }

        // Check for blending
        seg.has_blend_in = (i > 0 && waypoints[i].blend_factor > 0.0);
        seg.has_blend_out = (waypoints[i + 1].blend_factor > 0.0 && i + 2 < waypoints.size());

        spdlog::info("Segment {}: has_blend_out={}, wp[{}].blend_factor={:.4f}, i+2<size={}",
            i, seg.has_blend_out, i + 1, waypoints[i + 1].blend_factor, i + 2 < waypoints.size());

        // Compute blend arc geometry for the end waypoint if needed
        if (seg.has_blend_out) {
            seg.blend_arc = computeBlendGeometry(
                waypoints[i], waypoints[i + 1], waypoints[i + 2], current_joints);
            if (seg.blend_arc) {
                spdlog::info("  Blend arc computed for segment {}", i);
            } else {
                spdlog::warn("  Blend arc computation failed for segment {}", i);
            }
        }

        segments.push_back(seg);
        current_joints = seg.end_joints;
        current_time += seg.duration;
    }

    // Plan joint-space motion using Ruckig
    result.samples = planJointMotion(segments, waypoints, result.messages);

    if (result.samples.empty()) {
        result.valid = false;
        // Only add generic error if no specific errors were already added
        if (!result.hasErrors()) {
            result.messages.push_back({ValidationSeverity::Error,
                "Linear path not achievable - trajectory crosses a singularity or requires IK configuration change. "
                "Try moving waypoints to avoid shoulder/wrist singularities, or reduce blend radius.",
                std::nullopt, std::nullopt});
        }
        return result;
    }

    // Set total duration
    result.total_duration = result.samples.back().time;
    result.valid = !result.hasErrors();

    return result;
}

std::vector<TrajectorySample> TrajectoryPlanner::planJointMotion(
    const std::vector<Segment>& segments_in,
    const std::vector<Waypoint>& waypoints,
    std::vector<ValidationMessage>& messages) {

    std::vector<TrajectorySample> samples;

    if (segments_in.empty()) {
        return samples;
    }

    // =========================================================================
    // Phase 1: Build extended segments with distances and blend arcs
    // =========================================================================

    // Extended segment info for velocity planning
    struct ExtendedSegment {
        size_t start_index;
        size_t end_index;
        JointVector start_joints;
        double duration;
        double distance;           // Path distance in meters
        double entry_velocity;     // TCP velocity at segment start (m/s)
        double exit_velocity;      // TCP velocity at segment end (m/s)
        bool has_blend_out;        // Whether this segment ends with a blend
        bool coming_from_blend;    // Whether this segment starts from a blend midpoint
        std::optional<BlendArcInfo> blend_arc;
        std::optional<BlendArcInfo> prev_blend_arc;  // Arc from previous segment (for second half)
        Eigen::Isometry3d start_pose;
        Eigen::Isometry3d end_pose;
    };

    std::vector<ExtendedSegment> ext_segments;
    ext_segments.reserve(segments_in.size());

    JointVector current_joints = segments_in[0].start_joints;
    bool coming_from_blend = false;
    JointVector blend_end_joints = JointVector::Zero();
    std::optional<BlendArcInfo> prev_blend_arc;

    for (size_t seg_idx = 0; seg_idx < segments_in.size(); ++seg_idx) {
        const auto& seg = segments_in[seg_idx];

        ExtendedSegment ext;
        ext.start_index = seg.start_index;
        ext.end_index = seg.end_index;
        ext.start_joints = coming_from_blend ? blend_end_joints : current_joints;
        ext.duration = seg.duration;
        ext.coming_from_blend = coming_from_blend;
        ext.prev_blend_arc = prev_blend_arc;

        // Check if this segment needs a blend arc at the end
        bool needs_blend = (waypoints[seg.end_index].blend_factor > 0.0) &&
                           (seg_idx + 1 < segments_in.size()) &&
                           (waypoints[seg.end_index].pause_time <= 0.0);

        ext.has_blend_out = needs_blend;

        spdlog::info("Phase1 Segment {}: WP{}->WP{}, coming_from_blend={}, prev_arc={}, needs_blend={}, blend_factor={:.3f}",
            seg_idx, seg.start_index, seg.end_index,
            coming_from_blend, prev_blend_arc.has_value(), needs_blend,
            waypoints[seg.end_index].blend_factor);
        ext.start_pose = kinematics_.forward(ext.start_joints);
        ext.end_pose = waypoints[seg.end_index].toPose();

        // Compute blend geometry if needed
        if (needs_blend) {
            ext.blend_arc = computeBlendGeometry(
                waypoints[seg.start_index],
                waypoints[seg.end_index],
                waypoints[seg.end_index + 1],
                ext.start_joints);
        }

        // Compute path distance
        spdlog::info("Phase1 Segment {} condition check: coming_from_blend={}, prev_arc={}, blend_arc={}",
            seg_idx, coming_from_blend, prev_blend_arc.has_value(), ext.blend_arc.has_value());

        if (coming_from_blend && prev_blend_arc.has_value() && ext.blend_arc.has_value()) {
            // BOTH: starts from blend AND ends with blend
            // Path: second half of prev arc + linear + first half of new arc
            const auto& prev_arc = *prev_blend_arc;
            const auto& next_arc = *ext.blend_arc;

            double prev_half_arc = prev_arc.arc_length / 2.0;
            double linear_dist = (next_arc.arc_start - prev_arc.arc_end).norm();
            double next_half_arc = next_arc.arc_length / 2.0;
            ext.distance = prev_half_arc + linear_dist + next_half_arc;

            spdlog::info("Phase1 Segment {} -> BOTH ARCS: prev_arc={:.3f}, linear={:.3f}, next_arc={:.3f}",
                seg_idx, prev_half_arc, linear_dist, next_half_arc);

            if (ext.duration <= 0.0) {
                ext.duration = ext.distance / config_.max_linear_velocity;
                ext.duration = std::max(ext.duration, 0.1);
            }

            // Set up for next segment to start from arc midpoint of the NEW arc
            double half_angle = next_arc.arc_angle / 2.0;
            Eigen::Vector3d v_start = next_arc.arc_start - next_arc.center;
            Eigen::Vector3d v_mid =
                v_start * std::cos(half_angle) +
                next_arc.normal.cross(v_start) * std::sin(half_angle) +
                next_arc.normal * next_arc.normal.dot(v_start) * (1 - std::cos(half_angle));

            Eigen::Isometry3d mid_pose = Eigen::Isometry3d::Identity();
            mid_pose.translation() = next_arc.center + v_mid;
            Eigen::Quaterniond mid_quat = next_arc.start_orientation.slerp(0.5, next_arc.end_orientation);
            mid_pose.linear() = mid_quat.toRotationMatrix();

            auto mid_ik = solveIK(mid_pose, next_arc.start_joints);
            if (mid_ik.has_value()) {
                blend_end_joints = *mid_ik;
            } else {
                blend_end_joints = next_arc.start_joints;
            }

            // Keep coming_from_blend true, update prev_blend_arc to the new arc
            coming_from_blend = true;
            prev_blend_arc = ext.blend_arc;

        } else if (ext.blend_arc.has_value()) {
            // First half segment: linear to arc_start + half arc
            const auto& arc = *ext.blend_arc;
            double linear_dist = (arc.arc_start - ext.start_pose.translation()).norm();
            double half_arc = arc.arc_length / 2.0;
            ext.distance = linear_dist + half_arc;

            spdlog::info("Phase1 Segment {} -> FIRST HALF ARC: linear={:.3f}, half_arc={:.3f}",
                seg_idx, linear_dist, half_arc);

            // Auto-calculate duration if not specified
            if (ext.duration <= 0.0) {
                ext.duration = ext.distance / config_.max_linear_velocity;
                ext.duration = std::max(ext.duration, 0.1);
            }

            // Set up for next segment to start from arc midpoint
            // Compute midpoint position
            double half_angle = arc.arc_angle / 2.0;
            Eigen::Vector3d v_start = arc.arc_start - arc.center;
            Eigen::Vector3d v_mid =
                v_start * std::cos(half_angle) +
                arc.normal.cross(v_start) * std::sin(half_angle) +
                arc.normal * arc.normal.dot(v_start) * (1 - std::cos(half_angle));

            // Solve IK for arc midpoint to get blend_end_joints
            Eigen::Isometry3d mid_pose = Eigen::Isometry3d::Identity();
            mid_pose.translation() = arc.center + v_mid;
            Eigen::Quaterniond mid_quat = arc.start_orientation.slerp(0.5, arc.end_orientation);
            mid_pose.linear() = mid_quat.toRotationMatrix();

            auto mid_ik = solveIK(mid_pose, arc.start_joints);
            if (mid_ik.has_value()) {
                blend_end_joints = *mid_ik;
            } else {
                blend_end_joints = arc.start_joints;  // Fallback
            }

            coming_from_blend = true;
            prev_blend_arc = ext.blend_arc;
        } else if (coming_from_blend && prev_blend_arc.has_value()) {
            // Second half segment: half arc + linear to endpoint
            const auto& arc = *prev_blend_arc;
            double half_arc = arc.arc_length / 2.0;
            double linear_dist = (ext.end_pose.translation() - arc.arc_end).norm();
            ext.distance = half_arc + linear_dist;

            spdlog::info("Phase1 Segment {} -> SECOND HALF ARC: half_arc={:.3f}, linear={:.3f}",
                seg_idx, half_arc, linear_dist);

            if (ext.duration <= 0.0) {
                ext.duration = ext.distance / config_.max_linear_velocity;
                ext.duration = std::max(ext.duration, 0.1);
            }

            coming_from_blend = false;
            prev_blend_arc = std::nullopt;
            current_joints = seg.end_joints;  // Use pre-computed end joints
        } else {
            // Normal linear segment
            ext.distance = (ext.end_pose.translation() - ext.start_pose.translation()).norm();

            spdlog::info("Phase1 Segment {} -> NORMAL LINEAR: distance={:.3f}",
                seg_idx, ext.distance);

            if (ext.duration <= 0.0) {
                ext.duration = ext.distance / config_.max_linear_velocity;
                ext.duration = std::max(ext.duration, 0.1);
            }

            coming_from_blend = false;
            current_joints = seg.end_joints;
        }

        ext_segments.push_back(ext);
    }

    // =========================================================================
    // Phase 2: Backward velocity propagation using Ruckig
    // =========================================================================
    // Work backwards to compute required entry velocities using jerk-limited motion planning.
    // For each segment, Ruckig determines what entry velocity is needed to:
    // - Cover the segment distance
    // - In the specified duration
    // - Ending at the required exit velocity
    // - While respecting jerk, acceleration, and velocity limits
    //
    // If constraints are impossible, we report this to the operator.

    // Last segment always ends at velocity 0 (stopping at final waypoint)
    ext_segments.back().exit_velocity = 0.0;

    // Backward pass: starting from last segment, compute entry velocities
    // Key insight: first determine if coming from a blend, then compute velocity
    for (int i = static_cast<int>(ext_segments.size()) - 1; i >= 0; --i) {
        auto& seg = ext_segments[static_cast<size_t>(i)];

        // Step 1: Determine if this segment starts from rest
        // - Segment 0 always starts from rest
        // - Other segments start from rest if previous segment has no blend_out
        bool starts_from_rest = (i == 0);
        if (i > 0) {
            const auto& prev_seg = ext_segments[static_cast<size_t>(i - 1)];
            starts_from_rest = !prev_seg.has_blend_out;
        }

        if (starts_from_rest) {
            // This segment starts from rest - entry velocity is 0
            seg.entry_velocity = 0.0;

            // Also set previous segment's exit velocity to 0 (if exists)
            if (i > 0) {
                ext_segments[static_cast<size_t>(i - 1)].exit_velocity = 0.0;
            }

            spdlog::info("Segment {}: starts from rest (no blend from previous)", i);

            // If also ending at rest, verify symmetric profile is feasible
            if (seg.exit_velocity < 0.001) {
                // Check if symmetric profile can cover the distance in the given time
                double t_j = config_.max_linear_acceleration / config_.max_linear_jerk;
                double T_half = seg.duration / 2.0;
                double D = seg.distance;
                double A = config_.max_linear_acceleration;
                double J = config_.max_linear_jerk;

                // Max distance achievable with symmetric profile
                double D_max;
                if (T_half > 2.0 * t_j) {
                    // Trapezoidal
                    double t2_max = T_half - 2.0 * t_j;
                    double v_peak = config_.max_linear_acceleration * (t_j + t2_max);
                    D_max = v_peak * T_half;
                } else {
                    // Triangular
                    D_max = J * T_half * T_half * T_half / 4.0;
                }

                if (D_max >= seg.distance) {
                    // Symmetric profile is feasible
                    spdlog::info("Segment {}: symmetric profile feasible, D={:.3f}m, T={:.2f}s, D_max={:.3f}m",
                        i, seg.distance, seg.duration, D_max);
                } else {
                    // Cannot achieve required distance - report to operator
                    // Calculate minimum duration needed
                    double T_half_tri = std::cbrt(4.0 * D / J);
                    double T_min;

                    if (T_half_tri <= 2.0 * t_j) {
                        T_min = 2.0 * T_half_tri;
                    } else {
                        double T_half_trap = (t_j + std::sqrt(t_j * t_j + 4.0 * D / A)) / 2.0;
                        T_min = 2.0 * T_half_trap;
                    }

                    spdlog::error("Segment {}: cannot cover {:.3f}m in {:.2f}s (max achievable: {:.3f}m)",
                        i, seg.distance, seg.duration, D_max);

                    messages.push_back({ValidationSeverity::Error,
                        fmt::format("Segment {}: cannot cover {:.3f}m in {:.2f}s with rest-to-rest profile",
                            i, seg.distance, seg.duration),
                        static_cast<size_t>(i + 1), std::nullopt});

                    auto suggestion = fmt::format("Increase segment {} duration to at least {:.2f}s (currently {:.2f}s)",
                        i, T_min, seg.duration);
                    spdlog::info("{}", suggestion);
                    messages.push_back({ValidationSeverity::Info, suggestion,
                        static_cast<size_t>(i + 1), std::nullopt});

                    auto limits_msg = fmt::format("Current limits: accel={:.2f} m/s², jerk={:.1f} m/s³",
                        A, J);
                    spdlog::info("{}", limits_msg);
                    messages.push_back({ValidationSeverity::Info, limits_msg,
                        std::nullopt, std::nullopt});

                    return std::vector<TrajectorySample>{};
                }
            }

        } else {
            // Coming from blend - compute entry velocity based on exit velocity
            auto velocity_result = computeEntryVelocityAnalytical(
                seg.distance,
                seg.duration,
                seg.exit_velocity,
                config_.max_linear_velocity,
                config_.max_linear_acceleration,
                config_.max_linear_jerk);

            if (!velocity_result.success) {
                // Constraints are impossible - report to operator
                spdlog::error("Segment {} velocity planning failed: {}", i, velocity_result.error_message);

                messages.push_back({ValidationSeverity::Error,
                    fmt::format("Segment {}: {}",
                        i, velocity_result.error_message),
                    static_cast<size_t>(i + 1), std::nullopt});

                // Show current limits for reference
                auto limits_msg = fmt::format("Current limits: velocity={:.3f} m/s, accel={:.2f} m/s², jerk={:.1f} m/s³",
                    config_.max_linear_velocity, config_.max_linear_acceleration, config_.max_linear_jerk);
                spdlog::info("{}", limits_msg);
                messages.push_back({ValidationSeverity::Info, limits_msg,
                    std::nullopt, std::nullopt});

                // FAIL FAST - don't continue with invalid values
                return std::vector<TrajectorySample>{};
            }

            seg.entry_velocity = velocity_result.entry_velocity;

            // Propagate to previous segment (which must have blend_out = true)
            ext_segments[static_cast<size_t>(i - 1)].exit_velocity = seg.entry_velocity;

            spdlog::info("Segment {}: coming from blend, v_entry={:.3f}, v_exit={:.3f}",
                i, seg.entry_velocity, seg.exit_velocity);
        }
    }

    // =========================================================================
    // Phase 3: Forward sampling pass
    // =========================================================================

    // Reserve approximate number of samples
    double total_time = 0.0;
    for (const auto& seg : ext_segments) {
        total_time += seg.duration;
    }
    for (const auto& wp : waypoints) {
        total_time += wp.pause_time;
    }
    size_t estimated_samples = static_cast<size_t>(total_time * config_.sample_rate) + 100;
    samples.reserve(estimated_samples);

    double current_time = 0.0;
    JointVector current_pos = ext_segments[0].start_joints;

    for (size_t seg_idx = 0; seg_idx < ext_segments.size(); ++seg_idx) {
        const auto& seg = ext_segments[seg_idx];

        // Handle pause at start of segment (except first)
        if (seg_idx > 0 && waypoints[seg.start_index].pause_time > 0.0 && !seg.coming_from_blend) {
            double pause_duration = waypoints[seg.start_index].pause_time;
            double pause_end = current_time + pause_duration;

            while (current_time < pause_end) {
                samples.push_back(createSample(current_time, current_pos, JointVector::Zero()));
                current_time += 1.0 / config_.sample_rate;
            }
        }

        // Use current_pos (actual ending joints from previous segment) for continuity
        // Only use seg.start_joints for the first segment
        JointVector actual_start = (seg_idx > 0) ? current_pos : seg.start_joints;

        spdlog::info("Phase3 Segment {} condition check: coming_from_blend={}, prev_arc={}, blend_arc={}",
            seg_idx, seg.coming_from_blend, seg.prev_blend_arc.has_value(), seg.blend_arc.has_value());
        spdlog::info("Phase3 Segment {} J6: seg.start={:.1f}deg, actual_start={:.1f}deg",
            seg_idx, seg.start_joints[5] * 180.0 / M_PI, actual_start[5] * 180.0 / M_PI);

        if (seg.coming_from_blend && seg.prev_blend_arc.has_value() && seg.blend_arc.has_value()) {
            // BOTH: starts from blend AND ends with blend
            // Path: second half of prev arc + linear + first half of new arc
            const auto& prev_arc = *seg.prev_blend_arc;
            const auto& next_arc = *seg.blend_arc;

            spdlog::info("Phase3 Segment {} -> BOTH ARCS: duration={:.2f}, v_entry={:.3f}, v_exit={:.3f}",
                seg_idx, seg.duration, seg.entry_velocity, seg.exit_velocity);

            // Get previous segment's last joints for velocity continuity
            std::optional<JointVector> prev_seg_joints = samples.empty()
                ? std::nullopt
                : std::optional<JointVector>(samples.back().joints);

            auto segment_samples = planSegmentWithBothArcs(
                seg.start_pose, actual_start,
                prev_arc, next_arc,
                current_time, seg.duration,
                seg.entry_velocity, seg.exit_velocity,
                prev_seg_joints);

            if (segment_samples.empty()) {
                spdlog::error("Failed to plan segment {} (both blend arcs)", seg_idx);
                return {};
            }

            for (const auto& sample : segment_samples) {
                samples.push_back(sample);
            }

            current_time = segment_samples.back().time + 1.0 / config_.sample_rate;
            current_pos = segment_samples.back().joints;

        } else if (seg.blend_arc.has_value()) {
            // Plan first half of arc segment
            const auto& arc = *seg.blend_arc;

            spdlog::info("Sampling blend segment {}: duration={:.2f}, v_entry={:.3f}, v_exit={:.3f}",
                seg_idx, seg.duration, seg.entry_velocity, seg.exit_velocity);

            // Get previous segment's last joints for velocity continuity
            std::optional<JointVector> prev_seg_joints = samples.empty()
                ? std::nullopt
                : std::optional<JointVector>(samples.back().joints);

            auto segment_samples = planSegmentWithHalfArc(
                seg.start_pose, actual_start, arc,
                current_time, seg.duration,
                true,  // first_half = true
                seg.entry_velocity,
                seg.exit_velocity,
                Eigen::Isometry3d::Identity(),  // end_pose not used for first_half
                prev_seg_joints);

            if (segment_samples.empty()) {
                spdlog::error("Failed to plan segment {} (blend arc entry)", seg_idx);
                return {};
            }

            for (const auto& sample : segment_samples) {
                samples.push_back(sample);
            }

            current_time = segment_samples.back().time + 1.0 / config_.sample_rate;
            current_pos = segment_samples.back().joints;

        } else if (seg.coming_from_blend && seg.prev_blend_arc.has_value()) {
            // Plan second half of arc + linear
            const auto& arc = *seg.prev_blend_arc;

            spdlog::info("Sampling post-blend segment {}: duration={:.2f}, v_entry={:.3f}, v_exit={:.3f}",
                seg_idx, seg.duration, seg.entry_velocity, seg.exit_velocity);

            // Get previous segment's last joints for velocity continuity
            std::optional<JointVector> prev_seg_joints = samples.empty()
                ? std::nullopt
                : std::optional<JointVector>(samples.back().joints);

            auto segment_samples = planSegmentWithHalfArc(
                seg.start_pose, actual_start, arc,
                current_time, seg.duration,
                false,  // first_half = false (second half)
                seg.entry_velocity,
                seg.exit_velocity,
                seg.end_pose,
                prev_seg_joints);

            if (segment_samples.empty()) {
                spdlog::error("Failed to plan segment {} (blend arc exit)", seg_idx);
                return {};
            }

            for (const auto& sample : segment_samples) {
                samples.push_back(sample);
            }

            current_time = segment_samples.back().time + 1.0 / config_.sample_rate;
            current_pos = segment_samples.back().joints;

        } else {
            // Normal linear segment
            spdlog::info("Sampling linear segment {}: duration={:.2f}, v_entry={:.3f}, v_exit={:.3f}",
                seg_idx, seg.duration, seg.entry_velocity, seg.exit_velocity);

            // Get previous segment's last joints for velocity continuity
            std::optional<JointVector> prev_seg_joints = samples.empty()
                ? std::nullopt
                : std::optional<JointVector>(samples.back().joints);

            auto segment_samples = planSegmentLinearCartesian(
                seg.start_pose, seg.end_pose,
                actual_start,
                current_time, seg.duration,
                seg.entry_velocity,
                seg.exit_velocity,
                prev_seg_joints);

            if (segment_samples.empty()) {
                spdlog::error("Failed to plan segment {} - linear path not achievable", seg_idx);
                return {};
            }

            for (const auto& sample : segment_samples) {
                samples.push_back(sample);
            }

            current_time = segment_samples.back().time + 1.0 / config_.sample_rate;
            current_pos = segment_samples.back().joints;
        }
    }

    // Handle final waypoint pause
    if (!waypoints.empty() && waypoints.back().pause_time > 0.0) {
        double pause_end = current_time + waypoints.back().pause_time;
        while (current_time < pause_end) {
            samples.push_back(createSample(current_time, current_pos, JointVector::Zero()));
            current_time += 1.0 / config_.sample_rate;
        }
    }

    // Ensure final sample
    if (samples.empty() || samples.back().time < current_time - 1.0 / config_.sample_rate) {
        samples.push_back(createSample(current_time, current_pos, JointVector::Zero()));
    }

    return samples;
}

std::vector<TrajectorySample> TrajectoryPlanner::planSegmentWithRuckig(
    const JointVector& start_joints,
    const JointVector& end_joints,
    const JointVector& start_velocity,
    const JointVector& start_acceleration,
    double start_time,
    double desired_duration,
    const JointVector& target_velocity) {

    std::vector<TrajectorySample> samples;

    // Create Ruckig instance for 6 DOF
    ruckig::Ruckig<6> ruckig;
    ruckig::InputParameter<6> input;
    ruckig::Trajectory<6> trajectory;

    // If user specified a duration, set it as minimum duration
    if (desired_duration > 0.0) {
        input.minimum_duration = desired_duration;
    }

    // Set current state
    for (size_t i = 0; i < 6; ++i) {
        auto idx = static_cast<Eigen::Index>(i);
        input.current_position[i] = start_joints[idx];
        input.current_velocity[i] = start_velocity[idx];
        input.current_acceleration[i] = start_acceleration[idx];
    }

    // Set target state
    JointVector diff = jointDifference(end_joints, start_joints);
    for (size_t i = 0; i < 6; ++i) {
        auto idx = static_cast<Eigen::Index>(i);
        // Target position considering angle wrapping
        input.target_position[i] = start_joints[idx] + diff[idx];
        // Target velocity for blending (0 = stop, non-zero = blend through)
        input.target_velocity[i] = target_velocity[idx];
        input.target_acceleration[i] = 0.0;
    }

    // Set kinematic limits
    for (size_t i = 0; i < 6; ++i) {
        input.max_velocity[i] = config_.max_joint_velocity;
        input.max_acceleration[i] = config_.max_joint_acceleration;
        input.max_jerk[i] = config_.max_joint_jerk;
    }

    // Calculate trajectory
    ruckig::Result result = ruckig.calculate(input, trajectory);

    if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
        // Ruckig calculation failed, return empty to trigger fallback
        return samples;
    }

    // Sample the trajectory at our sample rate
    double duration = trajectory.get_duration();
    double sample_period = 1.0 / config_.sample_rate;

    std::array<double, 6> pos, vel, acc;

    for (double t = 0.0; t <= duration; t += sample_period) {
        trajectory.at_time(t, pos, vel, acc);

        JointVector joints, velocities;
        for (size_t i = 0; i < 6; ++i) {
            auto idx = static_cast<Eigen::Index>(i);
            joints[idx] = pos[i];
            velocities[idx] = vel[i];
        }

        samples.push_back(createSample(start_time + t, joints, velocities));
    }

    // Ensure we end at the target position
    if (samples.empty() || (duration - static_cast<double>(samples.size() - 1) * sample_period) > sample_period * 0.5) {
        trajectory.at_time(duration, pos, vel, acc);
        JointVector joints, velocities;
        for (size_t i = 0; i < 6; ++i) {
            auto idx = static_cast<Eigen::Index>(i);
            joints[idx] = pos[i];
            velocities[idx] = vel[i];
        }
        samples.push_back(createSample(start_time + duration, joints, velocities));
    }

    return samples;
}

std::vector<TrajectorySample> TrajectoryPlanner::planSegmentSmooth(
    const Segment& seg,
    double start_time) {

    std::vector<TrajectorySample> samples;
    double seg_duration = seg.duration;
    JointVector diff = jointDifference(seg.end_joints, seg.start_joints);
    double sample_period = 1.0 / config_.sample_rate;

    for (double t = 0.0; t < seg_duration; t += sample_period) {
        double normalized_t = t / seg_duration;
        double s = smoothStep(normalized_t);

        JointVector pos;
        JointVector vel;
        for (size_t i = 0; i < 6; ++i) {
            auto idx = static_cast<Eigen::Index>(i);
            pos[idx] = seg.start_joints[idx] + s * diff[idx];
            // Velocity from derivative of smooth step: 6*t*(1-t) / duration
            double ds_dt = 6.0 * normalized_t * (1.0 - normalized_t) / seg_duration;
            vel[idx] = diff[idx] * ds_dt;
        }

        samples.push_back(createSample(start_time + t, pos, vel));
    }

    return samples;
}

std::vector<TrajectorySample> TrajectoryPlanner::planSegmentLinearCartesian(
    const Eigen::Isometry3d& start_pose,
    const Eigen::Isometry3d& end_pose,
    const JointVector& start_joints,
    double start_time,
    double duration,
    double entry_velocity,
    double exit_velocity,
    std::optional<JointVector> prev_seg_joints) {

    std::vector<TrajectorySample> samples;

    if (duration <= 0.0) {
        duration = 1.0;  // Default to 1 second if not specified
    }

    // Extract positions
    Eigen::Vector3d start_pos = start_pose.translation();
    Eigen::Vector3d end_pos = end_pose.translation();
    Eigen::Vector3d pos_diff = end_pos - start_pos;
    double distance = pos_diff.norm();

    // Log the segment planning
    spdlog::info("planSegmentLinearCartesian: distance={:.4f}m, duration={:.2f}s, v_entry={:.3f}, v_exit={:.3f}",
        distance, duration, entry_velocity, exit_velocity);

    // Extract orientations as quaternions
    Eigen::Quaterniond start_quat(start_pose.rotation());
    Eigen::Quaterniond end_quat(end_pose.rotation());

    // Ensure shortest path for SLERP
    if (start_quat.dot(end_quat) < 0.0) {
        end_quat.coeffs() = -end_quat.coeffs();
    }

    // Create S-curve profile for path parameter
    // Path parameter goes from 0 to distance, with entry/exit velocities as constraints
    SCurveProfile scurve_profile(
        entry_velocity,
        exit_velocity,
        distance,
        duration,
        config_.max_linear_jerk
    );

    if (!scurve_profile.isFeasible()) {
        spdlog::error("S-curve profile infeasible for linear segment (jerk limit exceeded)");
        return samples;  // Return empty - caller will handle failure
    }

    // Generate samples by sampling the S-curve profile
    double sample_period = 1.0 / config_.sample_rate;
    int num_samples = static_cast<int>(duration * config_.sample_rate) + 1;
    if (num_samples < 2) num_samples = 2;

    // Track previous joints - angles accumulate naturally
    JointVector prev_joints = start_joints;

    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1) * duration;
        double sample_time = start_time + t;

        // Get path position from S-curve profile
        double path_position = scurve_profile.position(t);

        // Convert path position to normalized parameter s in [0, 1]
        double s = (distance > 0.001) ? (path_position / distance) : 0.0;
        s = std::clamp(s, 0.0, 1.0);

        // Interpolate position linearly (with S-curve timing)
        Eigen::Vector3d pos = start_pos + s * pos_diff;

        // Interpolate orientation with SLERP (with S-curve timing)
        Eigen::Quaterniond quat = start_quat.slerp(s, end_quat);

        // Create pose
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = pos;
        pose.linear() = quat.toRotationMatrix();

        // Numerical IK seeded from previous sample's joints — guarantees continuity
        JointVector joints;
        auto ik_solution = kinematics_.inverseNumerical(pose, prev_joints);
        if (!ik_solution.has_value()) {
            spdlog::error("Linear path unreachable at s={:.3f}, pos=[{:.3f}, {:.3f}, {:.3f}] - "
                "numerical IK failed",
                s, pos.x(), pos.y(), pos.z());
            return {};
        }
        joints = *ik_solution;

        // Unwrap IK solution to match previous joints (angles accumulate, no wrapping)
        for (int j = 0; j < 6; ++j) {
            double diff = joints[j] - prev_joints[j];
            while (diff > M_PI) {
                joints[j] -= 2.0 * M_PI;
                diff -= 2.0 * M_PI;
            }
            while (diff < -M_PI) {
                joints[j] += 2.0 * M_PI;
                diff += 2.0 * M_PI;
            }
        }

        // Log J6 at first and last sample
        if (i == 0 || i == num_samples - 1) {
            spdlog::info("  Sample {} (s={:.2f}): J6={:.1f}°",
                i, s, joints[5] * 180.0 / M_PI);
        }

        // Check for singularity (large joint velocity indicates singularity proximity)
        if (i > 0 && !samples.empty()) {
            JointVector delta = joints - prev_joints;
            double max_joint_delta = 0.0;
            for (int j = 0; j < 6; ++j) {
                max_joint_delta = std::max(max_joint_delta, std::abs(delta[j]));
            }

            // If joint delta is very large, we're near a singularity - log warning but continue
            double joint_velocity = max_joint_delta / sample_period;
            if (joint_velocity > config_.max_joint_velocity * 2.0) {
                spdlog::warn("Near singularity at s={:.3f}, pos=[{:.3f}, {:.3f}, {:.3f}] - "
                    "joint velocity {:.1f} rad/s exceeds limit. Consider slower motion.",
                    s, pos.x(), pos.y(), pos.z(), joint_velocity);
            }
        }

        // Compute joint velocities (finite difference)
        double sample_period = 1.0 / config_.sample_rate;
        JointVector velocities = JointVector::Zero();
        if (i > 0 && !samples.empty()) {
            for (size_t j = 0; j < 6; ++j) {
                auto idx = static_cast<Eigen::Index>(j);
                velocities[idx] = (joints[idx] - samples.back().joints[idx]) / sample_period;
            }
        } else if (i == 0 && prev_seg_joints.has_value()) {
            // Use previous segment's last joints for velocity continuity
            for (size_t j = 0; j < 6; ++j) {
                auto idx = static_cast<Eigen::Index>(j);
                velocities[idx] = (joints[idx] - (*prev_seg_joints)[idx]) / sample_period;
            }
        }

        samples.push_back(createSample(sample_time, joints, velocities));
        prev_joints = joints;  // Track for continuity checking
    }

    return samples;
}

std::vector<TrajectorySample> TrajectoryPlanner::planSegmentWithHalfArc(
    const Eigen::Isometry3d& start_pose,
    const JointVector& start_joints,
    const BlendArcInfo& arc_info,
    double start_time,
    double duration,
    bool first_half,
    double entry_velocity,
    double exit_velocity,
    const Eigen::Isometry3d& end_pose,
    std::optional<JointVector> prev_seg_joints) {

    std::vector<TrajectorySample> samples;

    if (duration <= 0.0) {
        duration = 1.0;
    }

    // Calculate distances
    double half_arc_length = arc_info.arc_length / 2.0;
    double linear_distance;
    Eigen::Vector3d linear_start, linear_end;
    Eigen::Quaterniond linear_start_quat, linear_end_quat;

    // Determine end pose for this segment and compute end joints
    Eigen::Isometry3d segment_end_pose;
    if (first_half) {
        // Linear portion: start_pose -> arc_start, then first half of arc to midpoint
        linear_start = start_pose.translation();
        linear_end = arc_info.arc_start;
        linear_start_quat = Eigen::Quaterniond(start_pose.rotation());
        linear_end_quat = arc_info.start_orientation;
        linear_distance = (linear_end - linear_start).norm();

        // End pose is at arc midpoint
        double half_angle = arc_info.arc_angle / 2.0;
        Eigen::Vector3d v_start = arc_info.arc_start - arc_info.center;
        Eigen::Vector3d v_mid =
            v_start * std::cos(half_angle) +
            arc_info.normal.cross(v_start) * std::sin(half_angle) +
            arc_info.normal * arc_info.normal.dot(v_start) * (1 - std::cos(half_angle));
        segment_end_pose = Eigen::Isometry3d::Identity();
        segment_end_pose.translation() = arc_info.center + v_mid;
        Eigen::Quaterniond mid_quat = arc_info.start_orientation.slerp(0.5, arc_info.end_orientation);
        segment_end_pose.linear() = mid_quat.toRotationMatrix();
    } else {
        // Second half of arc (from midpoint), then linear: arc_end -> end_pose
        linear_start = arc_info.arc_end;
        linear_end = end_pose.translation();
        linear_start_quat = arc_info.end_orientation;
        linear_end_quat = Eigen::Quaterniond(end_pose.rotation());
        linear_distance = (linear_end - linear_start).norm();
        segment_end_pose = end_pose;
    }

    double total_distance = linear_distance + half_arc_length;

    spdlog::info("planSegmentWithHalfArc: first_half={}, dist={:.4f}m, v_entry={:.3f}, v_exit={:.3f}",
        first_half, total_distance, entry_velocity, exit_velocity);

    // Ensure shortest path for SLERP
    if (linear_start_quat.dot(linear_end_quat) < 0.0) {
        linear_end_quat.coeffs() = -linear_end_quat.coeffs();
    }

    // Arc geometry
    Eigen::Vector3d v_start = arc_info.arc_start - arc_info.center;
    double half_arc_angle = arc_info.arc_angle / 2.0;

    // Create S-curve profile for path parameter
    SCurveProfile scurve_profile(
        entry_velocity,
        exit_velocity,
        total_distance,
        duration,
        config_.max_linear_jerk
    );

    if (!scurve_profile.isFeasible()) {
        spdlog::error("S-curve profile infeasible for half-arc segment");
        return samples;
    }

    // Calculate number of samples
    int num_samples = static_cast<int>(duration * config_.sample_rate) + 1;
    if (num_samples < 2) num_samples = 2;

    // Track previous joints - angles accumulate naturally
    JointVector prev_joints = start_joints;

    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1) * duration;
        double sample_time = start_time + t;

        // Get path position from S-curve profile
        double path_position = scurve_profile.position(t);

        // Distance along the combined path
        double dist_along_path = std::clamp(path_position, 0.0, total_distance);

        Eigen::Vector3d pos;
        Eigen::Quaterniond quat;

        if (first_half) {
            // First half: linear then arc
            if (dist_along_path <= linear_distance && linear_distance > 0.001) {
                // On linear portion
                double linear_t = dist_along_path / linear_distance;
                pos = linear_start + linear_t * (linear_end - linear_start);
                quat = linear_start_quat.slerp(linear_t, linear_end_quat);
            } else {
                // On first half of arc
                double arc_dist = dist_along_path - linear_distance;
                double arc_t = (half_arc_length > 0.001) ? (arc_dist / half_arc_length) : 0.0;
                arc_t = std::clamp(arc_t, 0.0, 1.0);

                // Rotate v_start around normal by (arc_t * half_arc_angle)
                double angle = arc_t * half_arc_angle;
                Eigen::Vector3d v_rotated =
                    v_start * std::cos(angle) +
                    arc_info.normal.cross(v_start) * std::sin(angle) +
                    arc_info.normal * arc_info.normal.dot(v_start) * (1 - std::cos(angle));
                pos = arc_info.center + v_rotated;

                // Interpolate orientation through first half
                quat = arc_info.start_orientation.slerp(arc_t * 0.5, arc_info.end_orientation);
            }
        } else {
            // Second half: arc then linear
            if (dist_along_path <= half_arc_length && half_arc_length > 0.001) {
                // On second half of arc (from midpoint to end)
                double arc_t = dist_along_path / half_arc_length;

                // Start from arc midpoint (half_arc_angle into the arc)
                double angle = half_arc_angle + arc_t * half_arc_angle;
                Eigen::Vector3d v_rotated =
                    v_start * std::cos(angle) +
                    arc_info.normal.cross(v_start) * std::sin(angle) +
                    arc_info.normal * arc_info.normal.dot(v_start) * (1 - std::cos(angle));
                pos = arc_info.center + v_rotated;

                // Interpolate orientation through second half (0.5 to 1.0)
                quat = arc_info.start_orientation.slerp(0.5 + arc_t * 0.5, arc_info.end_orientation);
            } else {
                // On linear portion after arc
                double linear_dist = dist_along_path - half_arc_length;
                double linear_t = (linear_distance > 0.001) ? (linear_dist / linear_distance) : 1.0;
                linear_t = std::clamp(linear_t, 0.0, 1.0);
                pos = linear_start + linear_t * (linear_end - linear_start);
                quat = linear_start_quat.slerp(linear_t, linear_end_quat);
            }
        }

        // Solve IK using configured method
        Eigen::Isometry3d sample_pose = Eigen::Isometry3d::Identity();
        sample_pose.translation() = pos;
        sample_pose.linear() = quat.toRotationMatrix();

        // Use prev_joints as seed for continuous motion
        auto ik_solution = solveIK(sample_pose, prev_joints);
        if (!ik_solution.has_value()) {
            spdlog::error("Blend path unreachable at t={:.3f}s, pos=[{:.3f}, {:.3f}, {:.3f}] - "
                "no IK solution found",
                t, pos.x(), pos.y(), pos.z());
            return {};
        }
        JointVector joints = *ik_solution;

        // Unwrap IK solution to match previous joints (angles accumulate, no wrapping)
        for (int j = 0; j < 6; ++j) {
            double diff = joints[j] - prev_joints[j];
            while (diff > M_PI) {
                joints[j] -= 2.0 * M_PI;
                diff -= 2.0 * M_PI;
            }
            while (diff < -M_PI) {
                joints[j] += 2.0 * M_PI;
                diff += 2.0 * M_PI;
            }
        }

        // Compute joint velocities (finite difference)
        double sample_period = 1.0 / config_.sample_rate;
        JointVector velocities = JointVector::Zero();
        if (i > 0 && !samples.empty()) {
            for (size_t j = 0; j < 6; ++j) {
                auto idx = static_cast<Eigen::Index>(j);
                velocities[idx] = (joints[idx] - samples.back().joints[idx]) / sample_period;
            }
        } else if (i == 0 && prev_seg_joints.has_value()) {
            // Use previous segment's last joints for velocity continuity
            for (size_t j = 0; j < 6; ++j) {
                auto idx = static_cast<Eigen::Index>(j);
                velocities[idx] = (joints[idx] - (*prev_seg_joints)[idx]) / sample_period;
            }
        }

        samples.push_back(createSample(sample_time, joints, velocities));
        prev_joints = joints;  // Track for next iteration
    }

    return samples;
}

std::vector<TrajectorySample> TrajectoryPlanner::planSegmentWithBothArcs(
    const Eigen::Isometry3d& /* start_pose */,
    const JointVector& start_joints,
    const BlendArcInfo& prev_arc,
    const BlendArcInfo& next_arc,
    double start_time,
    double duration,
    double entry_velocity,
    double exit_velocity,
    std::optional<JointVector> prev_seg_joints) {

    std::vector<TrajectorySample> samples;

    if (duration <= 0.0) {
        duration = 1.0;
    }

    // Calculate distances for all three portions:
    // 1. Second half of prev_arc (from midpoint to arc_end)
    // 2. Linear (from prev_arc.arc_end to next_arc.arc_start)
    // 3. First half of next_arc (from arc_start to midpoint)

    double prev_half_arc_length = prev_arc.arc_length / 2.0;
    double linear_distance = (next_arc.arc_start - prev_arc.arc_end).norm();
    double next_half_arc_length = next_arc.arc_length / 2.0;
    double total_distance = prev_half_arc_length + linear_distance + next_half_arc_length;

    spdlog::info("planSegmentWithBothArcs: prev_arc={:.3f}, linear={:.3f}, next_arc={:.3f}, total={:.4f}m",
        prev_half_arc_length, linear_distance, next_half_arc_length, total_distance);

    // Arc geometry for prev_arc (second half)
    Eigen::Vector3d prev_v_start = prev_arc.arc_start - prev_arc.center;
    double prev_half_angle = prev_arc.arc_angle / 2.0;

    // Arc geometry for next_arc (first half)
    Eigen::Vector3d next_v_start = next_arc.arc_start - next_arc.center;
    double next_half_angle = next_arc.arc_angle / 2.0;

    // Linear portion endpoints and orientations
    Eigen::Vector3d linear_start = prev_arc.arc_end;
    Eigen::Vector3d linear_end = next_arc.arc_start;
    Eigen::Quaterniond linear_start_quat = prev_arc.end_orientation;
    Eigen::Quaterniond linear_end_quat = next_arc.start_orientation;

    // Ensure shortest path for SLERP
    if (linear_start_quat.dot(linear_end_quat) < 0.0) {
        linear_end_quat.coeffs() = -linear_end_quat.coeffs();
    }

    // Create S-curve profile for path parameter
    SCurveProfile scurve_profile(
        entry_velocity,
        exit_velocity,
        total_distance,
        duration,
        config_.max_linear_jerk
    );

    if (!scurve_profile.isFeasible()) {
        spdlog::error("S-curve profile infeasible for both-arcs segment");
        return samples;
    }

    // Calculate number of samples
    int num_samples = static_cast<int>(duration * config_.sample_rate) + 1;
    if (num_samples < 2) num_samples = 2;

    // Track previous joints - angles accumulate naturally
    JointVector prev_joints = start_joints;

    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1) * duration;
        double sample_time = start_time + t;

        // Get path position from S-curve profile
        double path_position = scurve_profile.position(t);

        // Distance along the combined path
        double dist_along_path = std::clamp(path_position, 0.0, total_distance);

        Eigen::Vector3d pos;
        Eigen::Quaterniond quat;

        // Determine which portion we're in
        if (dist_along_path <= prev_half_arc_length && prev_half_arc_length > 0.001) {
            // Portion 1: Second half of prev_arc (from midpoint to arc_end)
            double arc_t = dist_along_path / prev_half_arc_length;

            // Start from arc midpoint (prev_half_angle into the arc)
            double angle = prev_half_angle + arc_t * prev_half_angle;
            Eigen::Vector3d v_rotated =
                prev_v_start * std::cos(angle) +
                prev_arc.normal.cross(prev_v_start) * std::sin(angle) +
                prev_arc.normal * prev_arc.normal.dot(prev_v_start) * (1 - std::cos(angle));
            pos = prev_arc.center + v_rotated;

            // Interpolate orientation through second half (0.5 to 1.0)
            quat = prev_arc.start_orientation.slerp(0.5 + arc_t * 0.5, prev_arc.end_orientation);

        } else if (dist_along_path <= prev_half_arc_length + linear_distance) {
            // Portion 2: Linear from prev_arc.arc_end to next_arc.arc_start
            double linear_dist = dist_along_path - prev_half_arc_length;
            double linear_t = (linear_distance > 0.001) ? (linear_dist / linear_distance) : 1.0;
            linear_t = std::clamp(linear_t, 0.0, 1.0);

            pos = linear_start + linear_t * (linear_end - linear_start);
            quat = linear_start_quat.slerp(linear_t, linear_end_quat);

        } else {
            // Portion 3: First half of next_arc (from arc_start to midpoint)
            double arc_dist = dist_along_path - prev_half_arc_length - linear_distance;
            double arc_t = (next_half_arc_length > 0.001) ? (arc_dist / next_half_arc_length) : 0.0;
            arc_t = std::clamp(arc_t, 0.0, 1.0);

            // Rotate from arc_start by (arc_t * half_angle)
            double angle = arc_t * next_half_angle;
            Eigen::Vector3d v_rotated =
                next_v_start * std::cos(angle) +
                next_arc.normal.cross(next_v_start) * std::sin(angle) +
                next_arc.normal * next_arc.normal.dot(next_v_start) * (1 - std::cos(angle));
            pos = next_arc.center + v_rotated;

            // Interpolate orientation through first half (0.0 to 0.5)
            quat = next_arc.start_orientation.slerp(arc_t * 0.5, next_arc.end_orientation);
        }

        // Solve IK using configured method
        Eigen::Isometry3d sample_pose = Eigen::Isometry3d::Identity();
        sample_pose.translation() = pos;
        sample_pose.linear() = quat.toRotationMatrix();

        // Use prev_joints as seed for continuous motion
        auto ik_solution = solveIK(sample_pose, prev_joints);
        if (!ik_solution.has_value()) {
            spdlog::error("Both-arc path unreachable at t={:.3f}s, pos=[{:.3f}, {:.3f}, {:.3f}]",
                t, pos.x(), pos.y(), pos.z());
            return {};
        }
        JointVector joints = *ik_solution;

        // Unwrap IK solution to match previous joints (angles accumulate, no wrapping)
        for (int j = 0; j < 6; ++j) {
            double diff = joints[j] - prev_joints[j];
            while (diff > M_PI) {
                joints[j] -= 2.0 * M_PI;
                diff -= 2.0 * M_PI;
            }
            while (diff < -M_PI) {
                joints[j] += 2.0 * M_PI;
                diff += 2.0 * M_PI;
            }
        }

        // Compute joint velocities (finite difference)
        double sample_period = 1.0 / config_.sample_rate;
        JointVector velocities = JointVector::Zero();
        if (i > 0 && !samples.empty()) {
            for (size_t j = 0; j < 6; ++j) {
                auto idx = static_cast<Eigen::Index>(j);
                velocities[idx] = (joints[idx] - samples.back().joints[idx]) / sample_period;
            }
        } else if (i == 0 && prev_seg_joints.has_value()) {
            // Use previous segment's last joints for velocity continuity
            for (size_t j = 0; j < 6; ++j) {
                auto idx = static_cast<Eigen::Index>(j);
                velocities[idx] = (joints[idx] - (*prev_seg_joints)[idx]) / sample_period;
            }
        }

        samples.push_back(createSample(sample_time, joints, velocities));
        prev_joints = joints;
    }

    return samples;
}

TrajectorySample TrajectoryPlanner::createSample(
    double time,
    const JointVector& joints,
    const JointVector& velocities) const {

    TrajectorySample sample;
    sample.time = time;
    sample.joints = joints;
    sample.pose = kinematics_.forward(joints);

    // Compute TCP velocity using Jacobian
    auto J = kinematics_.jacobian(joints);
    Eigen::Matrix<double, 6, 1> twist = J * velocities;
    sample.linear_velocity = twist.head<3>();
    sample.speed = sample.linear_velocity.norm();

    return sample;
}

std::vector<TrajectorySample> TrajectoryPlanner::computeBlendArc(
    const Waypoint& /* waypoint */,
    const JointVector& approach_joints,
    const JointVector& departure_joints,
    double blend_time,
    double start_time) {

    std::vector<TrajectorySample> samples;

    // Simple cubic interpolation for blending
    int num_samples = static_cast<int>(blend_time * config_.sample_rate);
    if (num_samples < 2) num_samples = 2;

    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        double s = smoothStep(t);

        JointVector joints;
        JointVector velocities;
        for (size_t j = 0; j < 6; ++j) {
            auto idx = static_cast<Eigen::Index>(j);
            double diff = wrapAngle(departure_joints[idx] - approach_joints[idx]);
            joints[idx] = approach_joints[idx] + s * diff;

            // Velocity from derivative of cubic
            double ds_dt = 6.0 * t * (1.0 - t) / blend_time;
            velocities[idx] = diff * ds_dt;
        }

        double sample_time = start_time + t * blend_time;
        samples.push_back(createSample(sample_time, joints, velocities));
    }

    return samples;
}

std::optional<BlendArcInfo> TrajectoryPlanner::computeBlendGeometry(
    const Waypoint& prev_wp,
    const Waypoint& blend_wp,
    const Waypoint& next_wp,
    const JointVector& prev_joints) {

    // Get Cartesian positions
    Eigen::Vector3d p_prev = prev_wp.position;
    Eigen::Vector3d p_blend = blend_wp.position;
    Eigen::Vector3d p_next = next_wp.position;
    double blend_factor = blend_wp.blend_factor;

    spdlog::info("computeBlendGeometry: blend_factor={:.4f}", blend_factor);
    spdlog::info("  prev=({:.3f},{:.3f},{:.3f}), blend=({:.3f},{:.3f},{:.3f}), next=({:.3f},{:.3f},{:.3f})",
        p_prev.x(), p_prev.y(), p_prev.z(),
        p_blend.x(), p_blend.y(), p_blend.z(),
        p_next.x(), p_next.y(), p_next.z());

    if (blend_factor < 0.001) {
        spdlog::info("  Blend factor too small ({:.4f}), skipping blend", blend_factor);
        return std::nullopt;
    }

    // Compute incoming and outgoing direction vectors
    Eigen::Vector3d dir_in = (p_blend - p_prev);
    Eigen::Vector3d dir_out = (p_next - p_blend);

    double dist_in = dir_in.norm();
    double dist_out = dir_out.norm();

    spdlog::info("  dist_in={:.4f}, dist_out={:.4f}", dist_in, dist_out);

    dir_in.normalize();
    dir_out.normalize();

    // Calculate the angle between incoming and outgoing directions
    double cos_angle = dir_in.dot(dir_out);
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    double angle = std::acos(cos_angle);  // Angle between directions (turning angle)

    spdlog::info("  Path angle={:.2f} degrees", angle * 180.0 / kPi);

    // If paths are nearly collinear, no blend needed
    if (angle < 0.01 || angle > kPi - 0.01) {
        spdlog::warn("  Paths nearly collinear (angle={:.4f}), skipping blend", angle);
        return std::nullopt;
    }

    // The arc angle is the supplement of the path angle
    double arc_angle = kPi - angle;

    spdlog::info("  arc_angle={:.2f} degrees", arc_angle * 180.0 / kPi);

    // NEW BLEND_FACTOR APPROACH:
    // blend_factor = distance from vertex to arc apex (along the bisector)
    //
    // Geometry:
    // - arc_radius = geometric radius of the inscribed circle
    // - The center is at distance (arc_radius / cos(angle/2)) from vertex along bisector
    // - The arc apex (closest point to vertex) is at distance:
    //   h = arc_radius / cos(angle/2) - arc_radius = arc_radius * (1 - cos(angle/2)) / cos(angle/2)
    //
    // Solving for arc_radius given blend_factor h:
    //   arc_radius = h * cos(angle/2) / (1 - cos(angle/2))
    //
    // The tangent distance (blend_radius) is:
    //   blend_radius = arc_radius * cos(angle/2)

    double half_angle = angle / 2.0;
    double cos_half = std::cos(half_angle);

    if (cos_half > 0.999) {
        // Nearly straight path, blend_factor would need huge arc
        spdlog::warn("  Path too straight for blend (cos_half={:.4f})", cos_half);
        return std::nullopt;
    }

    // Compute arc_radius from blend_factor
    double arc_radius = blend_factor * cos_half / (1.0 - cos_half);

    // Compute tangent distance (how far along each segment the arc starts/ends)
    double blend_radius = arc_radius * cos_half;

    spdlog::info("  Computed from blend_factor: arc_radius={:.4f}, blend_radius={:.4f}",
        arc_radius, blend_radius);

    // Check if blend_radius (tangent distance) exceeds half the segment lengths
    double max_blend_radius = std::min(dist_in, dist_out) * 0.5;
    if (blend_radius > max_blend_radius) {
        // Need to clamp - recompute blend_factor from the max allowed blend_radius
        double old_factor = blend_factor;
        blend_radius = max_blend_radius;
        arc_radius = blend_radius / cos_half;
        blend_factor = arc_radius * (1.0 - cos_half) / cos_half;
        spdlog::info("  Clamped: blend_factor {:.4f} -> {:.4f}, blend_radius={:.4f}",
            old_factor, blend_factor, blend_radius);
    }

    // Arc start and end points (tangent points on each segment)
    Eigen::Vector3d arc_start = p_blend - blend_radius * dir_in;
    Eigen::Vector3d arc_end = p_blend + blend_radius * dir_out;

    spdlog::info("  arc_start=({:.3f},{:.3f},{:.3f}), arc_end=({:.3f},{:.3f},{:.3f})",
        arc_start.x(), arc_start.y(), arc_start.z(),
        arc_end.x(), arc_end.y(), arc_end.z());

    // Compute arc center using robust method:
    // The center must satisfy:
    // 1. (center - arc_start) perpendicular to dir_in (tangency)
    // 2. (center - arc_end) perpendicular to dir_out (tangency)
    // 3. |center - arc_start| = |center - arc_end| (equal radius)

    // Normal to the path plane
    Eigen::Vector3d normal = dir_in.cross(dir_out);
    spdlog::info("  normal before normalize: norm={:.4f}", normal.norm());
    if (normal.norm() < 0.001) {
        spdlog::warn("  Collinear paths (normal.norm={:.4f}), skipping blend", normal.norm());
        return std::nullopt;  // Collinear paths
    }
    normal.normalize();

    // Perpendicular directions (in the plane, pointing towards the arc center)
    // For a corner that bends "inward", the center is on the inside of the corner
    Eigen::Vector3d perp_in = normal.cross(dir_in).normalized();   // Perpendicular to incoming, in plane
    Eigen::Vector3d perp_out = normal.cross(dir_out).normalized(); // Perpendicular to outgoing, in plane

    // Check which side the center should be on (inside the corner)
    // The center should be on the side where both perps point
    Eigen::Vector3d corner_bisector = (-dir_in + dir_out).normalized();
    if (perp_in.dot(corner_bisector) < 0) {
        perp_in = -perp_in;
    }
    if (perp_out.dot(corner_bisector) < 0) {
        perp_out = -perp_out;
    }

    spdlog::info("  perp_in=({:.3f},{:.3f},{:.3f})", perp_in.x(), perp_in.y(), perp_in.z());
    spdlog::info("  perp_out=({:.3f},{:.3f},{:.3f})", perp_out.x(), perp_out.y(), perp_out.z());

    // Find the arc center as intersection of two lines:
    // Line 1: arc_start + t * perp_in
    // Line 2: arc_end + s * perp_out
    // We need to find t and s such that these are equal (or closest point in 3D)

    // Using the formula for closest point between two lines in 3D:
    // We solve for t where the lines are closest
    Eigen::Vector3d w0 = arc_start - arc_end;
    double a = perp_in.dot(perp_in);      // always 1 (normalized)
    double b = perp_in.dot(perp_out);
    double c = perp_out.dot(perp_out);    // always 1 (normalized)
    double d = perp_in.dot(w0);
    double e = perp_out.dot(w0);

    double denom = a * c - b * b;
    if (std::abs(denom) < 0.0001) {
        spdlog::warn("  Parallel perpendiculars, cannot find arc center");
        return std::nullopt;
    }

    double t = (b * e - c * d) / denom;
    double s = (a * e - b * d) / denom;

    Eigen::Vector3d center1 = arc_start + t * perp_in;
    Eigen::Vector3d center2 = arc_end + s * perp_out;
    Eigen::Vector3d arc_center = (center1 + center2) / 2.0;  // Average in case of small numerical error

    spdlog::info("  t={:.4f}, s={:.4f}", t, s);
    spdlog::info("  arc_center=({:.3f},{:.3f},{:.3f})", arc_center.x(), arc_center.y(), arc_center.z());

    // Compute actual arc radius from the center
    double radius_to_start = (arc_start - arc_center).norm();
    double radius_to_end = (arc_end - arc_center).norm();
    arc_radius = (radius_to_start + radius_to_end) / 2.0;

    spdlog::info("  radius_to_start={:.4f}, radius_to_end={:.4f}, arc_radius={:.4f}",
        radius_to_start, radius_to_end, arc_radius);

    // Verify the radii are close enough
    if (std::abs(radius_to_start - radius_to_end) > 0.01) {
        spdlog::warn("  Radii mismatch: {:.4f} vs {:.4f}", radius_to_start, radius_to_end);
        return std::nullopt;
    }

    // Recalculate arc angle based on actual geometry
    Eigen::Vector3d v_start = (arc_start - arc_center).normalized();
    Eigen::Vector3d v_end = (arc_end - arc_center).normalized();
    arc_angle = std::acos(std::clamp(v_start.dot(v_end), -1.0, 1.0));

    // Arc length
    double arc_length = arc_radius * arc_angle;

    spdlog::info("  Arc geometry: arc_radius={:.4f}, arc_angle={:.2f} deg, arc_length={:.4f}",
        arc_radius, arc_angle * 180.0 / kPi, arc_length);

    // Compute IK for arc start and end positions
    // Create poses at arc start/end with interpolated orientation
    Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
    start_pose.translation() = arc_start;
    start_pose.linear() = blend_wp.toPose().rotation();  // Use blend waypoint orientation

    Eigen::Isometry3d end_pose = Eigen::Isometry3d::Identity();
    end_pose.translation() = arc_end;
    end_pose.linear() = blend_wp.toPose().rotation();

    // Solve IK for arc endpoints using configured method
    auto best_start = solveIK(start_pose, prev_joints);
    if (!best_start) {
        spdlog::warn("  Arc start not reachable - no IK solution");
        return std::nullopt;
    }

    auto best_end = solveIK(end_pose, *best_start);
    if (!best_end) {
        spdlog::warn("  Arc end not reachable - no IK solution");
        return std::nullopt;
    }
    spdlog::info("  Arc IK solved (numerical)");

    // Build BlendArcInfo
    BlendArcInfo arc_info;
    arc_info.center = arc_center;
    arc_info.radius = arc_radius;
    arc_info.arc_start = arc_start;
    arc_info.arc_end = arc_end;
    arc_info.start_tangent = dir_in;
    arc_info.end_tangent = dir_out;
    arc_info.normal = normal;
    arc_info.arc_angle = arc_angle;
    arc_info.arc_length = arc_length;
    arc_info.start_joints = *best_start;
    arc_info.end_joints = *best_end;
    arc_info.start_orientation = Eigen::Quaterniond(blend_wp.toPose().rotation());
    arc_info.end_orientation = arc_info.start_orientation;  // Keep same orientation through blend

    spdlog::info("  Blend arc geometry computed successfully!");
    return arc_info;
}

std::vector<TrajectorySample> TrajectoryPlanner::planBlendArcCartesian(
    const BlendArcInfo& arc_info,
    double start_time,
    double arc_duration) {

    std::vector<TrajectorySample> samples;

    // Use the provided arc_duration (calculated by caller for timing consistency)
    arc_duration = std::max(arc_duration, 0.05);  // Minimum 50ms

    // Compute vector from center to arc start
    Eigen::Vector3d v_start = arc_info.arc_start - arc_info.center;

    int num_samples = static_cast<int>(arc_duration * config_.sample_rate);
    if (num_samples < 2) num_samples = 2;

    double sample_period = arc_duration / (num_samples - 1);

    // Use Rodrigues' rotation formula to interpolate along the arc
    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        double current_angle = t * arc_info.arc_angle;

        // Rotate v_start around the normal by current_angle
        Eigen::Vector3d v_rotated =
            v_start * std::cos(current_angle) +
            arc_info.normal.cross(v_start) * std::sin(current_angle) +
            arc_info.normal * arc_info.normal.dot(v_start) * (1 - std::cos(current_angle));

        Eigen::Vector3d arc_point = arc_info.center + v_rotated;

        // Interpolate orientation using SLERP
        Eigen::Quaterniond orientation = arc_info.start_orientation.slerp(
            t, arc_info.end_orientation);

        // Solve IK using configured method
        JointVector prev_joints_val = (i == 0) ? arc_info.start_joints : samples.back().joints;
        Eigen::Isometry3d arc_pose = Eigen::Isometry3d::Identity();
        arc_pose.translation() = arc_point;
        arc_pose.linear() = orientation.toRotationMatrix();

        auto ik_solution = solveIK(arc_pose, prev_joints_val);
        if (!ik_solution.has_value()) {
            spdlog::error("Arc path unreachable at t={:.3f} - no IK solution found", t);
            return {};
        }
        JointVector joints = *ik_solution;

        // Compute velocity (finite difference)
        JointVector velocities = JointVector::Zero();
        if (i > 0) {
            double dt = sample_period;
            for (size_t j = 0; j < 6; ++j) {
                auto idx = static_cast<Eigen::Index>(j);
                velocities[idx] = (joints[idx] - samples.back().joints[idx]) / dt;
            }
        }

        samples.push_back(createSample(start_time + i * sample_period, joints, velocities));
    }

    return samples;
}

double TrajectoryPlanner::calculateSegmentDuration(
    const JointVector& start,
    const JointVector& end) const {

    JointVector diff = jointDifference(end, start);

    // Find the joint that requires the most time
    double max_time = 0.1;  // Minimum 100ms

    for (size_t i = 0; i < 6; ++i) {
        auto idx = static_cast<Eigen::Index>(i);
        // Time required considering velocity limit
        double t_vel = std::abs(diff[idx]) / config_.max_joint_velocity;

        // Time required considering acceleration (triangular profile)
        // t = 2 * sqrt(|diff| / max_accel)
        double t_acc = 2.0 * std::sqrt(std::abs(diff[idx]) / config_.max_joint_acceleration);

        double t_joint = std::max(t_vel, t_acc);
        max_time = std::max(max_time, t_joint);
    }

    // Add 20% margin for smooth motion
    return max_time * 1.2;
}

TrajectoryVisualization TrajectoryPlanner::generateVisualization(
    const PlannedTrajectory& trajectory,
    const std::vector<Waypoint>& waypoints) const {

    TrajectoryVisualization viz;

    if (trajectory.samples.empty()) {
        return viz;
    }

    // Sample path at regular intervals for visualization
    size_t viz_sample_step = std::max(size_t(1), trajectory.samples.size() / 500);

    for (size_t i = 0; i < trajectory.samples.size(); i += viz_sample_step) {
        const auto& sample = trajectory.samples[i];
        viz.path_points.push_back(sample.pose.translation());
        viz.path_times.push_back(sample.time);
        viz.times.push_back(sample.time);
        viz.speeds.push_back(sample.speed);

        // Compute acceleration (finite difference)
        if (i > 0 && i + viz_sample_step < trajectory.samples.size()) {
            double dt = trajectory.samples[i + viz_sample_step].time - trajectory.samples[i - viz_sample_step].time;
            double dv = trajectory.samples[i + viz_sample_step].speed - trajectory.samples[i - viz_sample_step].speed;
            viz.accelerations.push_back(dv / dt);
        } else {
            viz.accelerations.push_back(0.0);
        }
    }

    // Make sure to include the last point
    if (viz.path_points.empty() || viz.path_times.back() < trajectory.samples.back().time - 0.001) {
        const auto& last = trajectory.samples.back();
        viz.path_points.push_back(last.pose.translation());
        viz.path_times.push_back(last.time);
        viz.times.push_back(last.time);
        viz.speeds.push_back(last.speed);
        viz.accelerations.push_back(0.0);
    }

    // Compute jerk (derivative of acceleration using finite differences)
    viz.jerks.reserve(viz.accelerations.size());
    for (size_t i = 0; i < viz.accelerations.size(); ++i) {
        if (i > 0 && i + 1 < viz.accelerations.size()) {
            double dt = viz.times[i + 1] - viz.times[i - 1];
            double da = viz.accelerations[i + 1] - viz.accelerations[i - 1];
            viz.jerks.push_back(dt > 0.001 ? da / dt : 0.0);
        } else {
            viz.jerks.push_back(0.0);
        }
    }

    // Create waypoint markers and segment times for graph
    // Find actual times by searching for closest trajectory sample to each waypoint position
    viz.segment_times.push_back(0.0);  // First waypoint at t=0

    for (size_t i = 0; i < waypoints.size(); ++i) {
        WaypointMarker marker;
        marker.index = i;
        marker.position = waypoints[i].position;
        marker.blend_factor = waypoints[i].blend_factor;
        marker.has_pause = waypoints[i].pause_time > 0.0;

        if (i == 0) {
            marker.time = 0.0;
        } else {
            // Find the trajectory sample closest to this waypoint's position
            double best_time = trajectory.total_duration;
            double min_dist = std::numeric_limits<double>::max();

            for (const auto& sample : trajectory.samples) {
                double dist = (sample.pose.translation() - waypoints[i].position).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    best_time = sample.time;
                }
            }

            marker.time = best_time;
            viz.segment_times.push_back(marker.time);
        }

        viz.waypoints.push_back(marker);
    }

    // Joint data for graphs
    viz.joint_positions.reserve(trajectory.samples.size() / viz_sample_step + 1);
    viz.joint_velocities.reserve(trajectory.samples.size() / viz_sample_step + 1);

    for (size_t i = 0; i < trajectory.samples.size(); i += viz_sample_step) {
        const auto& sample = trajectory.samples[i];

        std::array<double, 6> pos{}, vel{};
        for (size_t j = 0; j < 6; ++j) {
            pos[j] = sample.joints[static_cast<Eigen::Index>(j)];
        }

        // Compute velocities (finite difference)
        if (i > 0 && i + viz_sample_step < trajectory.samples.size()) {
            double dt = trajectory.samples[i + viz_sample_step].time -
                        trajectory.samples[i - viz_sample_step].time;
            for (size_t j = 0; j < 6; ++j) {
                auto idx = static_cast<Eigen::Index>(j);
                double dq = trajectory.samples[i + viz_sample_step].joints[idx] -
                            trajectory.samples[i - viz_sample_step].joints[idx];
                vel[j] = dq / dt;
            }
        } else {
            for (size_t j = 0; j < 6; ++j) {
                vel[j] = 0.0;
            }
        }

        viz.joint_positions.push_back(pos);
        viz.joint_velocities.push_back(vel);
    }

    // Summary statistics
    viz.total_duration = trajectory.total_duration;

    // Calculate total distance
    viz.total_distance = 0.0;
    for (size_t i = 1; i < viz.path_points.size(); ++i) {
        viz.total_distance += (viz.path_points[i] - viz.path_points[i-1]).norm();
    }

    // Maximum speed
    viz.max_speed = 0.0;
    for (double speed : viz.speeds) {
        viz.max_speed = std::max(viz.max_speed, speed);
    }

    return viz;
}

// =============================================================================
// New API: SetupPose and Sequence planning
// =============================================================================

PlannedTrajectory TrajectoryPlanner::planSetupPose(
    const SetupPose& setup_pose,
    const JointVector& start_joints) {

    PlannedTrajectory result;
    result.sample_period = 1.0 / config_.sample_rate;

    // Validate target joints are within limits
    const auto& limits = kinematics_.jointLimits();
    if (!limits.withinLimits(setup_pose.joints)) {
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error,
            "Setup pose joints exceed joint limits", std::nullopt, std::nullopt});
        return result;
    }

    // Check if we're already at the target
    double total_diff = 0.0;
    for (int i = 0; i < 6; ++i) {
        double diff = std::abs(setup_pose.joints[i] - start_joints[i]);
        total_diff += diff;
    }
    if (total_diff < 1e-6) {
        // Already at target - create single sample
        result.valid = true;
        result.total_duration = 0.0;
        result.samples.push_back(createSample(0.0, start_joints, JointVector::Zero()));
        return result;
    }

    // Determine duration
    double duration = setup_pose.move_time;
    if (duration <= 0.0) {
        // Auto-compute based on joint velocity limits
        duration = calculateSegmentDuration(start_joints, setup_pose.joints);
    }

    // Plan using Ruckig for smooth joint-space motion
    result.samples = planSegmentWithRuckig(
        start_joints,
        setup_pose.joints,
        JointVector::Zero(),  // Start at rest
        JointVector::Zero(),  // No initial acceleration
        0.0,                  // Start time
        duration,
        JointVector::Zero()); // End at rest

    if (result.samples.empty()) {
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error,
            "Failed to plan joint motion to setup pose", std::nullopt, std::nullopt});
        return result;
    }

    result.total_duration = result.samples.back().time;
    result.valid = true;

    if (!setup_pose.name.empty()) {
        result.messages.push_back({ValidationSeverity::Info,
            "Setup pose '" + setup_pose.name + "' planned: " +
            std::to_string(result.total_duration) + "s",
            std::nullopt, std::nullopt});
    }

    return result;
}

PlannedTrajectory TrajectoryPlanner::planSequence(
    const Sequence& sequence,
    const JointVector& entry_joints) {

    PlannedTrajectory result;
    result.sample_period = 1.0 / config_.sample_rate;

    if (sequence.empty()) {
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error,
            "Sequence contains no waypoints", std::nullopt, std::nullopt});
        return result;
    }

    // Log entry configuration for debugging (informational only)
    auto entry_config = kinematics_.getConfiguration(entry_joints);
    spdlog::info("Sequence planning from entry config: shoulder={} elbow={} wrist={}",
        entry_config.shoulder, entry_config.elbow, entry_config.wrist);

    // Convert SequenceWaypoints to legacy Waypoints for validation
    std::vector<Waypoint> waypoints;
    waypoints.reserve(sequence.size());
    for (const auto& sw : sequence.waypoints) {
        Waypoint wp;
        wp.position = sw.position;
        wp.orientation = sw.orientation;
        wp.blend_factor = sw.blend_factor;
        wp.segment_time = sw.segment_time;
        wp.pause_time = sw.pause_time;
        wp.joints = sw.joints;  // Copy saved joint configuration
        waypoints.push_back(wp);
    }

    // Validate all waypoints are reachable via numerical IK seeded from
    // the entry joints (or the previous waypoint's saved joints).
    {
        JointVector reach_seed = entry_joints;
        for (size_t i = 0; i < waypoints.size(); ++i) {
            auto pose = waypoints[i].toPose();
            // Prefer the waypoint's own taught joints as seed when present
            JointVector seed = waypoints[i].joints.value_or(reach_seed);
            auto sol = kinematics_.inverseNumerical(pose, seed);
            if (!sol.has_value()) {
                result.valid = false;
                result.messages.push_back({ValidationSeverity::Error,
                    "Waypoint " + std::to_string(i + 1) + " is unreachable (numerical IK failed)",
                    i, std::nullopt});
                return result;
            }
            reach_seed = *sol;
        }
    }

    // Build segments with endpoint IK pre-computation (for duration estimation)
    // The actual path IK is solved incrementally in planSegmentLinearCartesian
    std::vector<Segment> segments;
    JointVector current_joints = entry_joints;
    double current_time = 0.0;

    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        Segment seg;
        seg.start_index = i;
        seg.end_index = i + 1;
        seg.start_joints = current_joints;
        seg.start_time = current_time;

        // Handle pause at current waypoint
        if (waypoints[i].pause_time > 0.0) {
            current_time += waypoints[i].pause_time;
        }
        seg.start_time = current_time;

        auto end_pose = waypoints[i + 1].toPose();
        Eigen::Vector3d wp_pos = end_pose.translation();
        spdlog::info("Segment {} creation: WP{} pose pos=[{:.4f}, {:.4f}, {:.4f}]",
            i, i + 2, wp_pos.x(), wp_pos.y(), wp_pos.z());

        // Use saved waypoint joints if available, otherwise solve IK
        if (waypoints[i + 1].joints.has_value()) {
            // Use the taught joint configuration directly
            seg.end_joints = waypoints[i + 1].joints.value();
            spdlog::info("  Segment {} using saved waypoint joints: [{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}]°",
                i, seg.end_joints[0]*180/M_PI, seg.end_joints[1]*180/M_PI, seg.end_joints[2]*180/M_PI,
                seg.end_joints[3]*180/M_PI, seg.end_joints[4]*180/M_PI, seg.end_joints[5]*180/M_PI);
        } else {
            // No saved joints - solve IK numerically from current joints
            auto ik_solution = solveIK(end_pose, current_joints);

            if (!ik_solution.has_value()) {
                spdlog::error("  No IK solution found for WP{}", i + 2);
                result.valid = false;
                result.messages.push_back({ValidationSeverity::Error,
                    "No IK solution for waypoint " + std::to_string(i + 2),
                    i + 1, std::nullopt});
                return result;
            }

            seg.end_joints = *ik_solution;
        }

        // Log significant joint changes
        for (int j = 0; j < 6; ++j) {
            double diff = kinematics::URKinematics::wrapAngle(seg.end_joints[j] - current_joints[j]);
            if (std::abs(diff) > 0.5) {  // More than ~30 degrees
                spdlog::info("  J{} changed by {:.1f}° ({:.1f}° -> {:.1f}°)",
                    j + 1, diff * 180.0 / 3.14159,
                    current_joints[j] * 180.0 / 3.14159,
                    seg.end_joints[j] * 180.0 / 3.14159);
            }
        }

        // Determine segment duration
        if (waypoints[i + 1].segment_time > 0.0) {
            seg.duration = waypoints[i + 1].segment_time;
        } else {
            seg.duration = calculateSegmentDuration(seg.start_joints, seg.end_joints);
        }

        // Check for blending
        seg.has_blend_in = (i > 0 && waypoints[i].blend_factor > 0.0);
        seg.has_blend_out = (waypoints[i + 1].blend_factor > 0.0 && i + 2 < waypoints.size());

        // Blend arc geometry computed during planning with actual joints
        seg.blend_arc = std::nullopt;

        segments.push_back(seg);
        current_joints = seg.end_joints;  // Chain the seed for next segment
        current_time += seg.duration;
    }

    // Plan joint-space motion
    result.samples = planJointMotion(segments, waypoints, result.messages);

    if (result.samples.empty()) {
        result.valid = false;
        // Only add generic error if no specific errors were already added
        if (!result.hasErrors()) {
            result.messages.push_back({ValidationSeverity::Error,
                "Linear path planning failed - path may cross singularity within locked configuration",
                std::nullopt, std::nullopt});
        }
        return result;
    }

    result.total_duration = result.samples.back().time;
    result.valid = !result.hasErrors();

    if (!sequence.name.empty()) {
        result.messages.push_back({ValidationSeverity::Info,
            "Sequence '" + sequence.name + "' planned: " +
            std::to_string(waypoints.size()) + " waypoints, " +
            std::to_string(result.total_duration) + "s",
            std::nullopt, std::nullopt});
    }

    return result;
}

PlannedTrajectory TrajectoryPlanner::planTrajectory(
    const Trajectory& trajectory,
    const JointVector& start_joints) {

    PlannedTrajectory result;
    result.sample_period = 1.0 / config_.sample_rate;

    if (trajectory.empty()) {
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error,
            "Trajectory contains no elements", std::nullopt, std::nullopt});
        return result;
    }

    JointVector current_joints = start_joints;
    double current_time = 0.0;

    for (size_t elem_idx = 0; elem_idx < trajectory.elements.size(); ++elem_idx) {
        const auto& element = trajectory.elements[elem_idx];
        PlannedTrajectory element_traj;

        if (element.type == TrajectoryElementType::SetupPose) {
            element_traj = planSetupPose(element.setup_pose, current_joints);
        } else {
            // Use stored entry_joints if available, otherwise use current_joints
            bool has_entry = element.sequence.hasEntryJoints();
            JointVector seq_entry = has_entry
                ? element.sequence.entry_joints.value()
                : current_joints;
            spdlog::info("Planning sequence: hasEntryJoints={}, using joints=[{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]",
                has_entry, seq_entry[0], seq_entry[1], seq_entry[2], seq_entry[3], seq_entry[4], seq_entry[5]);
            element_traj = planSequence(element.sequence, seq_entry);
        }

        // Check for errors
        if (!element_traj.valid) {
            result.valid = false;
            for (auto& msg : element_traj.messages) {
                msg.message = "Element " + std::to_string(elem_idx + 1) + ": " + msg.message;
                result.messages.push_back(msg);
            }
            return result;
        }

        // Copy messages
        for (auto& msg : element_traj.messages) {
            msg.time = msg.time.has_value() ?
                std::optional<double>(msg.time.value() + current_time) : std::nullopt;
            result.messages.push_back(msg);
        }

        // Append samples with time offset
        for (const auto& sample : element_traj.samples) {
            TrajectorySample offset_sample = sample;
            offset_sample.time += current_time;
            result.samples.push_back(offset_sample);
        }

        // Update current state for next element
        if (!element_traj.samples.empty()) {
            current_joints = element_traj.samples.back().joints;
            current_time += element_traj.total_duration;
        }
    }

    result.total_duration = current_time;
    result.valid = true;

    if (!trajectory.name.empty()) {
        result.messages.push_back({ValidationSeverity::Info,
            "Trajectory '" + trajectory.name + "' planned: " +
            std::to_string(trajectory.elements.size()) + " elements, " +
            std::to_string(result.total_duration) + "s",
            std::nullopt, std::nullopt});
    }

    return result;
}

}  // namespace trajectory
}  // namespace ur_controller
