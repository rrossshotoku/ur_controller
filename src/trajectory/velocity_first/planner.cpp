/// @file velocity_first/planner.cpp
/// @brief Implementation of velocity-first trajectory planning

#include "ur_controller/trajectory/velocity_first/planner.hpp"

#include <spdlog/spdlog.h>
#include <fmt/format.h>

#include <cmath>
#include <algorithm>

namespace ur_controller {
namespace trajectory {
namespace velocity_first {

// =============================================================================
// Construction
// =============================================================================

VelocityFirstPlanner::VelocityFirstPlanner(const kinematics::URKinematics& kinematics)
    : kinematics_(kinematics)
    , config_() {
}

VelocityFirstPlanner::VelocityFirstPlanner(
    const kinematics::URKinematics& kinematics,
    const VelocityFirstConfig& config)
    : kinematics_(kinematics)
    , config_(config) {
}

// =============================================================================
// Main Planning Entry Point
// =============================================================================

PlannedTrajectory VelocityFirstPlanner::plan(
    const std::vector<TimedWaypoint>& waypoints,
    const JointVector& start_joints) {

    PlannedTrajectory result;
    result.valid = false;
    result.sample_period = 1.0 / config_.sample_rate;

    // Validate input
    if (waypoints.size() < 2) {
        result.messages.push_back({
            ValidationSeverity::Error,
            "At least 2 waypoints required",
            std::nullopt,
            std::nullopt
        });
        return result;
    }

    spdlog::info("VelocityFirstPlanner: Planning trajectory with {} waypoints",
        waypoints.size());

    // Step 1: Compute target velocities for each segment
    auto velocities = computeTargetVelocities(waypoints);

    // Step 2: Validate velocities against limits
    if (!validateVelocities(velocities, result.messages)) {
        return result;
    }

    // Step 3: Compute velocity transitions
    auto transitions = computeTransitions(waypoints, velocities);

    // Step 4: Validate transitions don't overlap
    if (!validateTransitions(transitions, result.messages)) {
        return result;
    }

    // Step 5: Sample trajectory
    result.samples = sampleTrajectory(waypoints, velocities, transitions, start_joints);

    if (result.samples.empty()) {
        result.messages.push_back({
            ValidationSeverity::Error,
            "Failed to sample trajectory (IK failures)",
            std::nullopt,
            std::nullopt
        });
        return result;
    }

    // Step 6: Apply final position correction
    applyFinalCorrection(result.samples, waypoints.back().position,
        waypoints.back().time);

    // Set total duration
    result.total_duration = waypoints.back().time;
    result.valid = true;

    spdlog::info("VelocityFirstPlanner: Generated {} samples over {:.2f}s",
        result.samples.size(), result.total_duration);

    return result;
}

PlannedTrajectory VelocityFirstPlanner::planFromLegacy(
    const std::vector<Waypoint>& waypoints,
    const JointVector& start_joints) {

    // Convert legacy waypoints to timed waypoints
    std::vector<TimedWaypoint> timed_waypoints;
    double accumulated_time = 0.0;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];

        TimedWaypoint tw;
        tw.position = wp.position;
        tw.orientation = wp.orientation;
        tw.joints = wp.joints;

        // First waypoint is at t=0, subsequent use segment_time
        if (i > 0) {
            accumulated_time += wp.segment_time;
        }
        tw.time = accumulated_time;

        timed_waypoints.push_back(tw);
    }

    return plan(timed_waypoints, start_joints);
}

// =============================================================================
// Velocity Computation
// =============================================================================

std::vector<SegmentVelocity> VelocityFirstPlanner::computeTargetVelocities(
    const std::vector<TimedWaypoint>& waypoints) const {

    std::vector<SegmentVelocity> velocities;
    velocities.reserve(waypoints.size() - 1);

    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        const auto& wp1 = waypoints[i];
        const auto& wp2 = waypoints[i + 1];

        Eigen::Vector3d delta = wp2.position - wp1.position;
        double distance = delta.norm();
        double duration = wp2.time - wp1.time;

        SegmentVelocity sv;
        sv.duration = duration;

        if (distance < 1e-6 || duration < 1e-6) {
            // Near-zero segment - treat as pause
            sv.direction = Eigen::Vector3d::UnitX();  // Arbitrary
            sv.speed = 0.0;
        } else {
            sv.direction = delta / distance;
            sv.speed = distance / duration;
        }

        spdlog::debug("Segment {}: distance={:.3f}m, duration={:.2f}s, speed={:.3f}m/s",
            i, distance, duration, sv.speed);

        velocities.push_back(sv);
    }

    return velocities;
}

// =============================================================================
// Transition Computation
// =============================================================================

double VelocityFirstPlanner::computeMinTransitionDuration(
    double v1_speed,
    double v2_speed) const {

    double delta_v = std::abs(v2_speed - v1_speed);

    if (delta_v < 1e-6) {
        // No speed change needed - minimal transition for direction change
        return 0.1;  // 100ms minimum for direction interpolation
    }

    // Triangular jerk profile: T_min = 2 * sqrt(|dV| / J_max)
    double T_tri = 2.0 * std::sqrt(delta_v / config_.max_jerk);

    // Check if we need trapezoidal (longer) profile
    double t_j = config_.max_acceleration / config_.max_jerk;
    double dv_tri_max = 0.5 * config_.max_jerk * t_j * t_j * 2.0;  // Max dV for triangular

    if (delta_v > dv_tri_max) {
        // Trapezoidal profile needed
        double T_trap = delta_v / config_.max_acceleration + t_j;
        return T_trap;
    }

    return T_tri;
}

std::vector<VelocityTransition> VelocityFirstPlanner::computeTransitions(
    const std::vector<TimedWaypoint>& waypoints,
    const std::vector<SegmentVelocity>& velocities) const {

    std::vector<VelocityTransition> transitions;

    // First transition: rest to first segment velocity
    {
        VelocityTransition trans;
        SegmentVelocity rest;
        rest.speed = 0.0;
        rest.direction = velocities[0].direction;
        rest.duration = 0.0;

        trans.v_before = rest;
        trans.v_after = velocities[0];

        double T_min = computeMinTransitionDuration(0.0, velocities[0].speed);
        trans.t_center = waypoints[0].time;
        trans.t_start = trans.t_center;  // Start immediately
        trans.t_end = trans.t_center + T_min;

        transitions.push_back(trans);
        spdlog::debug("Initial transition: t=[{:.3f}, {:.3f}], 0 -> {:.3f} m/s",
            trans.t_start, trans.t_end, velocities[0].speed);
    }

    // Interior transitions: between segment velocities
    for (size_t i = 1; i < velocities.size(); ++i) {
        VelocityTransition trans;
        trans.v_before = velocities[i - 1];
        trans.v_after = velocities[i];

        double T_min = computeMinTransitionDuration(
            velocities[i - 1].speed, velocities[i].speed);

        // Center transition at waypoint time
        trans.t_center = waypoints[i].time;
        trans.t_start = trans.t_center - T_min / 2.0;
        trans.t_end = trans.t_center + T_min / 2.0;

        transitions.push_back(trans);
        spdlog::debug("Transition {}: t=[{:.3f}, {:.3f}], {:.3f} -> {:.3f} m/s",
            i, trans.t_start, trans.t_end,
            velocities[i - 1].speed, velocities[i].speed);
    }

    // Final transition: last segment velocity to rest
    {
        VelocityTransition trans;
        trans.v_before = velocities.back();
        SegmentVelocity rest;
        rest.speed = 0.0;
        rest.direction = velocities.back().direction;
        rest.duration = 0.0;
        trans.v_after = rest;

        double T_min = computeMinTransitionDuration(velocities.back().speed, 0.0);
        trans.t_center = waypoints.back().time;
        trans.t_start = trans.t_center - T_min;
        trans.t_end = trans.t_center;  // End exactly at final time

        transitions.push_back(trans);
        spdlog::debug("Final transition: t=[{:.3f}, {:.3f}], {:.3f} -> 0 m/s",
            trans.t_start, trans.t_end, velocities.back().speed);
    }

    return transitions;
}

// =============================================================================
// Validation
// =============================================================================

bool VelocityFirstPlanner::validateVelocities(
    const std::vector<SegmentVelocity>& velocities,
    std::vector<ValidationMessage>& messages) const {

    bool valid = true;

    for (size_t i = 0; i < velocities.size(); ++i) {
        const auto& v = velocities[i];

        if (v.speed > config_.max_speed) {
            double required_duration = (v.speed / config_.max_speed) * v.duration;
            messages.push_back({
                ValidationSeverity::Error,
                fmt::format("Segment {} requires {:.3f} m/s but limit is {:.3f} m/s. "
                           "Increase segment time from {:.2f}s to at least {:.2f}s.",
                           i + 1, v.speed, config_.max_speed,
                           v.duration, required_duration),
                i,
                std::nullopt
            });
            valid = false;
        }
    }

    return valid;
}

bool VelocityFirstPlanner::validateTransitions(
    const std::vector<VelocityTransition>& transitions,
    std::vector<ValidationMessage>& messages) const {

    bool valid = true;

    for (size_t i = 1; i < transitions.size(); ++i) {
        if (transitions[i].t_start < transitions[i - 1].t_end) {
            double overlap = transitions[i - 1].t_end - transitions[i].t_start;
            messages.push_back({
                ValidationSeverity::Error,
                fmt::format("Segment {} is {:.3f}s too short for velocity transitions. "
                           "Increase segment time.",
                           i, overlap),
                i,
                std::nullopt
            });
            valid = false;
        }
    }

    return valid;
}

// =============================================================================
// Velocity Evaluation
// =============================================================================

int VelocityFirstPlanner::findSegmentAt(
    double t,
    const std::vector<TimedWaypoint>& waypoints) const {

    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        if (t >= waypoints[i].time && t <= waypoints[i + 1].time) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int VelocityFirstPlanner::findTransitionAt(
    double t,
    const std::vector<VelocityTransition>& transitions) const {

    for (size_t i = 0; i < transitions.size(); ++i) {
        if (transitions[i].contains(t)) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

Eigen::Vector3d VelocityFirstPlanner::evaluateTransition(
    double t,
    const VelocityTransition& trans) const {

    // Normalize time within transition [0, 1]
    double u = (t - trans.t_start) / (trans.t_end - trans.t_start);
    u = std::clamp(u, 0.0, 1.0);

    // Speed profile: smooth step (S-curve approximation)
    double speed = trans.v_before.speed +
        (trans.v_after.speed - trans.v_before.speed) * smootherStep(u);

    // Direction: SLERP interpolation
    Eigen::Vector3d dir;
    if (trans.v_before.isRest()) {
        dir = trans.v_after.direction;
    } else if (trans.v_after.isRest()) {
        dir = trans.v_before.direction;
    } else {
        dir = slerpDirection(trans.v_before.direction, trans.v_after.direction, u);
    }

    return dir * speed;
}

Eigen::Vector3d VelocityFirstPlanner::velocityAt(
    double t,
    const std::vector<SegmentVelocity>& velocities,
    const std::vector<VelocityTransition>& transitions) const {

    // Check if we're in a transition
    int trans_idx = findTransitionAt(t, transitions);
    if (trans_idx >= 0) {
        return evaluateTransition(t, transitions[static_cast<size_t>(trans_idx)]);
    }

    // Not in transition - find which segment and return cruise velocity
    // Find the transition that just ended
    for (size_t i = 0; i + 1 < transitions.size(); ++i) {
        if (t > transitions[i].t_end && t < transitions[i + 1].t_start) {
            // Between transitions i and i+1 - cruising at velocity for segment i
            if (i < velocities.size()) {
                return velocities[i].vector();
            }
        }
    }

    // Edge case: before first transition or after last
    if (!transitions.empty()) {
        if (t <= transitions.front().t_start) {
            return Eigen::Vector3d::Zero();
        }
        if (t >= transitions.back().t_end) {
            return Eigen::Vector3d::Zero();
        }
    }

    return Eigen::Vector3d::Zero();
}

Eigen::Quaterniond VelocityFirstPlanner::orientationAt(
    double t,
    const std::vector<TimedWaypoint>& waypoints) const {

    // Find which segment we're in
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        if (t >= waypoints[i].time && t <= waypoints[i + 1].time) {
            double segment_duration = waypoints[i + 1].time - waypoints[i].time;
            double u = (segment_duration > 1e-6)
                ? (t - waypoints[i].time) / segment_duration
                : 0.0;
            u = std::clamp(u, 0.0, 1.0);

            return waypoints[i].orientation.slerp(u, waypoints[i + 1].orientation);
        }
    }

    // Before first or after last
    if (t <= waypoints.front().time) {
        return waypoints.front().orientation;
    }
    return waypoints.back().orientation;
}

// =============================================================================
// Trajectory Sampling
// =============================================================================

std::vector<TrajectorySample> VelocityFirstPlanner::sampleTrajectory(
    const std::vector<TimedWaypoint>& waypoints,
    const std::vector<SegmentVelocity>& velocities,
    const std::vector<VelocityTransition>& transitions,
    const JointVector& start_joints) const {

    std::vector<TrajectorySample> samples;

    double dt = 1.0 / config_.sample_rate;
    double total_time = waypoints.back().time;
    size_t num_samples = static_cast<size_t>(total_time * config_.sample_rate) + 1;
    samples.reserve(num_samples);

    // Helper to get interpolated reference joints based on time
    auto getInterpolatedReferenceJoints = [&](double t) -> std::optional<JointVector> {
        // Find which segment we're in
        int seg_idx = findSegmentAt(t, waypoints);
        if (seg_idx < 0) {
            seg_idx = 0;
        }
        size_t seg = static_cast<size_t>(seg_idx);

        // Get waypoints bounding this segment
        const auto& wp1 = waypoints[seg];
        const auto& wp2 = waypoints[std::min(seg + 1, waypoints.size() - 1)];

        // If both have taught joints, interpolate between them
        if (wp1.joints.has_value() && wp2.joints.has_value()) {
            double seg_duration = wp2.time - wp1.time;
            double u = (seg_duration > 1e-6) ? (t - wp1.time) / seg_duration : 0.0;
            u = std::clamp(u, 0.0, 1.0);

            // Linear interpolation of joints
            return (*wp1.joints) * (1.0 - u) + (*wp2.joints) * u;
        }

        // If only one has taught joints, use that
        if (wp1.joints.has_value()) {
            return *wp1.joints;
        }
        if (wp2.joints.has_value()) {
            return *wp2.joints;
        }

        return std::nullopt;
    };

    // Start at first waypoint
    Eigen::Vector3d position = waypoints.front().position;

    // Initialize prev_joints from first waypoint if available
    JointVector prev_joints = start_joints;
    if (waypoints.front().joints.has_value()) {
        prev_joints = *waypoints.front().joints;
        spdlog::info("Using taught joints from WP1 as initial configuration");
    }

    for (size_t i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) * dt;
        if (t > total_time) t = total_time;

        // Get velocity at this time
        Eigen::Vector3d velocity = velocityAt(t, velocities, transitions);

        // Integrate position (Euler integration)
        if (i > 0) {
            position = position + velocity * dt;
        }

        // Get orientation
        Eigen::Quaterniond orientation = orientationAt(t, waypoints);

        // Build pose
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = position;
        pose.linear() = orientation.toRotationMatrix();

        // Get interpolated reference joints for this time
        auto ref_joints = getInterpolatedReferenceJoints(t);

        // Numerical IK seeded from interpolated taught joints if available,
        // otherwise from the previous sample's joints
        JointVector seed = ref_joints.value_or(prev_joints);
        auto best_solution = kinematics_.inverseNumerical(pose, seed);

        if (!best_solution.has_value()) {
            spdlog::error("IK failed at t={:.3f}s, pos=({:.3f}, {:.3f}, {:.3f})",
                t, position.x(), position.y(), position.z());
        }

        JointVector joints = best_solution.value_or(prev_joints);

        // Compute joint velocities (finite difference)
        JointVector joint_velocities = JointVector::Zero();
        if (i > 0) {
            joint_velocities = (joints - prev_joints) / dt;
        }

        // Create sample
        TrajectorySample sample;
        sample.time = t;
        sample.joints = joints;
        sample.pose = pose;
        sample.linear_velocity = velocity;
        sample.speed = velocity.norm();

        samples.push_back(sample);
        prev_joints = joints;
    }

    return samples;
}

void VelocityFirstPlanner::applyFinalCorrection(
    std::vector<TrajectorySample>& samples,
    const Eigen::Vector3d& target_position,
    double total_duration) const {

    if (samples.empty()) return;

    // Find where to start correction (last 0.2s or 10% of trajectory)
    double correction_window = std::min(0.2, total_duration * 0.1);
    double correction_start = total_duration - correction_window;

    // Find sample index at correction start
    size_t start_idx = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
        if (samples[i].time >= correction_start) {
            start_idx = i;
            break;
        }
    }

    if (start_idx >= samples.size() - 1) return;

    // Compute position error at final sample
    Eigen::Vector3d actual_final = samples.back().pose.translation();
    Eigen::Vector3d error = target_position - actual_final;

    if (error.norm() < 0.001) {
        // Error is negligible
        spdlog::debug("Final position error: {:.4f}mm (no correction needed)",
            error.norm() * 1000);
        return;
    }

    spdlog::info("Applying final position correction: {:.2f}mm error",
        error.norm() * 1000);

    // Apply increasing correction over the window
    for (size_t i = start_idx; i < samples.size(); ++i) {
        double t_in_window = samples[i].time - correction_start;
        double blend = t_in_window / correction_window;
        blend = smootherStep(blend);  // Smooth blend

        // Apply correction
        samples[i].pose.translation() += error * blend;

        // Re-solve IK for corrected position via numerical IK
        JointVector seed = (i > 0) ? samples[i - 1].joints : samples[i].joints;
        auto best = kinematics_.inverseNumerical(samples[i].pose, seed);
        if (best.has_value()) {
            samples[i].joints = *best;
        }
    }
}

// =============================================================================
// Visualization
// =============================================================================

TrajectoryVisualization VelocityFirstPlanner::generateVisualization(
    const PlannedTrajectory& trajectory,
    const std::vector<TimedWaypoint>& waypoints) const {

    TrajectoryVisualization viz;

    if (trajectory.samples.empty()) {
        return viz;
    }

    // Subsample for visualization (target ~200 points)
    size_t step = std::max<size_t>(1, trajectory.samples.size() / 200);

    // Path points
    for (size_t i = 0; i < trajectory.samples.size(); i += step) {
        const auto& sample = trajectory.samples[i];
        viz.path_points.push_back(sample.pose.translation());
        viz.path_times.push_back(sample.time);
    }

    // Timing data (same subsampling)
    for (size_t i = 0; i < trajectory.samples.size(); i += step) {
        const auto& sample = trajectory.samples[i];
        viz.times.push_back(sample.time);
        viz.speeds.push_back(sample.speed);
    }

    // Compute accelerations (finite difference)
    viz.accelerations.resize(viz.speeds.size(), 0.0);
    for (size_t i = 1; i + 1 < viz.speeds.size(); ++i) {
        double dt = viz.times[i + 1] - viz.times[i - 1];
        double dv = viz.speeds[i + 1] - viz.speeds[i - 1];
        viz.accelerations[i] = (dt > 0.001) ? dv / dt : 0.0;
    }

    // Compute jerks
    viz.jerks.resize(viz.accelerations.size(), 0.0);
    for (size_t i = 1; i + 1 < viz.accelerations.size(); ++i) {
        double dt = viz.times[i + 1] - viz.times[i - 1];
        double da = viz.accelerations[i + 1] - viz.accelerations[i - 1];
        viz.jerks[i] = (dt > 0.001) ? da / dt : 0.0;
    }

    // Waypoint markers
    for (size_t i = 0; i < waypoints.size(); ++i) {
        WaypointMarker marker;
        marker.index = i;
        marker.position = waypoints[i].position;
        marker.time = waypoints[i].time;
        marker.blend_factor = 0.0;  // Velocity-first doesn't use explicit blend factors
        marker.has_pause = false;
        viz.waypoints.push_back(marker);
    }

    // Segment times
    for (const auto& wp : waypoints) {
        viz.segment_times.push_back(wp.time);
    }

    // Summary
    viz.total_duration = trajectory.total_duration;
    viz.total_distance = 0.0;
    for (size_t i = 1; i < trajectory.samples.size(); ++i) {
        viz.total_distance += (trajectory.samples[i].pose.translation() -
            trajectory.samples[i - 1].pose.translation()).norm();
    }
    viz.max_speed = *std::max_element(viz.speeds.begin(), viz.speeds.end());

    return viz;
}

}  // namespace velocity_first
}  // namespace trajectory
}  // namespace ur_controller
