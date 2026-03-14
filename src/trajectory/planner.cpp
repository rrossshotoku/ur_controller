/// @file planner.cpp
/// @brief Trajectory planner implementation with Ruckig motion profiles

#include "ur_controller/trajectory/planner.hpp"

#include <ruckig/ruckig.hpp>
#include <algorithm>
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

PlannedTrajectory TrajectoryPlanner::plan(const std::vector<Waypoint>& waypoints) {
    // Use IK for first waypoint to get start configuration
    if (waypoints.empty()) {
        PlannedTrajectory result;
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error, "No waypoints provided", std::nullopt, std::nullopt});
        return result;
    }

    // Get IK solution for first waypoint
    auto pose = waypoints[0].toPose();
    auto solutions = kinematics_.inverse(pose);
    if (solutions.empty()) {
        PlannedTrajectory result;
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error, "First waypoint is unreachable", 0, std::nullopt});
        return result;
    }

    // Select solution within limits
    JointVector start_joints = solutions[0];
    const auto& limits = kinematics_.jointLimits();
    for (const auto& sol : solutions) {
        if (limits.withinLimits(sol)) {
            start_joints = sol;
            break;
        }
    }

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

        // Get IK solution for end waypoint
        auto end_pose = waypoints[i + 1].toPose();
        auto solutions = kinematics_.inverse(end_pose);
        if (solutions.empty()) {
            // Already caught by validation, shouldn't happen
            result.valid = false;
            return result;
        }

        auto best_sol = kinematics_.selectBestSolution(solutions, current_joints, 1.0);
        seg.end_joints = best_sol.value_or(solutions[0]);

        // Determine segment duration (will be refined by Ruckig)
        if (waypoints[i + 1].segment_time > 0.0) {
            seg.duration = waypoints[i + 1].segment_time;
        } else {
            // Auto-compute duration based on joint velocity limits
            seg.duration = calculateSegmentDuration(seg.start_joints, seg.end_joints);
        }

        // Check for blending
        seg.has_blend_in = (i > 0 && waypoints[i].blend_radius > 0.0);
        seg.has_blend_out = (waypoints[i + 1].blend_radius > 0.0);

        segments.push_back(seg);
        current_joints = seg.end_joints;
        current_time += seg.duration;
    }

    // Plan joint-space motion using Ruckig
    result.samples = planJointMotion(segments, waypoints);

    if (result.samples.empty()) {
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error, "Failed to generate trajectory samples", std::nullopt, std::nullopt});
        return result;
    }

    // Set total duration
    result.total_duration = result.samples.back().time;
    result.valid = !result.hasErrors();

    return result;
}

std::vector<TrajectorySample> TrajectoryPlanner::planJointMotion(
    const std::vector<Segment>& segments,
    const std::vector<Waypoint>& waypoints) {

    std::vector<TrajectorySample> samples;

    if (segments.empty()) {
        return samples;
    }

    // Reserve approximate number of samples
    double total_time = 0.0;
    for (const auto& seg : segments) {
        total_time += seg.duration;
    }
    // Add pause times
    for (const auto& wp : waypoints) {
        total_time += wp.pause_time;
    }
    size_t estimated_samples = static_cast<size_t>(total_time * config_.sample_rate) + 100;
    samples.reserve(estimated_samples);

    // Process each segment using Ruckig for smooth, jerk-limited motion
    double current_time = 0.0;
    JointVector current_pos = segments[0].start_joints;
    JointVector current_vel = JointVector::Zero();
    JointVector current_acc = JointVector::Zero();

    for (size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx) {
        const auto& seg = segments[seg_idx];

        // Handle pause at start of segment (except first)
        if (seg_idx > 0 && waypoints[seg.start_index].pause_time > 0.0) {
            double pause_duration = waypoints[seg.start_index].pause_time;
            double pause_end = current_time + pause_duration;

            while (current_time < pause_end) {
                samples.push_back(createSample(current_time, current_pos, JointVector::Zero()));
                current_time += 1.0 / config_.sample_rate;
            }
            // Reset velocities after pause
            current_vel = JointVector::Zero();
            current_acc = JointVector::Zero();
        }

        // Use Ruckig for this segment
        auto segment_samples = planSegmentWithRuckig(
            seg.start_joints, seg.end_joints,
            current_vel, current_acc,
            current_time);

        if (segment_samples.empty()) {
            // Fallback to smooth interpolation if Ruckig fails
            segment_samples = planSegmentSmooth(seg, current_time);
        }

        // Append segment samples
        for (const auto& sample : segment_samples) {
            samples.push_back(sample);
        }

        // Update state for next segment
        if (!segment_samples.empty()) {
            current_time = segment_samples.back().time + 1.0 / config_.sample_rate;
            current_pos = seg.end_joints;
            // Ruckig ends at zero velocity/acceleration for position control
            current_vel = JointVector::Zero();
            current_acc = JointVector::Zero();
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
    double start_time) {

    std::vector<TrajectorySample> samples;

    // Create Ruckig instance for 6 DOF
    ruckig::Ruckig<6> ruckig;
    ruckig::InputParameter<6> input;
    ruckig::Trajectory<6> trajectory;

    // Set current state
    for (size_t i = 0; i < 6; ++i) {
        auto idx = static_cast<Eigen::Index>(i);
        input.current_position[i] = start_joints[idx];
        input.current_velocity[i] = start_velocity[idx];
        input.current_acceleration[i] = start_acceleration[idx];
    }

    // Set target state (stop at end position)
    JointVector diff = jointDifference(end_joints, start_joints);
    for (size_t i = 0; i < 6; ++i) {
        auto idx = static_cast<Eigen::Index>(i);
        // Target position considering angle wrapping
        input.target_position[i] = start_joints[idx] + diff[idx];
        input.target_velocity[i] = 0.0;  // Come to a stop
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

    // Create waypoint markers
    double accumulated_time = 0.0;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        WaypointMarker marker;
        marker.index = i;
        marker.position = waypoints[i].position;
        marker.blend_radius = waypoints[i].blend_radius;
        marker.has_pause = waypoints[i].pause_time > 0.0;

        // Find approximate time when this waypoint is reached
        if (i == 0) {
            marker.time = 0.0;
        } else {
            accumulated_time += waypoints[i].segment_time > 0 ?
                waypoints[i].segment_time : 1.0;  // Approximate
            marker.time = std::min(accumulated_time, trajectory.total_duration);
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

}  // namespace trajectory
}  // namespace ur_controller
