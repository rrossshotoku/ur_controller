/// @file planner.cpp
/// @brief Trajectory planner implementation with Ruckig motion profiles

#include "ur_controller/trajectory/planner.hpp"

#include <ruckig/ruckig.hpp>
#include <spdlog/spdlog.h>
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

/// @brief Per-joint maximum allowed change for continuous motion (radians)
/// J1, J2, J3, J5 are "configuration" joints - large changes indicate arm reconfiguration
/// J4, J6 are "free spin" joints - can rotate freely to orient wrist/tool
constexpr std::array<double, 6> kMaxJointChange = {
    1.57,  // J1 (base): ~90° - large change means reaching around
    1.57,  // J2 (shoulder): ~90° - large change affects overall arm pose
    2.09,  // J3 (elbow): ~120° - elbow flip is ~180°
    3.14,  // J4 (wrist 1): ~180° - free rotation, allow full spin
    1.57,  // J5 (wrist 2): ~90° - wrist flip is ~180°
    3.14   // J6 (flange): ~180° - free rotation, allow full spin
};

/// @brief Find an IK solution that can be reached with continuous joint motion
/// @param solutions All IK solutions for the target pose
/// @param previous Previous joint positions
/// @return The closest solution within per-joint thresholds, or nullopt
///
/// This function uses per-joint thresholds to detect configuration changes.
/// "Free spin" joints (J4, J6) allow large rotations for wrist/tool orientation.
/// "Configuration" joints (J1, J2, J3, J5) have stricter limits to catch real flips.
std::optional<JointVector> findContinuousSolution(
    const std::vector<JointVector>& solutions,
    const JointVector& previous) {

    if (solutions.empty()) {
        return std::nullopt;
    }

    // Find the solution with minimum total joint motion that respects per-joint limits
    std::optional<JointVector> best;
    double best_total_dist = std::numeric_limits<double>::max();

    for (const auto& sol : solutions) {
        bool within_limits = true;
        double total_dist = 0.0;

        for (int j = 0; j < 6; ++j) {
            double diff = std::abs(wrapAngle(sol[j] - previous[j]));
            total_dist += diff * diff;

            // Check against per-joint threshold
            if (diff > kMaxJointChange[static_cast<size_t>(j)]) {
                within_limits = false;
                break;
            }
        }

        // Only consider solutions within per-joint limits
        if (within_limits) {
            // Among valid solutions, prefer the one with smallest total motion
            if (!best.has_value() || total_dist < best_total_dist) {
                best = sol;
                best_total_dist = total_dist;
            }
        }
    }

    return best;
}

/// @brief Check if a configuration is viable along a linear path between two poses
/// @param start_pos Start position
/// @param end_pos End position
/// @param orientation Orientation to use (typically from start waypoint)
/// @param config Configuration to test
/// @param kinematics Kinematics instance
/// @param num_samples Number of points to sample along path
/// @return true if configuration is viable for entire path
bool isConfigViableAlongPath(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& end_pos,
    const Eigen::Quaterniond& orientation,
    const kinematics::ArmConfiguration& config,
    const kinematics::URKinematics& kinematics,
    int num_samples = 10) {

    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        Eigen::Vector3d pos = start_pos + t * (end_pos - start_pos);

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = pos;
        pose.linear() = orientation.toRotationMatrix();

        auto solutions = kinematics.inverse(pose);
        auto same_config = kinematics.filterByConfiguration(solutions, config);

        if (same_config.empty()) {
            return false;  // No solution in this configuration at this point
        }
    }
    return true;
}

/// @brief Find a configuration that is viable for the entire trajectory
/// @param waypoints The waypoints to plan through
/// @param kinematics Kinematics instance
/// @return Pair of (viable_start_joints, viable_config), or nullopt if none found
std::optional<std::pair<JointVector, kinematics::ArmConfiguration>> findViableConfiguration(
    const std::vector<Waypoint>& waypoints,
    const kinematics::URKinematics& kinematics) {

    if (waypoints.empty()) {
        return std::nullopt;
    }

    // Get all IK solutions for first waypoint
    auto first_pose = waypoints[0].toPose();
    auto first_solutions = kinematics.inverse(first_pose);
    if (first_solutions.empty()) {
        return std::nullopt;
    }

    // Get unique configurations from first waypoint solutions
    std::vector<std::pair<kinematics::ArmConfiguration, JointVector>> configs_to_try;
    for (const auto& sol : first_solutions) {
        auto config = kinematics.getConfiguration(sol);
        // Check if we already have this configuration
        bool already_have = false;
        for (const auto& [existing_config, _] : configs_to_try) {
            if (existing_config == config) {
                already_have = true;
                break;
            }
        }
        if (!already_have) {
            configs_to_try.emplace_back(config, sol);
        }
    }

    spdlog::info("Testing {} unique configurations for viability", configs_to_try.size());

    // Test each configuration against all segments
    for (const auto& [config, start_joints] : configs_to_try) {
        bool config_viable = true;

        // Check each segment
        for (size_t i = 0; i + 1 < waypoints.size() && config_viable; ++i) {
            const auto& wp_start = waypoints[i];
            const auto& wp_end = waypoints[i + 1];

            // Use the orientation from the starting waypoint for consistency
            if (!isConfigViableAlongPath(
                    wp_start.position,
                    wp_end.position,
                    wp_start.orientation,
                    config,
                    kinematics,
                    10)) {
                config_viable = false;
                spdlog::debug("Config (s={} e={} w={}) fails on segment {}",
                    config.shoulder, config.elbow, config.wrist, i);
            }
        }

        if (config_viable) {
            spdlog::info("Found viable configuration: shoulder={} elbow={} wrist={}",
                config.shoulder, config.elbow, config.wrist);
            return std::make_pair(start_joints, config);
        }
    }

    spdlog::warn("No single configuration is viable for entire trajectory");
    return std::nullopt;
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

    if (config_.ik_method == IKMethod::Numerical) {
        // Numerical IK using damped least squares
        spdlog::info("solveIK: attempting numerical IK from seed J6={:.1f}°",
            seed[5] * 180.0 / 3.14159);
        auto result = kinematics_.inverseNumerical(pose, seed);
        if (!result.has_value()) {
            spdlog::warn("solveIK: numerical IK did not converge");
        }
        if (result.has_value()) {
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
            spdlog::info("solveIK: numerical IK succeeded, J6={:.1f}°",
                (*result)[5] * 180.0 / 3.14159);

            // Log joint changes for debugging
            for (int i = 0; i < 6; ++i) {
                double diff = kinematics::URKinematics::wrapAngle((*result)[i] - seed[i]);
                if (std::abs(diff) > 0.35) {  // More than ~20 degrees
                    spdlog::info("Numerical IK: J{} changed by {:.1f}° (seed={:.1f}°, result={:.1f}°)",
                        i + 1, diff * 180.0 / 3.14159,
                        seed[i] * 180.0 / 3.14159,
                        (*result)[i] * 180.0 / 3.14159);
                }
            }
        }
        return result;
    } else {
        // Analytical IK with continuous solution selection
        auto solutions = kinematics_.inverse(pose);

        if (solutions.empty()) {
            return std::nullopt;
        }

        // Filter out numerically corrupt solutions using FK verification
        constexpr double kMaxFKError = 0.005;  // 5mm tolerance
        Eigen::Vector3d target_pos = pose.translation();

        std::vector<JointVector> valid_solutions;
        valid_solutions.reserve(solutions.size());

        for (const auto& sol : solutions) {
            auto fk_check = kinematics_.forward(sol);
            double pos_error = (fk_check.translation() - target_pos).norm();
            if (pos_error <= kMaxFKError) {
                valid_solutions.push_back(sol);
            }
        }

        if (valid_solutions.empty()) {
            spdlog::warn("All analytical IK solutions failed FK verification");
            return std::nullopt;
        }

        // Find continuous solution within per-joint thresholds
        return findContinuousSolution(valid_solutions, seed);
    }
}

// =============================================================================
// Planning
// =============================================================================

PlannedTrajectory TrajectoryPlanner::plan(const std::vector<Waypoint>& waypoints) {
    // Use IK for first waypoint to get start configuration
    if (waypoints.empty()) {
        PlannedTrajectory result;
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error, "No waypoints provided", std::nullopt, std::nullopt});
        return result;
    }

    // Find a configuration that is viable for the entire trajectory
    auto viable = findViableConfiguration(waypoints, kinematics_);
    if (!viable.has_value()) {
        // Fall back to checking just the first waypoint
        auto pose = waypoints[0].toPose();
        auto solutions = kinematics_.inverse(pose);
        if (solutions.empty()) {
            PlannedTrajectory result;
            result.valid = false;
            result.messages.push_back({ValidationSeverity::Error, "First waypoint is unreachable", 0, std::nullopt});
            return result;
        }

        // No viable configuration for entire trajectory
        PlannedTrajectory result;
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error,
            "No arm configuration is viable for the entire trajectory. "
            "The path crosses configuration boundaries that cannot be avoided. "
            "Try repositioning waypoints to stay within one arm configuration.",
            std::nullopt, std::nullopt});
        return result;
    }

    auto [start_joints, viable_config] = *viable;

    // Verify start joints are within limits
    const auto& limits = kinematics_.jointLimits();
    if (!limits.withinLimits(start_joints)) {
        // Try to find another solution in the same config that's within limits
        auto pose = waypoints[0].toPose();
        auto solutions = kinematics_.inverse(pose);
        auto same_config_sols = kinematics_.filterByConfiguration(solutions, viable_config);

        bool found_valid = false;
        for (const auto& sol : same_config_sols) {
            if (limits.withinLimits(sol)) {
                start_joints = sol;
                found_valid = true;
                break;
            }
        }

        if (!found_valid) {
            PlannedTrajectory result;
            result.valid = false;
            result.messages.push_back({ValidationSeverity::Error,
                "Viable configuration exists but all solutions exceed joint limits",
                0, std::nullopt});
            return result;
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
            result.valid = false;
            result.messages.push_back({ValidationSeverity::Error,
                "Waypoint " + std::to_string(i + 2) + " is unreachable - no IK solution",
                i + 1, std::nullopt});
            return result;
        }

        // Select best IK solution for endpoint - allow configuration changes but warn
        // The velocity limits during execution will protect against unsafe motion
        auto best_sol = kinematics_.selectBestSolution(solutions, current_joints, 2.0);

        if (!best_sol.has_value()) {
            best_sol = solutions[0];  // Use first solution as fallback
        }

        seg.end_joints = *best_sol;

        // Warn if configuration is different (but don't reject)
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
        seg.has_blend_in = (i > 0 && waypoints[i].blend_radius > 0.0);
        seg.has_blend_out = (waypoints[i + 1].blend_radius > 0.0 && i + 2 < waypoints.size());

        spdlog::info("Segment {}: has_blend_out={}, wp[{}].blend_radius={:.4f}, i+2<size={}",
            i, seg.has_blend_out, i + 1, waypoints[i + 1].blend_radius, i + 2 < waypoints.size());

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
    result.samples = planJointMotion(segments, waypoints);

    if (result.samples.empty()) {
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error,
            "Linear path not achievable - trajectory crosses a singularity or requires IK configuration change. "
            "Try moving waypoints to avoid shoulder/wrist singularities, or reduce blend radius.",
            std::nullopt, std::nullopt});
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

    // Process each segment
    double current_time = 0.0;
    JointVector current_pos = segments[0].start_joints;
    JointVector current_vel = JointVector::Zero();
    JointVector current_acc = JointVector::Zero();

    // Track if we're coming from a blend arc (affects start position of next segment)
    bool coming_from_blend = false;
    JointVector blend_end_joints = JointVector::Zero();
    std::optional<BlendArcInfo> prev_blend_arc;  // Arc info from previous segment for second half

    for (size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx) {
        const auto& seg = segments[seg_idx];

        // Handle pause at start of segment (except first)
        if (seg_idx > 0 && waypoints[seg.start_index].pause_time > 0.0 && !coming_from_blend) {
            double pause_duration = waypoints[seg.start_index].pause_time;
            double pause_end = current_time + pause_duration;

            while (current_time < pause_end) {
                samples.push_back(createSample(current_time, current_pos, JointVector::Zero()));
                current_time += 1.0 / config_.sample_rate;
            }
            current_vel = JointVector::Zero();
            current_acc = JointVector::Zero();
        }

        // Determine actual start and end positions
        // If coming from a blend, start from blend arc end instead of waypoint
        // Use actual joints from previous segment, not pre-computed start_joints
        JointVector actual_start = coming_from_blend ? blend_end_joints : current_pos;

        // Check if this segment needs a blend arc at the end
        // Compute blend geometry on-the-fly using actual current joints
        bool needs_blend = (waypoints[seg.end_index].blend_radius > 0.0) &&
                           (seg_idx + 1 < segments.size()) &&
                           (waypoints[seg.end_index].pause_time <= 0.0);

        std::optional<BlendArcInfo> blend_arc;
        if (needs_blend) {
            // Compute blend geometry using actual current joints
            blend_arc = computeBlendGeometry(
                waypoints[seg.start_index],
                waypoints[seg.end_index],
                waypoints[seg.end_index + 1],
                actual_start);
        }

        if (blend_arc.has_value()) {
            const auto& arc = *blend_arc;

            // Plan segment 1: linear portion + first half of arc (unified, evenly spaced)
            Eigen::Isometry3d start_pose = kinematics_.forward(actual_start);

            // Use segment duration for the combined path
            double segment_duration = seg.duration;
            if (segment_duration <= 0.0) {
                // Auto-calculate based on distance and max velocity
                Eigen::Vector3d start_pos = start_pose.translation();
                double linear_distance = (arc.arc_start - start_pos).norm();
                double half_arc_length = arc.arc_length / 2.0;
                segment_duration = (linear_distance + half_arc_length) / config_.max_linear_velocity;
                segment_duration = std::max(segment_duration, 0.1);
            }

            spdlog::info("Blend segment: duration={:.2f}, arc_length={:.3f}",
                segment_duration, arc.arc_length);

            auto segment_samples = planSegmentWithHalfArc(
                start_pose, actual_start, arc,
                current_time, segment_duration,
                true);  // first_half = true

            // Check for planning failure
            if (segment_samples.empty()) {
                spdlog::error("Failed to plan segment {} (blend arc entry) - linear path not achievable",
                    seg_idx);
                return {};  // Return empty to signal failure
            }

            // Append samples
            for (const auto& sample : segment_samples) {
                samples.push_back(sample);
            }

            current_time = segment_samples.back().time + 1.0 / config_.sample_rate;
            current_pos = segment_samples.back().joints;
            current_vel = JointVector::Zero();
            current_acc = JointVector::Zero();

            // Mark that the next segment should continue from arc midpoint
            coming_from_blend = true;
            blend_end_joints = current_pos;  // Joints at arc midpoint
            prev_blend_arc = arc;  // Store arc info for next segment's second half

        } else {
            // No blend arc at end - plan to the waypoint
            // Get end pose from waypoint directly (NOT from pre-computed joints)
            Eigen::Isometry3d end_pose = waypoints[seg.end_index].toPose();
            Eigen::Vector3d end_pos = end_pose.translation();

            spdlog::info("Segment {}: target pos=[{:.4f}, {:.4f}, {:.4f}]",
                seg_idx, end_pos.x(), end_pos.y(), end_pos.z());

            if (coming_from_blend && prev_blend_arc.has_value()) {
                // Coming from a blend: plan second half of arc + linear to waypoint
                const auto& arc = *prev_blend_arc;

                // Segment duration covers: second half of arc + linear portion
                double segment_duration = seg.duration;
                if (segment_duration <= 0.0) {
                    // Auto-calculate
                    double half_arc_length = arc.arc_length / 2.0;
                    double linear_distance = (end_pos - arc.arc_end).norm();
                    segment_duration = (half_arc_length + linear_distance) / config_.max_linear_velocity;
                    segment_duration = std::max(segment_duration, 0.1);
                }

                spdlog::info("Post-blend segment: duration={:.2f}", segment_duration);

                // Start from arc midpoint (stored in actual_start/blend_end_joints)
                Eigen::Isometry3d start_pose = kinematics_.forward(actual_start);

                auto segment_samples = planSegmentWithHalfArc(
                    start_pose, actual_start, arc,
                    current_time, segment_duration,
                    false,  // first_half = false (second half of arc)
                    end_pose);

                // Check for planning failure
                if (segment_samples.empty()) {
                    spdlog::error("Failed to plan segment {} (blend arc exit) - linear path not achievable",
                        seg_idx);
                    return {};  // Return empty to signal failure
                }

                // Append segment samples
                for (const auto& sample : segment_samples) {
                    samples.push_back(sample);
                }

                current_time = segment_samples.back().time + 1.0 / config_.sample_rate;
                current_pos = segment_samples.back().joints;  // Use actual computed joints
                current_vel = JointVector::Zero();
                current_acc = JointVector::Zero();

                prev_blend_arc = std::nullopt;
            } else {
                // Normal segment - just plan linear to waypoint
                Eigen::Isometry3d start_pose = kinematics_.forward(actual_start);

                double segment_duration = seg.duration;
                if (segment_duration <= 0.0) {
                    double linear_distance = (end_pos - start_pose.translation()).norm();
                    segment_duration = linear_distance / config_.max_linear_velocity;
                    segment_duration = std::max(segment_duration, 0.1);
                }

                auto segment_samples = planSegmentLinearCartesian(
                    start_pose, end_pose,
                    actual_start,
                    current_time, segment_duration,
                    false);  // S-curve to decelerate at end

                // Check for planning failure
                if (segment_samples.empty()) {
                    spdlog::error("Failed to plan segment {} - linear path not achievable", seg_idx);
                    return {};  // Return empty to signal failure
                }

                // Append segment samples
                for (const auto& sample : segment_samples) {
                    samples.push_back(sample);
                }

                current_time = segment_samples.back().time + 1.0 / config_.sample_rate;
                current_pos = segment_samples.back().joints;  // Use actual computed joints
                current_vel = JointVector::Zero();
                current_acc = JointVector::Zero();
            }

            coming_from_blend = false;
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
    bool constant_velocity) {

    std::vector<TrajectorySample> samples;

    if (duration <= 0.0) {
        duration = 1.0;  // Default to 1 second if not specified
    }

    // Log the segment planning
    Eigen::Vector3d dbg_end_pos = end_pose.translation();
    spdlog::info("planSegmentLinearCartesian: start J6={:.1f}°, end_pos=[{:.4f}, {:.4f}, {:.4f}]",
        start_joints[5] * 180.0 / M_PI,
        dbg_end_pos.x(), dbg_end_pos.y(), dbg_end_pos.z());

    // Extract positions
    Eigen::Vector3d start_pos = start_pose.translation();
    Eigen::Vector3d end_pos = end_pose.translation();
    Eigen::Vector3d pos_diff = end_pos - start_pos;

    // Extract orientations as quaternions
    Eigen::Quaterniond start_quat(start_pose.rotation());
    Eigen::Quaterniond end_quat(end_pose.rotation());

    // Ensure shortest path for SLERP
    if (start_quat.dot(end_quat) < 0.0) {
        end_quat.coeffs() = -end_quat.coeffs();
    }

    // Generate samples
    double sample_period = 1.0 / config_.sample_rate;
    int num_samples = static_cast<int>(duration * config_.sample_rate) + 1;
    if (num_samples < 2) num_samples = 2;

    // Track previous joints - angles accumulate naturally
    JointVector prev_joints = start_joints;

    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        double sample_time = start_time + t * duration;

        // Choose velocity profile:
        // - constant_velocity=true: linear interpolation (for blending into arcs)
        // - constant_velocity=false: S-curve with accel/decel (for stopping at waypoints)
        double s = constant_velocity ? t : smoothStep(t);

        // Interpolate position linearly (with S-curve timing)
        Eigen::Vector3d pos = start_pos + s * pos_diff;

        // Interpolate orientation with SLERP (with S-curve timing)
        Eigen::Quaterniond quat = start_quat.slerp(s, end_quat);

        // Create pose
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = pos;
        pose.linear() = quat.toRotationMatrix();

        // Solve IK and select solution closest to previous sample
        JointVector joints;
        if (config_.ik_method == IKMethod::Numerical) {
            // Numerical IK seeds from previous joints
            auto ik_solution = kinematics_.inverseNumerical(pose, prev_joints);
            if (!ik_solution.has_value()) {
                spdlog::error("Linear path unreachable at t={:.3f}, pos=[{:.3f}, {:.3f}, {:.3f}] - "
                    "numerical IK failed",
                    t, pos.x(), pos.y(), pos.z());
                return {};
            }
            joints = *ik_solution;
        } else {
            // Analytical IK: get all solutions and pick closest to previous
            auto solutions = kinematics_.inverse(pose);
            if (solutions.empty()) {
                spdlog::error("Linear path unreachable at t={:.3f}, pos=[{:.3f}, {:.3f}, {:.3f}] - "
                    "no IK solution found",
                    t, pos.x(), pos.y(), pos.z());
                return {};
            }

            // Select solution closest to previous sample (continuous motion)
            auto best = kinematics_.selectBestSolution(solutions, prev_joints, 1.0);
            joints = best.value_or(solutions[0]);
        }

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
            spdlog::info("  Sample {} (t={:.2f}): J6={:.1f}°",
                i, t, joints[5] * 180.0 / M_PI);
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
                spdlog::warn("Near singularity at t={:.3f}, pos=[{:.3f}, {:.3f}, {:.3f}] - "
                    "joint velocity {:.1f} rad/s exceeds limit. Consider slower motion.",
                    t, pos.x(), pos.y(), pos.z(), joint_velocity);
            }
        }

        // Compute joint velocities (finite difference)
        JointVector velocities = JointVector::Zero();
        if (i > 0 && !samples.empty()) {
            for (size_t j = 0; j < 6; ++j) {
                auto idx = static_cast<Eigen::Index>(j);
                velocities[idx] = (joints[idx] - samples.back().joints[idx]) / sample_period;
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
    const Eigen::Isometry3d& end_pose) {

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

    // Ensure shortest path for SLERP
    if (linear_start_quat.dot(linear_end_quat) < 0.0) {
        linear_end_quat.coeffs() = -linear_end_quat.coeffs();
    }

    // Arc geometry
    Eigen::Vector3d v_start = arc_info.arc_start - arc_info.center;
    double half_arc_angle = arc_info.arc_angle / 2.0;

    // =========================================================================
    // Singularity preview: Find max condition number along path
    // =========================================================================
    constexpr int kPreviewSamples = 30;
    double max_condition_number = 1.0;
    JointVector preview_joints = start_joints;

    for (int i = 0; i < kPreviewSamples; ++i) {
        double t = static_cast<double>(i) / (kPreviewSamples - 1);
        double dist_along_path = t * total_distance;

        Eigen::Vector3d pos;
        Eigen::Quaterniond quat;

        if (first_half) {
            if (dist_along_path <= linear_distance && linear_distance > 0.001) {
                double linear_t = dist_along_path / linear_distance;
                pos = linear_start + linear_t * (linear_end - linear_start);
                quat = linear_start_quat.slerp(linear_t, linear_end_quat);
            } else {
                double arc_dist = dist_along_path - linear_distance;
                double arc_t = (half_arc_length > 0.001) ? (arc_dist / half_arc_length) : 0.0;
                arc_t = std::clamp(arc_t, 0.0, 1.0);
                double angle = arc_t * half_arc_angle;
                Eigen::Vector3d v_rotated =
                    v_start * std::cos(angle) +
                    arc_info.normal.cross(v_start) * std::sin(angle) +
                    arc_info.normal * arc_info.normal.dot(v_start) * (1 - std::cos(angle));
                pos = arc_info.center + v_rotated;
                quat = arc_info.start_orientation.slerp(arc_t * 0.5, arc_info.end_orientation);
            }
        } else {
            if (dist_along_path <= half_arc_length && half_arc_length > 0.001) {
                double arc_t = dist_along_path / half_arc_length;
                double angle = half_arc_angle + arc_t * half_arc_angle;
                Eigen::Vector3d v_rotated =
                    v_start * std::cos(angle) +
                    arc_info.normal.cross(v_start) * std::sin(angle) +
                    arc_info.normal * arc_info.normal.dot(v_start) * (1 - std::cos(angle));
                pos = arc_info.center + v_rotated;
                quat = arc_info.start_orientation.slerp(0.5 + arc_t * 0.5, arc_info.end_orientation);
            } else {
                double linear_dist = dist_along_path - half_arc_length;
                double linear_t = (linear_distance > 0.001) ? (linear_dist / linear_distance) : 1.0;
                linear_t = std::clamp(linear_t, 0.0, 1.0);
                pos = linear_start + linear_t * (linear_end - linear_start);
                quat = linear_start_quat.slerp(linear_t, linear_end_quat);
            }
        }

        Eigen::Isometry3d preview_pose = Eigen::Isometry3d::Identity();
        preview_pose.translation() = pos;
        preview_pose.linear() = quat.toRotationMatrix();

        auto ik_solution = solveIK(preview_pose, preview_joints);
        if (!ik_solution.has_value()) {
            spdlog::error("Blend path unreachable at position [{:.3f}, {:.3f}, {:.3f}] "
                "(t={:.1f}%). No IK solution found.",
                pos.x(), pos.y(), pos.z(), static_cast<double>(i) / (kPreviewSamples - 1) * 100.0);
            return {};
        }
        preview_joints = *ik_solution;
        auto singularity_info = kinematics_.analyzeSingularity(preview_joints);
        max_condition_number = std::max(max_condition_number, singularity_info.condition_number);
    }

    // Compute time scaling
    double nominal_tcp_velocity = total_distance / duration;
    double max_allowed_tcp_velocity = config_.max_joint_velocity / max_condition_number;
    double time_scale = 1.0;

    if (nominal_tcp_velocity > max_allowed_tcp_velocity && max_allowed_tcp_velocity > 0.0) {
        time_scale = nominal_tcp_velocity / max_allowed_tcp_velocity;
        constexpr double kMaxTimeScale = 100.0;
        if (time_scale > kMaxTimeScale) {
            time_scale = kMaxTimeScale;
        }
    }

    double scaled_duration = duration * time_scale;

    // Calculate number of samples with scaled duration
    double sample_period = 1.0 / config_.sample_rate;
    int num_samples = static_cast<int>(scaled_duration * config_.sample_rate) + 1;
    if (num_samples < 2) num_samples = 2;

    // Track previous joints - angles accumulate naturally
    JointVector prev_joints = start_joints;

    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        double sample_time = start_time + t * scaled_duration;

        // Distance along the combined path
        double dist_along_path = t * total_distance;

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
            spdlog::error("Blend path unreachable at t={:.3f}, pos=[{:.3f}, {:.3f}, {:.3f}] - "
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

        // Compute velocities
        JointVector velocities = JointVector::Zero();
        if (i > 0 && !samples.empty()) {
            for (size_t j = 0; j < 6; ++j) {
                auto idx = static_cast<Eigen::Index>(j);
                velocities[idx] = (joints[idx] - samples.back().joints[idx]) / sample_period;
            }
        }

        samples.push_back(createSample(sample_time, joints, velocities));
        prev_joints = joints;  // Track for next iteration
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
    double blend_radius = blend_wp.blend_radius;

    spdlog::info("computeBlendGeometry: blend_radius={:.4f}", blend_radius);
    spdlog::info("  prev=({:.3f},{:.3f},{:.3f}), blend=({:.3f},{:.3f},{:.3f}), next=({:.3f},{:.3f},{:.3f})",
        p_prev.x(), p_prev.y(), p_prev.z(),
        p_blend.x(), p_blend.y(), p_blend.z(),
        p_next.x(), p_next.y(), p_next.z());

    // Compute incoming and outgoing direction vectors
    Eigen::Vector3d dir_in = (p_blend - p_prev);
    Eigen::Vector3d dir_out = (p_next - p_blend);

    double dist_in = dir_in.norm();
    double dist_out = dir_out.norm();

    spdlog::info("  dist_in={:.4f}, dist_out={:.4f}", dist_in, dist_out);

    // Check if blend radius is feasible (not larger than half the segment lengths)
    if (blend_radius > dist_in * 0.5 || blend_radius > dist_out * 0.5) {
        // Blend radius too large - reduce it
        double old_radius = blend_radius;
        blend_radius = std::min(dist_in * 0.5, dist_out * 0.5);
        spdlog::info("  Reduced blend_radius from {:.4f} to {:.4f}", old_radius, blend_radius);
    }

    if (blend_radius < 0.001) {
        spdlog::warn("  Blend radius too small ({:.4f}), skipping blend", blend_radius);
        return std::nullopt;  // Blend radius too small
    }

    dir_in.normalize();
    dir_out.normalize();

    // Calculate the angle between incoming and outgoing directions
    double cos_angle = dir_in.dot(dir_out);
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    double angle = std::acos(cos_angle);  // Angle between directions

    spdlog::info("  Path angle={:.2f} degrees", angle * 180.0 / kPi);

    // If paths are nearly collinear, no blend needed
    if (angle < 0.01 || angle > kPi - 0.01) {
        spdlog::warn("  Paths nearly collinear (angle={:.4f}), skipping blend", angle);
        return std::nullopt;
    }

    // Arc start and end points
    Eigen::Vector3d arc_start = p_blend - blend_radius * dir_in;
    Eigen::Vector3d arc_end = p_blend + blend_radius * dir_out;

    spdlog::info("  arc_start=({:.3f},{:.3f},{:.3f}), arc_end=({:.3f},{:.3f},{:.3f})",
        arc_start.x(), arc_start.y(), arc_start.z(),
        arc_end.x(), arc_end.y(), arc_end.z());

    // The arc angle is the supplement of the path angle
    // (arc sweeps from incoming direction to outgoing direction)
    double arc_angle = kPi - angle;

    spdlog::info("  arc_angle={:.2f} degrees", arc_angle * 180.0 / kPi);

    // Arc radius using geometry:
    // The arc connects arc_start to arc_end, tangent to both directions
    // Arc radius = blend_radius / sin(arc_angle/2)
    double sin_half = std::sin(arc_angle / 2.0);
    spdlog::info("  sin_half={:.4f}", sin_half);
    if (std::abs(sin_half) < 0.001) {
        spdlog::warn("  Degenerate arc (sin_half={:.4f}), skipping blend", sin_half);
        return std::nullopt;  // Degenerate case
    }
    double arc_radius = blend_radius / sin_half;

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
    spdlog::info("  Arc IK solved using {} method",
        config_.ik_method == IKMethod::Numerical ? "numerical" : "analytical");

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

    // =========================================================================
    // Singularity preview: Find max condition number along arc
    // =========================================================================
    constexpr int kPreviewSamples = 20;
    double max_condition_number = 1.0;
    JointVector preview_joints = arc_info.start_joints;

    for (int i = 0; i < kPreviewSamples; ++i) {
        double t = static_cast<double>(i) / (kPreviewSamples - 1);
        double current_angle = t * arc_info.arc_angle;

        Eigen::Vector3d v_rotated =
            v_start * std::cos(current_angle) +
            arc_info.normal.cross(v_start) * std::sin(current_angle) +
            arc_info.normal * arc_info.normal.dot(v_start) * (1 - std::cos(current_angle));

        Eigen::Vector3d arc_point = arc_info.center + v_rotated;
        Eigen::Quaterniond orientation = arc_info.start_orientation.slerp(t, arc_info.end_orientation);

        Eigen::Isometry3d preview_pose = Eigen::Isometry3d::Identity();
        preview_pose.translation() = arc_point;
        preview_pose.linear() = orientation.toRotationMatrix();

        auto ik_solution = solveIK(preview_pose, preview_joints);
        if (!ik_solution.has_value()) {
            spdlog::error("Blend arc unreachable at position [{:.3f}, {:.3f}, {:.3f}] "
                "(t={:.1f}%). No IK solution found.",
                arc_point.x(), arc_point.y(), arc_point.z(), t * 100.0);
            return {};
        }
        preview_joints = *ik_solution;
        auto singularity_info = kinematics_.analyzeSingularity(preview_joints);
        max_condition_number = std::max(max_condition_number, singularity_info.condition_number);
    }

    // Compute time scaling
    double nominal_tcp_velocity = arc_info.arc_length / arc_duration;
    double max_allowed_tcp_velocity = config_.max_joint_velocity / max_condition_number;
    double time_scale = 1.0;

    if (nominal_tcp_velocity > max_allowed_tcp_velocity && max_allowed_tcp_velocity > 0.0) {
        time_scale = nominal_tcp_velocity / max_allowed_tcp_velocity;
        constexpr double kMaxTimeScale = 100.0;
        if (time_scale > kMaxTimeScale) {
            time_scale = kMaxTimeScale;
        }
    }

    double scaled_duration = arc_duration * time_scale;

    int num_samples = static_cast<int>(scaled_duration * config_.sample_rate);
    if (num_samples < 2) num_samples = 2;

    double sample_period = scaled_duration / (num_samples - 1);

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
        wp.blend_radius = sw.blend_radius;
        wp.segment_time = sw.segment_time;
        wp.pause_time = sw.pause_time;
        wp.joints = sw.joints;  // Copy saved joint configuration
        waypoints.push_back(wp);
    }

    // Validate all waypoints can be reached with continuous joint motion
    // This replaces strict configuration checking - we allow joints to cross
    // arbitrary angular boundaries as long as the motion is smooth
    JointVector current_joints = entry_joints;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        auto pose = waypoints[i].toPose();
        auto solutions = kinematics_.inverse(pose);

        if (solutions.empty()) {
            result.valid = false;
            result.messages.push_back({ValidationSeverity::Error,
                "Waypoint " + std::to_string(i + 1) + " is unreachable (no IK solution)",
                i, std::nullopt});
            return result;
        }

        // Find a solution reachable with continuous motion (max ~120° per joint)
        auto continuous_solution = findContinuousSolution(solutions, current_joints);

        if (!continuous_solution.has_value()) {
            // No continuous solution - this is a real reconfiguration
            result.valid = false;
            result.messages.push_back({ValidationSeverity::Error,
                "Waypoint " + std::to_string(i + 1) + " requires arm reconfiguration "
                "(large motion on J1/J2/J3/J5). "
                "Add intermediate waypoints or a setup pose to create a smoother path.",
                i, std::nullopt});
            return result;
        }

        current_joints = *continuous_solution;

        // Log if configuration changed (informational)
        auto wp_config = kinematics_.getConfiguration(current_joints);
        if (wp_config != entry_config) {
            spdlog::info("WP{}: Config changed to shoulder={} elbow={} wrist={} (continuous motion OK)",
                i + 1, wp_config.shoulder, wp_config.elbow, wp_config.wrist);
        }
    }

    // Build segments with endpoint IK pre-computation (for duration estimation)
    // The actual path IK is solved incrementally in planSegmentLinearCartesian
    std::vector<Segment> segments;
    current_joints = entry_joints;
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
            // No saved joints - solve IK
            auto ik_solution = solveIK(end_pose, current_joints);
            spdlog::info("  Segment {} endpoint IK using {} method",
                i, config_.ik_method == IKMethod::Numerical ? "numerical" : "analytical");

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
        seg.has_blend_in = (i > 0 && waypoints[i].blend_radius > 0.0);
        seg.has_blend_out = (waypoints[i + 1].blend_radius > 0.0 && i + 2 < waypoints.size());

        // Blend arc geometry computed during planning with actual joints
        seg.blend_arc = std::nullopt;

        segments.push_back(seg);
        current_joints = seg.end_joints;  // Chain the seed for next segment
        current_time += seg.duration;
    }

    // Plan joint-space motion
    result.samples = planJointMotion(segments, waypoints);

    if (result.samples.empty()) {
        result.valid = false;
        result.messages.push_back({ValidationSeverity::Error,
            "Linear path planning failed - path may cross singularity within locked configuration",
            std::nullopt, std::nullopt});
        return result;
    }

    result.total_duration = result.samples.back().time;
    result.valid = true;

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
