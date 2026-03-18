/// @file validator.cpp
/// @brief Trajectory validation implementation

#include "ur_controller/trajectory/validator.hpp"

#include <cmath>
#include <algorithm>

namespace ur_controller {
namespace trajectory {

namespace {

constexpr double kPi = 3.14159265358979323846;

/// @brief Format a validation message
ValidationMessage makeMessage(ValidationSeverity severity,
                               const std::string& msg,
                               std::optional<size_t> waypoint = std::nullopt,
                               std::optional<double> time = std::nullopt) {
    ValidationMessage m;
    m.severity = severity;
    m.message = msg;
    m.waypoint_index = waypoint;
    m.time = time;
    return m;
}

}  // namespace

// =============================================================================
// TrajectoryValidation
// =============================================================================

bool TrajectoryValidation::hasErrors() const {
    return std::any_of(all_messages.begin(), all_messages.end(),
        [](const ValidationMessage& m) {
            return m.severity == ValidationSeverity::Error;
        });
}

bool TrajectoryValidation::hasWarnings() const {
    return std::any_of(all_messages.begin(), all_messages.end(),
        [](const ValidationMessage& m) {
            return m.severity == ValidationSeverity::Warning;
        });
}

size_t TrajectoryValidation::errorCount() const {
    return static_cast<size_t>(std::count_if(all_messages.begin(), all_messages.end(),
        [](const ValidationMessage& m) {
            return m.severity == ValidationSeverity::Error;
        }));
}

size_t TrajectoryValidation::warningCount() const {
    return static_cast<size_t>(std::count_if(all_messages.begin(), all_messages.end(),
        [](const ValidationMessage& m) {
            return m.severity == ValidationSeverity::Warning;
        }));
}

// =============================================================================
// TrajectoryValidator
// =============================================================================

TrajectoryValidator::TrajectoryValidator(const kinematics::URKinematics& kinematics)
    : kinematics_(kinematics), config_{} {}

TrajectoryValidator::TrajectoryValidator(const kinematics::URKinematics& kinematics,
                                         const ValidationConfig& config)
    : kinematics_(kinematics), config_(config) {}

TrajectoryValidation TrajectoryValidator::validate(
    const std::vector<Waypoint>& waypoints,
    const TrajectoryConfig& trajectory_config) const {

    TrajectoryValidation result;

    if (waypoints.empty()) {
        result.all_messages.push_back(
            makeMessage(ValidationSeverity::Error, "No waypoints provided"));
        result.valid = false;
        return result;
    }

    if (waypoints.size() == 1) {
        result.all_messages.push_back(
            makeMessage(ValidationSeverity::Warning,
                        "Only one waypoint - no trajectory to execute"));
    }

    // Validate each waypoint
    std::optional<JointVector> previous_joints = std::nullopt;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        auto wp_validation = validateWaypoint(waypoints[i], previous_joints);
        wp_validation.index = i;

        // Copy messages to combined list
        for (auto& msg : wp_validation.messages) {
            msg.waypoint_index = i;
            result.all_messages.push_back(msg);
        }

        // Update previous joints for next waypoint
        if (wp_validation.best_solution.has_value()) {
            previous_joints = wp_validation.best_solution;
        }

        result.waypoints.push_back(std::move(wp_validation));
    }

    // Validate path segments between waypoints
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        const auto& start_wp = result.waypoints[i];
        const auto& end_wp = result.waypoints[i + 1];

        // Skip segment validation if either endpoint is unreachable
        if (!start_wp.best_solution.has_value() || !end_wp.best_solution.has_value()) {
            PathValidation pv;
            pv.segment_index = i;
            pv.singularity_free = false;
            pv.velocity_feasible = false;
            pv.messages.push_back(
                makeMessage(ValidationSeverity::Error,
                            "Cannot validate segment - endpoint unreachable"));
            result.all_messages.push_back(pv.messages.back());
            result.segments.push_back(std::move(pv));
            continue;
        }

        // Determine segment time
        double segment_time = waypoints[i + 1].segment_time;
        if (segment_time <= 0.0) {
            // Auto-compute based on distance and max velocity
            Eigen::Vector3d delta = waypoints[i + 1].position - waypoints[i].position;
            double distance = delta.norm();
            segment_time = distance / trajectory_config.max_linear_velocity;
            segment_time = std::max(segment_time, 0.1);  // Minimum 100ms
        }

        auto path_validation = validateSegment(
            waypoints[i], waypoints[i + 1],
            *start_wp.best_solution, segment_time, trajectory_config);
        path_validation.segment_index = i;

        // Copy messages
        for (const auto& msg : path_validation.messages) {
            result.all_messages.push_back(msg);
        }

        result.segments.push_back(std::move(path_validation));
    }

    // Determine overall validity
    result.valid = !result.hasErrors();

    return result;
}

WaypointValidation TrajectoryValidator::validateWaypoint(
    const Waypoint& waypoint,
    const std::optional<JointVector>& current_joints) const {

    WaypointValidation result;

    // Convert waypoint to pose
    Eigen::Isometry3d pose = waypoint.toPose();

    // Get IK solutions
    auto solutions = kinematics_.inverse(pose);
    result.num_solutions = solutions.size();

    if (solutions.empty()) {
        result.reachable = false;
        result.within_limits = false;
        result.messages.push_back(
            makeMessage(ValidationSeverity::Error,
                        "Waypoint is outside robot reach (no IK solution)"));
        return result;
    }

    result.reachable = true;

    // Select best solution
    if (current_joints.has_value()) {
        result.best_solution = kinematics_.selectBestSolution(
            solutions, *current_joints, config_.max_joint_jump);
    } else {
        // No current joints - select first solution within limits
        const auto& limits = kinematics_.jointLimits();
        for (const auto& sol : solutions) {
            if (limits.withinLimits(sol)) {
                result.best_solution = sol;
                break;
            }
        }
        // If none within limits, take first solution anyway and flag warning
        if (!result.best_solution.has_value() && !solutions.empty()) {
            result.best_solution = solutions[0];
        }
    }

    if (!result.best_solution.has_value()) {
        result.within_limits = false;
        result.messages.push_back(
            makeMessage(ValidationSeverity::Warning,
                        "No IK solution within joint limits or jump threshold"));
    } else {
        // Check if selected solution is within limits
        result.within_limits = kinematics_.jointLimits().withinLimits(*result.best_solution);

        if (!result.within_limits) {
            result.messages.push_back(
                makeMessage(ValidationSeverity::Warning,
                            "Selected IK solution exceeds joint limits"));
        }

        // Check for singularity at waypoint
        if (kinematics_.nearSingularity(*result.best_solution, config_.singularity_threshold)) {
            auto info = kinematics_.analyzeSingularity(*result.best_solution,
                                                        config_.singularity_threshold);
            std::string msg = "Waypoint is near ";
            switch (info.type) {
                case kinematics::SingularityType::Shoulder:
                    msg += "shoulder singularity";
                    break;
                case kinematics::SingularityType::Elbow:
                    msg += "elbow singularity";
                    break;
                case kinematics::SingularityType::Wrist:
                    msg += "wrist singularity";
                    break;
                default:
                    msg += "singularity";
            }
            result.messages.push_back(
                makeMessage(ValidationSeverity::Warning, msg));
        }
    }

    // Validate blend factor if specified
    if (waypoint.blend_factor < 0.0) {
        result.messages.push_back(
            makeMessage(ValidationSeverity::Error,
                        "Blend factor cannot be negative"));
    }

    return result;
}

PathValidation TrajectoryValidator::validateSegment(
    const Waypoint& start,
    const Waypoint& end,
    const JointVector& start_joints,
    double segment_time,
    const TrajectoryConfig& config) const {

    PathValidation result;

    if (segment_time <= 0.0) {
        result.velocity_feasible = false;
        result.messages.push_back(
            makeMessage(ValidationSeverity::Error, "Segment time must be positive"));
        return result;
    }

    Eigen::Isometry3d start_pose = start.toPose();
    Eigen::Isometry3d end_pose = end.toPose();

    // Get end joint configuration
    auto end_solutions = kinematics_.inverse(end_pose);
    if (end_solutions.empty()) {
        result.singularity_free = false;
        result.velocity_feasible = false;
        result.messages.push_back(
            makeMessage(ValidationSeverity::Error, "End pose unreachable"));
        return result;
    }

    auto end_joints_opt = kinematics_.selectBestSolution(
        end_solutions, start_joints, config_.max_joint_jump);

    if (!end_joints_opt.has_value()) {
        result.velocity_feasible = false;
        result.messages.push_back(
            makeMessage(ValidationSeverity::Warning,
                        "Large joint jump required for this segment"));
        // Use closest solution for velocity estimation
        double min_dist = std::numeric_limits<double>::max();
        JointVector best_sol = end_solutions[0];
        for (const auto& sol : end_solutions) {
            double dist = (sol - start_joints).norm();
            if (dist < min_dist) {
                min_dist = dist;
                best_sol = sol;
            }
        }
        end_joints_opt = best_sol;
    }

    JointVector end_joints = *end_joints_opt;

    // Check velocity limits
    if (config_.check_velocities) {
        JointVector velocities = estimateJointVelocities(start_joints, end_joints, segment_time);

        // Calculate ratio to velocity limit
        double max_ratio = 0.0;
        for (int i = 0; i < 6; ++i) {
            double limit = config.max_joint_velocity;
            double ratio = std::abs(velocities[i]) / limit;
            max_ratio = std::max(max_ratio, ratio);
        }
        result.max_velocity_ratio = max_ratio;

        if (max_ratio > 1.0) {
            result.velocity_feasible = false;
            result.messages.push_back(
                makeMessage(ValidationSeverity::Warning,
                            "Segment timing exceeds joint velocity limits by " +
                            std::to_string(static_cast<int>((max_ratio - 1.0) * 100)) + "%"));
        } else {
            result.velocity_feasible = true;
        }
    }

    // Check for singularities along path
    if (config_.check_singularities) {
        // Sample path at regular intervals
        double path_length = (end.position - start.position).norm();
        int num_samples = std::max(2, static_cast<int>(path_length / config_.path_sample_distance));

        JointVector current_joints = start_joints;
        for (int i = 1; i < num_samples; ++i) {
            double t = static_cast<double>(i) / (num_samples - 1);
            Eigen::Isometry3d sample_pose = interpolatePose(start_pose, end_pose, t);

            // Get IK for sampled pose
            auto solutions = kinematics_.inverse(sample_pose);
            if (solutions.empty()) {
                result.singularity_free = false;
                result.messages.push_back(
                    makeMessage(ValidationSeverity::Warning,
                                "Path passes through unreachable region"));
                break;
            }

            auto sample_joints_opt = kinematics_.selectBestSolution(
                solutions, current_joints, config_.max_joint_jump * 0.5);

            if (!sample_joints_opt.has_value()) {
                sample_joints_opt = solutions[0];  // Fallback
            }

            JointVector sample_joints = *sample_joints_opt;

            if (kinematics_.nearSingularity(sample_joints, config_.singularity_threshold)) {
                result.singularity_free = false;
                auto info = kinematics_.analyzeSingularity(sample_joints,
                                                           config_.singularity_threshold);
                std::string msg = "Path passes near ";
                switch (info.type) {
                    case kinematics::SingularityType::Shoulder:
                        msg += "shoulder singularity";
                        break;
                    case kinematics::SingularityType::Elbow:
                        msg += "elbow singularity";
                        break;
                    case kinematics::SingularityType::Wrist:
                        msg += "wrist singularity";
                        break;
                    default:
                        msg += "singularity";
                }
                msg += " at " + std::to_string(static_cast<int>(t * 100)) + "% of segment";
                result.messages.push_back(
                    makeMessage(ValidationSeverity::Warning, msg));
                break;  // Only report first singularity
            }

            current_joints = sample_joints;
        }
    }

    return result;
}

bool TrajectoryValidator::isReachable(const Eigen::Isometry3d& pose) const {
    auto solutions = kinematics_.inverse(pose);
    return !solutions.empty();
}

bool TrajectoryValidator::isSingularityFree(
    const Eigen::Isometry3d& start,
    const Eigen::Isometry3d& end,
    const JointVector& start_joints) const {

    double path_length = (end.translation() - start.translation()).norm();
    int num_samples = std::max(2, static_cast<int>(path_length / config_.path_sample_distance));

    JointVector current_joints = start_joints;
    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        Eigen::Isometry3d pose = interpolatePose(start, end, t);

        auto solutions = kinematics_.inverse(pose);
        if (solutions.empty()) {
            return false;
        }

        auto joints_opt = kinematics_.selectBestSolution(
            solutions, current_joints, config_.max_joint_jump * 0.5);

        if (!joints_opt.has_value()) {
            joints_opt = solutions[0];
        }

        if (kinematics_.nearSingularity(*joints_opt, config_.singularity_threshold)) {
            return false;
        }

        current_joints = *joints_opt;
    }

    return true;
}

Eigen::Isometry3d TrajectoryValidator::interpolatePose(
    const Eigen::Isometry3d& start,
    const Eigen::Isometry3d& end,
    double t) const {

    // Linear interpolation for position
    Eigen::Vector3d position = (1.0 - t) * start.translation() + t * end.translation();

    // SLERP for orientation
    Eigen::Quaterniond q_start(start.rotation());
    Eigen::Quaterniond q_end(end.rotation());
    Eigen::Quaterniond q_interp = q_start.slerp(t, q_end);

    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = position;
    result.linear() = q_interp.toRotationMatrix();

    return result;
}

JointVector TrajectoryValidator::estimateJointVelocities(
    const JointVector& start,
    const JointVector& end,
    double time) const {

    JointVector velocities;
    for (int i = 0; i < 6; ++i) {
        double diff = end[i] - start[i];
        // Wrap angle difference to [-pi, pi]
        while (diff > kPi) diff -= 2.0 * kPi;
        while (diff < -kPi) diff += 2.0 * kPi;
        velocities[i] = diff / time;
    }
    return velocities;
}

bool TrajectoryValidator::velocitiesWithinLimits(
    const JointVector& velocities,
    const TrajectoryConfig& config) const {

    for (int i = 0; i < 6; ++i) {
        if (std::abs(velocities[i]) > config.max_joint_velocity) {
            return false;
        }
    }
    return true;
}

}  // namespace trajectory
}  // namespace ur_controller
