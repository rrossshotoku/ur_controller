/// @file types.cpp
/// @brief Implementation of trajectory type methods

#include "ur_controller/trajectory/types.hpp"

#include <algorithm>
#include <cmath>

namespace ur_controller {
namespace trajectory {

TrajectorySample PlannedTrajectory::sampleAt(double t) const {
    if (samples.empty()) {
        return TrajectorySample{};
    }

    // Clamp time to valid range
    t = std::clamp(t, 0.0, total_duration);

    // Fast path: direct index lookup for uniform sampling
    size_t index = static_cast<size_t>(t / sample_period);
    index = std::min(index, samples.size() - 1);

    // If we're at or past the last sample, return it
    if (index >= samples.size() - 1) {
        return samples.back();
    }

    // Get surrounding samples
    const auto& s0 = samples[index];
    const auto& s1 = samples[index + 1];

    // Interpolation factor
    double dt = t - s0.time;
    double alpha = dt / sample_period;
    alpha = std::clamp(alpha, 0.0, 1.0);

    // Interpolate
    TrajectorySample result;
    result.time = t;

    // Linear interpolation for joints
    result.joints = (1.0 - alpha) * s0.joints + alpha * s1.joints;

    // Linear interpolation for position
    Eigen::Vector3d pos = (1.0 - alpha) * s0.pose.translation() +
                          alpha * s1.pose.translation();

    // SLERP for orientation
    Eigen::Quaterniond q0(s0.pose.rotation());
    Eigen::Quaterniond q1(s1.pose.rotation());
    Eigen::Quaterniond q_interp = q0.slerp(alpha, q1);

    result.pose = Eigen::Isometry3d::Identity();
    result.pose.translation() = pos;
    result.pose.linear() = q_interp.toRotationMatrix();

    // Linear interpolation for velocity
    result.linear_velocity = (1.0 - alpha) * s0.linear_velocity +
                             alpha * s1.linear_velocity;
    result.speed = (1.0 - alpha) * s0.speed + alpha * s1.speed;

    return result;
}

bool PlannedTrajectory::hasErrors() const {
    return std::any_of(messages.begin(), messages.end(),
        [](const ValidationMessage& m) {
            return m.severity == ValidationSeverity::Error;
        });
}

bool PlannedTrajectory::hasWarnings() const {
    return std::any_of(messages.begin(), messages.end(),
        [](const ValidationMessage& m) {
            return m.severity == ValidationSeverity::Warning;
        });
}

}  // namespace trajectory
}  // namespace ur_controller
