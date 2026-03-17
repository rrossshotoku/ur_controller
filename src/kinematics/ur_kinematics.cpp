/// @file ur_kinematics.cpp
/// @brief Analytical kinematics implementation for Universal Robots

#include "ur_controller/kinematics/ur_kinematics.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace ur_controller {
namespace kinematics {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-10;

// Sign function
inline double sign(double x) {
    return (x >= 0.0) ? 1.0 : -1.0;
}

}  // namespace

URKinematics::URKinematics(URModel model)
    : model_(model)
    , dh_(DHParameters::forModel(model))
    , limits_(defaultJointLimits(model)) {
    // Cache frequently used DH parameters for IK
    d1_ = dh_.joints[0].d;
    a2_ = dh_.joints[1].a;
    a3_ = dh_.joints[2].a;
    d4_ = dh_.joints[3].d;
    d5_ = dh_.joints[4].d;
    d6_ = dh_.joints[5].d;
}

// -----------------------------------------------------------------------------
// Utility Functions
// -----------------------------------------------------------------------------

double URKinematics::wrapAngle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

JointVector URKinematics::wrapAngles(const JointVector& q) {
    JointVector wrapped;
    for (int i = 0; i < 6; ++i) {
        wrapped[i] = wrapAngle(q[i]);
    }
    return wrapped;
}

Eigen::Isometry3d URKinematics::dhTransform(
    double a, double d, double alpha, double theta) const {
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);

    // Standard DH convention:
    // T = Rot(z, theta) * Trans(z, d) * Trans(x, a) * Rot(x, alpha)
    Eigen::Matrix4d m;
    m << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
         0.0,     sa,       ca,      d,
         0.0,    0.0,      0.0,    1.0;

    Eigen::Isometry3d T;
    T.matrix() = m;
    return T;
}

// -----------------------------------------------------------------------------
// Forward Kinematics
// -----------------------------------------------------------------------------

Eigen::Isometry3d URKinematics::forward(const JointVector& q) const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    for (int i = 0; i < 6; ++i) {
        const auto& joint = dh_.joints[static_cast<size_t>(i)];
        double theta = q[i] + joint.theta_offset;
        T = T * dhTransform(joint.a, joint.d, joint.alpha, theta);
    }

    return T;
}

std::array<Eigen::Isometry3d, 7> URKinematics::allLinkTransforms(
    const JointVector& q) const {
    std::array<Eigen::Isometry3d, 7> transforms;

    // Base frame
    transforms[0] = Eigen::Isometry3d::Identity();

    // Cumulative transforms
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    for (int i = 0; i < 6; ++i) {
        const auto& joint = dh_.joints[static_cast<size_t>(i)];
        double theta = q[i] + joint.theta_offset;
        T = T * dhTransform(joint.a, joint.d, joint.alpha, theta);
        transforms[static_cast<size_t>(i) + 1] = T;
    }

    return transforms;
}

// -----------------------------------------------------------------------------
// Inverse Kinematics
// -----------------------------------------------------------------------------

Eigen::Vector3d URKinematics::wristCenter(
    const Eigen::Isometry3d& tcp_pose) const {
    // Wrist center is d6 back from TCP along the approach vector (z-axis of TCP)
    Eigen::Vector3d z_tcp = tcp_pose.rotation().col(2);
    return tcp_pose.translation() - d6_ * z_tcp;
}

std::vector<JointVector> URKinematics::inverse(
    const Eigen::Isometry3d& target) const {
    std::vector<JointVector> solutions;
    solutions.reserve(8);

    // Extract the 4x4 transformation matrix
    const Eigen::Matrix4d T = target.matrix();

    // Extract rotation matrix elements (column-major: T(row, col))
    // n = x-axis (normal), o = y-axis (orientation), a = z-axis (approach)
    double nx = T(0, 0), ox = T(0, 1), ax = T(0, 2), px = T(0, 3);
    double ny = T(1, 0), oy = T(1, 1), ay = T(1, 2), py = T(1, 3);
    // az unused after q234 calculation was simplified

    // =========================================================================
    // Step 1: Solve for q1 (shoulder pan) - 2 solutions
    // =========================================================================
    // The wrist center position is: p05 = p06 - d6 * a (where a is the approach vector)
    double p05x = px - d6_ * ax;
    double p05y = py - d6_ * ay;

    // q1 depends on the xy projection of the wrist center
    // The shoulder offset d4 creates an offset from the base z-axis
    double R = std::sqrt(p05x * p05x + p05y * p05y);

    // Wrist center must be at least d4 away from base z-axis
    // Use small epsilon to allow edge cases at shoulder singularity
    if (R < std::abs(d4_) - kEpsilon) {
        return solutions;  // Wrist center too close to base z-axis
    }

    // Two solutions for q1
    double q1[2];
    double alpha = std::atan2(p05y, p05x);
    double beta = std::acos(std::clamp(d4_ / R, -1.0, 1.0));
    q1[0] = alpha + beta + kPi / 2.0;
    q1[1] = alpha - beta + kPi / 2.0;

    // =========================================================================
    // For each q1, solve remaining joints
    // =========================================================================
    for (int i1 = 0; i1 < 2; ++i1) {
        double theta1 = q1[i1];
        double s1 = std::sin(theta1);
        double c1 = std::cos(theta1);

        // =====================================================================
        // Step 2: Solve for q5 (wrist 2) - 2 solutions per q1
        // =====================================================================
        // From the UR kinematic chain analysis (verified numerically):
        // cos(q5) = ax*s1 - ay*c1
        // sin(q5) = ±sqrt(1 - cos²(q5)) - two solutions for wrist up/down
        double cos_q5 = ax * s1 - ay * c1;

        if (std::abs(cos_q5) > 1.0 + kEpsilon) {
            continue;  // Invalid configuration
        }
        cos_q5 = std::clamp(cos_q5, -1.0, 1.0);

        // Two solutions: positive and negative sin(q5)
        double sin_q5_abs = std::sqrt(1.0 - cos_q5 * cos_q5);
        double q5[2];
        q5[0] = std::atan2(sin_q5_abs, cos_q5);   // Positive sin(q5)
        q5[1] = std::atan2(-sin_q5_abs, cos_q5);  // Negative sin(q5)

        for (int i5 = 0; i5 < 2; ++i5) {
            double theta5 = q5[i5];
            double s5 = std::sin(theta5);

            // =================================================================
            // Step 3: Solve for q6 (wrist 3)
            // =================================================================
            double theta6;
            if (std::abs(s5) < kEpsilon) {
                // Wrist singularity (q5 = 0 or pi) - infinite q6 solutions
                // Choose q6 = 0 by convention
                theta6 = 0.0;
            } else {
                // From orientation constraints (verified numerically):
                // sin(q6) = -(ox*s1 - oy*c1) / sin(q5)
                // cos(q6) = (nx*s1 - ny*c1) / sin(q5)
                // The sin(q5) cancels in atan2
                double sin_q6 = -ox * s1 + oy * c1;
                double cos_q6 = nx * s1 - ny * c1;
                theta6 = std::atan2(sin_q6, cos_q6);
            }

            // =================================================================
            // Step 4: Compute T14 transform (needed for both q234 and arm triangle)
            // =================================================================
            Eigen::Isometry3d T01 = dhTransform(
                dh_.joints[0].a, dh_.joints[0].d, dh_.joints[0].alpha, theta1);
            Eigen::Isometry3d T45 = dhTransform(
                dh_.joints[4].a, dh_.joints[4].d, dh_.joints[4].alpha, theta5);
            Eigen::Isometry3d T56 = dhTransform(
                dh_.joints[5].a, dh_.joints[5].d, dh_.joints[5].alpha, theta6);

            Eigen::Isometry3d T14 = T01.inverse() * target * T56.inverse() * T45.inverse();
            Eigen::Vector3d p14 = T14.translation();

            // =================================================================
            // Step 5: Solve for q234 = q2 + q3 + q4
            // =================================================================
            // T14 = T12 * T23 * T34 = Rz(q2) * Rz(q3) * Rz(q4) * Rx(α4)
            //     = Rz(q234) * Rx(π/2)
            // So we can extract q234 from the T14 rotation matrix
            // R14 = Rz(q234) * Rx(π/2), which gives:
            // R14(0,0) = cos(q234), R14(1,0) = sin(q234)
            double q234 = std::atan2(T14.rotation()(1, 0), T14.rotation()(0, 0));

            // The arm plane is the xy-plane of frame 1 (due to α1 = π/2)
            double p14x = p14.x();
            double p14y = p14.y();

            // Law of cosines for q3: |p14|^2 = a2^2 + a3^2 + 2*a2*a3*cos(q3)
            double D = p14x * p14x + p14y * p14y;
            double cos_q3 = (D - a2_ * a2_ - a3_ * a3_) / (2.0 * a2_ * a3_);

            if (std::abs(cos_q3) > 1.0 + kEpsilon) {
                continue;  // Position not reachable
            }
            cos_q3 = std::clamp(cos_q3, -1.0, 1.0);

            double q3[2];
            q3[0] = std::acos(cos_q3);
            q3[1] = -std::acos(cos_q3);

            for (int i3 = 0; i3 < 2; ++i3) {
                double theta3 = q3[i3];
                double s3 = std::sin(theta3);
                double c3 = std::cos(theta3);

                // =============================================================
                // Step 6: Solve for q2 (shoulder lift)
                // =============================================================
                // From geometry in xy-plane: p14_x = a2*c2 + a3*c23, p14_y = a2*s2 + a3*s23
                // This gives: q2 = atan2(p14y, p14x) - atan2(a3*s3, a2 + a3*c3)
                double theta2 = std::atan2(p14y, p14x) -
                               std::atan2(a3_ * s3, a2_ + a3_ * c3);

                // =============================================================
                // Step 7: Solve for q4 (wrist 1)
                // =============================================================
                // q4 = q234 - q2 - q3
                double theta4 = q234 - theta2 - theta3;

                // Build and store the solution
                JointVector sol;
                sol << theta1, theta2, theta3, theta4, theta5, theta6;
                sol = wrapAngles(sol);
                solutions.push_back(sol);
            }
        }
    }

    return solutions;
}

std::optional<JointVector> URKinematics::inverseNumerical(
    const Eigen::Isometry3d& target,
    const JointVector& seed,
    int max_iterations,
    double tolerance) const {

    JointVector q = seed;
    const double damping = 0.05;  // Damping factor for DLS (λ)

    // Target position and orientation
    const Eigen::Vector3d target_pos = target.translation();
    const Eigen::Quaterniond target_quat(target.rotation());

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Current pose from FK
        Eigen::Isometry3d current = forward(q);
        Eigen::Vector3d current_pos = current.translation();
        Eigen::Quaterniond current_quat(current.rotation());

        // Position error (3D vector)
        Eigen::Vector3d pos_error = target_pos - current_pos;

        // Orientation error using quaternion
        // Error quaternion: q_error = q_target * q_current^(-1)
        Eigen::Quaterniond q_error = target_quat * current_quat.inverse();

        // Ensure shortest rotation path
        if (q_error.w() < 0) {
            q_error.coeffs() = -q_error.coeffs();
        }

        // Convert to angle-axis representation for 3D angular error
        Eigen::AngleAxisd aa(q_error);
        Eigen::Vector3d rot_error = aa.angle() * aa.axis();

        // Combined 6D error vector [position; orientation]
        Eigen::Matrix<double, 6, 1> error;
        error.head<3>() = pos_error;
        error.tail<3>() = rot_error;

        // Check convergence
        double pos_norm = pos_error.norm();
        double rot_norm = rot_error.norm();
        if (pos_norm < tolerance && rot_norm < tolerance) {
            // Converged - angles stayed close to seed (no wrapping during iteration)
            return q;
        }

        // Compute geometric Jacobian at current configuration
        Jacobian J = jacobian(q);

        // Damped least squares (DLS) pseudo-inverse:
        // J+ = J^T * (J*J^T + λ²I)^(-1)
        Eigen::Matrix<double, 6, 6> JJT = J * J.transpose();
        JJT.diagonal().array() += damping * damping;

        // Solve for Cartesian velocity that reduces error
        Eigen::Matrix<double, 6, 1> delta_x = JJT.ldlt().solve(error);

        // Convert to joint velocity: delta_q = J^T * delta_x
        JointVector delta_q = J.transpose() * delta_x;

        // Limit step size to prevent overshooting
        double max_step = delta_q.cwiseAbs().maxCoeff();
        if (max_step > 0.5) {  // Max 0.5 rad per iteration (~29 deg)
            delta_q *= 0.5 / max_step;
        }

        // Update joint angles - NO WRAPPING during iteration
        // This preserves continuity when near ±π (like J6 near 180°)
        q += delta_q;

        // Don't wrap angles during iteration - sin/cos in FK are periodic anyway
        // Wrapping disrupts convergence when angles cross ±π boundary
    }

    // Failed to converge within max iterations
    return std::nullopt;
}

std::optional<JointVector> URKinematics::selectBestSolution(
    const std::vector<JointVector>& solutions,
    const JointVector& current_q,
    double max_joint_jump) const {
    return selectBestSolution(solutions, current_q, limits_, max_joint_jump);
}

std::optional<JointVector> URKinematics::selectBestSolution(
    const std::vector<JointVector>& solutions,
    const JointVector& current_q,
    const JointLimits& limits,
    double max_joint_jump) const {

    std::optional<JointVector> best;
    double best_distance = std::numeric_limits<double>::max();

    for (const auto& solution : solutions) {
        // Check joint limits
        if (!limits.withinLimits(solution)) {
            continue;
        }

        // Check for large joint jumps
        JointVector diff = solution - current_q;
        for (int i = 0; i < 6; ++i) {
            diff[i] = wrapAngle(diff[i]);
        }

        bool valid = true;
        for (int i = 0; i < 6; ++i) {
            if (std::abs(diff[i]) > max_joint_jump) {
                valid = false;
                break;
            }
        }

        if (!valid) {
            continue;
        }

        // Compute weighted distance (wrist joints weighted less)
        double distance = 0.0;
        for (int i = 0; i < 6; ++i) {
            double weight = (i < 3) ? 1.0 : 0.5;  // Wrist joints weighted less
            distance += weight * diff[i] * diff[i];
        }
        distance = std::sqrt(distance);

        if (distance < best_distance) {
            best_distance = distance;
            best = solution;
        }
    }

    return best;
}

// -----------------------------------------------------------------------------
// Configuration Tracking
// -----------------------------------------------------------------------------

ArmConfiguration URKinematics::getConfiguration(const JointVector& q) const {
    ArmConfiguration config;

    // Wrap angles to [-π, π] for consistent analysis
    JointVector qw = wrapAngles(q);

    // Shoulder: front/back based on q1
    // Front: q1 in roughly [-π/2, π/2] (robot reaching "forward")
    // Back: q1 near ±π (robot reaching "backward")
    // Use π/2 as the boundary
    config.shoulder = (std::abs(qw[0]) < kPi / 2.0) ? 1 : -1;

    // Elbow: up/down based on q3 sign
    // In UR convention with the DH parameters used:
    // - q3 < 0 typically means "elbow up" (forearm above shoulder-elbow line)
    // - q3 > 0 typically means "elbow down"
    config.elbow = (qw[2] < 0) ? 1 : -1;

    // Wrist: based on q5
    // The two q5 solutions come from ±sqrt(1 - cos²(q5))
    // - Positive sin(q5) → q5 in (0, π)
    // - Negative sin(q5) → q5 in (-π, 0)
    config.wrist = (qw[4] >= 0) ? 1 : -1;

    return config;
}

std::vector<JointVector> URKinematics::filterByConfiguration(
    const std::vector<JointVector>& solutions,
    const ArmConfiguration& target_config) const {

    std::vector<JointVector> filtered;
    filtered.reserve(solutions.size());

    for (const auto& sol : solutions) {
        if (getConfiguration(sol) == target_config) {
            filtered.push_back(sol);
        }
    }

    return filtered;
}

std::optional<JointVector> URKinematics::selectBestSolutionSameConfig(
    const std::vector<JointVector>& solutions,
    const JointVector& current_q) const {

    // Get current configuration
    ArmConfiguration current_config = getConfiguration(current_q);

    // Filter to same configuration
    std::vector<JointVector> same_config = filterByConfiguration(solutions, current_config);

    if (same_config.empty()) {
        return std::nullopt;  // No solution in same configuration
    }

    // Find closest solution (no need for large threshold since config is same)
    std::optional<JointVector> best;
    double best_distance = std::numeric_limits<double>::max();

    for (const auto& sol : same_config) {
        // Check joint limits
        if (!limits_.withinLimits(sol)) {
            continue;
        }

        // Calculate weighted distance (favor arm joints over wrist joints)
        double distance = 0.0;
        for (int i = 0; i < 6; ++i) {
            double diff = wrapAngle(sol[i] - current_q[i]);
            // Weight arm joints (0-2) more than wrist joints (3-5)
            double weight = (i < 3) ? 2.0 : 1.0;
            distance += weight * diff * diff;
        }

        if (distance < best_distance) {
            best_distance = distance;
            best = sol;
        }
    }

    return best;
}

// -----------------------------------------------------------------------------
// Jacobian
// -----------------------------------------------------------------------------

Jacobian URKinematics::jacobian(const JointVector& q) const {
    // Compute all link transforms
    auto transforms = allLinkTransforms(q);

    // TCP position
    Eigen::Vector3d p_tcp = transforms[6].translation();

    Jacobian J;
    J.setZero();

    for (int i = 0; i < 6; ++i) {
        // Z-axis of joint i frame
        Eigen::Vector3d z_i = transforms[static_cast<size_t>(i)].rotation().col(2);

        // Position of joint i
        Eigen::Vector3d p_i = transforms[static_cast<size_t>(i)].translation();

        // Vector from joint i to TCP
        Eigen::Vector3d r = p_tcp - p_i;

        // Linear velocity contribution (z_i x r)
        J.block<3, 1>(0, i) = z_i.cross(r);

        // Angular velocity contribution (z_i)
        J.block<3, 1>(3, i) = z_i;
    }

    return J;
}

// -----------------------------------------------------------------------------
// Singularity Detection
// -----------------------------------------------------------------------------

bool URKinematics::nearSingularity(const JointVector& q, double threshold) const {
    return analyzeSingularity(q, threshold).near_singularity;
}

SingularityInfo URKinematics::analyzeSingularity(
    const JointVector& q, double threshold) const {
    SingularityInfo info;

    // Compute Jacobian
    Jacobian J = jacobian(q);

    // Compute SVD for condition number
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    Eigen::VectorXd singular_values = svd.singularValues();

    double sigma_max = singular_values(0);
    double sigma_min = singular_values(5);

    if (sigma_min < kEpsilon) {
        info.condition_number = std::numeric_limits<double>::infinity();
        info.near_singularity = true;
    } else {
        info.condition_number = sigma_max / sigma_min;
        info.near_singularity = (info.condition_number > threshold);
    }

    if (!info.near_singularity) {
        info.type = SingularityType::None;
        return info;
    }

    // Determine singularity type
    // Wrist singularity: q5 near 0 or pi
    double q5 = q[4];
    if (std::abs(q5) < 0.1 || std::abs(std::abs(q5) - kPi) < 0.1) {
        info.type = SingularityType::Wrist;
        return info;
    }

    // Elbow singularity: q3 near 0 or pi (arm extended/folded)
    double q3 = q[2];
    if (std::abs(q3) < 0.1 || std::abs(std::abs(q3) - kPi) < 0.1) {
        info.type = SingularityType::Elbow;
        return info;
    }

    // Shoulder singularity: wrist center on z-axis of base
    auto transforms = allLinkTransforms(q);
    Eigen::Vector3d wrist = transforms[4].translation();
    double xy_dist = std::sqrt(wrist.x() * wrist.x() + wrist.y() * wrist.y());
    if (xy_dist < 0.05) {
        info.type = SingularityType::Shoulder;
        return info;
    }

    // Generic singularity (combination or unknown cause)
    info.type = SingularityType::None;
    return info;
}

}  // namespace kinematics
}  // namespace ur_controller
