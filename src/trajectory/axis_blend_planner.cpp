/// @file axis_blend_planner.cpp
/// @brief Per-axis polyline-with-blends planner implementation

#include "ur_controller/trajectory/axis_blend_planner.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace ur_controller {
namespace trajectory {

namespace {

constexpr double kEps = 1e-12;

/// @brief Lower bound on blend half-width to avoid zero-duration knots.
/// One sample period = 1 / sample_rate.
double minBlendWidth(double sample_rate) {
    return (sample_rate > 0.0) ? (1.0 / sample_rate) : 1e-3;
}

/// @brief Auto-derive segment time from path distance and translational limit.
double autoSegmentTime(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                       const TrajectoryConfig& cfg) {
    double dist = (b - a).norm();
    double v = std::max(cfg.max_linear_velocity, 1e-3);
    return std::max(dist / v, 1e-3);
}

}  // namespace

AxisBlendPlanner::AxisBlendPlanner(const kinematics::URKinematics& kin)
    : kinematics_(kin), config_{}, validator_(kin) {}

AxisBlendPlanner::AxisBlendPlanner(const kinematics::URKinematics& kin,
                                   const TrajectoryConfig& config)
    : kinematics_(kin), config_(config), validator_(kin) {}

// =============================================================================
// Quaternion log / exp helpers
// =============================================================================

Eigen::Vector3d AxisBlendPlanner::quatLog(const Eigen::Quaterniond& q) {
    Eigen::Vector3d xyz(q.x(), q.y(), q.z());
    double xyz_norm = xyz.norm();
    if (xyz_norm < kEps) {
        return Eigen::Vector3d::Zero();
    }
    double w = q.w();
    double half_angle = std::atan2(xyz_norm, w);
    return xyz * (half_angle / xyz_norm);
}

Eigen::Quaterniond AxisBlendPlanner::quatExp(const Eigen::Vector3d& v) {
    double half_angle = v.norm();
    if (half_angle < kEps) {
        return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    }
    Eigen::Vector3d axis = v / half_angle;
    double s = std::sin(half_angle);
    return Eigen::Quaterniond(std::cos(half_angle),
                              axis.x() * s, axis.y() * s, axis.z() * s);
}

// =============================================================================
// Profile building
// =============================================================================

AxisBlendPlanner::ProfilePlan AxisBlendPlanner::buildProfile(
    const std::vector<Waypoint>& waypoints,
    std::vector<ValidationMessage>& messages) const
{
    ProfilePlan plan;
    const size_t N = waypoints.size();
    if (N < 2) {
        return plan;
    }

    plan.seg_T.resize(N - 1);
    plan.seg_v.resize(N - 1);
    plan.seg_w.resize(N - 1);
    plan.tau.assign(N, 0.0);
    plan.wp_time.assign(N, 0.0);

    // 1) Per-segment cruise velocity vectors and durations
    for (size_t i = 0; i + 1 < N; ++i) {
        double T = waypoints[i + 1].segment_time;
        if (T <= 0.0) {
            T = autoSegmentTime(waypoints[i].position,
                                waypoints[i + 1].position, config_);
            ValidationMessage m;
            m.severity = ValidationSeverity::Info;
            m.message = "Segment " + std::to_string(i + 1) +
                        ": auto segment_time = " + std::to_string(T) + "s";
            m.waypoint_index = i + 1;
            messages.push_back(m);
        }
        plan.seg_T[i] = T;

        // Linear velocity (world frame)
        plan.seg_v[i] = (waypoints[i + 1].position - waypoints[i].position) / T;

        // Angular velocity in world frame.
        // q_target = exp(½ ω T) * q_start  ⇒  ω = 2 · log(q_target · q_start⁻¹) / T
        Eigen::Quaterniond q_a = waypoints[i].orientation.normalized();
        Eigen::Quaterniond q_b = waypoints[i + 1].orientation.normalized();
        // Pick short-arc representative (handle quaternion double cover)
        if (q_a.dot(q_b) < 0.0) {
            q_b.coeffs() = -q_b.coeffs();
        }
        Eigen::Quaterniond delta_world = q_b * q_a.conjugate();
        plan.seg_w[i] = quatLog(delta_world) * (2.0 / T);
    }

    // 2) Cumulative waypoint times
    plan.wp_time[0] = 0.0;
    for (size_t i = 1; i < N; ++i) {
        plan.wp_time[i] = plan.wp_time[i - 1] + plan.seg_T[i - 1];
    }
    plan.total_duration = plan.wp_time.back();

    // 3) Resolve cruise velocities and blend widths.
    //
    //    Approach: standard LSPB ("linear segments with parabolic blends" in
    //    position; equivalently constant-velocity cruise with linear-velocity
    //    ramps). Interior segments cruise on the polyline V_i = Δp/T_i.
    //    First / last segments use the *tangent* construction the user
    //    requested: accelerate at f·a_max from rest until the parabola is
    //    tangent to a straight line passing through the next waypoint, then
    //    cruise to that waypoint along that line.
    //
    //    Tangent geometry for first segment:
    //        V_0 = (p_1 − p_0) / (T_0 − τ_0/2)
    //        τ_0 = ‖V_0‖ / (f·a_max)        (linear axis)
    //    ⇒ ‖V_0‖² − 2·f·a_max·T_0·‖V_0‖ + 2·f·a_max·D = 0
    //    ⇒ ‖V_0‖ = f·a_max·T_0 − √(f·a_max·(f·a_max·T_0² − 2D))   (smaller root)
    //    Symmetric construction for the last segment. Same form solved for
    //    angular axis; the binding axis (whichever gives larger τ) wins.
    //    The other axis is automatically below its limit at the chosen τ.
    //
    //    For N=2 (rest-rest single segment) τ_0 and τ_1 share the segment;
    //    iterate to a fixed point.
    const double tau_min = minBlendWidth(config_.sample_rate);
    const double a_max = std::max(config_.max_linear_acceleration, 1e-6);
    const double alpha_max = std::max(config_.max_angular_acceleration, 1e-6);
    constexpr double kFracMin = 0.01;

    auto getF = [&](size_t i) {
        double f = waypoints[i].blend_factor;
        if (!std::isfinite(f) || f <= 0.0) f = 1.0;
        if (f > 1.0) f = 1.0;
        if (f < kFracMin) f = kFracMin;
        return f;
    };

    auto computeRotVec = [&](size_t i) {
        Eigen::Quaterniond q_a = waypoints[i].orientation.normalized();
        Eigen::Quaterniond q_b = waypoints[i + 1].orientation.normalized();
        if (q_a.dot(q_b) < 0.0) q_b.coeffs() = -q_b.coeffs();
        return quatLog(q_b * q_a.conjugate()) * 2.0;  // axis × angle
    };

    if (N == 2) {
        // Rest-rest single segment. τ_0 and τ_1 both eat into segment 0.
        const double T = plan.seg_T[0];
        const Eigen::Vector3d dp = waypoints[1].position - waypoints[0].position;
        const Eigen::Vector3d dr = computeRotVec(0);
        const double D_lin = dp.norm();
        const double D_ang = dr.norm();
        const double f0 = getF(0);
        const double f1 = getF(1);

        double tau0 = 0.0, tau1 = 0.0;
        bool feasible = true;
        for (int it = 0; it < 50; ++it) {
            double denom = T - 0.5 * (tau0 + tau1);
            if (denom < 1e-9) {
                feasible = false;
                break;
            }
            const double Vmag = D_lin / denom;
            const double Wmag = D_ang / denom;
            const double new0 = std::max(Vmag / (f0 * a_max),
                                         Wmag / (f0 * alpha_max));
            const double new1 = std::max(Vmag / (f1 * a_max),
                                         Wmag / (f1 * alpha_max));
            if (std::abs(new0 - tau0) < 1e-7 &&
                std::abs(new1 - tau1) < 1e-7) {
                tau0 = new0;
                tau1 = new1;
                break;
            }
            tau0 = new0;
            tau1 = new1;
        }
        if (!feasible || (tau0 + tau1) > T - 1e-9) {
            // Couldn't fit within segment time. Clamp proportionally and warn.
            const double sum = std::max(tau0 + tau1, 1e-9);
            const double scale = (T - 1e-3) / sum;
            tau0 *= scale;
            tau1 *= scale;
            ValidationMessage m;
            m.severity = ValidationSeverity::Warning;
            m.message = "Segment time " + std::to_string(T) +
                        "s too short to accelerate from rest to cruise and "
                        "back at f·a_max — clamping τ. Increase segment_time "
                        "or f for a feasible profile.";
            m.waypoint_index = 0;
            messages.push_back(m);
        }
        const double denom = T - 0.5 * (tau0 + tau1);
        if (denom > 1e-9) {
            plan.seg_v[0] = dp / denom;
            plan.seg_w[0] = dr / denom;
        } else {
            plan.seg_v[0] = dp / std::max(T, 1e-9);
            plan.seg_w[0] = dr / std::max(T, 1e-9);
        }
        plan.tau[0] = std::max(tau0, tau_min);
        plan.tau[1] = std::max(tau1, tau_min);
    } else {
        // N ≥ 3: handle first and last segments independently. Interior
        // segments keep their polyline cruise velocities (already set above).

        auto restRampTau = [&](double T, double D_lin, double D_ang, double f) {
            // Solve τ such that ‖V‖ = D / (T − τ/2) and ‖V‖/τ = f·a_lim.
            // Returns the smaller (physical) root. -1 if infeasible.
            auto solveQuad = [&](double D, double f_lim) -> double {
                if (D <= 1e-12) return 0.0;
                const double disc = f_lim * (f_lim * T * T - 2.0 * D);
                if (disc < 0.0) return -1.0;
                const double Vmag = f_lim * T - std::sqrt(disc);
                return Vmag / f_lim;
            };
            const double t_lin = solveQuad(D_lin, f * a_max);
            const double t_ang = solveQuad(D_ang, f * alpha_max);
            if (t_lin < 0.0 || t_ang < 0.0) return -1.0;
            return std::max(t_lin, t_ang);
        };

        // First segment: rest start → interior end at WP_1
        {
            const double T = plan.seg_T[0];
            const Eigen::Vector3d dp = waypoints[1].position - waypoints[0].position;
            const Eigen::Vector3d dr = computeRotVec(0);
            double tau0 = restRampTau(T, dp.norm(), dr.norm(), getF(0));
            if (tau0 < 0.0) {
                tau0 = T - 1e-3;
                ValidationMessage m;
                m.severity = ValidationSeverity::Warning;
                m.message = "Segment 1: time too short to accelerate from rest "
                            "at f·a_max — clamping τ_0 = " +
                            std::to_string(tau0) +
                            "s. Increase segment_time or f.";
                m.waypoint_index = 0;
                messages.push_back(m);
            }
            tau0 = std::min(tau0, T - 1e-9);
            plan.tau[0] = std::max(tau0, tau_min);

            const double denom = T - 0.5 * tau0;
            if (denom > 1e-9) {
                plan.seg_v[0] = dp / denom;
                plan.seg_w[0] = dr / denom;
            }
        }

        // Last segment: interior start → rest end at WP_{N-1}
        {
            const size_t i_last = N - 2;
            const double T = plan.seg_T[i_last];
            const Eigen::Vector3d dp =
                waypoints[N - 1].position - waypoints[i_last].position;
            const Eigen::Vector3d dr = computeRotVec(i_last);
            double tauN = restRampTau(T, dp.norm(), dr.norm(), getF(N - 1));
            if (tauN < 0.0) {
                tauN = T - 1e-3;
                ValidationMessage m;
                m.severity = ValidationSeverity::Warning;
                m.message = "Segment " + std::to_string(i_last + 1) +
                            ": time too short to decelerate to rest at "
                            "f·a_max — clamping τ = " +
                            std::to_string(tauN) +
                            "s. Increase segment_time or f.";
                m.waypoint_index = N - 1;
                messages.push_back(m);
            }
            tauN = std::min(tauN, T - 1e-9);
            plan.tau[N - 1] = std::max(tauN, tau_min);

            const double denom = T - 0.5 * tauN;
            if (denom > 1e-9) {
                plan.seg_v[i_last] = dp / denom;
                plan.seg_w[i_last] = dr / denom;
            }
        }
    }

    // 4) Interior waypoint blends. τ_i is the *half*-blend window; full
    //    interior blend duration is 2τ_i. τ_i sized so peak accel during the
    //    blend is f_i · a_max (or f_i · α_max, whichever binds).
    for (size_t i = 1; i + 1 < N; ++i) {
        const Eigen::Vector3d dv = plan.seg_v[i] - plan.seg_v[i - 1];
        const Eigen::Vector3d dw = plan.seg_w[i] - plan.seg_w[i - 1];
        const double dv_norm = dv.norm();
        const double dw_norm = dw.norm();
        const double f = getF(i);

        const double tau_lin = (dv_norm > kEps)
                                   ? (dv_norm / (2.0 * f * a_max))
                                   : 0.0;
        const double tau_ang = (dw_norm > kEps)
                                   ? (dw_norm / (2.0 * f * alpha_max))
                                   : 0.0;
        double tau = std::max(tau_lin, tau_ang);

        // Geometric cap: must fit within both adjacent segments without
        // colliding with rest ramps or other blends.
        double cap = std::min(plan.seg_T[i - 1] * 0.5, plan.seg_T[i] * 0.5);
        if (i == 1) {
            // Shares segment 0 with the rest-start ramp τ_0.
            const double room = plan.seg_T[0] - plan.tau[0] - 1e-9;
            cap = std::min(cap, room);
        }
        if (i + 1 == N - 1) {
            // Shares segment N-2 with the rest-end ramp τ_{N-1}.
            const double room = plan.seg_T[N - 2] - plan.tau[N - 1] - 1e-9;
            cap = std::min(cap, room);
        }
        if (cap < tau_min) cap = tau_min;

        if (tau > cap) {
            plan.tau[i] = cap;
            const double D_eff = cap * 2.0;
            const double a_eff = (dv_norm > kEps) ? (dv_norm / D_eff) : 0.0;
            const double alpha_eff = (dw_norm > kEps) ? (dw_norm / D_eff) : 0.0;
            const bool over = a_eff > a_max + 1e-9 || alpha_eff > alpha_max + 1e-9;
            ValidationMessage m;
            m.severity = over ? ValidationSeverity::Warning
                              : ValidationSeverity::Info;
            m.message = "Waypoint " + std::to_string(i + 1) +
                        ": interior blend τ capped at " +
                        std::to_string(cap) +
                        "s by adjacent segment_time; achieved accel = " +
                        std::to_string(a_eff) + " m/s² (limit " +
                        std::to_string(a_max) + "), angular = " +
                        std::to_string(alpha_eff) + " rad/s² (limit " +
                        std::to_string(alpha_max) + ")";
            m.waypoint_index = i;
            messages.push_back(m);
        } else {
            plan.tau[i] = std::max(tau, tau_min);
        }
    }

    return plan;
}

// =============================================================================
// Knot list
// =============================================================================

std::vector<AxisBlendPlanner::Knot> AxisBlendPlanner::buildKnots(
    const ProfilePlan& plan) const
{
    std::vector<Knot> knots;
    const size_t N = plan.tau.size();
    if (N < 2) {
        return knots;
    }

    // First waypoint: rest → cruise of segment 0
    {
        Knot k;
        k.t = 0.0;
        k.v.setZero();
        k.w.setZero();
        knots.push_back(k);

        Knot k2;
        k2.t = plan.tau[0];          // half-blend of width τ_0 at start
        k2.v = plan.seg_v[0];
        k2.w = plan.seg_w[0];
        knots.push_back(k2);
    }

    // Interior waypoints: blend from segment i-1 cruise to segment i cruise
    for (size_t i = 1; i + 1 < N; ++i) {
        const double t_wp = plan.wp_time[i];
        const double tau = plan.tau[i];

        Knot k_pre;
        k_pre.t = t_wp - tau;
        k_pre.v = plan.seg_v[i - 1];
        k_pre.w = plan.seg_w[i - 1];
        knots.push_back(k_pre);

        Knot k_post;
        k_post.t = t_wp + tau;
        k_post.v = plan.seg_v[i];
        k_post.w = plan.seg_w[i];
        knots.push_back(k_post);
    }

    // Last waypoint: cruise of segment N-2 → rest
    {
        const double t_end = plan.total_duration;
        Knot k_pre;
        k_pre.t = t_end - plan.tau[N - 1];
        k_pre.v = plan.seg_v[N - 2];
        k_pre.w = plan.seg_w[N - 2];
        knots.push_back(k_pre);

        Knot k_end;
        k_end.t = t_end;
        k_end.v.setZero();
        k_end.w.setZero();
        knots.push_back(k_end);
    }

    // Knots may end up out of order if τ values are aggressive against the
    // adjacent segment time; sort to be safe and de-duplicate identical times.
    std::sort(knots.begin(), knots.end(),
              [](const Knot& a, const Knot& b) { return a.t < b.t; });

    return knots;
}

// =============================================================================
// Sampling / integration
// =============================================================================

PlannedTrajectory AxisBlendPlanner::sample(
    const std::vector<Knot>& knots,
    const ProfilePlan& profile,
    const std::vector<Waypoint>& waypoints,
    const JointVector& start_joints,
    std::vector<ValidationMessage>& messages) const
{
    PlannedTrajectory result;
    result.sample_period = 1.0 / config_.sample_rate;

    if (knots.size() < 2 || waypoints.size() < 2) {
        ValidationMessage m;
        m.severity = ValidationSeverity::Error;
        m.message = "AxisBlendPlanner: not enough knots/waypoints to sample";
        messages.push_back(m);
        result.messages = messages;
        return result;
    }

    const size_t N = waypoints.size();
    const double T_total = knots.back().t;
    result.total_duration = T_total;

    const double dt = result.sample_period;
    const size_t num_samples =
        static_cast<size_t>(std::floor(T_total / dt)) + 1;
    result.samples.reserve(num_samples);

    // Pre-normalise waypoint orientations and resolve quaternion double cover
    // so neighbouring quaternions are in the same hemisphere (slerp / SQUAD
    // both want the short-arc representative).
    std::vector<Eigen::Quaterniond> Q(N);
    Q[0] = waypoints[0].orientation.normalized();
    for (size_t i = 1; i < N; ++i) {
        Eigen::Quaterniond qi = waypoints[i].orientation.normalized();
        if (Q[i - 1].dot(qi) < 0.0) qi.coeffs() = -qi.coeffs();
        Q[i] = qi;
    }

    // SQUAD intermediate control quaternions per waypoint.
    //   s_i = q_i · exp( -( log(q_i⁻¹·q_{i+1}) + log(q_i⁻¹·q_{i-1}) ) / 4 )
    // (Shoemake's formula; here quatLog returns θ/2·n̂ so the /4 matches.)
    // Boundaries: s_0 = q_0 and s_{N-1} = q_{N-1}, which makes the SQUAD
    // tangent zero at trajectory start/end — i.e. ω = 0 at rest endpoints.
    std::vector<Eigen::Quaterniond> S(N);
    S[0] = Q[0];
    S[N - 1] = Q[N - 1];
    for (size_t i = 1; i + 1 < N; ++i) {
        const Eigen::Quaterniond q_inv = Q[i].conjugate();
        const Eigen::Vector3d log_next = quatLog(q_inv * Q[i + 1]);
        const Eigen::Vector3d log_prev = quatLog(q_inv * Q[i - 1]);
        S[i] = Q[i] * quatExp((log_next + log_prev) * (-0.25));
        S[i].normalize();
    }

    // SQUAD evaluation: Q(u) = slerp( slerp(q_a, q_b, u),
    //                                  slerp(s_a, s_b, u),
    //                                  2u(1-u) )
    auto squad = [](const Eigen::Quaterniond& q_a,
                    const Eigen::Quaterniond& s_a,
                    const Eigen::Quaterniond& s_b,
                    const Eigen::Quaterniond& q_b,
                    double u) {
        const Eigen::Quaterniond p1 = q_a.slerp(u, q_b);
        const Eigen::Quaterniond p2 = s_a.slerp(u, s_b);
        const double w = 2.0 * u * (1.0 - u);
        return p1.slerp(w, p2);
    };

    // Find segment index containing time t (linear scan; trajectories rarely
    // have more than ~20 segments).
    auto findSegment = [&](double t) -> size_t {
        for (size_t i = 0; i + 1 < profile.wp_time.size(); ++i) {
            if (t < profile.wp_time[i + 1]) return i;
        }
        return profile.seg_T.empty() ? 0 : profile.seg_T.size() - 1;
    };

    auto orientationAt = [&](double t) -> Eigen::Quaterniond {
        const size_t i = findSegment(t);
        const double T = (profile.seg_T[i] > 1e-9) ? profile.seg_T[i] : 1e-9;
        double u = (t - profile.wp_time[i]) / T;
        u = std::clamp(u, 0.0, 1.0);
        return squad(Q[i], S[i], S[i + 1], Q[i + 1], u).normalized();
    };

    // Lambda: lookup v(t) by linear interpolation between knots. ω is no
    // longer integrated — orientation comes from SQUAD.
    size_t k_idx = 0;
    auto interpV = [&](double t) -> Eigen::Vector3d {
        while (k_idx + 1 < knots.size() && knots[k_idx + 1].t < t) {
            ++k_idx;
        }
        const Knot& a = knots[k_idx];
        const Knot& b = knots[std::min(k_idx + 1, knots.size() - 1)];
        const double span = b.t - a.t;
        if (span <= kEps) return b.v;
        double u = std::clamp((t - a.t) / span, 0.0, 1.0);
        return a.v + (b.v - a.v) * u;
    };

    // Initial state: position/orientation come from the first waypoint.
    Eigen::Vector3d pos = waypoints.front().position;
    Eigen::Quaterniond q = orientationAt(0.0);
    JointVector joints = start_joints;
    Eigen::Vector3d v_curr = interpV(0.0);

    // Emit sample at t=0 directly (no integration step yet)
    {
        TrajectorySample s;
        s.time = 0.0;
        s.joints = joints;
        s.pose = Eigen::Isometry3d::Identity();
        s.pose.translation() = pos;
        s.pose.linear() = q.toRotationMatrix();
        s.linear_velocity = v_curr;
        s.speed = v_curr.norm();
        result.samples.push_back(s);
    }

    size_t ik_failures = 0;

    for (size_t k = 1; k < num_samples; ++k) {
        const double t_next = std::min(static_cast<double>(k) * dt, T_total);
        const Eigen::Vector3d v_next = interpV(t_next);

        double step = t_next - (static_cast<double>(k - 1) * dt);
        if (step < 0.0) step = 0.0;

        // Trapezoidal integration of position
        pos += 0.5 * (v_curr + v_next) * step;

        // Orientation evaluated directly from SQUAD curve
        q = orientationAt(t_next);

        // IK seeded by previous sample's joints
        Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
        target.translation() = pos;
        target.linear() = q.toRotationMatrix();

        auto ik = kinematics_.inverseNumerical(target, joints);
        if (ik.has_value()) {
            joints = *ik;
        } else {
            ++ik_failures;
            // Hold previous joint solution rather than producing a NaN sample
        }

        TrajectorySample s;
        s.time = t_next;
        s.joints = joints;
        s.pose = target;
        s.linear_velocity = v_next;
        s.speed = v_next.norm();
        result.samples.push_back(s);

        v_curr = v_next;
    }

    if (ik_failures > 0) {
        ValidationMessage m;
        m.severity = ValidationSeverity::Warning;
        m.message = "AxisBlendPlanner: " + std::to_string(ik_failures) +
                    " IK failures during sampling (held last solution)";
        messages.push_back(m);
    }

    // Joint-velocity sanity check
    if (result.samples.size() >= 2) {
        double max_jv = 0.0;
        size_t max_jv_idx = 0;
        int max_jv_joint = 0;
        for (size_t i = 1; i < result.samples.size(); ++i) {
            JointVector dq = result.samples[i].joints - result.samples[i - 1].joints;
            for (int j = 0; j < 6; ++j) {
                double jv = std::abs(dq[j]) / dt;
                if (jv > max_jv) {
                    max_jv = jv;
                    max_jv_idx = i;
                    max_jv_joint = j;
                }
            }
        }
        if (max_jv > config_.max_joint_velocity) {
            ValidationMessage m;
            m.severity = ValidationSeverity::Warning;
            m.message = "AxisBlendPlanner: joint J" + std::to_string(max_jv_joint + 1) +
                        " velocity " + std::to_string(max_jv) +
                        " rad/s exceeds limit " +
                        std::to_string(config_.max_joint_velocity) +
                        " (sample " + std::to_string(max_jv_idx) + ")";
            messages.push_back(m);
        }
    }

    result.messages = messages;
    result.valid = true;
    for (const auto& m : result.messages) {
        if (m.severity == ValidationSeverity::Error) {
            result.valid = false;
            break;
        }
    }
    return result;
}

// =============================================================================
// Public API
// =============================================================================

PlannedTrajectory AxisBlendPlanner::plan(const std::vector<Waypoint>& waypoints,
                                         const JointVector& start_joints)
{
    PlannedTrajectory result;
    result.sample_period = 1.0 / config_.sample_rate;
    std::vector<ValidationMessage> messages;

    if (waypoints.size() < 2) {
        ValidationMessage m;
        m.severity = ValidationSeverity::Error;
        m.message = "AxisBlendPlanner: need at least 2 waypoints";
        messages.push_back(m);
        result.messages = messages;
        return result;
    }

    spdlog::info("AxisBlendPlanner: planning {} waypoints, sample rate {} Hz",
                 waypoints.size(), config_.sample_rate);

    // Up-front reachability + limits check on user-supplied waypoints
    auto wp_validation = validator_.validate(waypoints, config_);
    for (const auto& m : wp_validation.all_messages) {
        messages.push_back(m);
    }

    ProfilePlan profile = buildProfile(waypoints, messages);
    if (profile.seg_T.empty()) {
        ValidationMessage m;
        m.severity = ValidationSeverity::Error;
        m.message = "AxisBlendPlanner: failed to build profile";
        messages.push_back(m);
        result.messages = messages;
        return result;
    }

    spdlog::info("AxisBlendPlanner: total duration {:.3f}s, {} segments",
                 profile.total_duration, profile.seg_T.size());

    auto knots = buildKnots(profile);
    return sample(knots, profile, waypoints, start_joints, messages);
}

PlannedTrajectory AxisBlendPlanner::plan(const std::vector<Waypoint>& waypoints)
{
    JointVector start = JointVector::Zero();

    if (!waypoints.empty() && waypoints[0].joints.has_value()) {
        start = *waypoints[0].joints;
    } else if (!waypoints.empty()) {
        // Seed numerical IK from joint-limit midpoints
        const auto& limits = kinematics_.jointLimits();
        for (int i = 0; i < 6; ++i) {
            start[i] = 0.5 * (limits.joints[static_cast<size_t>(i)].min +
                              limits.joints[static_cast<size_t>(i)].max);
        }
        auto ik = kinematics_.inverseNumerical(waypoints[0].toPose(), start);
        if (ik.has_value()) {
            start = *ik;
        }
    }
    return plan(waypoints, start);
}

}  // namespace trajectory
}  // namespace ur_controller
