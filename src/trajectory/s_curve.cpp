#include "ur_controller/trajectory/s_curve.hpp"
#include <cmath>
#include <algorithm>
#include <spdlog/spdlog.h>

namespace ur_controller::trajectory {

SCurveProfile::SCurveProfile(const SCurveParams& params)
    : v_entry_(params.v_entry)
    , v_exit_(params.v_exit)
    , distance_(params.distance)
    , duration_(params.duration)
    , max_accel_(params.max_accel)
    , max_jerk_(params.max_jerk)
{
    // Handle edge cases
    if (duration_ <= 0.0 || distance_ < 0.0) {
        valid_ = false;
        return;
    }

    double dv = v_entry_ - v_exit_;  // Velocity change needed

    constexpr double kVelocityTolerance = 0.001;  // 1 mm/s

    // Check if both entry and exit are near zero (symmetric rest-to-rest profile)
    bool start_at_rest = std::abs(v_entry_) < kVelocityTolerance;
    bool end_at_rest = std::abs(v_exit_) < kVelocityTolerance;

    if (start_at_rest && end_at_rest && distance_ > 0.001) {
        // Symmetric profile: accel up to peak, then decel back to rest
        type_ = ProfileType::Symmetric;
        computeSymmetricProfile();
    } else if (std::abs(dv) < kVelocityTolerance) {
        // Pure cruise - velocity stays constant
        type_ = ProfileType::CruiseOnly;
        computeCruiseOnly();
    } else if (dv > 0) {
        // Need to decelerate (v_entry > v_exit)
        type_ = ProfileType::Deceleration;
        computeDecelProfile(dv);
    } else {
        // Need to accelerate (v_entry < v_exit)
        type_ = ProfileType::Acceleration;
        computeAccelProfile(-dv);  // Pass positive magnitude
    }

    computeCumulativeTimes();
    verifyDistanceConstraint();
}

void SCurveProfile::computeCruiseOnly() {
    // All time goes to cruise phase
    t4_ = duration_;
    v4_ = v_entry_;
    s4_ = v_entry_ * duration_;
    valid_ = (std::abs(s4_ - distance_) < 0.001 * distance_ + 0.0001);
}

void SCurveProfile::computeDecelProfile(double dv) {
    // dv = v_entry - v_exit > 0 (amount to decelerate)
    double t_j = max_accel_ / max_jerk_;

    // Velocity change achievable with triangular decel (no constant phase)
    // In triangular: a_peak = J * t_j_actual, where t_j_actual = T_decel / 2
    // dv = a_peak * t_j_actual = J * (T_decel/2)² * 2 = J * T_decel² / 2
    // So T_decel = sqrt(2 * dv / J)
    double dv_triangular_limit = max_jerk_ * t_j * t_j;  // dv achievable at max accel

    double T_decel;  // Time needed for deceleration phases (5,6,7)

    if (dv <= dv_triangular_limit) {
        // Triangular profile - acceleration never reaches max
        // T_decel = 2 * sqrt(dv / J)
        T_decel = 2.0 * std::sqrt(dv / max_jerk_);
        t5_ = T_decel / 2.0;
        t6_ = 0.0;
        t7_ = T_decel / 2.0;
        a5_ = -max_jerk_ * t5_;  // Peak deceleration (negative)
    } else {
        // Trapezoidal profile - constant deceleration phase exists
        // T_decel = dv / A + A / J  (time for jerk phases + constant phase)
        T_decel = dv / max_accel_ + max_accel_ / max_jerk_;
        t5_ = t_j;
        t6_ = (dv - dv_triangular_limit) / max_accel_;
        t7_ = t_j;
        a5_ = -max_accel_;  // Peak deceleration (negative)
    }

    // Remaining time goes to cruise (phase 4)
    t4_ = duration_ - T_decel;
    if (t4_ < -0.001) {
        // Duration too short for required deceleration
        spdlog::warn("SCurve: Duration {:.3f}s too short for decel, need {:.3f}s",
            duration_, T_decel);
        t4_ = 0.0;
        // Scale down the profile to fit
        double scale = duration_ / T_decel;
        t5_ *= scale;
        t6_ *= scale;
        t7_ *= scale;
    } else if (t4_ < 0.0) {
        t4_ = 0.0;
    }

    // Compute states at phase boundaries
    // Phase 4 (cruise): constant velocity at v_entry
    s4_ = v_entry_ * t4_;
    v4_ = v_entry_;

    // Phase 5 (jerk down): a goes from 0 to a5_ (negative)
    // j5 = -max_jerk (or scaled)
    double j5 = (t5_ > 0.001) ? a5_ / t5_ : 0.0;
    v5_ = v4_ + 0.5 * j5 * t5_ * t5_;  // v = v0 + 0.5*j*t²
    s5_ = s4_ + v4_ * t5_ + (1.0/6.0) * j5 * t5_ * t5_ * t5_;

    // Phase 6 (constant decel): a = a5_
    v6_ = v5_ + a5_ * t6_;
    s6_ = s5_ + v5_ * t6_ + 0.5 * a5_ * t6_ * t6_;
    a6_ = a5_;

    // Phase 7 (jerk up): a goes from a5_ to 0
    // j7 = +max_jerk (or scaled), magnitude = -a5_ / t7_
    double j7 = (t7_ > 0.001) ? -a5_ / t7_ : 0.0;
    v7_ = v6_ + a5_ * t7_ + 0.5 * j7 * t7_ * t7_;
    s7_ = s6_ + v6_ * t7_ + 0.5 * a5_ * t7_ * t7_ + (1.0/6.0) * j7 * t7_ * t7_ * t7_;

    valid_ = true;
}

void SCurveProfile::computeAccelProfile(double dv) {
    // dv = v_exit - v_entry > 0 (amount to accelerate)
    double t_j = max_accel_ / max_jerk_;
    double dv_triangular_limit = max_jerk_ * t_j * t_j;

    double T_accel;  // Time needed for acceleration phases (1,2,3)

    if (dv <= dv_triangular_limit) {
        // Triangular profile
        T_accel = 2.0 * std::sqrt(dv / max_jerk_);
        t1_ = T_accel / 2.0;
        t2_ = 0.0;
        t3_ = T_accel / 2.0;
        a1_ = max_jerk_ * t1_;  // Peak acceleration (positive)
    } else {
        // Trapezoidal profile
        T_accel = dv / max_accel_ + max_accel_ / max_jerk_;
        t1_ = t_j;
        t2_ = (dv - dv_triangular_limit) / max_accel_;
        t3_ = t_j;
        a1_ = max_accel_;  // Peak acceleration (positive)
    }

    // Remaining time goes to cruise (phase 4, after acceleration)
    t4_ = duration_ - T_accel;
    if (t4_ < -0.001) {
        spdlog::warn("SCurve: Duration {:.3f}s too short for accel, need {:.3f}s",
            duration_, T_accel);
        t4_ = 0.0;
        double scale = duration_ / T_accel;
        t1_ *= scale;
        t2_ *= scale;
        t3_ *= scale;
    } else if (t4_ < 0.0) {
        t4_ = 0.0;
    }

    // Compute states at phase boundaries
    // Phase 1 (jerk up): a goes from 0 to a1_
    double j1 = (t1_ > 0.001) ? a1_ / t1_ : 0.0;
    v1_ = v_entry_ + 0.5 * j1 * t1_ * t1_;
    s1_ = v_entry_ * t1_ + (1.0/6.0) * j1 * t1_ * t1_ * t1_;

    // Phase 2 (constant accel): a = a1_
    a2_ = a1_;
    v2_ = v1_ + a1_ * t2_;
    s2_ = s1_ + v1_ * t2_ + 0.5 * a1_ * t2_ * t2_;

    // Phase 3 (jerk down): a goes from a1_ to 0
    double j3 = (t3_ > 0.001) ? -a1_ / t3_ : 0.0;
    v3_ = v2_ + a1_ * t3_ + 0.5 * j3 * t3_ * t3_;
    s3_ = s2_ + v2_ * t3_ + 0.5 * a1_ * t3_ * t3_ + (1.0/6.0) * j3 * t3_ * t3_ * t3_;
    a3_ = 0.0;

    // Phase 4 (cruise at v_exit)
    v4_ = v3_;  // Should equal v_exit
    s4_ = s3_ + v4_ * t4_;

    valid_ = true;
}

void SCurveProfile::computeSymmetricProfile() {
    // Rest-to-rest profile: accelerate from 0 to v_peak, then decelerate back to 0
    // Uses all 7 phases: 1,2,3 (accel), 4 (cruise), 5,6,7 (decel)
    // By symmetry: t1=t7, t2=t6, t3=t5
    //
    // Analytical solution:
    // For trapezoidal accel: D_accel = v_peak * T_accel / 2 (area of trapezoid)
    // Total: D = 2*D_accel + v_peak*T_cruise = v_peak*T_accel + v_peak*(T - 2*T_accel)
    //          = v_peak*(T - T_accel)
    // Where T_accel = v_peak/A + t_j
    // So: D = v_peak*(T - v_peak/A - t_j)
    //     v_peak² + A*v_peak*(t_j - T) + A*D = 0
    // Quadratic solution: v_peak = [A*(T - t_j) - sqrt(A²*(T - t_j)² - 4*A*D)] / 2

    spdlog::info("Symmetric SCurve: D={:.4f}m, T={:.3f}s, A={:.2f}, J={:.2f}",
        distance_, duration_, max_accel_, max_jerk_);

    double t_j = max_accel_ / max_jerk_;
    double dv_triangular = max_jerk_ * t_j * t_j;  // Velocity achievable at max accel
    double T_half = duration_ / 2.0;

    spdlog::info("  t_j={:.4f}s, T_half={:.4f}s, dv_tri={:.4f}", t_j, T_half, dv_triangular);

    double v_peak = 0.0;
    double T_accel = 0.0;

    // First check if we need trapezoidal or triangular profile
    // Compute max distance achievable with full trapezoidal (no cruise)
    double D_max_trapezoidal = 0.0;
    double D_max_triangular = 0.0;

    if (T_half > 2.0 * t_j) {
        // Trapezoidal is possible
        double t2_max = T_half - 2.0 * t_j;
        double v_peak_trap = max_accel_ * (t_j + t2_max);
        D_max_trapezoidal = v_peak_trap * T_half;  // D_accel = v_peak * T_accel / 2, total = 2*D_accel
    }

    // Triangular max (using full T_half with triangular profile)
    // D = 2*D_accel = 2*J*t1³ where t1 = T_half/2
    // D_max = 2*J*(T_half/2)³ = J*T_half³/4
    D_max_triangular = max_jerk_ * T_half * T_half * T_half / 4.0;

    double D_max = std::max(D_max_trapezoidal, D_max_triangular);

    if (D_max < distance_) {
        // Cannot cover required distance in given time - use maximum possible
        spdlog::warn("Symmetric SCurve: D_max={:.4f}m < D_required={:.4f}m, using max capability",
            D_max, distance_);

        if (T_half > 2.0 * t_j) {
            // Use trapezoidal at max
            double t2_max = T_half - 2.0 * t_j;
            v_peak = max_accel_ * (t_j + t2_max);
            T_accel = T_half;
        } else {
            // Use triangular at max
            double t1_max = T_half / 2.0;
            v_peak = max_jerk_ * t1_max * t1_max;
            T_accel = T_half;
        }
    } else {
        // Analytical solution for v_peak
        // Try trapezoidal first (more common case)
        if (T_half > 2.0 * t_j) {
            // Quadratic: v_peak² + A*(t_j - T)*v_peak + A*D = 0
            double a_coef = 1.0;
            double b_coef = max_accel_ * (t_j - duration_);
            double c_coef = max_accel_ * distance_;

            double discriminant = b_coef * b_coef - 4.0 * a_coef * c_coef;

            if (discriminant >= 0.0) {
                // Take smaller root (less v_peak = more cruise time)
                v_peak = (-b_coef - std::sqrt(discriminant)) / (2.0 * a_coef);

                // Verify this v_peak requires trapezoidal (v_peak > dv_triangular)
                if (v_peak > dv_triangular && v_peak > 0.0) {
                    T_accel = v_peak / max_accel_ + t_j;

                    // Verify T_cruise >= 0
                    if (duration_ - 2.0 * T_accel >= -0.001) {
                        // Valid trapezoidal solution
                    } else {
                        // Need triangular solution
                        v_peak = 0.0;
                    }
                } else {
                    // v_peak too low for trapezoidal, need triangular
                    v_peak = 0.0;
                }
            }
        }

        // If trapezoidal didn't work, use triangular
        if (v_peak <= 0.0 || v_peak <= dv_triangular) {
            // For triangular: D = 2*J*t1³ + v_peak*T_cruise
            // where v_peak = J*t1², T_accel = 2*t1, T_cruise = T - 4*t1
            // D = 2*J*t1³ + J*t1²*(T - 4*t1) = 2*J*t1³ + J*t1²*T - 4*J*t1³
            //   = J*t1²*T - 2*J*t1³ = J*t1²*(T - 2*t1)
            // This is a cubic in t1. Let's solve:
            // D/J = t1²*T - 2*t1³
            // 2*t1³ - T*t1² + D/J = 0

            // Use Newton's method or cubic formula
            // For simplicity, use the closed form when T_cruise = 0:
            // D = 2*J*t1³ => t1 = (D/(2*J))^(1/3)
            double t1_no_cruise = std::cbrt(distance_ / (2.0 * max_jerk_));
            double T_accel_no_cruise = 2.0 * t1_no_cruise;

            if (T_accel_no_cruise * 2.0 <= duration_) {
                // There's room for cruise - solve the cubic
                // 2*t1³ - T*t1² + D/J = 0
                // Use Newton-Raphson starting from a good initial guess
                // Start from T/4 (midpoint of valid range) rather than t1_no_cruise
                double t1 = duration_ / 4.0;
                for (int iter = 0; iter < 30; ++iter) {
                    double f = 2.0 * t1 * t1 * t1 - duration_ * t1 * t1 + distance_ / max_jerk_;
                    double df = 6.0 * t1 * t1 - 2.0 * duration_ * t1;
                    if (std::abs(df) < 1e-12) break;
                    double t1_new = t1 - f / df;
                    // Clamp to valid range [0, T/2]
                    t1_new = std::clamp(t1_new, 0.001, duration_ / 2.0);
                    if (std::abs(t1_new - t1) < 1e-9) {
                        t1 = t1_new;
                        break;
                    }
                    t1 = t1_new;
                }
                v_peak = max_jerk_ * t1 * t1;
                T_accel = 2.0 * t1;
            } else {
                // No room for cruise, use max triangular
                v_peak = max_jerk_ * t1_no_cruise * t1_no_cruise;
                T_accel = 2.0 * t1_no_cruise;
            }
        }
    }

    spdlog::info("  After analytical: v_peak={:.4f}, T_accel={:.4f}", v_peak, T_accel);

    // Validate that we got a reasonable v_peak
    // If analytical solution failed, fall back to using max capability
    if (v_peak < 0.001 || !std::isfinite(v_peak)) {
        spdlog::warn("Symmetric SCurve: analytical solution failed (v_peak={:.6f}), using max capability", v_peak);
        if (T_half > 2.0 * t_j) {
            double t2_max = T_half - 2.0 * t_j;
            v_peak = max_accel_ * (t_j + t2_max);
        } else {
            double t1_max = T_half / 2.0;
            v_peak = max_jerk_ * t1_max * t1_max;
        }
        T_accel = T_half;
        spdlog::info("  After fallback: v_peak={:.4f}, T_accel={:.4f}", v_peak, T_accel);
    }

    // Compute actual phase times with final v_peak
    if (v_peak <= dv_triangular) {
        // Triangular accel/decel
        double t1 = std::sqrt(v_peak / max_jerk_);
        t1_ = t1;
        t2_ = 0.0;
        t3_ = t1;
        t5_ = t1;
        t6_ = 0.0;
        t7_ = t1;
        a1_ = max_jerk_ * t1;  // Peak accel (positive)
        a5_ = -a1_;            // Peak decel (negative)
    } else {
        // Trapezoidal accel/decel
        double t2 = (v_peak - dv_triangular) / max_accel_;
        t1_ = t_j;
        t2_ = t2;
        t3_ = t_j;
        t5_ = t_j;
        t6_ = t2;
        t7_ = t_j;
        a1_ = max_accel_;
        a5_ = -max_accel_;
    }

    // Cruise time is whatever remains
    t4_ = duration_ - 2.0 * T_accel;
    if (t4_ < 0.0) t4_ = 0.0;

    // Compute states at phase boundaries
    // Phase 1 (jerk up): j = +J, a: 0 -> a1_
    double j1 = (t1_ > 0.001) ? a1_ / t1_ : 0.0;
    v1_ = 0.5 * j1 * t1_ * t1_;  // Starting from v=0
    s1_ = (1.0/6.0) * j1 * t1_ * t1_ * t1_;

    // Phase 2 (constant accel): a = a1_
    a2_ = a1_;
    v2_ = v1_ + a1_ * t2_;
    s2_ = s1_ + v1_ * t2_ + 0.5 * a1_ * t2_ * t2_;

    // Phase 3 (jerk down): j = -J, a: a1_ -> 0
    double j3 = (t3_ > 0.001) ? -a1_ / t3_ : 0.0;
    v3_ = v2_ + a1_ * t3_ + 0.5 * j3 * t3_ * t3_;
    s3_ = s2_ + v2_ * t3_ + 0.5 * a1_ * t3_ * t3_ + (1.0/6.0) * j3 * t3_ * t3_ * t3_;
    a3_ = 0.0;

    // v3_ should equal v_peak
    // Phase 4 (cruise): a = 0, v = v_peak
    v4_ = v3_;
    s4_ = s3_ + v4_ * t4_;

    // Phase 5 (jerk down): j = -J, a: 0 -> a5_ (negative)
    double j5 = (t5_ > 0.001) ? a5_ / t5_ : 0.0;
    v5_ = v4_ + 0.5 * j5 * t5_ * t5_;
    s5_ = s4_ + v4_ * t5_ + (1.0/6.0) * j5 * t5_ * t5_ * t5_;

    // Phase 6 (constant decel): a = a5_
    a6_ = a5_;
    v6_ = v5_ + a5_ * t6_;
    s6_ = s5_ + v5_ * t6_ + 0.5 * a5_ * t6_ * t6_;

    // Phase 7 (jerk up): j = +J, a: a5_ -> 0
    double j7 = (t7_ > 0.001) ? -a5_ / t7_ : 0.0;
    v7_ = v6_ + a5_ * t7_ + 0.5 * j7 * t7_ * t7_;
    s7_ = s6_ + v6_ * t7_ + 0.5 * a5_ * t7_ * t7_ + (1.0/6.0) * j7 * t7_ * t7_ * t7_;

    valid_ = true;
}

void SCurveProfile::computeCumulativeTimes() {
    if (type_ == ProfileType::Symmetric) {
        // Order: 1, 2, 3, 4, 5, 6, 7 (accel, cruise, decel)
        T1_ = t1_;
        T2_ = T1_ + t2_;
        T3_ = T2_ + t3_;
        T4_ = T3_ + t4_;
        T5_ = T4_ + t5_;
        T6_ = T5_ + t6_;
        T7_ = T6_ + t7_;
    } else if (type_ == ProfileType::Acceleration) {
        // Order: 1, 2, 3, 4 (accel then cruise)
        T1_ = t1_;
        T2_ = T1_ + t2_;
        T3_ = T2_ + t3_;
        T4_ = T3_ + t4_;
        T5_ = T4_;
        T6_ = T4_;
        T7_ = T4_;
    } else {
        // Order: 4, 5, 6, 7 (cruise then decel) or just 4 for cruise only
        T1_ = 0.0;
        T2_ = 0.0;
        T3_ = 0.0;
        T4_ = t4_;
        T5_ = T4_ + t5_;
        T6_ = T5_ + t6_;
        T7_ = T6_ + t7_;
    }
}

void SCurveProfile::verifyDistanceConstraint() {
    // Verify that computed distance matches requested distance
    double computed_distance;

    if (type_ == ProfileType::CruiseOnly) {
        computed_distance = s4_;
    } else if (type_ == ProfileType::Acceleration) {
        computed_distance = s4_;
    } else if (type_ == ProfileType::Symmetric) {
        computed_distance = s7_;
    } else {  // Deceleration
        computed_distance = s7_;
    }

    double error = std::abs(computed_distance - distance_);
    double tolerance = 0.001 * distance_ + 0.0001;  // 0.1% + 0.1mm

    if (error > tolerance) {
        spdlog::warn("SCurve distance mismatch: computed={:.4f}m, requested={:.4f}m, error={:.4f}m",
            computed_distance, distance_, error);
        // Adjust - the analytical entry velocity should ensure distance matches
        // If there's a mismatch, it's likely due to rounding in phase times
    }
}

SCurveState SCurveProfile::evaluate(double t) const {
    t = std::clamp(t, 0.0, duration_);

    if (type_ == ProfileType::CruiseOnly) {
        return evaluateCruise(t);
    } else if (type_ == ProfileType::Symmetric) {
        // Full 7-phase profile: accel (1,2,3), cruise (4), decel (5,6,7)
        if (t <= T1_) return evaluateAccelPhase1(t);
        if (t <= T2_) return evaluateAccelPhase2(t - T1_);
        if (t <= T3_) return evaluateAccelPhase3(t - T2_);
        if (t <= T4_) return evaluateSymmetricCruise(t - T3_);
        if (t <= T5_) return evaluateSymmetricDecelPhase5(t - T4_);
        if (t <= T6_) return evaluateSymmetricDecelPhase6(t - T5_);
        return evaluateSymmetricDecelPhase7(t - T6_);
    } else if (type_ == ProfileType::Acceleration) {
        if (t <= T1_) return evaluateAccelPhase1(t);
        if (t <= T2_) return evaluateAccelPhase2(t - T1_);
        if (t <= T3_) return evaluateAccelPhase3(t - T2_);
        return evaluateCruise(t - T3_);  // Phase 4 cruise
    } else {  // Deceleration
        if (t <= T4_) return evaluateCruise(t);  // Phase 4 cruise first
        if (t <= T5_) return evaluateDecelPhase5(t - T4_);
        if (t <= T6_) return evaluateDecelPhase6(t - T5_);
        return evaluateDecelPhase7(t - T6_);
    }
}

SCurveState SCurveProfile::evaluateCruise(double dt) const {
    SCurveState state;
    if (type_ == ProfileType::Acceleration) {
        // Cruise after acceleration (at v_exit)
        state.position = s3_ + v3_ * dt;
        state.velocity = v3_;
    } else {
        // Cruise before deceleration (at v_entry) or cruise-only
        state.position = v_entry_ * dt;
        state.velocity = v_entry_;
    }
    state.acceleration = 0.0;
    return state;
}

SCurveState SCurveProfile::evaluateDecelPhase5(double dt) const {
    // Jerk down: j = -J (or scaled), a goes from 0 to a5_ (negative)
    double j = (t5_ > 0.001) ? a5_ / t5_ : 0.0;  // j is negative

    SCurveState state;
    state.acceleration = j * dt;
    state.velocity = v4_ + 0.5 * j * dt * dt;
    state.position = s4_ + v4_ * dt + (1.0/6.0) * j * dt * dt * dt;
    return state;
}

SCurveState SCurveProfile::evaluateDecelPhase6(double dt) const {
    // Constant decel: a = a5_ (negative)
    SCurveState state;
    state.acceleration = a5_;
    state.velocity = v5_ + a5_ * dt;
    state.position = s5_ + v5_ * dt + 0.5 * a5_ * dt * dt;
    return state;
}

SCurveState SCurveProfile::evaluateDecelPhase7(double dt) const {
    // Jerk up: j = +J (or scaled), a goes from a5_ to 0
    double j = (t7_ > 0.001) ? -a5_ / t7_ : 0.0;  // j is positive

    SCurveState state;
    state.acceleration = a6_ + j * dt;
    state.velocity = v6_ + a6_ * dt + 0.5 * j * dt * dt;
    state.position = s6_ + v6_ * dt + 0.5 * a6_ * dt * dt + (1.0/6.0) * j * dt * dt * dt;
    return state;
}

SCurveState SCurveProfile::evaluateAccelPhase1(double dt) const {
    // Jerk up: j = +J (or scaled), a goes from 0 to a1_ (positive)
    double j = (t1_ > 0.001) ? a1_ / t1_ : 0.0;

    SCurveState state;
    state.acceleration = j * dt;
    state.velocity = v_entry_ + 0.5 * j * dt * dt;
    state.position = v_entry_ * dt + (1.0/6.0) * j * dt * dt * dt;
    return state;
}

SCurveState SCurveProfile::evaluateAccelPhase2(double dt) const {
    // Constant accel: a = a1_ (positive)
    SCurveState state;
    state.acceleration = a1_;
    state.velocity = v1_ + a1_ * dt;
    state.position = s1_ + v1_ * dt + 0.5 * a1_ * dt * dt;
    return state;
}

SCurveState SCurveProfile::evaluateAccelPhase3(double dt) const {
    // Jerk down: j = -J (or scaled), a goes from a1_ to 0
    double j = (t3_ > 0.001) ? -a1_ / t3_ : 0.0;

    SCurveState state;
    state.acceleration = a2_ + j * dt;
    state.velocity = v2_ + a2_ * dt + 0.5 * j * dt * dt;
    state.position = s2_ + v2_ * dt + 0.5 * a2_ * dt * dt + (1.0/6.0) * j * dt * dt * dt;
    return state;
}

SCurveState SCurveProfile::evaluateSymmetricCruise(double dt) const {
    // Cruise after acceleration at v_peak (v3_)
    SCurveState state;
    state.position = s3_ + v3_ * dt;
    state.velocity = v3_;
    state.acceleration = 0.0;
    return state;
}

SCurveState SCurveProfile::evaluateSymmetricDecelPhase5(double dt) const {
    // Jerk down: j = -J, a goes from 0 to a5_ (negative)
    // Starts from end of cruise phase (s4_, v4_)
    double j = (t5_ > 0.001) ? a5_ / t5_ : 0.0;  // j is negative

    SCurveState state;
    state.acceleration = j * dt;
    state.velocity = v4_ + 0.5 * j * dt * dt;
    state.position = s4_ + v4_ * dt + (1.0/6.0) * j * dt * dt * dt;
    return state;
}

SCurveState SCurveProfile::evaluateSymmetricDecelPhase6(double dt) const {
    // Constant decel: a = a5_ (negative)
    SCurveState state;
    state.acceleration = a5_;
    state.velocity = v5_ + a5_ * dt;
    state.position = s5_ + v5_ * dt + 0.5 * a5_ * dt * dt;
    return state;
}

SCurveState SCurveProfile::evaluateSymmetricDecelPhase7(double dt) const {
    // Jerk up: j = +J, a goes from a5_ to 0
    double j = (t7_ > 0.001) ? -a5_ / t7_ : 0.0;  // j is positive

    SCurveState state;
    state.acceleration = a6_ + j * dt;
    state.velocity = v6_ + a6_ * dt + 0.5 * j * dt * dt;
    state.position = s6_ + v6_ * dt + 0.5 * a6_ * dt * dt + (1.0/6.0) * j * dt * dt * dt;
    return state;
}

}  // namespace ur_controller::trajectory
