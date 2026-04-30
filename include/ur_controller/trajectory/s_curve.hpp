/// @file s_curve.hpp
/// @brief Jerk-limited S-curve velocity profile for trajectory segments

#pragma once

#include <cmath>
#include <algorithm>

namespace ur_controller {
namespace trajectory {

/// @brief Jerk-limited S-curve velocity profile
///
/// Computes a smooth velocity transition from v_entry to v_exit over a given
/// duration. Two modes are supported:
///
/// 1. **Asymmetric mode** (v_entry ≠ v_exit): Two phases
///    - Phase 1: Constant jerk (acceleration ramps up or down)
///    - Phase 2: Opposite jerk (acceleration returns to zero)
///
/// 2. **Symmetric mode** (v_entry ≈ v_exit): Four phases (accelerate then decelerate)
///    - Phase 1-2: S-curve from v_entry to v_peak
///    - Phase 3-4: S-curve from v_peak to v_exit
///
/// Both modes produce smooth acceleration that starts and ends at zero.
///
/// Usage:
/// @code
///   SCurveProfile profile(0.2, 0.0, 0.1, 1.0, 10.0);
///   if (profile.isFeasible()) {
///       double pos = profile.position(0.5);
///       double vel = profile.velocity(0.5);
///   }
/// @endcode
class SCurveProfile {
public:
    /// @brief Create a jerk-limited S-curve profile
    /// @param v_entry Starting velocity (m/s)
    /// @param v_exit Ending velocity (m/s)
    /// @param distance Segment distance (m)
    /// @param duration Segment time (s)
    /// @param j_max Maximum allowed jerk (m/s³)
    SCurveProfile(
        double v_entry,
        double v_exit,
        double distance,
        double duration,
        double j_max)
        : v_entry_(v_entry)
        , v_exit_(v_exit)
        , distance_(distance)
        , duration_(duration)
        , j_max_(j_max)
    {
        compute();
    }

    /// @brief Check if the profile is feasible within jerk limits
    [[nodiscard]] bool isFeasible() const { return feasible_; }

    /// @brief Get the actual jerk used (may be less than j_max)
    [[nodiscard]] double actualJerk() const { return std::abs(jerk_); }

    /// @brief Get the peak acceleration achieved
    [[nodiscard]] double peakAcceleration() const {
        if (symmetric_) {
            return std::abs(jerk_) * t_quarter_;
        }
        return std::abs(jerk_) * t1_;
    }

    /// @brief Get the peak velocity (for symmetric profiles)
    [[nodiscard]] double peakVelocity() const { return v_peak_; }

    /// @brief Get the profile duration
    [[nodiscard]] double duration() const { return duration_; }

    /// @brief Get entry velocity
    [[nodiscard]] double entryVelocity() const { return v_entry_; }

    /// @brief Get exit velocity
    [[nodiscard]] double exitVelocity() const { return v_exit_; }

    /// @brief Evaluate position (distance traveled) at time t
    /// @param t Time from segment start (0 to duration)
    [[nodiscard]] double position(double t) const {
        t = std::clamp(t, 0.0, duration_);

        if (symmetric_) {
            return positionSymmetric(t);
        }

        if (t <= t1_) {
            // Phase 1: jerk = jerk_
            return v_entry_ * t + (1.0/6.0) * jerk_ * t * t * t;
        } else {
            // Phase 2: jerk = -jerk_
            double dt = t - t1_;
            return s1_ + v1_ * dt + 0.5 * a1_ * dt * dt - (1.0/6.0) * jerk_ * dt * dt * dt;
        }
    }

    /// @brief Evaluate velocity at time t
    [[nodiscard]] double velocity(double t) const {
        t = std::clamp(t, 0.0, duration_);

        if (symmetric_) {
            return velocitySymmetric(t);
        }

        if (t <= t1_) {
            // Phase 1: jerk = jerk_
            return v_entry_ + 0.5 * jerk_ * t * t;
        } else {
            // Phase 2: jerk = -jerk_
            double dt = t - t1_;
            return v1_ + a1_ * dt - 0.5 * jerk_ * dt * dt;
        }
    }

    /// @brief Evaluate acceleration at time t
    [[nodiscard]] double acceleration(double t) const {
        t = std::clamp(t, 0.0, duration_);

        if (symmetric_) {
            return accelerationSymmetric(t);
        }

        if (t <= t1_) {
            return jerk_ * t;
        } else {
            double dt = t - t1_;
            return a1_ - jerk_ * dt;
        }
    }

    /// @brief Evaluate jerk at time t
    [[nodiscard]] double jerk(double t) const {
        t = std::clamp(t, 0.0, duration_);

        if (symmetric_) {
            return jerkSymmetric(t);
        }

        return (t <= t1_) ? jerk_ : -jerk_;
    }

    // =========================================================================
    // Static utilities for trajectory planning
    // =========================================================================

    /// @brief Compute entry velocity for backward propagation
    /// @param v_exit Exit velocity (m/s)
    /// @param distance Segment distance (m)
    /// @param duration Segment time (s)
    /// @return Entry velocity (m/s)
    [[nodiscard]] static double computeEntryVelocity(
        double v_exit,
        double distance,
        double duration)
    {
        // D = (v_entry + v_exit) * T / 2
        // v_entry = 2*D/T - v_exit
        return 2.0 * distance / duration - v_exit;
    }

    /// @brief Compute minimum time for a velocity change given jerk limit
    /// @param delta_v Velocity change magnitude (m/s)
    /// @param j_max Maximum jerk (m/s³)
    /// @return Minimum time required (s)
    [[nodiscard]] static double minimumTime(double delta_v, double j_max) {
        // ΔV = J * T² / 4
        // T = 2 * sqrt(ΔV / J)
        return 2.0 * std::sqrt(std::abs(delta_v) / j_max);
    }

    /// @brief Check if a velocity change is feasible within time and jerk limits
    [[nodiscard]] static bool isVelocityChangeFeasible(
        double delta_v,
        double duration,
        double j_max)
    {
        // ΔV ≤ J * T² / 4
        double max_delta_v = j_max * duration * duration / 4.0;
        return std::abs(delta_v) <= max_delta_v;
    }

private:
    double v_entry_;
    double v_exit_;
    double distance_;
    double duration_;
    double j_max_;

    double jerk_{0.0};      ///< Actual jerk used (signed, for accel phase in symmetric mode)
    double t1_{0.0};        ///< Time of first phase (half of duration for asymmetric)
    bool feasible_{false};
    bool symmetric_{false}; ///< True if using 4-phase symmetric profile

    // Cached values at phase boundary (t = t1) for asymmetric mode
    double a1_{0.0};        ///< Acceleration at end of phase 1
    double v1_{0.0};        ///< Velocity at end of phase 1
    double s1_{0.0};        ///< Position at end of phase 1

    // Symmetric mode parameters
    double v_peak_{0.0};    ///< Peak velocity at midpoint
    double t_quarter_{0.0}; ///< T/4 - each jerk phase duration
    double t_half_{0.0};    ///< T/2 - midpoint
    double s_half_{0.0};    ///< Position at midpoint

    // Symmetric mode helper: position
    [[nodiscard]] double positionSymmetric(double t) const {
        if (t <= t_quarter_) {
            // Phase 1: jerk = +jerk_ (accelerating, accel increasing)
            return v_entry_ * t + (1.0/6.0) * jerk_ * t * t * t;
        } else if (t <= t_half_) {
            // Phase 2: jerk = -jerk_ (accelerating, accel decreasing)
            double dt = t - t_quarter_;
            // State at t_quarter
            double a_q = jerk_ * t_quarter_;
            double v_q = v_entry_ + 0.5 * jerk_ * t_quarter_ * t_quarter_;
            double s_q = v_entry_ * t_quarter_ + (1.0/6.0) * jerk_ * t_quarter_ * t_quarter_ * t_quarter_;
            return s_q + v_q * dt + 0.5 * a_q * dt * dt - (1.0/6.0) * jerk_ * dt * dt * dt;
        } else if (t <= t_half_ + t_quarter_) {
            // Phase 3: jerk = -jerk_ (decelerating, accel decreasing/going negative)
            double dt = t - t_half_;
            return s_half_ + v_peak_ * dt - (1.0/6.0) * jerk_ * dt * dt * dt;
        } else {
            // Phase 4: jerk = +jerk_ (decelerating, accel increasing toward zero)
            double dt = t - t_half_ - t_quarter_;
            // State at 3*t_quarter
            double a_3q = -jerk_ * t_quarter_;
            double v_3q = v_peak_ - 0.5 * jerk_ * t_quarter_ * t_quarter_;
            double s_3q = s_half_ + v_peak_ * t_quarter_ - (1.0/6.0) * jerk_ * t_quarter_ * t_quarter_ * t_quarter_;
            return s_3q + v_3q * dt + 0.5 * a_3q * dt * dt + (1.0/6.0) * jerk_ * dt * dt * dt;
        }
    }

    // Symmetric mode helper: velocity
    [[nodiscard]] double velocitySymmetric(double t) const {
        if (t <= t_quarter_) {
            // Phase 1: jerk = +jerk_
            return v_entry_ + 0.5 * jerk_ * t * t;
        } else if (t <= t_half_) {
            // Phase 2: jerk = -jerk_
            double dt = t - t_quarter_;
            double a_q = jerk_ * t_quarter_;
            double v_q = v_entry_ + 0.5 * jerk_ * t_quarter_ * t_quarter_;
            return v_q + a_q * dt - 0.5 * jerk_ * dt * dt;
        } else if (t <= t_half_ + t_quarter_) {
            // Phase 3: jerk = -jerk_
            double dt = t - t_half_;
            return v_peak_ - 0.5 * jerk_ * dt * dt;
        } else {
            // Phase 4: jerk = +jerk_
            double dt = t - t_half_ - t_quarter_;
            double a_3q = -jerk_ * t_quarter_;
            double v_3q = v_peak_ - 0.5 * jerk_ * t_quarter_ * t_quarter_;
            return v_3q + a_3q * dt + 0.5 * jerk_ * dt * dt;
        }
    }

    // Symmetric mode helper: acceleration
    [[nodiscard]] double accelerationSymmetric(double t) const {
        if (t <= t_quarter_) {
            return jerk_ * t;
        } else if (t <= t_half_) {
            double dt = t - t_quarter_;
            return jerk_ * t_quarter_ - jerk_ * dt;
        } else if (t <= t_half_ + t_quarter_) {
            double dt = t - t_half_;
            return -jerk_ * dt;
        } else {
            double dt = t - t_half_ - t_quarter_;
            return -jerk_ * t_quarter_ + jerk_ * dt;
        }
    }

    // Symmetric mode helper: jerk
    [[nodiscard]] double jerkSymmetric(double t) const {
        if (t <= t_quarter_) {
            return jerk_;
        } else if (t <= t_half_) {
            return -jerk_;
        } else if (t <= t_half_ + t_quarter_) {
            return -jerk_;
        } else {
            return jerk_;
        }
    }

    void compute() {
        if (duration_ <= 0.0) {
            feasible_ = false;
            return;
        }

        double delta_v = v_exit_ - v_entry_;

        // Check if this is a symmetric profile (v_entry ≈ v_exit)
        // For symmetric profiles, we need to accelerate to v_peak then decelerate
        // BUT: if average velocity matches v_entry, it's truly constant velocity
        constexpr double kVelocityTolerance = 1e-6;
        double avg_velocity = distance_ / duration_;
        bool is_constant_velocity = std::abs(avg_velocity - v_entry_) < kVelocityTolerance;

        if (std::abs(delta_v) < kVelocityTolerance && !is_constant_velocity) {
            computeSymmetric();
            return;
        }

        // Asymmetric profile: standard 2-phase S-curve
        symmetric_ = false;
        t1_ = duration_ / 2.0;

        // Compute required jerk: ΔV = J * T² / 4 => J = 4 * ΔV / T²
        jerk_ = 4.0 * delta_v / (duration_ * duration_);

        // Check feasibility
        feasible_ = std::abs(jerk_) <= j_max_;

        // Compute state at end of phase 1
        a1_ = jerk_ * t1_;
        v1_ = v_entry_ + 0.5 * jerk_ * t1_ * t1_;
        s1_ = v_entry_ * t1_ + (1.0/6.0) * jerk_ * t1_ * t1_ * t1_;

        // Set peak velocity (for asymmetric, it's v1)
        v_peak_ = v1_;
    }

    void computeSymmetric() {
        symmetric_ = true;
        t_half_ = duration_ / 2.0;
        t_quarter_ = duration_ / 4.0;

        // For symmetric profile: D = v_peak * T / 2 (triangle area)
        // v_peak = 2 * D / T
        v_peak_ = 2.0 * distance_ / duration_;

        // Velocity change in first half: ΔV = v_peak - v_entry
        double delta_v_half = v_peak_ - v_entry_;

        // Jerk for the acceleration phase (first half):
        // ΔV = J * (T/2)² / 4 = J * T² / 16
        // J = 16 * ΔV / T²
        jerk_ = 16.0 * delta_v_half / (duration_ * duration_);

        // Check feasibility
        feasible_ = std::abs(jerk_) <= j_max_ && v_peak_ >= 0.0;

        // Compute position at midpoint
        // First quarter state
        double v_q = v_entry_ + 0.5 * jerk_ * t_quarter_ * t_quarter_;
        double s_q = v_entry_ * t_quarter_ + (1.0/6.0) * jerk_ * t_quarter_ * t_quarter_ * t_quarter_;
        double a_q = jerk_ * t_quarter_;

        // Second quarter
        double dt = t_quarter_;
        s_half_ = s_q + v_q * dt + 0.5 * a_q * dt * dt - (1.0/6.0) * jerk_ * dt * dt * dt;
    }
};

}  // namespace trajectory
}  // namespace ur_controller
