/// @file test_s_curve.cpp
/// @brief Unit tests for S-curve velocity profile

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "ur_controller/trajectory/s_curve.hpp"

#include <cmath>

using namespace ur_controller::trajectory;
using Catch::Approx;

// =============================================================================
// Basic Profile Creation
// =============================================================================

TEST_CASE("SCurveProfile construction", "[s_curve]") {
    SECTION("valid profile is created") {
        // v_entry=0.2, v_exit=0.0, D=0.1, T=1.0, j_max=10.0
        SCurveProfile profile(0.2, 0.0, 0.1, 1.0, 10.0);

        REQUIRE(profile.isFeasible());
        REQUIRE(profile.duration() == Approx(1.0));
        REQUIRE(profile.entryVelocity() == Approx(0.2));
        REQUIRE(profile.exitVelocity() == Approx(0.0));
    }

    SECTION("zero duration is infeasible") {
        SCurveProfile profile(0.1, 0.0, 0.05, 0.0, 10.0);
        REQUIRE_FALSE(profile.isFeasible());
    }

    SECTION("jerk within limits is feasible") {
        // ΔV = 0.2, T = 1.0 => J_required = 4 * 0.2 / 1 = 0.8
        SCurveProfile profile(0.0, 0.2, 0.1, 1.0, 1.0);
        REQUIRE(profile.isFeasible());
        REQUIRE(profile.actualJerk() == Approx(0.8));
    }

    SECTION("jerk exceeds limits is infeasible") {
        // ΔV = 0.2, T = 0.1 => J_required = 4 * 0.2 / 0.01 = 80
        SCurveProfile profile(0.0, 0.2, 0.01, 0.1, 10.0);
        REQUIRE_FALSE(profile.isFeasible());
    }
}

// =============================================================================
// Deceleration Profile (v_entry > v_exit)
// =============================================================================

TEST_CASE("SCurveProfile deceleration", "[s_curve][decel]") {
    double v_entry = 0.2;
    double v_exit = 0.0;
    double duration = 1.0;
    double distance = (v_entry + v_exit) / 2.0 * duration;  // 0.1 m
    double j_max = 10.0;

    SCurveProfile profile(v_entry, v_exit, distance, duration, j_max);

    REQUIRE(profile.isFeasible());

    SECTION("boundary conditions") {
        REQUIRE(profile.velocity(0.0) == Approx(v_entry));
        REQUIRE(profile.velocity(duration) == Approx(v_exit).margin(0.001));
        REQUIRE(profile.position(0.0) == Approx(0.0));
    }

    SECTION("velocity decreases from entry to exit") {
        double mid_vel = profile.velocity(duration / 2.0);
        REQUIRE(mid_vel < v_entry);
        REQUIRE(mid_vel > v_exit);
    }

    SECTION("distance is correct") {
        REQUIRE(profile.position(duration) == Approx(distance).margin(0.001));
    }

    SECTION("acceleration starts and ends at zero") {
        REQUIRE(profile.acceleration(0.0) == Approx(0.0));
        REQUIRE(profile.acceleration(duration) == Approx(0.0).margin(0.001));
    }

    SECTION("peak acceleration at midpoint") {
        double a_mid = profile.acceleration(duration / 2.0);
        double a_start = profile.acceleration(0.0);
        double a_end = profile.acceleration(duration);

        // For deceleration, peak acceleration should be negative
        REQUIRE(a_mid < a_start);
        REQUIRE(a_mid < a_end);
    }
}

// =============================================================================
// Acceleration Profile (v_entry < v_exit)
// =============================================================================

TEST_CASE("SCurveProfile acceleration", "[s_curve][accel]") {
    double v_entry = 0.0;
    double v_exit = 0.2;
    double duration = 1.0;
    double distance = (v_entry + v_exit) / 2.0 * duration;  // 0.1 m
    double j_max = 10.0;

    SCurveProfile profile(v_entry, v_exit, distance, duration, j_max);

    REQUIRE(profile.isFeasible());

    SECTION("boundary conditions") {
        REQUIRE(profile.velocity(0.0) == Approx(v_entry));
        REQUIRE(profile.velocity(duration) == Approx(v_exit).margin(0.001));
    }

    SECTION("velocity increases from entry to exit") {
        double mid_vel = profile.velocity(duration / 2.0);
        REQUIRE(mid_vel > v_entry);
        REQUIRE(mid_vel < v_exit);
    }

    SECTION("peak acceleration is positive") {
        double a_mid = profile.acceleration(duration / 2.0);
        REQUIRE(a_mid > 0.0);
    }
}

// =============================================================================
// Constant Velocity Profile (v_entry ≈ v_exit)
// =============================================================================

TEST_CASE("SCurveProfile constant velocity", "[s_curve][const]") {
    double velocity = 0.1;
    double duration = 1.0;
    double distance = velocity * duration;
    double j_max = 10.0;

    SCurveProfile profile(velocity, velocity, distance, duration, j_max);

    REQUIRE(profile.isFeasible());
    REQUIRE(profile.actualJerk() == Approx(0.0).margin(0.0001));

    SECTION("velocity is constant") {
        for (double t = 0.0; t <= duration; t += 0.1) {
            REQUIRE(profile.velocity(t) == Approx(velocity).margin(0.001));
        }
    }

    SECTION("acceleration is zero") {
        for (double t = 0.0; t <= duration; t += 0.1) {
            REQUIRE(profile.acceleration(t) == Approx(0.0).margin(0.001));
        }
    }
}

// =============================================================================
// Symmetric (Rest-to-Rest) Profile
// =============================================================================

TEST_CASE("SCurveProfile symmetric rest-to-rest", "[s_curve][symmetric]") {
    double v_entry = 0.0;
    double v_exit = 0.0;
    double distance = 0.1;
    double duration = 1.0;
    double j_max = 10.0;

    SCurveProfile profile(v_entry, v_exit, distance, duration, j_max);

    // Symmetric profile: accelerate to v_peak, then decelerate
    // v_peak = 2 * D / T = 0.2 m/s
    // jerk = 16 * v_peak / T² = 3.2 m/s³
    REQUIRE(profile.isFeasible());
    REQUIRE(profile.actualJerk() == Approx(3.2).margin(0.001));
    REQUIRE(profile.peakVelocity() == Approx(0.2).margin(0.001));

    SECTION("boundary conditions") {
        REQUIRE(profile.velocity(0.0) == Approx(v_entry));
        REQUIRE(profile.velocity(duration) == Approx(v_exit).margin(0.001));
        REQUIRE(profile.position(0.0) == Approx(0.0));
        REQUIRE(profile.position(duration) == Approx(distance).margin(0.001));
    }

    SECTION("peak velocity at midpoint") {
        REQUIRE(profile.velocity(duration / 2.0) == Approx(0.2).margin(0.001));
    }

    SECTION("acceleration starts and ends at zero") {
        REQUIRE(profile.acceleration(0.0) == Approx(0.0));
        REQUIRE(profile.acceleration(duration) == Approx(0.0).margin(0.001));
    }

    SECTION("velocity is non-negative throughout") {
        for (double t = 0.0; t <= duration; t += 0.05) {
            REQUIRE(profile.velocity(t) >= -0.001);
        }
    }
}

// =============================================================================
// Static Utility Functions
// =============================================================================

TEST_CASE("SCurveProfile static utilities", "[s_curve][static]") {
    SECTION("computeEntryVelocity") {
        // D = 0.1, T = 1.0, v_exit = 0.0 => v_entry = 0.2
        double v_entry = SCurveProfile::computeEntryVelocity(0.0, 0.1, 1.0);
        REQUIRE(v_entry == Approx(0.2));

        // D = 0.15, T = 1.0, v_exit = 0.1 => v_entry = 0.2
        v_entry = SCurveProfile::computeEntryVelocity(0.1, 0.15, 1.0);
        REQUIRE(v_entry == Approx(0.2));
    }

    SECTION("minimumTime") {
        // ΔV = 1.0, J = 4.0 => T_min = 2 * sqrt(1/4) = 1.0
        double t_min = SCurveProfile::minimumTime(1.0, 4.0);
        REQUIRE(t_min == Approx(1.0));

        // ΔV = 0.2, J = 10.0 => T_min = 2 * sqrt(0.02) ≈ 0.283
        t_min = SCurveProfile::minimumTime(0.2, 10.0);
        REQUIRE(t_min == Approx(0.2828).margin(0.001));
    }

    SECTION("isVelocityChangeFeasible") {
        // ΔV = 0.8, T = 1.0, J = 4.0 => max_ΔV = 4 * 1 / 4 = 1.0
        REQUIRE(SCurveProfile::isVelocityChangeFeasible(0.8, 1.0, 4.0));
        REQUIRE(SCurveProfile::isVelocityChangeFeasible(1.0, 1.0, 4.0));
        REQUIRE_FALSE(SCurveProfile::isVelocityChangeFeasible(1.1, 1.0, 4.0));
    }
}

// =============================================================================
// Position Continuity and Monotonicity
// =============================================================================

TEST_CASE("SCurveProfile position continuity", "[s_curve][continuity]") {
    SCurveProfile profile(0.2, 0.0, 0.1, 1.0, 10.0);

    REQUIRE(profile.isFeasible());

    SECTION("position is continuous") {
        double dt = 0.001;
        auto prev = profile.position(0.0);

        for (double t = dt; t <= profile.duration(); t += dt) {
            auto curr = profile.position(t);
            double jump = std::abs(curr - prev);

            // No large jumps (should be smooth)
            REQUIRE(jump < 0.01);
            prev = curr;
        }
    }

    SECTION("position is monotonically increasing") {
        double dt = 0.01;
        double prev_pos = -0.001;

        for (double t = 0.0; t <= profile.duration(); t += dt) {
            double pos = profile.position(t);
            REQUIRE(pos >= prev_pos - 0.0001);
            prev_pos = pos;
        }
    }
}

// =============================================================================
// Backward Propagation
// =============================================================================

TEST_CASE("SCurveProfile backward propagation roundtrip", "[s_curve][backward]") {
    double v_exit = 0.0;
    double D = 0.1;
    double T = 1.0;
    double j_max = 10.0;

    // Compute entry velocity using static function
    double v_entry = SCurveProfile::computeEntryVelocity(v_exit, D, T);

    // Create profile with computed entry velocity
    SCurveProfile profile(v_entry, v_exit, D, T, j_max);

    REQUIRE(profile.isFeasible());
    REQUIRE(profile.position(T) == Approx(D).margin(0.001));
    REQUIRE(profile.velocity(T) == Approx(v_exit).margin(0.001));
}

// =============================================================================
// Edge Cases
// =============================================================================

TEST_CASE("SCurveProfile edge cases", "[s_curve][edge]") {
    SECTION("time clamping") {
        SCurveProfile profile(0.2, 0.0, 0.1, 1.0, 10.0);

        // Negative time clamps to 0
        REQUIRE(profile.position(-0.5) == Approx(profile.position(0.0)));
        REQUIRE(profile.velocity(-0.5) == Approx(profile.velocity(0.0)));

        // Time > duration clamps to duration
        REQUIRE(profile.position(1.5) == Approx(profile.position(1.0)));
        REQUIRE(profile.velocity(1.5) == Approx(profile.velocity(1.0)));
    }

    SECTION("very small velocity change") {
        SCurveProfile profile(0.1, 0.1001, 0.10005, 1.0, 10.0);

        REQUIRE(profile.isFeasible());
        REQUIRE(profile.actualJerk() < 0.01);
    }
}

// =============================================================================
// Jerk Profile
// =============================================================================

TEST_CASE("SCurveProfile jerk profile", "[s_curve][jerk]") {
    SCurveProfile profile(0.0, 0.2, 0.1, 1.0, 10.0);

    REQUIRE(profile.isFeasible());

    SECTION("jerk is positive in phase 1, negative in phase 2") {
        double j_phase1 = profile.jerk(0.25);  // First quarter
        double j_phase2 = profile.jerk(0.75);  // Third quarter

        REQUIRE(j_phase1 > 0.0);
        REQUIRE(j_phase2 < 0.0);
        REQUIRE(std::abs(j_phase1) == Approx(std::abs(j_phase2)));
    }

    SECTION("jerk magnitude matches actualJerk") {
        double j = profile.jerk(0.25);
        REQUIRE(std::abs(j) == Approx(profile.actualJerk()));
    }
}
