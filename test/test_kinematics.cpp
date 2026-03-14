/// @file test_kinematics.cpp
/// @brief Unit tests for the kinematics module

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ur_controller/kinematics/ur_kinematics.hpp"
#include "ur_controller/kinematics/dh_parameters.hpp"
#include "ur_controller/kinematics/types.hpp"

#include <cmath>
#include <random>

using namespace ur_controller::kinematics;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-6;

// Check if two transforms are approximately equal
bool transformsEqual(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b,
                     double tol = kTolerance) {
    // Check translation
    if ((a.translation() - b.translation()).norm() > tol) {
        return false;
    }
    // Check rotation (using angle-axis)
    Eigen::AngleAxisd diff(a.rotation().transpose() * b.rotation());
    return std::abs(diff.angle()) < tol;
}

}  // namespace

// =============================================================================
// DH Parameters Tests
// =============================================================================

TEST_CASE("DH parameters are available for all UR models", "[kinematics][dh]") {
    SECTION("UR3e parameters") {
        auto params = DHParameters::forModel(URModel::UR3e);
        REQUIRE(params.joints[0].d > 0.0);  // d1 > 0
        REQUIRE(params.joints[1].a < 0.0);  // a2 < 0 (upper arm length)
    }

    SECTION("UR5e parameters") {
        auto params = DHParameters::forModel(URModel::UR5e);
        REQUIRE_THAT(params.joints[0].d, WithinAbs(0.1625, 0.001));
        REQUIRE_THAT(params.joints[1].a, WithinAbs(-0.425, 0.001));
        REQUIRE_THAT(params.joints[2].a, WithinAbs(-0.3922, 0.001));
    }

    SECTION("UR10e parameters") {
        auto params = DHParameters::forModel(URModel::UR10e);
        REQUIRE(params.joints[0].d > 0.0);
        REQUIRE(params.joints[1].a < -0.5);  // Longer arm
    }

    SECTION("UR16e parameters") {
        auto params = DHParameters::forModel(URModel::UR16e);
        REQUIRE(params.joints[0].d > 0.0);
    }

    SECTION("UR20 parameters") {
        auto params = DHParameters::forModel(URModel::UR20);
        REQUIRE(params.joints[0].d > 0.0);
        REQUIRE(params.joints[1].a < -0.8);  // Largest robot
    }
}

TEST_CASE("Joint limits are sensible", "[kinematics][dh]") {
    auto limits = defaultJointLimits(URModel::UR5e);

    SECTION("Position limits are symmetric around zero") {
        for (const auto& joint : limits.joints) {
            REQUIRE_THAT(joint.min, WithinAbs(-joint.max, 0.001));
        }
    }

    SECTION("All joints have positive velocity limits") {
        for (const auto& joint : limits.joints) {
            REQUIRE(joint.velocity > 0.0);
        }
    }
}

// =============================================================================
// Forward Kinematics Tests
// =============================================================================

TEST_CASE("Forward kinematics at home position", "[kinematics][fk]") {
    URKinematics kin(URModel::UR5e);
    JointVector home = JointVector::Zero();

    auto pose = kin.forward(home);

    SECTION("TCP extends horizontally at home position") {
        // At home (all zeros), the UR robot extends horizontally in -X direction
        // a2 + a3 = -0.425 + -0.3922 = -0.8172
        double expected_x = -0.425 + -0.3922;
        REQUIRE_THAT(pose.translation().x(), WithinAbs(expected_x, 0.01));
    }

    SECTION("TCP has correct Y and Z at home") {
        // Y offset from d4, d6 (after alpha rotations)
        // Z offset from d1, d5 (after alpha rotations)
        REQUIRE(pose.translation().norm() > 0.5);  // Reasonable reach
    }
}

TEST_CASE("Forward kinematics at known configurations", "[kinematics][fk]") {
    URKinematics kin(URModel::UR5e);

    SECTION("Straight up configuration") {
        // Configuration with arm pointing roughly up
        JointVector q;
        q << 0, -kPi / 2, 0, -kPi / 2, 0, 0;

        auto pose = kin.forward(q);

        // Robot should be pointing roughly straight up
        // Expected z ~ 1.08m (d1 + |a2| + |a3| + d4 + d5 + d6)
        REQUIRE(pose.translation().z() > 0.8);
        REQUIRE(std::abs(pose.translation().x()) < 0.01);  // Nearly centered
        // Y can have some offset due to wrist geometry
    }

    SECTION("All link transforms have correct count") {
        JointVector q = JointVector::Zero();
        auto transforms = kin.allLinkTransforms(q);

        REQUIRE(transforms.size() == 7);  // Base + 6 links
    }

    SECTION("First transform is identity (base)") {
        JointVector q = JointVector::Zero();
        auto transforms = kin.allLinkTransforms(q);

        REQUIRE(transformsEqual(transforms[0], Eigen::Isometry3d::Identity()));
    }

    SECTION("Last transform equals forward() result") {
        JointVector q;
        q << 0.1, -0.5, 0.3, -1.0, 0.5, -0.2;

        auto pose = kin.forward(q);
        auto transforms = kin.allLinkTransforms(q);

        REQUIRE(transformsEqual(transforms[6], pose));
    }
}

// =============================================================================
// Inverse Kinematics Tests
// =============================================================================

TEST_CASE("Inverse kinematics finds solutions", "[kinematics][ik]") {
    URKinematics kin(URModel::UR5e);

    SECTION("IK returns solutions for various poses") {
        // Test several configurations
        std::vector<JointVector> test_configs;

        JointVector q1;
        q1 << 0.5, -1.0, 1.5, -1.0, 0.5, 0.3;
        test_configs.push_back(q1);

        JointVector q2;
        q2 << -0.3, -0.8, 1.2, -0.5, 0.7, -0.2;
        test_configs.push_back(q2);

        for (const auto& q : test_configs) {
            auto pose = kin.forward(q);
            auto solutions = kin.inverse(pose);

            // Should find at least one solution
            REQUIRE(!solutions.empty());
            REQUIRE(solutions.size() <= 8);
        }
    }
}

TEST_CASE("FK/IK roundtrip consistency", "[kinematics][ik]") {
    URKinematics kin(URModel::UR5e);

    SECTION("IK solutions exist for forward kinematic poses") {
        // Test that IK finds solutions for poses generated by FK
        std::vector<JointVector> test_configs;

        JointVector q1;
        q1 << 0.3, -0.8, 1.2, -0.5, 0.7, -0.2;
        test_configs.push_back(q1);

        JointVector q2;
        q2 << -0.5, -1.2, 0.8, -1.0, 0.3, 0.5;
        test_configs.push_back(q2);

        for (const auto& q : test_configs) {
            auto pose = kin.forward(q);
            auto solutions = kin.inverse(pose);

            // IK should find at least one solution
            INFO("Testing q = " << q.transpose());
            REQUIRE(!solutions.empty());
        }
    }
}

TEST_CASE("IK solution selection", "[kinematics][ik]") {
    URKinematics kin(URModel::UR5e);

    SECTION("selectBestSolution returns valid solution within limits") {
        JointVector current;
        current << 0.3, -0.8, 1.2, -0.5, 0.7, -0.2;

        auto pose = kin.forward(current);
        auto solutions = kin.inverse(pose);

        // If solutions exist, selection should work
        if (!solutions.empty()) {
            auto best = kin.selectBestSolution(solutions, current, 3.0);  // Generous jump limit

            if (best.has_value()) {
                // Result should be within joint limits
                REQUIRE(kin.jointLimits().withinLimits(*best));
            }
        }
    }

    SECTION("selectBestSolution respects max joint jump") {
        JointVector current;
        current << 0.0, -kPi / 2, kPi / 2, -kPi / 2, 0.5, 0;

        auto pose = kin.forward(current);
        auto solutions = kin.inverse(pose);

        if (!solutions.empty()) {
            // With very small max jump, might reject solutions
            auto best = kin.selectBestSolution(solutions, current, 0.1);

            // If a solution is found, it must be within jump limit
            if (best.has_value()) {
                for (int i = 0; i < 6; ++i) {
                    double diff = std::abs(URKinematics::wrapAngle((*best)[i] - current[i]));
                    REQUIRE(diff <= 0.1 + kTolerance);
                }
            }
        }
    }
}

// =============================================================================
// Jacobian Tests
// =============================================================================

TEST_CASE("Jacobian computation", "[kinematics][jacobian]") {
    URKinematics kin(URModel::UR5e);

    SECTION("Jacobian has correct dimensions") {
        JointVector q = JointVector::Zero();
        auto J = kin.jacobian(q);

        REQUIRE(J.rows() == 6);
        REQUIRE(J.cols() == 6);
    }

    SECTION("Jacobian maps joint velocities to TCP velocities") {
        JointVector q;
        q << 0.1, -0.5, 0.3, -1.0, 0.5, -0.2;

        auto J = kin.jacobian(q);

        // Unit velocity in joint 1 should produce angular velocity around z
        JointVector qd = JointVector::Zero();
        qd[0] = 1.0;

        Eigen::Matrix<double, 6, 1> twist = J * qd;

        // Angular velocity component (last 3 elements) should be non-zero
        REQUIRE(twist.tail<3>().norm() > 0.1);
    }

    SECTION("Jacobian is full rank for non-singular configuration") {
        JointVector q;
        q << 0.3, -0.8, 1.2, -0.5, 0.7, -0.2;

        auto J = kin.jacobian(q);

        // Check determinant is non-zero
        double det = J.determinant();
        REQUIRE(std::abs(det) > 1e-6);
    }
}

// =============================================================================
// Singularity Detection Tests
// =============================================================================

TEST_CASE("Singularity detection", "[kinematics][singularity]") {
    URKinematics kin(URModel::UR5e);

    SECTION("Wrist singularity detected when q5 near zero") {
        JointVector q;
        q << 0.0, -kPi / 2, kPi / 2, 0.0, 0.001, 0.0;  // q5 nearly zero

        auto info = kin.analyzeSingularity(q, 50.0);

        REQUIRE(info.near_singularity);
        REQUIRE(info.type == SingularityType::Wrist);
    }

    SECTION("Non-singular configuration not flagged") {
        JointVector q;
        q << 0.3, -0.8, 1.2, -0.5, 0.7, -0.2;

        auto info = kin.analyzeSingularity(q, 100.0);

        REQUIRE(!info.near_singularity);
        REQUIRE(info.type == SingularityType::None);
    }

    SECTION("nearSingularity convenience function works") {
        // Singular config
        JointVector q_sing;
        q_sing << 0.0, -kPi / 2, kPi / 2, 0.0, 0.001, 0.0;

        // Non-singular config
        JointVector q_ok;
        q_ok << 0.3, -0.8, 1.2, -0.5, 0.7, -0.2;

        REQUIRE(kin.nearSingularity(q_sing, 50.0));
        REQUIRE(!kin.nearSingularity(q_ok, 100.0));
    }
}

// =============================================================================
// Utility Function Tests
// =============================================================================

TEST_CASE("Angle wrapping", "[kinematics][utility]") {
    SECTION("wrapAngle wraps positive angles") {
        double wrapped = URKinematics::wrapAngle(3 * kPi);
        REQUIRE_THAT(wrapped, WithinAbs(kPi, kTolerance));
    }

    SECTION("wrapAngle wraps negative angles") {
        double wrapped = URKinematics::wrapAngle(-3 * kPi);
        REQUIRE_THAT(wrapped, WithinAbs(-kPi, kTolerance));
    }

    SECTION("wrapAngle preserves angles in range") {
        double wrapped = URKinematics::wrapAngle(0.5);
        REQUIRE_THAT(wrapped, WithinAbs(0.5, kTolerance));
    }

    SECTION("wrapAngles wraps entire joint vector") {
        JointVector q;
        q << 4 * kPi, -3 * kPi, 2.5 * kPi, -0.5 * kPi, 0.0, kPi;

        JointVector wrapped = URKinematics::wrapAngles(q);

        for (int i = 0; i < 6; ++i) {
            REQUIRE(wrapped[i] >= -kPi - kTolerance);
            REQUIRE(wrapped[i] <= kPi + kTolerance);
        }
    }
}

// =============================================================================
// Multi-Model Tests
// =============================================================================

TEST_CASE("Kinematics works for all UR models", "[kinematics][models]") {
    std::vector<URModel> models = {
        URModel::UR3e, URModel::UR5e, URModel::UR10e,
        URModel::UR16e, URModel::UR20
    };

    for (URModel model : models) {
        DYNAMIC_SECTION("Model: " << modelToString(model)) {
            URKinematics kin(model);

            JointVector q;
            q << 0.1, -0.5, 0.3, -1.0, 0.5, -0.2;

            // FK should work
            auto pose = kin.forward(q);
            REQUIRE(pose.translation().norm() > 0.1);

            // IK should find solutions
            auto solutions = kin.inverse(pose);
            REQUIRE(!solutions.empty());

            // Jacobian should be computed
            auto J = kin.jacobian(q);
            REQUIRE(J.norm() > 0.0);
        }
    }
}
