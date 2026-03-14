/// @file test_trajectory.cpp
/// @brief Unit tests for the trajectory module

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ur_controller/trajectory/types.hpp"
#include "ur_controller/trajectory/validator.hpp"
#include "ur_controller/trajectory/planner.hpp"
#include "ur_controller/trajectory/executor.hpp"
#include "ur_controller/kinematics/ur_kinematics.hpp"

#include <cmath>
#include <thread>
#include <chrono>

using namespace ur_controller::trajectory;
using namespace ur_controller::kinematics;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-6;

// Create a reachable waypoint for UR5e
Waypoint createReachableWaypoint(double x, double y, double z) {
    Waypoint wp;
    wp.position = Eigen::Vector3d(x, y, z);
    wp.orientation = Eigen::Quaterniond::Identity();
    return wp;
}

}  // namespace

// =============================================================================
// Waypoint and Types Tests
// =============================================================================

TEST_CASE("Waypoint construction and conversion", "[trajectory][types]") {
    SECTION("Default waypoint has identity orientation") {
        Waypoint wp;
        REQUIRE(wp.orientation.w() == 1.0);
        REQUIRE(wp.orientation.x() == 0.0);
        REQUIRE(wp.orientation.y() == 0.0);
        REQUIRE(wp.orientation.z() == 0.0);
    }

    SECTION("Waypoint toPose creates valid Isometry3d") {
        Waypoint wp;
        wp.position = Eigen::Vector3d(0.5, 0.0, 0.4);
        wp.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(kPi/4, Eigen::Vector3d::UnitZ()));

        auto pose = wp.toPose();

        REQUIRE_THAT(pose.translation().x(), WithinAbs(0.5, kTolerance));
        REQUIRE_THAT(pose.translation().y(), WithinAbs(0.0, kTolerance));
        REQUIRE_THAT(pose.translation().z(), WithinAbs(0.4, kTolerance));
    }

    SECTION("Waypoint fromPose creates correct waypoint") {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(0.3, 0.2, 0.5);
        pose.linear() = Eigen::AngleAxisd(kPi/3, Eigen::Vector3d::UnitY()).toRotationMatrix();

        auto wp = Waypoint::fromPose(pose);

        REQUIRE_THAT(wp.position.x(), WithinAbs(0.3, kTolerance));
        REQUIRE_THAT(wp.position.y(), WithinAbs(0.2, kTolerance));
        REQUIRE_THAT(wp.position.z(), WithinAbs(0.5, kTolerance));
    }
}

TEST_CASE("PlannedTrajectory sampleAt interpolation", "[trajectory][types]") {
    PlannedTrajectory traj;
    traj.sample_period = 0.1;
    traj.total_duration = 1.0;

    // Create 11 samples from t=0 to t=1
    for (int i = 0; i <= 10; ++i) {
        TrajectorySample sample;
        sample.time = i * 0.1;
        sample.joints = JointVector::Constant(static_cast<double>(i) * 0.1);
        sample.pose = Eigen::Isometry3d::Identity();
        sample.pose.translation().x() = static_cast<double>(i) * 0.1;
        sample.speed = static_cast<double>(i) * 0.1;
        traj.samples.push_back(sample);
    }

    SECTION("sampleAt returns exact values at sample times") {
        auto sample = traj.sampleAt(0.5);
        REQUIRE_THAT(sample.time, WithinAbs(0.5, kTolerance));
        REQUIRE_THAT(sample.joints[0], WithinAbs(0.5, 0.01));
    }

    SECTION("sampleAt interpolates between samples") {
        auto sample = traj.sampleAt(0.55);
        REQUIRE_THAT(sample.time, WithinAbs(0.55, kTolerance));
        // Should be between 0.5 and 0.6
        REQUIRE(sample.joints[0] > 0.5);
        REQUIRE(sample.joints[0] < 0.6);
    }

    SECTION("sampleAt clamps to valid range") {
        auto sample_low = traj.sampleAt(-0.5);
        REQUIRE_THAT(sample_low.time, WithinAbs(0.0, kTolerance));

        auto sample_high = traj.sampleAt(1.5);
        REQUIRE_THAT(sample_high.time, WithinAbs(1.0, 0.01));
    }
}

// =============================================================================
// Validator Tests
// =============================================================================

TEST_CASE("TrajectoryValidator validates reachable waypoints", "[trajectory][validator]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryValidator validator(kin);

    SECTION("Reachable waypoint passes validation") {
        // Create waypoint at known reachable position
        Waypoint wp = createReachableWaypoint(-0.4, 0.2, 0.3);

        auto result = validator.validateWaypoint(wp);

        REQUIRE(result.reachable);
        REQUIRE(result.num_solutions > 0);
        REQUIRE(result.best_solution.has_value());
    }

    SECTION("Unreachable waypoint fails validation") {
        // Create waypoint far outside reach
        Waypoint wp = createReachableWaypoint(5.0, 5.0, 5.0);

        auto result = validator.validateWaypoint(wp);

        REQUIRE_FALSE(result.reachable);
        REQUIRE(result.num_solutions == 0);
        REQUIRE_FALSE(result.best_solution.has_value());
    }
}

TEST_CASE("TrajectoryValidator validates full trajectories", "[trajectory][validator]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryValidator validator(kin);
    TrajectoryConfig config;

    SECTION("Empty waypoints returns error") {
        std::vector<Waypoint> waypoints;
        auto result = validator.validate(waypoints, config);

        REQUIRE_FALSE(result.valid);
        REQUIRE(result.hasErrors());
    }

    SECTION("Single waypoint returns warning") {
        std::vector<Waypoint> waypoints = {
            createReachableWaypoint(-0.4, 0.2, 0.3)
        };
        auto result = validator.validate(waypoints, config);

        REQUIRE(result.hasWarnings());
    }

    SECTION("Two reachable waypoints validates successfully") {
        std::vector<Waypoint> waypoints = {
            createReachableWaypoint(-0.4, 0.2, 0.3),
            createReachableWaypoint(-0.4, 0.3, 0.3)
        };

        auto result = validator.validate(waypoints, config);

        REQUIRE(result.valid);
        REQUIRE(result.waypoints.size() == 2);
        REQUIRE(result.segments.size() == 1);
    }
}

// =============================================================================
// Planner Tests
// =============================================================================

TEST_CASE("TrajectoryPlanner generates valid trajectories", "[trajectory][planner]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);

    SECTION("Plan with two waypoints produces samples") {
        std::vector<Waypoint> waypoints = {
            createReachableWaypoint(-0.4, 0.2, 0.3),
            createReachableWaypoint(-0.4, 0.3, 0.35)
        };
        waypoints[1].segment_time = 1.0;  // 1 second motion

        auto trajectory = planner.plan(waypoints);

        REQUIRE(trajectory.valid);
        REQUIRE(trajectory.samples.size() > 0);
        REQUIRE(trajectory.total_duration > 0.0);
    }

    SECTION("Plan with unreachable waypoint fails") {
        std::vector<Waypoint> waypoints = {
            createReachableWaypoint(-0.4, 0.2, 0.3),
            createReachableWaypoint(5.0, 5.0, 5.0)
        };

        auto trajectory = planner.plan(waypoints);

        REQUIRE_FALSE(trajectory.valid);
        REQUIRE(trajectory.hasErrors());
    }

    SECTION("Trajectory samples are monotonically increasing in time") {
        std::vector<Waypoint> waypoints = {
            createReachableWaypoint(-0.4, 0.2, 0.3),
            createReachableWaypoint(-0.4, 0.3, 0.35),
            createReachableWaypoint(-0.3, 0.3, 0.4)
        };

        auto trajectory = planner.plan(waypoints);

        REQUIRE(trajectory.valid);
        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            REQUIRE(trajectory.samples[i].time >= trajectory.samples[i-1].time);
        }
    }
}

TEST_CASE("TrajectoryPlanner generates visualization data", "[trajectory][planner]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);

    std::vector<Waypoint> waypoints = {
        createReachableWaypoint(-0.4, 0.2, 0.3),
        createReachableWaypoint(-0.4, 0.3, 0.35)
    };
    waypoints[1].segment_time = 1.0;

    auto trajectory = planner.plan(waypoints);
    REQUIRE(trajectory.valid);

    auto viz = planner.generateVisualization(trajectory, waypoints);

    SECTION("Path points are generated") {
        REQUIRE(viz.path_points.size() > 0);
        REQUIRE(viz.path_times.size() == viz.path_points.size());
    }

    SECTION("Timing data is generated") {
        REQUIRE(viz.times.size() > 0);
        REQUIRE(viz.speeds.size() == viz.times.size());
    }

    SECTION("Waypoint markers are created") {
        REQUIRE(viz.waypoints.size() == 2);
    }

    SECTION("Summary statistics are computed") {
        REQUIRE(viz.total_duration > 0.0);
        REQUIRE(viz.total_distance >= 0.0);
    }
}

// =============================================================================
// Executor Tests
// =============================================================================

TEST_CASE("TrajectoryExecutor state management", "[trajectory][executor]") {
    TrajectoryExecutor executor;

    SECTION("Initial state is Idle") {
        REQUIRE(executor.state() == ExecutorState::Idle);
        REQUIRE_FALSE(executor.hasTrajectory());
        REQUIRE_FALSE(executor.isRunning());
    }

    SECTION("Cannot start without loaded trajectory") {
        REQUIRE_FALSE(executor.start());
        REQUIRE(executor.state() == ExecutorState::Idle);
    }
}

TEST_CASE("TrajectoryExecutor loads and executes trajectory", "[trajectory][executor]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);
    TrajectoryExecutor executor;

    ExecutorConfig config;
    config.dry_run = true;  // Don't actually send commands
    config.control_rate = 100.0;  // Lower rate for testing
    executor.setConfig(config);

    // Create a simple trajectory
    std::vector<Waypoint> waypoints = {
        createReachableWaypoint(-0.4, 0.2, 0.3),
        createReachableWaypoint(-0.4, 0.25, 0.32)
    };
    waypoints[1].segment_time = 0.2;  // Quick motion

    auto trajectory = planner.plan(waypoints);
    REQUIRE(trajectory.valid);

    SECTION("Load transitions to Ready state") {
        REQUIRE(executor.load(trajectory));
        REQUIRE(executor.state() == ExecutorState::Ready);
        REQUIRE(executor.hasTrajectory());
    }

    SECTION("Start transitions to Running state") {
        executor.load(trajectory);
        REQUIRE(executor.start());
        REQUIRE(executor.state() == ExecutorState::Running);
        REQUIRE(executor.isRunning());

        // Clean up
        executor.stop();
    }

    SECTION("Stop returns to Idle state") {
        executor.load(trajectory);
        executor.start();

        executor.stop();

        REQUIRE(executor.state() == ExecutorState::Idle);
        REQUIRE_FALSE(executor.hasTrajectory());
    }

    SECTION("Progress updates during execution") {
        executor.load(trajectory);

        auto progress = executor.progress();
        REQUIRE(progress.state == ExecutorState::Ready);
        REQUIRE(progress.total_duration > 0.0);
        REQUIRE(progress.total_samples > 0);

        executor.stop();
    }
}

TEST_CASE("TrajectoryExecutor pause and resume", "[trajectory][executor]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);
    TrajectoryExecutor executor;

    ExecutorConfig config;
    config.dry_run = true;
    config.control_rate = 100.0;
    executor.setConfig(config);

    std::vector<Waypoint> waypoints = {
        createReachableWaypoint(-0.4, 0.2, 0.3),
        createReachableWaypoint(-0.4, 0.25, 0.32)
    };
    waypoints[1].segment_time = 0.5;

    auto trajectory = planner.plan(waypoints);
    executor.load(trajectory);
    executor.start();

    SECTION("Pause transitions to Paused state") {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        executor.pause();
        REQUIRE(executor.state() == ExecutorState::Paused);
    }

    SECTION("Resume transitions back to Running state") {
        executor.pause();
        executor.resume();
        REQUIRE(executor.state() == ExecutorState::Running);
    }

    executor.stop();
}

// =============================================================================
// Integration Tests
// =============================================================================

TEST_CASE("Full trajectory workflow", "[trajectory][integration]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);
    TrajectoryExecutor executor;

    ExecutorConfig config;
    config.dry_run = false;  // Allow callback to be called for testing
    config.control_rate = 100.0;
    executor.setConfig(config);

    // Track commands
    std::vector<std::pair<JointVector, double>> commands;
    executor.setCommandCallback([&](const JointVector& joints, double time) {
        commands.push_back({joints, time});
    });

    // Create multi-waypoint trajectory
    std::vector<Waypoint> waypoints = {
        createReachableWaypoint(-0.4, 0.2, 0.3),
        createReachableWaypoint(-0.4, 0.25, 0.35),
        createReachableWaypoint(-0.35, 0.25, 0.35)
    };
    waypoints[1].segment_time = 0.1;
    waypoints[2].segment_time = 0.1;

    auto trajectory = planner.plan(waypoints);
    REQUIRE(trajectory.valid);
    REQUIRE(trajectory.samples.size() > 5);

    executor.load(trajectory);
    executor.start();

    // Wait for trajectory to complete (with timeout)
    auto start = std::chrono::steady_clock::now();
    while (executor.isRunning()) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > std::chrono::seconds(5)) {
            executor.stop();
            FAIL("Trajectory execution timed out");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    SECTION("Commands were sent during execution") {
        REQUIRE(commands.size() > 0);
    }

    SECTION("Commands have monotonically increasing time") {
        for (size_t i = 1; i < commands.size(); ++i) {
            REQUIRE(commands[i].second >= commands[i-1].second);
        }
    }
}
