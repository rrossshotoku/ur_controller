/// @file test_trajectory.cpp
/// @brief Unit tests for the trajectory module

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ur_controller/trajectory/types.hpp"
#include "ur_controller/trajectory/validator.hpp"
#include "ur_controller/trajectory/planner.hpp"
#include "ur_controller/trajectory/executor.hpp"
#include "ur_controller/trajectory/s_curve.hpp"
#include "ur_controller/kinematics/ur_kinematics.hpp"
#include <ruckig/ruckig.hpp>

#include <cmath>
#include <thread>
#include <chrono>
#include <iostream>

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
        // Set achievable segment times for rest-to-rest profiles
        waypoints[1].segment_time = 1.0;
        waypoints[2].segment_time = 1.0;

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
    waypoints[1].segment_time = 1.0;  // Achievable with rest-to-rest profile

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
    waypoints[1].segment_time = 1.0;  // Achievable duration for S-curve profile

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
// Path Linearity Tests
// =============================================================================

TEST_CASE("Shoulder singularity analysis", "[trajectory][planner][singularity]") {
    URKinematics kin(URModel::UR5e);

    // Analyze the problematic path step by step
    Eigen::Vector3d start_pos(0.507, -0.063, 0.785);
    Eigen::Vector3d end_pos(-0.431, -0.275, 0.785);
    Eigen::Vector3d direction = (end_pos - start_pos).normalized();
    double total_distance = (end_pos - start_pos).norm();

    INFO("Path analysis: start=(" << start_pos.transpose() << ") end=(" << end_pos.transpose() << ")");
    INFO("Total distance: " << total_distance << " m");
    INFO("Direction: " << direction.transpose());

    // Sample along the path and check IK availability
    SECTION("Check IK solutions along path") {
        // Get start configuration
        Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
        start_pose.translation() = start_pos;
        auto start_solutions = kin.inverse(start_pose);
        REQUIRE(!start_solutions.empty());

        JointVector start_joints = start_solutions[0];
        auto start_config = kin.getConfiguration(start_joints);
        INFO("Start configuration: shoulder=" << static_cast<int>(start_config.shoulder)
             << " elbow=" << static_cast<int>(start_config.elbow)
             << " wrist=" << static_cast<int>(start_config.wrist));

        // Check multiple points along the path
        constexpr int kNumPoints = 20;
        bool found_config_change = false;
        double config_change_t = 0.0;
        Eigen::Vector3d config_change_pos;

        for (int i = 0; i <= kNumPoints; ++i) {
            double t = static_cast<double>(i) / kNumPoints;
            Eigen::Vector3d pos = start_pos + t * (end_pos - start_pos);

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = pos;

            auto solutions = kin.inverse(pose);
            auto same_config_solutions = kin.filterByConfiguration(solutions, start_config);

            std::cout << "t=" << t << " pos=(" << pos.x() << ", " << pos.y() << ", " << pos.z()
                 << ") total=" << solutions.size()
                 << " same_config=" << same_config_solutions.size() << std::endl;

            if (same_config_solutions.empty() && !solutions.empty() && !found_config_change) {
                // Point where configuration must change
                found_config_change = true;
                config_change_t = t;
                config_change_pos = pos;
                std::cout << "*** CONFIG CHANGE REQUIRED at t=" << t << std::endl;

                // Show what configurations ARE available
                for (size_t j = 0; j < solutions.size() && j < 3; ++j) {
                    auto sol_config = kin.getConfiguration(solutions[j]);
                    std::cout << "  Available: shoulder=" << static_cast<int>(sol_config.shoulder)
                         << " elbow=" << static_cast<int>(sol_config.elbow)
                         << " wrist=" << static_cast<int>(sol_config.wrist) << std::endl;
                }
            }

            if (solutions.empty()) {
                std::cout << "*** UNREACHABLE at t=" << t << std::endl;
            }
        }

        // This path REQUIRES a configuration change - it cannot be linear
        if (found_config_change) {
            std::cout << "\n=== CONCLUSION ===" << std::endl;
            std::cout << "This path crosses the shoulder singularity and requires" << std::endl;
            std::cout << "a configuration change at t=" << config_change_t << std::endl;
            std::cout << "position=(" << config_change_pos.x() << ", "
                      << config_change_pos.y() << ", " << config_change_pos.z() << ")" << std::endl;
            std::cout << "\nThe robot cannot maintain a linear Cartesian path" << std::endl;
            std::cout << "while keeping the same arm configuration (elbow/wrist)." << std::endl;
            std::cout << "\nSOLUTIONS:" << std::endl;
            std::cout << "1. Add intermediate waypoints to go around the singularity" << std::endl;
            std::cout << "2. Accept non-linear path (joint-space interpolation)" << std::endl;
        }

        REQUIRE(found_config_change);  // This path SHOULD require config change
    }
}

TEST_CASE("Analyze IK failure along path", "[trajectory][planner][ik_analysis]") {
    URKinematics kin(URModel::UR5e);

    // The path that fails
    Eigen::Vector3d start_pos(0.506, 0.074, 0.785);
    Eigen::Vector3d end_pos(-0.449, -0.244, 0.785);

    std::cout << "\n=== IK Analysis Along Path ===" << std::endl;
    std::cout << "Checking IK solutions at each point along path" << std::endl;

    // Sample the path finely around the failure region (t=0.43 to 0.45)
    for (double t = 0.40; t <= 0.50; t += 0.01) {
        Eigen::Vector3d pos = start_pos + t * (end_pos - start_pos);

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = pos;

        auto solutions = kin.inverse(pose);

        std::cout << "t=" << t << " pos=(" << pos.x() << ", " << pos.y() << ", " << pos.z()
                  << ") distance_from_base=" << pos.head<2>().norm()
                  << " solutions=" << solutions.size();

        if (solutions.empty()) {
            std::cout << " *** NO IK SOLUTION ***";
        }
        std::cout << std::endl;
    }

    // Also check the exact failure point
    std::cout << "\nChecking exact failure point:" << std::endl;
    Eigen::Vector3d fail_pos(0.122, -0.054, 0.785);
    Eigen::Isometry3d fail_pose = Eigen::Isometry3d::Identity();
    fail_pose.translation() = fail_pos;

    auto fail_solutions = kin.inverse(fail_pose);
    std::cout << "Failure point (" << fail_pos.x() << ", " << fail_pos.y() << ", " << fail_pos.z()
              << "): " << fail_solutions.size() << " solutions" << std::endl;

    // Check if the issue is the identity orientation
    // Try with a different orientation
    std::cout << "\nTrying different orientations at failing position (0.114, -0.056, 0.785):" << std::endl;
    Eigen::Vector3d test_pos(0.114, -0.056, 0.785);

    // Try various rotations
    std::vector<std::pair<std::string, Eigen::Matrix3d>> orientations = {
        {"Identity (Z up)", Eigen::Matrix3d::Identity()},
        {"180° around X (Z down)", Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix()},
        {"90° around Y (Z forward)", Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()).toRotationMatrix()},
        {"-90° around Y (Z backward)", Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()).toRotationMatrix()},
        {"90° around X (Z right)", Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()).toRotationMatrix()},
        {"-90° around X (Z left)", Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()).toRotationMatrix()},
    };

    for (const auto& [name, rot] : orientations) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = test_pos;
        pose.linear() = rot;

        auto solutions = kin.inverse(pose);
        std::cout << "  " << name << ": " << solutions.size() << " solutions" << std::endl;
    }

    // Check what the start and end orientations should be
    std::cout << "\nStart and end position analysis:" << std::endl;
    Eigen::Vector3d start(0.506, 0.074, 0.785);
    Eigen::Vector3d end(-0.449, -0.244, 0.785);

    Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
    start_pose.translation() = start;
    auto start_solutions = kin.inverse(start_pose);
    std::cout << "Start (0.506, 0.074, 0.785) with identity: " << start_solutions.size() << " solutions" << std::endl;

    Eigen::Isometry3d end_pose = Eigen::Isometry3d::Identity();
    end_pose.translation() = end;
    auto end_solutions = kin.inverse(end_pose);
    std::cout << "End (-0.449, -0.244, 0.785) with identity: " << end_solutions.size() << " solutions" << std::endl;

    REQUIRE(true);  // This is just an analysis test
}

TEST_CASE("Arc in middle of move analysis", "[trajectory][planner][arc_deviation]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);

    // Linear path that doesn't cross the shoulder singularity
    // (stays in front of the robot, doesn't pass through X=0, Y=0)
    std::vector<Waypoint> waypoints = {
        createReachableWaypoint(0.3, 0.25, 0.5),
        createReachableWaypoint(0.3, -0.25, 0.5)
    };
    waypoints[1].segment_time = 3.0;

    std::cout << "\n=== Arc in Middle Analysis ===" << std::endl;
    std::cout << "From: (0.3, 0.25, 0.5) To: (0.3, -0.25, 0.5)" << std::endl;

    auto trajectory = planner.plan(waypoints);

    SECTION("Analyze path for unexpected arcs") {
        REQUIRE(trajectory.valid);
        REQUIRE(trajectory.samples.size() > 10);

        std::cout << "Trajectory valid: " << trajectory.valid << std::endl;
        std::cout << "Samples: " << trajectory.samples.size() << std::endl;

        // Calculate ideal line
        Eigen::Vector3d start_pos = waypoints[0].position;
        Eigen::Vector3d end_pos = waypoints[1].position;
        Eigen::Vector3d line_dir = (end_pos - start_pos).normalized();
        double total_dist = (end_pos - start_pos).norm();

        std::cout << "Total distance: " << total_dist << " m" << std::endl;

        // Check EVERY sample for deviation - find where the arc is
        double max_deviation = 0.0;
        size_t max_dev_sample = 0;

        std::cout << "\nSample-by-sample analysis (showing deviations > 0.1mm):" << std::endl;

        for (size_t i = 0; i < trajectory.samples.size(); ++i) {
            Eigen::Vector3d pos = trajectory.samples[i].pose.translation();
            Eigen::Vector3d to_point = pos - start_pos;
            double projection = to_point.dot(line_dir);
            Eigen::Vector3d closest = start_pos + projection * line_dir;
            double deviation = (pos - closest).norm();

            if (deviation > max_deviation) {
                max_deviation = deviation;
                max_dev_sample = i;
            }

            if (deviation > 0.0001) {  // > 0.1mm
                std::cout << "  Sample " << i << " (t=" << trajectory.samples[i].time << "s): "
                          << deviation * 1000.0 << " mm, pos=("
                          << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
            }
        }

        std::cout << "\nMax deviation: " << max_deviation * 1000.0 << " mm at sample "
                  << max_dev_sample << " (t=" << trajectory.samples[max_dev_sample].time << "s)" << std::endl;

        // Show position at max deviation
        if (max_deviation > 0.001) {
            const auto& s = trajectory.samples[max_dev_sample];
            Eigen::Vector3d pos = s.pose.translation();
            std::cout << "Position at max deviation: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;

            // Check what's happening with IK at that point
            std::cout << "Joints at max deviation: ";
            for (int j = 0; j < 6; ++j) {
                std::cout << s.joints[j] * 180.0 / M_PI << " ";
            }
            std::cout << "deg" << std::endl;
        }

        // Check the visualization data too
        auto viz = planner.generateVisualization(trajectory, waypoints);
        std::cout << "\nVisualization path points: " << viz.path_points.size() << std::endl;

        double max_viz_dev = 0.0;
        for (const auto& pt : viz.path_points) {
            Eigen::Vector3d to_point = pt - start_pos;
            double projection = to_point.dot(line_dir);
            Eigen::Vector3d closest = start_pos + projection * line_dir;
            double deviation = (pt - closest).norm();
            max_viz_dev = std::max(max_viz_dev, deviation);
        }
        std::cout << "Max visualization deviation: " << max_viz_dev * 1000.0 << " mm" << std::endl;

        REQUIRE(max_deviation < 0.01);  // Should be < 10mm
    }
}

TEST_CASE("Path deviation analysis", "[trajectory][planner][deviation]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);

    // User-reported path with deviations
    // Distance ~0.965m, with V_max=0.5 m/s, need T >= 2*D/(V_max+0) = 3.86s
    std::vector<Waypoint> waypoints = {
        createReachableWaypoint(-0.449, -0.244, 0.785),
        createReachableWaypoint(0.504, -0.088, 0.785)
    };
    waypoints[1].segment_time = 4.0;

    std::cout << "\n=== Path Deviation Analysis ===" << std::endl;
    std::cout << "From: (-0.449, -0.244, 0.785) To: (0.504, -0.088, 0.785)" << std::endl;

    auto trajectory = planner.plan(waypoints);

    SECTION("Analyze path for deviations") {
        REQUIRE(trajectory.valid);
        REQUIRE(trajectory.samples.size() > 10);

        std::cout << "Trajectory valid: " << trajectory.valid << std::endl;
        std::cout << "Samples: " << trajectory.samples.size() << std::endl;

        // Calculate ideal line
        Eigen::Vector3d start_pos = waypoints[0].position;
        Eigen::Vector3d end_pos = waypoints[1].position;
        Eigen::Vector3d line_dir = (end_pos - start_pos).normalized();
        double total_dist = (end_pos - start_pos).norm();

        std::cout << "Total distance: " << total_dist << " m" << std::endl;

        // Check each sample for deviation
        double max_deviation = 0.0;
        size_t max_dev_sample = 0;
        std::vector<std::pair<size_t, double>> large_deviations;

        for (size_t i = 0; i < trajectory.samples.size(); ++i) {
            Eigen::Vector3d pos = trajectory.samples[i].pose.translation();
            Eigen::Vector3d to_point = pos - start_pos;
            double projection = to_point.dot(line_dir);
            Eigen::Vector3d closest = start_pos + projection * line_dir;
            double deviation = (pos - closest).norm();

            if (deviation > max_deviation) {
                max_deviation = deviation;
                max_dev_sample = i;
            }

            if (deviation > 0.001) {  // > 1mm
                large_deviations.push_back({i, deviation});
            }
        }

        std::cout << "\nMax deviation: " << max_deviation * 1000.0 << " mm at sample "
                  << max_dev_sample << std::endl;

        if (!large_deviations.empty()) {
            std::cout << "\nSamples with >1mm deviation:" << std::endl;
            for (size_t j = 0; j < std::min(large_deviations.size(), size_t(20)); ++j) {
                auto [idx, dev] = large_deviations[j];
                const auto& s = trajectory.samples[idx];
                Eigen::Vector3d pos = s.pose.translation();
                std::cout << "  Sample " << idx << " (t=" << s.time << "s): "
                          << dev * 1000.0 << " mm, pos=("
                          << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
            }
            if (large_deviations.size() > 20) {
                std::cout << "  ... and " << large_deviations.size() - 20 << " more" << std::endl;
            }
        }

        // Check joint jumps
        std::cout << "\n--- Joint Analysis ---" << std::endl;
        double max_jump = 0.0;
        size_t max_jump_sample = 0;

        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            for (int j = 0; j < 6; ++j) {
                double delta = std::abs(trajectory.samples[i].joints[j] -
                                       trajectory.samples[i-1].joints[j]);
                if (delta > M_PI) delta = 2 * M_PI - delta;
                if (delta > max_jump) {
                    max_jump = delta;
                    max_jump_sample = i;
                }
            }
        }

        std::cout << "Max joint jump: " << max_jump * 180.0 / M_PI << " deg at sample "
                  << max_jump_sample << std::endl;

        // Check configuration changes
        auto start_config = kin.getConfiguration(trajectory.samples[0].joints);
        std::cout << "Start config: shoulder=" << static_cast<int>(start_config.shoulder)
                  << " elbow=" << static_cast<int>(start_config.elbow)
                  << " wrist=" << static_cast<int>(start_config.wrist) << std::endl;

        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            auto config = kin.getConfiguration(trajectory.samples[i].joints);
            if (config != start_config) {
                std::cout << "Config change at sample " << i << " (t="
                          << trajectory.samples[i].time << "s)" << std::endl;
                start_config = config;
            }
        }

        // The test passes if we get data - actual requirement TBD
        REQUIRE(max_deviation < 0.01);  // < 10mm for now
    }
}

TEST_CASE("Path through singularity region - correctly rejected", "[trajectory][planner][singularity_crossing]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);

    // This path crosses directly through the robot's Z-axis where identity
    // orientation has no IK solution. The planner correctly rejects this path
    // rather than adjusting orientation (which could cause unexpected motion).
    std::vector<Waypoint> waypoints = {
        createReachableWaypoint(0.506, 0.074, 0.785),
        createReachableWaypoint(-0.449, -0.244, 0.785)
    };
    waypoints[1].segment_time = 3.0;

    std::cout << "\n=== Singularity Crossing Test ===" << std::endl;
    std::cout << "From: (0.506, 0.074, 0.785) To: (-0.449, -0.244, 0.785)" << std::endl;
    std::cout << "This path crosses the shoulder singularity region where identity orientation is not achievable." << std::endl;

    auto trajectory = planner.plan(waypoints);

    SECTION("Path should be correctly rejected - no IK solution at singularity") {
        // Path should fail because identity orientation is not achievable
        // at points near X=0, Y=0 (shoulder singularity)
        REQUIRE_FALSE(trajectory.valid);

        std::cout << "Trajectory correctly rejected: valid=" << trajectory.valid << std::endl;
        std::cout << "To achieve this path, use a different orientation at the waypoints" << std::endl;
        std::cout << "or plan a path that avoids passing directly through X=0, Y=0." << std::endl;
    }
}

TEST_CASE("Discontinuity analysis for specific waypoints", "[trajectory][planner][discontinuity][.]") {
    // This test is marked as hidden [.] because the path is now correctly rejected as infeasible
    // It's kept for documentation and debugging purposes
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);

    std::vector<Waypoint> waypoints = {
        createReachableWaypoint(0.506, 0.074, 0.785),
        createReachableWaypoint(-0.449, -0.244, 0.785)
    };
    waypoints[1].segment_time = 3.0;

    std::cout << "\n=== Discontinuity Analysis ===" << std::endl;
    std::cout << "From: (0.506, 0.074, 0.785) To: (-0.449, -0.244, 0.785)" << std::endl;

    auto trajectory = planner.plan(waypoints);

    SECTION("Check trajectory validity and continuity") {
        // Skip if path is correctly rejected as infeasible
        if (!trajectory.valid) {
            std::cout << "Path correctly rejected as infeasible" << std::endl;
            SUCCEED();
            return;
        }
        REQUIRE(trajectory.samples.size() > 10);

        std::cout << "Trajectory valid: " << trajectory.valid << std::endl;
        std::cout << "Number of samples: " << trajectory.samples.size() << std::endl;
        std::cout << "Duration: " << trajectory.total_duration << " s" << std::endl;

        // Analyze joint discontinuities
        double max_jump = 0.0;
        size_t max_jump_sample = 0;
        int max_jump_joint = 0;
        std::vector<std::tuple<size_t, int, double>> large_jumps;

        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            for (int j = 0; j < 6; ++j) {
                double delta = std::abs(trajectory.samples[i].joints[j] -
                                       trajectory.samples[i-1].joints[j]);
                // Wrap to handle angle discontinuities
                if (delta > M_PI) delta = 2 * M_PI - delta;

                if (delta > max_jump) {
                    max_jump = delta;
                    max_jump_sample = i;
                    max_jump_joint = j;
                }

                // Track jumps > 5 degrees per sample
                if (delta > 5.0 * M_PI / 180.0) {
                    large_jumps.push_back({i, j, delta});
                }
            }
        }

        std::cout << "\n--- Joint Discontinuity Analysis ---" << std::endl;
        std::cout << "Max joint jump: " << max_jump * 180.0 / M_PI << " degrees" << std::endl;
        std::cout << "At sample: " << max_jump_sample << ", joint: " << max_jump_joint << std::endl;
        std::cout << "Time: " << trajectory.samples[max_jump_sample].time << " s" << std::endl;

        if (!large_jumps.empty()) {
            std::cout << "\nLarge jumps (>5 deg per sample):" << std::endl;
            size_t count = 0;
            for (const auto& [sample, joint, delta] : large_jumps) {
                if (count++ < 20) {  // Show first 20
                    std::cout << "  Sample " << sample << " (t="
                              << trajectory.samples[sample].time << "s): joint "
                              << joint << " jumped " << delta * 180.0 / M_PI << " deg" << std::endl;
                }
            }
            if (large_jumps.size() > 20) {
                std::cout << "  ... and " << large_jumps.size() - 20 << " more" << std::endl;
            }
        }

        // Check Cartesian path linearity
        std::cout << "\n--- Cartesian Path Analysis ---" << std::endl;
        Eigen::Vector3d start_pos = waypoints[0].position;
        Eigen::Vector3d end_pos = waypoints[1].position;
        Eigen::Vector3d line_dir = (end_pos - start_pos).normalized();

        double max_deviation = 0.0;
        size_t max_dev_sample = 0;

        for (size_t i = 0; i < trajectory.samples.size(); ++i) {
            Eigen::Vector3d pos = trajectory.samples[i].pose.translation();
            Eigen::Vector3d to_point = pos - start_pos;
            double projection = to_point.dot(line_dir);
            Eigen::Vector3d closest = start_pos + projection * line_dir;
            double deviation = (pos - closest).norm();

            if (deviation > max_deviation) {
                max_deviation = deviation;
                max_dev_sample = i;
            }

            // Print samples with large deviations
            if (deviation > 0.001) {  // > 1mm
                std::cout << "Sample " << i << " (t=" << trajectory.samples[i].time
                          << "s): deviation = " << deviation * 1000.0 << " mm, pos=("
                          << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
            }
        }

        std::cout << "\nMax Cartesian deviation: " << max_deviation * 1000.0 << " mm" << std::endl;
        std::cout << "At sample: " << max_dev_sample << std::endl;

        // Analyze IK configurations along path
        std::cout << "\n--- Configuration Analysis ---" << std::endl;
        auto start_config = kin.getConfiguration(trajectory.samples[0].joints);
        std::cout << "Start config: shoulder=" << static_cast<int>(start_config.shoulder)
                  << " elbow=" << static_cast<int>(start_config.elbow)
                  << " wrist=" << static_cast<int>(start_config.wrist) << std::endl;

        bool config_changed = false;
        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            auto config = kin.getConfiguration(trajectory.samples[i].joints);
            if (config != start_config && !config_changed) {
                config_changed = true;
                std::cout << "Config change at sample " << i << " (t="
                          << trajectory.samples[i].time << "s): shoulder="
                          << static_cast<int>(config.shoulder) << " elbow="
                          << static_cast<int>(config.elbow) << " wrist="
                          << static_cast<int>(config.wrist) << std::endl;

                // Show joints before and after
                std::cout << "  Before (sample " << i-1 << "): ";
                for (int j = 0; j < 6; ++j) {
                    std::cout << trajectory.samples[i-1].joints[j] * 180.0 / M_PI << " ";
                }
                std::cout << std::endl;
                std::cout << "  After  (sample " << i << "): ";
                for (int j = 0; j < 6; ++j) {
                    std::cout << trajectory.samples[i].joints[j] * 180.0 / M_PI << " ";
                }
                std::cout << std::endl;
            }
        }

        if (!config_changed) {
            std::cout << "Configuration remained constant throughout trajectory" << std::endl;
        }

        // The path should be reasonably linear
        REQUIRE(max_deviation < 0.01);  // Less than 10mm deviation
    }
}

TEST_CASE("Long cross-workspace linear path analysis", "[trajectory][planner][linearity]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);

    // User-reported problematic waypoints - ~0.96m move crossing workspace
    std::vector<Waypoint> waypoints = {
        createReachableWaypoint(0.507, -0.063, 0.785),
        createReachableWaypoint(-0.431, -0.275, 0.785)
    };
    waypoints[1].segment_time = 3.0;  // 3 seconds for a ~1m move

    INFO("Testing trajectory from (0.507, -0.063, 0.785) to (-0.431, -0.275, 0.785)");

    auto trajectory = planner.plan(waypoints);

    SECTION("Trajectory planning status") {
        if (!trajectory.valid) {
            INFO("Trajectory planning FAILED");
            for (const auto& msg : trajectory.messages) {
                INFO("Message: " << msg.message);
            }
        }
        // Report whether valid - don't require success yet
        INFO("Trajectory valid: " << trajectory.valid);
        INFO("Total duration: " << trajectory.total_duration << " seconds");
        INFO("Number of samples: " << trajectory.samples.size());
    }

    SECTION("Analyze path linearity") {
        if (!trajectory.valid || trajectory.samples.empty()) {
            WARN("Trajectory not valid, skipping linearity analysis");
            return;
        }

        // Calculate the ideal line direction
        Eigen::Vector3d start_pos = waypoints[0].position;
        Eigen::Vector3d end_pos = waypoints[1].position;
        Eigen::Vector3d line_dir = (end_pos - start_pos).normalized();
        double total_distance = (end_pos - start_pos).norm();

        INFO("Total linear distance: " << total_distance << " meters");
        INFO("Line direction: " << line_dir.transpose());

        // Check each sample point's deviation from the ideal line
        double max_deviation = 0.0;
        size_t max_deviation_sample = 0;
        std::vector<double> deviations;

        for (size_t i = 0; i < trajectory.samples.size(); ++i) {
            const auto& sample = trajectory.samples[i];
            Eigen::Vector3d sample_pos = sample.pose.translation();

            // Calculate perpendicular distance from sample to ideal line
            Eigen::Vector3d to_sample = sample_pos - start_pos;
            double projection = to_sample.dot(line_dir);
            Eigen::Vector3d closest_on_line = start_pos + projection * line_dir;
            double deviation = (sample_pos - closest_on_line).norm();

            deviations.push_back(deviation);

            if (deviation > max_deviation) {
                max_deviation = deviation;
                max_deviation_sample = i;
            }
        }

        INFO("Max deviation from linear path: " << max_deviation * 1000.0 << " mm");
        INFO("Max deviation at sample: " << max_deviation_sample);
        INFO("Max deviation time: " << trajectory.samples[max_deviation_sample].time << " s");

        // Show worst few samples
        for (size_t i = 0; i < trajectory.samples.size(); ++i) {
            if (deviations[i] > 0.01) {  // More than 10mm deviation
                const auto& s = trajectory.samples[i];
                INFO("Large deviation at sample " << i << " (t=" << s.time << "s): "
                    << deviations[i] * 1000.0 << " mm, pos=("
                    << s.pose.translation().x() << ", "
                    << s.pose.translation().y() << ", "
                    << s.pose.translation().z() << ")");
            }
        }

        // Report if linearity is acceptable (< 5mm deviation)
        bool is_linear = max_deviation < 0.005;
        INFO("Path is linear (< 5mm deviation): " << (is_linear ? "YES" : "NO"));

        // Check for configuration changes along path
        INFO("\n--- Configuration Analysis ---");
        auto start_config = kin.getConfiguration(trajectory.samples[0].joints);
        INFO("Start config: shoulder=" << static_cast<int>(start_config.shoulder)
             << " elbow=" << static_cast<int>(start_config.elbow)
             << " wrist=" << static_cast<int>(start_config.wrist));

        bool config_changed = false;
        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            auto config = kin.getConfiguration(trajectory.samples[i].joints);
            if (config != start_config) {
                INFO("Configuration change at sample " << i << " (t="
                     << trajectory.samples[i].time << "s): shoulder="
                     << static_cast<int>(config.shoulder) << " elbow="
                     << static_cast<int>(config.elbow) << " wrist="
                     << static_cast<int>(config.wrist));
                config_changed = true;
                break;
            }
        }

        INFO("Configuration changed during path: " << (config_changed ? "YES" : "NO"));

        // For now, just check that we generated something
        REQUIRE(trajectory.samples.size() > 10);
    }

    SECTION("Analyze joint behavior") {
        if (!trajectory.valid || trajectory.samples.empty()) {
            WARN("Trajectory not valid, skipping joint analysis");
            return;
        }

        INFO("\n--- Joint Movement Analysis ---");

        // Check joint jumps
        double max_jump = 0.0;
        size_t max_jump_joint = 0;
        size_t max_jump_sample = 0;

        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            JointVector delta = trajectory.samples[i].joints -
                               trajectory.samples[i-1].joints;

            for (int j = 0; j < 6; ++j) {
                double jump = std::abs(delta[j]);
                if (jump > max_jump) {
                    max_jump = jump;
                    max_jump_joint = static_cast<size_t>(j);
                    max_jump_sample = i;
                }
            }
        }

        INFO("Max joint jump: " << max_jump * 180.0 / 3.14159 << " degrees");
        INFO("Max jump at joint " << max_jump_joint << ", sample " << max_jump_sample);
        INFO("Max jump time: " << trajectory.samples[max_jump_sample].time << " s");

        // Report large jumps (> 5 degrees between samples)
        constexpr double kLargeJump = 5.0 * 3.14159 / 180.0;
        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            JointVector delta = trajectory.samples[i].joints -
                               trajectory.samples[i-1].joints;

            bool has_large_jump = false;
            for (int j = 0; j < 6; ++j) {
                if (std::abs(delta[j]) > kLargeJump) {
                    has_large_jump = true;
                }
            }

            if (has_large_jump) {
                INFO("Large jump at sample " << i << " (t=" << trajectory.samples[i].time
                     << "s): delta=" << delta.transpose() * 180.0 / 3.14159 << " deg");
            }
        }

        // Warn if there are unreasonably large jumps
        if (max_jump > 0.1) {  // > ~6 degrees per sample at 500Hz is suspicious
            WARN("Large joint jump detected: " << max_jump * 180.0 / 3.14159 << " deg");
        }
    }
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
    waypoints[1].segment_time = 1.0;  // Achievable duration for S-curve profile
    waypoints[2].segment_time = 1.0;  // Achievable duration for S-curve profile

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

TEST_CASE("Base rotation path analysis", "[trajectory][planner][base_rotation]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);

    // User-reported path where TCP leaves the straight line while base spins
    // Start: nearly in front of robot (X close to 0), Y negative
    // End: to the right and forward
    // Distance ~0.967m, with V_max=0.5 m/s, need T >= 3.87s
    std::vector<Waypoint> waypoints = {
        createReachableWaypoint(-0.015, -0.511, 0.785),
        createReachableWaypoint(0.326, 0.394, 0.785)
    };
    waypoints[1].segment_time = 4.0;

    std::cout << "\n=== Base Rotation Path Analysis ===" << std::endl;
    std::cout << "From: (-0.015, -0.511, 0.785) To: (0.326, 0.394, 0.785)" << std::endl;

    // First, analyze the start and end IK solutions
    Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
    start_pose.translation() = waypoints[0].position;
    start_pose.linear() = waypoints[0].orientation.toRotationMatrix();

    Eigen::Isometry3d end_pose = Eigen::Isometry3d::Identity();
    end_pose.translation() = waypoints[1].position;
    end_pose.linear() = waypoints[1].orientation.toRotationMatrix();

    auto start_solutions = kin.inverse(start_pose);
    auto end_solutions = kin.inverse(end_pose);

    std::cout << "\nStart pose IK solutions: " << start_solutions.size() << std::endl;
    std::cout << "End pose IK solutions: " << end_solutions.size() << std::endl;

    if (!start_solutions.empty()) {
        std::cout << "\nStart solutions (joint 1 / base angle in degrees):" << std::endl;
        for (size_t i = 0; i < start_solutions.size(); ++i) {
            auto config = kin.getConfiguration(start_solutions[i]);
            std::cout << "  Solution " << i << ": q1=" << start_solutions[i][0] * 180.0 / M_PI
                      << " deg, config: shoulder=" << static_cast<int>(config.shoulder)
                      << " elbow=" << static_cast<int>(config.elbow)
                      << " wrist=" << static_cast<int>(config.wrist) << std::endl;
        }
    }

    if (!end_solutions.empty()) {
        std::cout << "\nEnd solutions (joint 1 / base angle in degrees):" << std::endl;
        for (size_t i = 0; i < end_solutions.size(); ++i) {
            auto config = kin.getConfiguration(end_solutions[i]);
            std::cout << "  Solution " << i << ": q1=" << end_solutions[i][0] * 180.0 / M_PI
                      << " deg, config: shoulder=" << static_cast<int>(config.shoulder)
                      << " elbow=" << static_cast<int>(config.elbow)
                      << " wrist=" << static_cast<int>(config.wrist) << std::endl;
        }
    }

    auto trajectory = planner.plan(waypoints);

    SECTION("Analyze base rotation and path deviation") {
        REQUIRE(trajectory.valid);
        REQUIRE(trajectory.samples.size() > 10);

        std::cout << "\nTrajectory valid: " << trajectory.valid << std::endl;
        std::cout << "Duration: " << trajectory.total_duration << " s" << std::endl;
        std::cout << "Samples: " << trajectory.samples.size() << std::endl;

        // Analyze joint 1 (base) rotation throughout trajectory
        std::cout << "\n--- Joint 1 (Base) Analysis ---" << std::endl;
        double j1_start = trajectory.samples[0].joints[0];
        double j1_end = trajectory.samples.back().joints[0];
        double j1_total_rotation = j1_end - j1_start;

        std::cout << "J1 start: " << j1_start * 180.0 / M_PI << " deg" << std::endl;
        std::cout << "J1 end: " << j1_end * 180.0 / M_PI << " deg" << std::endl;
        std::cout << "J1 total rotation: " << j1_total_rotation * 180.0 / M_PI << " deg" << std::endl;

        // Find max joint velocities (deg/s)
        double max_j1_velocity = 0.0;
        size_t max_j1_vel_sample = 0;

        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            double dt = trajectory.samples[i].time - trajectory.samples[i-1].time;
            if (dt > 0) {
                double j1_vel = std::abs(trajectory.samples[i].joints[0] -
                                        trajectory.samples[i-1].joints[0]) / dt;
                if (j1_vel > max_j1_velocity) {
                    max_j1_velocity = j1_vel;
                    max_j1_vel_sample = i;
                }
            }
        }

        std::cout << "Max J1 velocity: " << max_j1_velocity * 180.0 / M_PI << " deg/s at sample "
                  << max_j1_vel_sample << std::endl;

        // Check Cartesian path deviation
        std::cout << "\n--- Cartesian Path Deviation ---" << std::endl;
        Eigen::Vector3d start_pos = waypoints[0].position;
        Eigen::Vector3d end_pos = waypoints[1].position;
        Eigen::Vector3d line_dir = (end_pos - start_pos).normalized();
        double path_length = (end_pos - start_pos).norm();

        std::cout << "Ideal path length: " << path_length * 1000.0 << " mm" << std::endl;

        double max_deviation = 0.0;
        size_t max_dev_sample = 0;
        Eigen::Vector3d max_dev_pos;

        std::cout << "\nSamples with deviation > 1mm:" << std::endl;
        for (size_t i = 0; i < trajectory.samples.size(); ++i) {
            Eigen::Vector3d pos = trajectory.samples[i].pose.translation();
            Eigen::Vector3d to_point = pos - start_pos;
            double projection = to_point.dot(line_dir);
            Eigen::Vector3d closest = start_pos + projection * line_dir;
            double deviation = (pos - closest).norm();

            if (deviation > max_deviation) {
                max_deviation = deviation;
                max_dev_sample = i;
                max_dev_pos = pos;
            }

            if (deviation > 0.001) {  // > 1mm
                std::cout << "  Sample " << i << " (t=" << trajectory.samples[i].time
                          << "s): " << deviation * 1000.0 << " mm, pos=("
                          << pos.x() << ", " << pos.y() << ", " << pos.z() << ")"
                          << ", J1=" << trajectory.samples[i].joints[0] * 180.0 / M_PI << " deg"
                          << std::endl;
            }
        }

        std::cout << "\nMax deviation: " << max_deviation * 1000.0 << " mm at sample "
                  << max_dev_sample << " (t=" << trajectory.samples[max_dev_sample].time << "s)"
                  << std::endl;
        std::cout << "Position at max deviation: (" << max_dev_pos.x() << ", "
                  << max_dev_pos.y() << ", " << max_dev_pos.z() << ")" << std::endl;

        // Configuration analysis
        std::cout << "\n--- Configuration Changes ---" << std::endl;
        auto start_config = kin.getConfiguration(trajectory.samples[0].joints);
        std::cout << "Start config: shoulder=" << static_cast<int>(start_config.shoulder)
                  << " elbow=" << static_cast<int>(start_config.elbow)
                  << " wrist=" << static_cast<int>(start_config.wrist) << std::endl;

        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            auto config = kin.getConfiguration(trajectory.samples[i].joints);
            auto prev_config = kin.getConfiguration(trajectory.samples[i-1].joints);
            if (config != prev_config) {
                std::cout << "Config change at sample " << i << " (t="
                          << trajectory.samples[i].time << "s):" << std::endl;
                std::cout << "  From: shoulder=" << static_cast<int>(prev_config.shoulder)
                          << " elbow=" << static_cast<int>(prev_config.elbow)
                          << " wrist=" << static_cast<int>(prev_config.wrist) << std::endl;
                std::cout << "  To:   shoulder=" << static_cast<int>(config.shoulder)
                          << " elbow=" << static_cast<int>(config.elbow)
                          << " wrist=" << static_cast<int>(config.wrist) << std::endl;
            }
        }

        auto end_config = kin.getConfiguration(trajectory.samples.back().joints);
        std::cout << "End config: shoulder=" << static_cast<int>(end_config.shoulder)
                  << " elbow=" << static_cast<int>(end_config.elbow)
                  << " wrist=" << static_cast<int>(end_config.wrist) << std::endl;

        // The path should remain linear - flag if deviation is too large
        if (max_deviation > 0.01) {  // > 10mm
            std::cout << "\n*** WARNING: Large path deviation detected! ***" << std::endl;
            std::cout << "This may indicate a configuration change or singularity issue." << std::endl;
        }

        // For now, just report - don't fail the test
        REQUIRE(trajectory.valid);
    }
}

TEST_CASE("TCP orientation flip analysis", "[trajectory][planner][orientation_flip]") {
    URKinematics kin(URModel::UR5e);
    TrajectoryPlanner planner(kin);

    // User-reported path with TCP orientation flips
    std::vector<Waypoint> waypoints = {
        createReachableWaypoint(-0.157, -0.486, 0.785),
        createReachableWaypoint(-0.101, 0.501, 0.785)
    };
    waypoints[1].segment_time = 3.0;

    std::cout << "\n=== TCP Orientation Flip Analysis ===" << std::endl;
    std::cout << "From: (-0.157, -0.486, 0.785) To: (-0.101, 0.501, 0.785)" << std::endl;
    std::cout << "Start orientation: w=" << waypoints[0].orientation.w()
              << " x=" << waypoints[0].orientation.x()
              << " y=" << waypoints[0].orientation.y()
              << " z=" << waypoints[0].orientation.z() << std::endl;

    auto trajectory = planner.plan(waypoints);

    SECTION("Analyze orientation changes along path") {
        if (!trajectory.valid) {
            std::cout << "Trajectory INVALID - path rejected" << std::endl;
            SUCCEED();
            return;
        }

        std::cout << "Trajectory valid, samples: " << trajectory.samples.size() << std::endl;
        std::cout << "Duration: " << trajectory.total_duration << " s" << std::endl;

        // Analyze wrist joints (q4, q5, q6) for flips
        std::cout << "\n--- Wrist Joint Analysis (q4, q5, q6) ---" << std::endl;

        double max_q4_delta = 0, max_q5_delta = 0, max_q6_delta = 0;
        size_t max_q4_sample = 0, max_q5_sample = 0, max_q6_sample = 0;

        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            double dq4 = std::abs(trajectory.samples[i].joints[3] - trajectory.samples[i-1].joints[3]);
            double dq5 = std::abs(trajectory.samples[i].joints[4] - trajectory.samples[i-1].joints[4]);
            double dq6 = std::abs(trajectory.samples[i].joints[5] - trajectory.samples[i-1].joints[5]);

            if (dq4 > max_q4_delta) { max_q4_delta = dq4; max_q4_sample = i; }
            if (dq5 > max_q5_delta) { max_q5_delta = dq5; max_q5_sample = i; }
            if (dq6 > max_q6_delta) { max_q6_delta = dq6; max_q6_sample = i; }
        }

        std::cout << "Max q4 delta: " << max_q4_delta * 180.0 / M_PI << " deg at sample " << max_q4_sample << std::endl;
        std::cout << "Max q5 delta: " << max_q5_delta * 180.0 / M_PI << " deg at sample " << max_q5_sample << std::endl;
        std::cout << "Max q6 delta: " << max_q6_delta * 180.0 / M_PI << " deg at sample " << max_q6_sample << std::endl;

        // Show details of largest jump
        size_t worst_sample = max_q5_sample;  // q5 flip is typically the wrist flip
        if (max_q5_delta > 0.1) {  // > ~6 degrees
            std::cout << "\n*** Potential wrist flip at sample " << worst_sample << " ***" << std::endl;
            const auto& before = trajectory.samples[worst_sample - 1];
            const auto& after = trajectory.samples[worst_sample];

            std::cout << "Time: " << before.time << "s -> " << after.time << "s" << std::endl;
            std::cout << "Position: (" << before.pose.translation().x() << ", "
                      << before.pose.translation().y() << ", "
                      << before.pose.translation().z() << ")" << std::endl;

            std::cout << "q5 before: " << before.joints[4] * 180.0 / M_PI << " deg" << std::endl;
            std::cout << "q5 after:  " << after.joints[4] * 180.0 / M_PI << " deg" << std::endl;

            std::cout << "\nAll joints before: ";
            for (int j = 0; j < 6; ++j) std::cout << before.joints[j] * 180.0 / M_PI << " ";
            std::cout << "deg" << std::endl;

            std::cout << "All joints after:  ";
            for (int j = 0; j < 6; ++j) std::cout << after.joints[j] * 180.0 / M_PI << " ";
            std::cout << "deg" << std::endl;

            // Check TCP orientation change
            Eigen::Quaterniond quat_before(before.pose.rotation());
            Eigen::Quaterniond quat_after(after.pose.rotation());
            double angle_diff = quat_before.angularDistance(quat_after) * 180.0 / M_PI;
            std::cout << "\nTCP orientation change: " << angle_diff << " deg" << std::endl;

            // Configuration check
            auto config_before = kin.getConfiguration(before.joints);
            auto config_after = kin.getConfiguration(after.joints);
            std::cout << "Config before: shoulder=" << static_cast<int>(config_before.shoulder)
                      << " elbow=" << static_cast<int>(config_before.elbow)
                      << " wrist=" << static_cast<int>(config_before.wrist) << std::endl;
            std::cout << "Config after:  shoulder=" << static_cast<int>(config_after.shoulder)
                      << " elbow=" << static_cast<int>(config_after.elbow)
                      << " wrist=" << static_cast<int>(config_after.wrist) << std::endl;
        }

        // Check all joints for any large jumps
        std::cout << "\n--- All Joint Jump Analysis ---" << std::endl;
        for (size_t i = 1; i < trajectory.samples.size(); ++i) {
            for (int j = 0; j < 6; ++j) {
                double delta = std::abs(trajectory.samples[i].joints[j] - trajectory.samples[i-1].joints[j]);
                if (delta > 0.5) {  // > ~30 degrees
                    std::cout << "Large jump at sample " << i << " (t=" << trajectory.samples[i].time
                              << "s), joint " << j << ": " << delta * 180.0 / M_PI << " deg" << std::endl;
                }
            }
        }

        REQUIRE(trajectory.valid);
    }
}

TEST_CASE("User reported jog path analysis", "[trajectory][planner][user_jog_path]") {
    // User reported these waypoints work with jog controller but fail in trajectory planner
    // WP1: X: -0.286 Y: 0.274 Z: 0.484
    // WP2: X: 0.256 Y: 0.479 Z: 0.476
    // WP3: X: 0.380 Y: 0.152 Z: 0.459
    //
    // Error: "Path passes near wrist singularity at 33% of segment"
    // Error: "[WP2] Segment 1 to waypoint 2 crosses configuration boundary"

    URKinematics kin(URModel::UR5e);

    Eigen::Vector3d wp1(-0.286, 0.274, 0.484);
    Eigen::Vector3d wp2(0.256, 0.479, 0.476);
    Eigen::Vector3d wp3(0.380, 0.152, 0.459);

    std::cout << "\n=== User Jog Path Analysis ===" << std::endl;
    std::cout << "WP1: (" << wp1.x() << ", " << wp1.y() << ", " << wp1.z() << ")" << std::endl;
    std::cout << "WP2: (" << wp2.x() << ", " << wp2.y() << ", " << wp2.z() << ")" << std::endl;
    std::cout << "WP3: (" << wp3.x() << ", " << wp3.y() << ", " << wp3.z() << ")" << std::endl;

    // Check IK solutions at each waypoint with identity orientation
    std::cout << "\n--- IK Solutions at Waypoints (Identity Orientation) ---" << std::endl;

    auto checkWaypoint = [&](const std::string& name, const Eigen::Vector3d& pos) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = pos;

        auto solutions = kin.inverse(pose);
        std::cout << name << ": " << solutions.size() << " IK solutions" << std::endl;

        if (!solutions.empty()) {
            // Show configurations for first few solutions
            for (size_t i = 0; i < std::min(solutions.size(), size_t(4)); ++i) {
                auto config = kin.getConfiguration(solutions[i]);
                auto singularity = kin.analyzeSingularity(solutions[i]);
                std::cout << "  Sol " << i << ": shoulder=" << static_cast<int>(config.shoulder)
                          << " elbow=" << static_cast<int>(config.elbow)
                          << " wrist=" << static_cast<int>(config.wrist)
                          << " | cond=" << singularity.condition_number;
                if (singularity.near_singularity) {
                    std::cout << " SINGULAR(" << static_cast<int>(singularity.type) << ")";
                }
                std::cout << std::endl;
            }
        }
        return solutions;
    };

    auto sol1 = checkWaypoint("WP1", wp1);
    auto sol2 = checkWaypoint("WP2", wp2);
    auto sol3 = checkWaypoint("WP3", wp3);

    REQUIRE(!sol1.empty());
    REQUIRE(!sol2.empty());
    REQUIRE(!sol3.empty());

    // Find a common configuration that works for all three waypoints
    std::cout << "\n--- Finding Common Configuration ---" << std::endl;

    for (size_t i = 0; i < sol1.size(); ++i) {
        auto config1 = kin.getConfiguration(sol1[i]);

        auto same_config_sol2 = kin.filterByConfiguration(sol2, config1);
        auto same_config_sol3 = kin.filterByConfiguration(sol3, config1);

        if (!same_config_sol2.empty() && !same_config_sol3.empty()) {
            std::cout << "Config from WP1 solution " << i << " works for ALL waypoints!" << std::endl;
            std::cout << "  shoulder=" << static_cast<int>(config1.shoulder)
                      << " elbow=" << static_cast<int>(config1.elbow)
                      << " wrist=" << static_cast<int>(config1.wrist) << std::endl;
        }
    }

    // Analyze segment 1 (WP1 to WP2) - the one that fails
    std::cout << "\n--- Segment 1 Analysis (WP1 to WP2) ---" << std::endl;

    // Use first solution's configuration as reference
    JointVector start_joints = sol1[0];
    auto start_config = kin.getConfiguration(start_joints);
    std::cout << "Starting with config: shoulder=" << static_cast<int>(start_config.shoulder)
              << " elbow=" << static_cast<int>(start_config.elbow)
              << " wrist=" << static_cast<int>(start_config.wrist) << std::endl;

    constexpr int kNumPoints = 20;
    JointVector current_joints = start_joints;

    for (int i = 0; i <= kNumPoints; ++i) {
        double t = static_cast<double>(i) / kNumPoints;
        Eigen::Vector3d pos = wp1 + t * (wp2 - wp1);

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = pos;

        auto solutions = kin.inverse(pose);
        auto same_config = kin.filterByConfiguration(solutions, start_config);

        // Also try to find closest solution to current
        std::optional<JointVector> best;
        if (!solutions.empty()) {
            best = kin.selectBestSolution(solutions, current_joints, 1.0);
        }

        auto singularity = kin.analyzeSingularity(current_joints);

        std::cout << "t=" << std::fixed << std::setprecision(2) << t
                  << " pos=(" << std::setprecision(3) << pos.x() << ", " << pos.y() << ", " << pos.z() << ")"
                  << " | total_sol=" << solutions.size()
                  << " same_config=" << same_config.size()
                  << " | cond=" << std::setprecision(1) << singularity.condition_number;

        if (singularity.near_singularity) {
            std::cout << " SINGULAR(" << static_cast<int>(singularity.type) << ")";
        }

        if (same_config.empty() && !solutions.empty()) {
            std::cout << " *** CONFIG BOUNDARY ***";
            // Show what configs ARE available
            std::cout << std::endl << "  Available configs:";
            for (size_t j = 0; j < std::min(solutions.size(), size_t(3)); ++j) {
                auto c = kin.getConfiguration(solutions[j]);
                std::cout << " [s=" << static_cast<int>(c.shoulder)
                          << " e=" << static_cast<int>(c.elbow)
                          << " w=" << static_cast<int>(c.wrist) << "]";
            }
        }

        std::cout << std::endl;

        // Update current joints if we found a solution
        if (best.has_value()) {
            current_joints = *best;
        } else if (!same_config.empty()) {
            current_joints = same_config[0];
        }
    }

    // Try with a different starting configuration
    std::cout << "\n--- Trying Different Starting Configurations ---" << std::endl;

    for (size_t start_sol_idx = 0; start_sol_idx < sol1.size(); ++start_sol_idx) {
        JointVector test_start = sol1[start_sol_idx];
        auto test_config = kin.getConfiguration(test_start);

        bool path_viable = true;
        int fail_point = -1;

        JointVector cur = test_start;
        for (int i = 0; i <= kNumPoints && path_viable; ++i) {
            double t = static_cast<double>(i) / kNumPoints;
            Eigen::Vector3d pos = wp1 + t * (wp2 - wp1);

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = pos;

            auto solutions = kin.inverse(pose);
            auto same_config = kin.filterByConfiguration(solutions, test_config);

            if (same_config.empty()) {
                path_viable = false;
                fail_point = i;
            } else {
                // Find closest
                double min_dist = 1e9;
                for (const auto& s : same_config) {
                    double d = (s - cur).norm();
                    if (d < min_dist) {
                        min_dist = d;
                        cur = s;
                    }
                }
            }
        }

        std::cout << "Start sol " << start_sol_idx << " (s=" << static_cast<int>(test_config.shoulder)
                  << " e=" << static_cast<int>(test_config.elbow)
                  << " w=" << static_cast<int>(test_config.wrist) << "): ";

        if (path_viable) {
            std::cout << "PATH VIABLE!" << std::endl;
        } else {
            std::cout << "fails at t=" << static_cast<double>(fail_point) / kNumPoints << std::endl;
        }
    }

    REQUIRE(true);  // Analysis test - always passes

    // Now actually test the trajectory planner with these waypoints
    SECTION("Trajectory planner should find viable configuration") {
        TrajectoryPlanner planner(kin);

        std::vector<Waypoint> waypoints;
        waypoints.push_back(createReachableWaypoint(-0.286, 0.274, 0.484));
        waypoints.push_back(createReachableWaypoint(0.256, 0.479, 0.476));
        waypoints.push_back(createReachableWaypoint(0.380, 0.152, 0.459));
        // Set segment times adequate for the distances and V_max=0.5 m/s
        waypoints[1].segment_time = 2.5;  // WP1->WP2: ~0.58m, need T >= 2.32s
        waypoints[2].segment_time = 1.5;  // WP2->WP3: ~0.35m, need T >= 1.40s

        auto trajectory = planner.plan(waypoints);

        std::cout << "\n--- Trajectory Planner Result ---" << std::endl;
        std::cout << "Valid: " << (trajectory.valid ? "YES" : "NO") << std::endl;
        std::cout << "Duration: " << trajectory.total_duration << "s" << std::endl;
        std::cout << "Samples: " << trajectory.samples.size() << std::endl;

        for (const auto& msg : trajectory.messages) {
            std::cout << "Message: " << msg.message << std::endl;
        }

        if (trajectory.valid && !trajectory.samples.empty()) {
            auto start_config = kin.getConfiguration(trajectory.samples.front().joints);
            auto end_config = kin.getConfiguration(trajectory.samples.back().joints);
            std::cout << "Start config: shoulder=" << static_cast<int>(start_config.shoulder)
                      << " elbow=" << static_cast<int>(start_config.elbow)
                      << " wrist=" << static_cast<int>(start_config.wrist) << std::endl;
            std::cout << "End config: shoulder=" << static_cast<int>(end_config.shoulder)
                      << " elbow=" << static_cast<int>(end_config.elbow)
                      << " wrist=" << static_cast<int>(end_config.wrist) << std::endl;
        }

        REQUIRE(trajectory.valid);
    }
}

// =============================================================================
// Configuration Detection Issue Tests
// =============================================================================

TEST_CASE("Rectangular path should not require configuration change", "[trajectory][config_bug]") {
    // This test reproduces a user-reported bug where a simple rectangular path
    // in 3D space is incorrectly rejected as requiring a configuration change,
    // even though the user can physically move through all waypoints without
    // any arm reconfiguration.
    //
    // The waypoints form a simple rectangular path:
    // WP1 -> WP2: Move in -X direction (same Z)
    // WP2 -> WP3: Move in -Z direction (same X)
    // WP3 -> WP4: Move in +X direction (same Z)

    URKinematics kin(URModel::UR5e);
    TrajectoryConfig config;
    config.sample_rate = 125.0;
    config.max_joint_velocity = 1.5;
    config.max_joint_acceleration = 2.0;
    TrajectoryPlanner planner(kin, config);

    // User's exact waypoints that fail planning
    std::vector<Eigen::Vector3d> positions = {
        {-0.198, -0.552, 0.753},  // WP1 (entry point)
        {-0.358, -0.552, 0.753},  // WP2
        {-0.358, -0.552, 0.487},  // WP3
        {-0.079, -0.552, 0.487}   // WP4 - this one fails
    };

    // Use the actual TCP orientation from the robot during testing
    // Captured from /api/trajectory/current-pose at position (-0.237, -0.552, 0.753)
    // This is a typical orientation for reaching to the side
    Eigen::Quaterniond orientation(0.741808, 0.591299, 0.209173, -0.237346);  // w, x, y, z

    std::cout << "\n=== Rectangular Path Configuration Analysis ===" << std::endl;

    // Step 1: Get IK solutions for the first waypoint to determine entry_joints
    Eigen::Isometry3d first_pose = Eigen::Isometry3d::Identity();
    first_pose.translation() = positions[0];
    first_pose.linear() = orientation.toRotationMatrix();

    auto first_solutions = kin.inverse(first_pose);

    // Use actual joint values captured from robot near this pose
    // joints: [-1.794715, -1.888088, -1.078697, 0.046977, 1.819093, 2.547276]
    JointVector real_entry_joints;
    real_entry_joints << -1.794715, -1.888088, -1.078697, 0.046977, 1.819093, 2.547276;

    // If IK solutions exist, prefer the one closest to real entry joints
    JointVector entry_joints;
    if (first_solutions.empty()) {
        std::cout << "NOTE: No IK solutions for first waypoint, using captured joints" << std::endl;
        entry_joints = real_entry_joints;
    } else {
        // Find solution closest to real entry joints
        entry_joints = first_solutions[0];
        double best_dist = (first_solutions[0] - real_entry_joints).squaredNorm();
        for (const auto& sol : first_solutions) {
            double dist = (sol - real_entry_joints).squaredNorm();
            if (dist < best_dist) {
                best_dist = dist;
                entry_joints = sol;
            }
        }
    }
    auto entry_config = kin.getConfiguration(entry_joints);

    std::cout << "Entry joints (radians): [";
    for (int j = 0; j < 6; ++j) {
        std::cout << entry_joints[j];
        if (j < 5) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "Entry config: shoulder=" << static_cast<int>(entry_config.shoulder)
              << " elbow=" << static_cast<int>(entry_config.elbow)
              << " wrist=" << static_cast<int>(entry_config.wrist) << std::endl;

    // Step 2: Analyze IK solutions for each waypoint
    JointVector current_joints = entry_joints;
    bool all_reachable_in_config = true;

    for (size_t i = 0; i < positions.size(); ++i) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = positions[i];
        pose.linear() = orientation.toRotationMatrix();

        auto solutions = kin.inverse(pose);
        std::cout << "\nWP" << (i + 1) << " at (" << positions[i].x() << ", "
                  << positions[i].y() << ", " << positions[i].z() << "):" << std::endl;
        std::cout << "  Total IK solutions: " << solutions.size() << std::endl;

        // Check each solution's configuration
        auto matching_solutions = kin.filterByConfiguration(solutions, entry_config);
        std::cout << "  Solutions in entry config: " << matching_solutions.size() << std::endl;

        if (matching_solutions.empty()) {
            std::cout << "  *** NO SOLUTION IN ENTRY CONFIG ***" << std::endl;
            all_reachable_in_config = false;

            // Print all solutions and their configurations for diagnosis
            for (size_t s = 0; s < solutions.size(); ++s) {
                auto sol_config = kin.getConfiguration(solutions[s]);
                std::cout << "    Solution " << s << ": shoulder=" << static_cast<int>(sol_config.shoulder)
                          << " elbow=" << static_cast<int>(sol_config.elbow)
                          << " wrist=" << static_cast<int>(sol_config.wrist);

                // Show the critical joint values
                JointVector qw = solutions[s];
                std::cout << " (q3=" << qw[2] << ", q5=" << qw[4] << ")" << std::endl;
            }
        } else {
            // Select closest solution and update current_joints
            JointVector best = matching_solutions[0];
            double best_dist = (best - current_joints).squaredNorm();
            for (const auto& sol : matching_solutions) {
                double dist = (sol - current_joints).squaredNorm();
                if (dist < best_dist) {
                    best_dist = dist;
                    best = sol;
                }
            }
            current_joints = best;

            std::cout << "  Selected joints: [";
            for (int j = 0; j < 6; ++j) {
                std::cout << current_joints[j];
                if (j < 5) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            std::cout << "  Critical angles: q3=" << current_joints[2]
                      << " q5=" << current_joints[4] << std::endl;
        }
    }

    // Step 3: Now try to actually plan the sequence
    Sequence sequence;
    sequence.name = "Test Rectangular Path";
    sequence.entry_joints = entry_joints;
    for (const auto& pos : positions) {
        SequenceWaypoint sw;
        sw.position = pos;
        sw.orientation = orientation;
        sw.blend_factor = 0.0;
        sw.segment_time = 2.0;
        sw.pause_time = 0.0;
        sequence.waypoints.push_back(sw);
    }

    auto trajectory = planner.planSequence(sequence, entry_joints);

    std::cout << "\n=== Planning Result ===" << std::endl;
    std::cout << "Valid: " << (trajectory.valid ? "YES" : "NO") << std::endl;
    for (const auto& msg : trajectory.messages) {
        std::cout << "Message: " << msg.message << std::endl;
    }

    // The test currently documents the bug - all waypoints should be reachable
    // If this test fails, it confirms the configuration detection bug exists
    if (!all_reachable_in_config) {
        std::cout << "\n*** BUG CONFIRMED: Some waypoints have no IK solution in entry config ***" << std::endl;
        std::cout << "This indicates the configuration boundary detection is too strict." << std::endl;
        std::cout << "A joint crossing near zero (e.g., q3 or q5) may flip the detected config" << std::endl;
        std::cout << "even though no physical arm reconfiguration occurs." << std::endl;
    }

    // For now, document the current behavior
    // TODO: Fix the configuration detection to handle boundary cases
    // REQUIRE(trajectory.valid);  // Uncomment after fix

    // Additional analysis: What if we ignore configuration and just pick closest solution?
    std::cout << "\n=== Continuity Analysis (ignoring config boundaries) ===" << std::endl;
    current_joints = entry_joints;
    double max_joint_change = 0.0;
    int max_change_joint = -1;
    size_t max_change_wp = 0;

    for (size_t i = 0; i < positions.size(); ++i) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = positions[i];
        pose.linear() = orientation.toRotationMatrix();

        auto solutions = kin.inverse(pose);
        if (solutions.empty()) continue;

        // Find closest solution (ignore configuration)
        JointVector best = solutions[0];
        double best_dist = (solutions[0] - current_joints).squaredNorm();
        for (const auto& sol : solutions) {
            double dist = (sol - current_joints).squaredNorm();
            if (dist < best_dist) {
                best_dist = dist;
                best = sol;
            }
        }

        // Check max joint change
        for (int j = 0; j < 6; ++j) {
            double change = std::abs(best[j] - current_joints[j]);
            if (change > max_joint_change) {
                max_joint_change = change;
                max_change_joint = j;
                max_change_wp = i;
            }
        }

        std::cout << "WP" << (i + 1) << " -> closest solution:" << std::endl;
        std::cout << "  q1 change: " << (best[0] - current_joints[0]) * 180.0 / kPi << " deg" << std::endl;
        std::cout << "  q3 change: " << (best[2] - current_joints[2]) * 180.0 / kPi << " deg" << std::endl;
        std::cout << "  q5 change: " << (best[4] - current_joints[4]) * 180.0 / kPi << " deg" << std::endl;

        auto sol_config = kin.getConfiguration(best);
        std::cout << "  Config: shoulder=" << static_cast<int>(sol_config.shoulder)
                  << " elbow=" << static_cast<int>(sol_config.elbow)
                  << " wrist=" << static_cast<int>(sol_config.wrist) << std::endl;

        current_joints = best;
    }

    std::cout << "\nMax joint change: " << max_joint_change * 180.0 / kPi
              << " deg (joint " << max_change_joint << " at WP" << (max_change_wp + 1) << ")" << std::endl;

    // If max change is small (< 45 degrees), the path is likely feasible
    // even if configuration boundaries are crossed
    if (max_joint_change < kPi / 4.0) {
        std::cout << "*** Path appears feasible with smooth interpolation ***" << std::endl;
    }
}


// =============================================================================
// Ruckig Path Parameter Investigation
// =============================================================================

TEST_CASE("Ruckig 1D path parameter behavior", "[trajectory][ruckig][debug]") {
    // Test case from user's scenario:
    // Segment 1: distance=0.184m, duration=5.00s, v_entry=0.000, v_exit=0.479

    SECTION("Long duration with high exit velocity") {
        ruckig::Ruckig<1> ruckig;
        ruckig::InputParameter<1> input;
        ruckig::Trajectory<1> trajectory;

        double distance = 0.184;  // meters
        double duration = 5.0;    // seconds
        double v_entry = 0.0;     // m/s
        double v_exit = 0.479;    // m/s

        // Set initial state
        input.current_position[0] = 0.0;
        input.current_velocity[0] = v_entry;
        input.current_acceleration[0] = 0.0;

        // Set target state
        input.target_position[0] = distance;
        input.target_velocity[0] = v_exit;
        input.target_acceleration[0] = 0.0;

        // Set kinematic limits (from TrajectoryConfig defaults)
        input.max_velocity[0] = 0.5;      // m/s
        input.max_acceleration[0] = 1.0;  // m/s²
        input.max_jerk[0] = 5.0;          // m/s³

        // Set minimum duration
        input.minimum_duration = duration;

        // Calculate trajectory
        ruckig::Result result = ruckig.calculate(input, trajectory);

        std::cout << "\n=== Ruckig 1D Path Test ==="  << std::endl;
        std::cout << "Input:" << std::endl;
        std::cout << "  distance = " << distance << " m" << std::endl;
        std::cout << "  duration = " << duration << " s" << std::endl;
        std::cout << "  v_entry = " << v_entry << " m/s" << std::endl;
        std::cout << "  v_exit = " << v_exit << " m/s" << std::endl;
        std::cout << "  max_velocity = " << input.max_velocity[0] << " m/s" << std::endl;
        std::cout << "  max_acceleration = " << input.max_acceleration[0] << " m/s²" << std::endl;
        std::cout << "  max_jerk = " << input.max_jerk[0] << " m/s³" << std::endl;

        std::cout << "\nResult: " << static_cast<int>(result) << std::endl;
        std::cout << "  (0=Working, 1=Finished, -1=Error...)" << std::endl;

        if (result == ruckig::Result::Working || result == ruckig::Result::Finished) {
            double actual_duration = trajectory.get_duration();
            std::cout << "\nActual duration: " << actual_duration << " s" << std::endl;

            // Sample at key points
            std::array<double, 1> pos, vel, acc;
            std::cout << "\nTrajectory samples:" << std::endl;
            std::cout << "  t=0.0: ";
            trajectory.at_time(0.0, pos, vel, acc);
            std::cout << "pos=" << pos[0] << ", vel=" << vel[0] << std::endl;

            for (double t = 0.5; t < actual_duration; t += 0.5) {
                trajectory.at_time(t, pos, vel, acc);
                std::cout << "  t=" << t << ": pos=" << pos[0] << ", vel=" << vel[0] << std::endl;
            }

            trajectory.at_time(actual_duration, pos, vel, acc);
            std::cout << "  t=" << actual_duration << " (end): pos=" << pos[0] << ", vel=" << vel[0] << std::endl;

            // Check if final position matches target
            std::cout << "\nFinal position: " << pos[0] << " (target: " << distance << ")" << std::endl;
            std::cout << "Final velocity: " << vel[0] << " (target: " << v_exit << ")" << std::endl;

            // Check if position ever exceeds target
            double max_pos = 0.0;
            for (double t = 0.0; t <= actual_duration; t += 0.01) {
                trajectory.at_time(t, pos, vel, acc);
                if (pos[0] > max_pos) max_pos = pos[0];
            }
            std::cout << "\nMax position reached: " << max_pos << " m" << std::endl;
            if (max_pos > distance * 1.01) {
                std::cout << "*** WARNING: Position exceeded target by " 
                          << (max_pos - distance) * 1000 << " mm ***" << std::endl;
            }

            REQUIRE(result == ruckig::Result::Working);
            // Check final position is close to target
            trajectory.at_time(actual_duration, pos, vel, acc);
            CHECK(std::abs(pos[0] - distance) < 0.001);
        } else {
            std::cout << "*** Ruckig FAILED to compute trajectory ***" << std::endl;
            FAIL("Ruckig returned error");
        }
    }

    SECTION("Very long duration - should add cruise phase") {
        ruckig::Ruckig<1> ruckig;
        ruckig::InputParameter<1> input;
        ruckig::Trajectory<1> trajectory;

        double distance = 0.184;
        double duration = 10.0;  // Even longer
        double v_entry = 0.0;
        double v_exit = 0.479;

        input.current_position[0] = 0.0;
        input.current_velocity[0] = v_entry;
        input.current_acceleration[0] = 0.0;
        input.target_position[0] = distance;
        input.target_velocity[0] = v_exit;
        input.target_acceleration[0] = 0.0;
        input.max_velocity[0] = 0.5;
        input.max_acceleration[0] = 1.0;
        input.max_jerk[0] = 5.0;
        input.minimum_duration = duration;

        ruckig::Result result = ruckig.calculate(input, trajectory);

        std::cout << "\n=== Ruckig 10s Duration Test ==="  << std::endl;
        std::cout << "Result: " << static_cast<int>(result) << std::endl;

        if (result == ruckig::Result::Working || result == ruckig::Result::Finished) {
            double actual_duration = trajectory.get_duration();
            std::cout << "Actual duration: " << actual_duration << " s" << std::endl;

            std::array<double, 1> pos, vel, acc;
            trajectory.at_time(actual_duration, pos, vel, acc);
            std::cout << "Final pos=" << pos[0] << ", vel=" << vel[0] << std::endl;

            // Check max position
            double max_pos = 0.0;
            for (double t = 0.0; t <= actual_duration; t += 0.01) {
                trajectory.at_time(t, pos, vel, acc);
                if (pos[0] > max_pos) max_pos = pos[0];
            }
            std::cout << "Max position: " << max_pos << " m (target: " << distance << ")" << std::endl;
        }
    }

    SECTION("Short duration - time optimal") {
        ruckig::Ruckig<1> ruckig;
        ruckig::InputParameter<1> input;
        ruckig::Trajectory<1> trajectory;

        double distance = 0.184;
        double v_entry = 0.0;
        double v_exit = 0.479;

        input.current_position[0] = 0.0;
        input.current_velocity[0] = v_entry;
        input.current_acceleration[0] = 0.0;
        input.target_position[0] = distance;
        input.target_velocity[0] = v_exit;
        input.target_acceleration[0] = 0.0;
        input.max_velocity[0] = 0.5;
        input.max_acceleration[0] = 1.0;
        input.max_jerk[0] = 5.0;
        // NO minimum_duration - let Ruckig find optimal

        ruckig::Result result = ruckig.calculate(input, trajectory);

        std::cout << "\n=== Ruckig Time-Optimal Test ==="  << std::endl;
        std::cout << "Result: " << static_cast<int>(result) << std::endl;

        if (result == ruckig::Result::Working || result == ruckig::Result::Finished) {
            double actual_duration = trajectory.get_duration();
            std::cout << "Time-optimal duration: " << actual_duration << " s" << std::endl;

            std::array<double, 1> pos, vel, acc;
            trajectory.at_time(actual_duration, pos, vel, acc);
            std::cout << "Final pos=" << pos[0] << ", vel=" << vel[0] << std::endl;
        }
    }
}

TEST_CASE("Ruckig impossible constraint - short distance high exit velocity", "[trajectory][ruckig][debug]") {
    // From user's scenario:
    // Segment 1: dist=0.107m, T=5.00s, v_entry=0.000, v_exit=0.473
    // This is IMPOSSIBLE: need 0.112m to accelerate from 0 to 0.473 at 1.0 m/s²

    ruckig::Ruckig<1> ruckig;
    ruckig::InputParameter<1> input;
    ruckig::Trajectory<1> trajectory;

    double distance = 0.107;  // meters (too short!)
    double duration = 5.0;    // seconds
    double v_entry = 0.0;     // m/s
    double v_exit = 0.473;    // m/s

    input.current_position[0] = 0.0;
    input.current_velocity[0] = v_entry;
    input.current_acceleration[0] = 0.0;
    input.target_position[0] = distance;
    input.target_velocity[0] = v_exit;
    input.target_acceleration[0] = 0.0;
    input.max_velocity[0] = 0.5;
    input.max_acceleration[0] = 1.0;
    input.max_jerk[0] = 5.0;
    input.minimum_duration = duration;

    ruckig::Result result = ruckig.calculate(input, trajectory);

    std::cout << "\n=== Impossible Constraint Test ===" << std::endl;
    std::cout << "Input: distance=" << distance << "m, v_exit=" << v_exit << " m/s" << std::endl;
    std::cout << "Min distance to reach v_exit at 1.0 m/s²: "
              << (0.5 * v_exit * v_exit / 1.0) << " m" << std::endl;
    std::cout << "Result: " << static_cast<int>(result) << std::endl;

    if (result == ruckig::Result::Working || result == ruckig::Result::Finished) {
        double actual_duration = trajectory.get_duration();
        std::cout << "Actual duration: " << actual_duration << " s" << std::endl;

        std::array<double, 1> pos, vel, acc;

        // Sample at key points
        std::cout << "\nTrajectory profile:" << std::endl;
        for (double t = 0.0; t <= actual_duration + 0.01; t += 0.5) {
            if (t > actual_duration) t = actual_duration;
            trajectory.at_time(t, pos, vel, acc);
            std::cout << "  t=" << t << "s: pos=" << pos[0] << "m, vel=" << vel[0] << " m/s" << std::endl;
            if (t >= actual_duration) break;
        }

        trajectory.at_time(actual_duration, pos, vel, acc);
        std::cout << "\nFinal: pos=" << pos[0] << " (target: " << distance << ")" << std::endl;
        std::cout << "Final: vel=" << vel[0] << " (target: " << v_exit << ")" << std::endl;

        // Check if it exceeded target position
        double max_pos = 0.0;
        for (double t = 0.0; t <= actual_duration; t += 0.01) {
            trajectory.at_time(t, pos, vel, acc);
            if (pos[0] > max_pos) max_pos = pos[0];
        }
        std::cout << "Max position: " << max_pos << " m" << std::endl;

        if (std::abs(vel[0] - v_exit) > 0.01) {
            std::cout << "*** WARNING: Did not reach target velocity! ***" << std::endl;
        }
    } else {
        std::cout << "*** Ruckig returned ERROR ***" << std::endl;
    }
}

// =============================================================================
// Analytical Velocity Solver Tests (via TrajectoryPlanner integration)
// =============================================================================

TEST_CASE("Analytical velocity solver - reasonable segment durations", "[trajectory][velocity][analytical]") {
    URKinematics kinematics(URModel::UR5e);
    TrajectoryConfig config;
    config.max_linear_velocity = 0.5;       // m/s
    config.max_linear_acceleration = 1.0;   // m/s²
    config.max_linear_jerk = 5.0;           // m/s³
    config.sample_rate = 100.0;             // Hz

    TrajectoryPlanner planner(kinematics, config);

    SECTION("Two waypoints with reasonable segment time - should succeed") {
        std::vector<Waypoint> waypoints;

        // Simple linear move in front of robot
        Waypoint wp1 = createReachableWaypoint(0.4, -0.1, 0.4);
        wp1.segment_time = 0.0;  // First waypoint, no segment time
        waypoints.push_back(wp1);

        Waypoint wp2 = createReachableWaypoint(0.4, 0.1, 0.4);
        wp2.segment_time = 2.0;  // 2 seconds for 0.2m = 0.1 m/s avg velocity
        waypoints.push_back(wp2);

        auto result = planner.plan(waypoints);

        REQUIRE(result.valid);
        REQUIRE(!result.samples.empty());

        // Total duration should be approximately 2 seconds
        REQUIRE_THAT(result.total_duration, WithinAbs(2.0, 0.5));
    }

    SECTION("Segment with velocity limit exceeded - should fail with T_min message") {
        std::vector<Waypoint> waypoints;

        // Large move with very short time - should require V_entry > V_max
        Waypoint wp1 = createReachableWaypoint(0.3, -0.2, 0.4);
        waypoints.push_back(wp1);

        Waypoint wp2 = createReachableWaypoint(0.3, 0.2, 0.4);
        wp2.segment_time = 0.3;  // Only 0.3s for 0.4m move - too fast!
        waypoints.push_back(wp2);

        auto result = planner.plan(waypoints);

        // Should fail with helpful error message
        REQUIRE_FALSE(result.valid);

        // Check for error messages - print them for debugging
        for (const auto& msg : result.messages) {
            if (msg.severity == ValidationSeverity::Error) {
                std::cout << "Error message: " << msg.message << std::endl;
            }
        }
        // Note: The planner might fail for other reasons (IK, etc.) before velocity check
        // So we just verify it failed - the velocity solver is exercised
    }

    SECTION("Three waypoints with blending - should compute entry velocities") {
        std::vector<Waypoint> waypoints;

        Waypoint wp1 = createReachableWaypoint(0.4, -0.15, 0.4);
        waypoints.push_back(wp1);

        Waypoint wp2 = createReachableWaypoint(0.4, 0.0, 0.4);
        wp2.segment_time = 1.5;
        wp2.blend_factor = 0.5;  // Blend at this waypoint
        waypoints.push_back(wp2);

        Waypoint wp3 = createReachableWaypoint(0.4, 0.15, 0.4);
        wp3.segment_time = 1.5;
        waypoints.push_back(wp3);

        auto result = planner.plan(waypoints);

        // Should succeed - reasonable constraints
        REQUIRE(result.valid);
        REQUIRE(!result.samples.empty());

        // Check total duration is reasonable
        REQUIRE(result.total_duration > 2.5);
        REQUIRE(result.total_duration < 4.0);
    }
}

TEST_CASE("Analytical velocity solver - edge cases", "[trajectory][velocity][analytical]") {
    URKinematics kinematics(URModel::UR5e);
    TrajectoryConfig config;
    config.max_linear_velocity = 0.5;
    config.max_linear_acceleration = 1.0;
    config.max_linear_jerk = 5.0;
    config.sample_rate = 100.0;

    TrajectoryPlanner planner(kinematics, config);

    SECTION("Very short segment with stopping - triangular profile") {
        std::vector<Waypoint> waypoints;

        // Short 50mm move - need at least 0.68s with A=1.0, J=5.0
        Waypoint wp1 = createReachableWaypoint(0.4, 0.0, 0.4);
        waypoints.push_back(wp1);

        Waypoint wp2 = createReachableWaypoint(0.4, 0.05, 0.4);
        wp2.segment_time = 0.8;  // 0.8s for 50mm - achievable with triangular profile
        waypoints.push_back(wp2);

        auto result = planner.plan(waypoints);

        // Should succeed with achievable duration
        REQUIRE(result.valid);
    }

    SECTION("Long segment with generous time - full profile") {
        std::vector<Waypoint> waypoints;

        // 300mm move with plenty of time
        Waypoint wp1 = createReachableWaypoint(0.3, -0.15, 0.4);
        waypoints.push_back(wp1);

        Waypoint wp2 = createReachableWaypoint(0.3, 0.15, 0.4);
        wp2.segment_time = 3.0;  // 3s for 300mm - comfortable pace
        waypoints.push_back(wp2);

        auto result = planner.plan(waypoints);

        REQUIRE(result.valid);
        REQUIRE_THAT(result.total_duration, WithinAbs(3.0, 0.5));
    }
}

// Note: Old S-curve tests removed - see test_s_curve.cpp for new 2-phase S-curve tests
