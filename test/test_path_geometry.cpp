/// @file test_path_geometry.cpp
/// @brief Unit tests for PathGeometry module

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ur_controller/trajectory/path_geometry.hpp"

#include <cmath>

using namespace ur_controller::trajectory;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-6;

// Helper to create a simple waypoint
SequenceWaypoint makeWaypoint(double x, double y, double z, double blend_factor = 0.0) {
    SequenceWaypoint wp;
    wp.position = Eigen::Vector3d(x, y, z);
    wp.orientation = Eigen::Quaterniond::Identity();
    wp.blend_factor = blend_factor;
    return wp;
}

}  // namespace

// =============================================================================
// Basic Construction Tests
// =============================================================================

TEST_CASE("PathGeometry empty path", "[path_geometry]") {
    SECTION("Default constructor creates empty path") {
        PathGeometry path;
        REQUIRE(path.empty());
        REQUIRE(path.numSegments() == 0);
        REQUIRE(path.totalLength() == 0.0);
    }

    SECTION("Single waypoint creates empty path") {
        std::vector<SequenceWaypoint> waypoints = {
            makeWaypoint(0.5, 0.0, 0.4)
        };

        auto path = PathGeometry::fromWaypoints(waypoints);

        REQUIRE(path.empty());
        REQUIRE(path.numWaypoints() == 1);
        REQUIRE(path.waypointDistances().size() == 1);
        REQUIRE(path.waypointDistances()[0] == 0.0);
    }
}

TEST_CASE("PathGeometry linear path - two waypoints", "[path_geometry]") {
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0),
        makeWaypoint(1.0, 0.0, 0.0)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Path has correct structure") {
        REQUIRE(path.numSegments() == 1);
        REQUIRE(path.numWaypoints() == 2);
        REQUIRE_THAT(path.totalLength(), WithinAbs(1.0, kTolerance));
    }

    SECTION("Segment has correct properties") {
        const auto& seg = path.segment(0);
        REQUIRE(seg.type == PathSegment::Type::Linear);
        REQUIRE_THAT(seg.length, WithinAbs(1.0, kTolerance));
        REQUIRE_THAT(seg.start_distance, WithinAbs(0.0, kTolerance));
    }

    SECTION("Waypoint distances are correct") {
        const auto& distances = path.waypointDistances();
        REQUIRE(distances.size() == 2);
        REQUIRE_THAT(distances[0], WithinAbs(0.0, kTolerance));
        REQUIRE_THAT(distances[1], WithinAbs(1.0, kTolerance));
    }
}

TEST_CASE("PathGeometry linear path - three waypoints", "[path_geometry]") {
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0),
        makeWaypoint(0.3, 0.0, 0.0),
        makeWaypoint(0.3, 0.4, 0.0)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Path has correct structure") {
        REQUIRE(path.numSegments() == 2);
        REQUIRE(path.numWaypoints() == 3);
        REQUIRE_THAT(path.totalLength(), WithinAbs(0.7, kTolerance));
    }

    SECTION("First segment is correct") {
        const auto& seg = path.segment(0);
        REQUIRE_THAT(seg.length, WithinAbs(0.3, kTolerance));
        REQUIRE_THAT(seg.start_distance, WithinAbs(0.0, kTolerance));
    }

    SECTION("Second segment is correct") {
        const auto& seg = path.segment(1);
        REQUIRE_THAT(seg.length, WithinAbs(0.4, kTolerance));
        REQUIRE_THAT(seg.start_distance, WithinAbs(0.3, kTolerance));
    }

    SECTION("Waypoint distances are correct") {
        const auto& distances = path.waypointDistances();
        REQUIRE(distances.size() == 3);
        REQUIRE_THAT(distances[0], WithinAbs(0.0, kTolerance));
        REQUIRE_THAT(distances[1], WithinAbs(0.3, kTolerance));
        REQUIRE_THAT(distances[2], WithinAbs(0.7, kTolerance));
    }
}

// =============================================================================
// Evaluation Tests
// =============================================================================

TEST_CASE("PathGeometry evaluate at endpoints", "[path_geometry]") {
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.5, 0.0, 0.4),
        makeWaypoint(0.5, 0.3, 0.4)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Evaluate at s=0 returns start position") {
        auto point = path.evaluate(0.0);

        REQUIRE_THAT(point.position.x(), WithinAbs(0.5, kTolerance));
        REQUIRE_THAT(point.position.y(), WithinAbs(0.0, kTolerance));
        REQUIRE_THAT(point.position.z(), WithinAbs(0.4, kTolerance));
        REQUIRE(point.segment_index == 0);
        REQUIRE_THAT(point.segment_progress, WithinAbs(0.0, kTolerance));
    }

    SECTION("Evaluate at s=total_length returns end position") {
        auto point = path.evaluate(path.totalLength());

        REQUIRE_THAT(point.position.x(), WithinAbs(0.5, kTolerance));
        REQUIRE_THAT(point.position.y(), WithinAbs(0.3, kTolerance));
        REQUIRE_THAT(point.position.z(), WithinAbs(0.4, kTolerance));
        REQUIRE(point.segment_index == 0);
        REQUIRE_THAT(point.segment_progress, WithinAbs(1.0, kTolerance));
    }
}

TEST_CASE("PathGeometry evaluate at midpoint", "[path_geometry]") {
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0),
        makeWaypoint(1.0, 0.0, 0.0)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Midpoint has correct position") {
        auto point = path.evaluate(0.5);

        REQUIRE_THAT(point.position.x(), WithinAbs(0.5, kTolerance));
        REQUIRE_THAT(point.position.y(), WithinAbs(0.0, kTolerance));
        REQUIRE_THAT(point.position.z(), WithinAbs(0.0, kTolerance));
        REQUIRE_THAT(point.segment_progress, WithinAbs(0.5, kTolerance));
    }

    SECTION("Quarter point has correct position") {
        auto point = path.evaluate(0.25);

        REQUIRE_THAT(point.position.x(), WithinAbs(0.25, kTolerance));
        REQUIRE_THAT(point.segment_progress, WithinAbs(0.25, kTolerance));
    }
}

TEST_CASE("PathGeometry evaluate across multiple segments", "[path_geometry]") {
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0),
        makeWaypoint(1.0, 0.0, 0.0),
        makeWaypoint(1.0, 1.0, 0.0)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);
    // Total length = 2.0 (1.0 + 1.0)

    SECTION("Point in first segment") {
        auto point = path.evaluate(0.5);

        REQUIRE(point.segment_index == 0);
        REQUIRE_THAT(point.position.x(), WithinAbs(0.5, kTolerance));
        REQUIRE_THAT(point.position.y(), WithinAbs(0.0, kTolerance));
    }

    SECTION("Point at segment boundary") {
        auto point = path.evaluate(1.0);

        REQUIRE(point.segment_index == 0);  // At end of first segment
        REQUIRE_THAT(point.position.x(), WithinAbs(1.0, kTolerance));
        REQUIRE_THAT(point.position.y(), WithinAbs(0.0, kTolerance));
    }

    SECTION("Point in second segment") {
        auto point = path.evaluate(1.5);

        REQUIRE(point.segment_index == 1);
        REQUIRE_THAT(point.position.x(), WithinAbs(1.0, kTolerance));
        REQUIRE_THAT(point.position.y(), WithinAbs(0.5, kTolerance));
        REQUIRE_THAT(point.segment_progress, WithinAbs(0.5, kTolerance));
    }

    SECTION("Point at end of path") {
        auto point = path.evaluate(2.0);

        REQUIRE(point.segment_index == 1);
        REQUIRE_THAT(point.position.x(), WithinAbs(1.0, kTolerance));
        REQUIRE_THAT(point.position.y(), WithinAbs(1.0, kTolerance));
    }
}

TEST_CASE("PathGeometry evaluate clamping", "[path_geometry]") {
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0),
        makeWaypoint(1.0, 0.0, 0.0)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Negative s is clamped to 0") {
        auto point = path.evaluate(-0.5);

        REQUIRE_THAT(point.position.x(), WithinAbs(0.0, kTolerance));
    }

    SECTION("s beyond total length is clamped") {
        auto point = path.evaluate(5.0);

        REQUIRE_THAT(point.position.x(), WithinAbs(1.0, kTolerance));
    }
}

// =============================================================================
// Orientation Tests
// =============================================================================

TEST_CASE("PathGeometry orientation interpolation", "[path_geometry]") {
    std::vector<SequenceWaypoint> waypoints;

    // Start with identity
    auto wp1 = makeWaypoint(0.0, 0.0, 0.0);
    wp1.orientation = Eigen::Quaterniond::Identity();

    // End with 90 degree rotation around Z
    auto wp2 = makeWaypoint(1.0, 0.0, 0.0);
    wp2.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(kPi / 2, Eigen::Vector3d::UnitZ()));

    waypoints.push_back(wp1);
    waypoints.push_back(wp2);

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Orientation at start") {
        auto point = path.evaluate(0.0);

        REQUIRE_THAT(point.orientation.w(), WithinAbs(1.0, kTolerance));
        REQUIRE_THAT(point.orientation.x(), WithinAbs(0.0, kTolerance));
    }

    SECTION("Orientation at midpoint (SLERP)") {
        auto point = path.evaluate(0.5);

        // At midpoint, should have 45 degree rotation
        Eigen::Quaterniond expected(Eigen::AngleAxisd(kPi / 4, Eigen::Vector3d::UnitZ()));

        REQUIRE_THAT(point.orientation.w(), WithinAbs(expected.w(), kTolerance));
        REQUIRE_THAT(point.orientation.z(), WithinAbs(expected.z(), kTolerance));
    }

    SECTION("Orientation at end") {
        auto point = path.evaluate(1.0);

        Eigen::Quaterniond expected(Eigen::AngleAxisd(kPi / 2, Eigen::Vector3d::UnitZ()));

        REQUIRE_THAT(point.orientation.w(), WithinAbs(expected.w(), kTolerance));
        REQUIRE_THAT(point.orientation.z(), WithinAbs(expected.z(), kTolerance));
    }
}

// =============================================================================
// PathPoint toPose Tests
// =============================================================================

TEST_CASE("PathPoint toPose conversion", "[path_geometry]") {
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.5, 0.2, 0.4),
        makeWaypoint(0.5, 0.5, 0.4)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);
    auto point = path.evaluate(0.15);  // Midpoint

    auto pose = point.toPose();

    SECTION("Translation is correct") {
        REQUIRE_THAT(pose.translation().x(), WithinAbs(0.5, kTolerance));
        REQUIRE_THAT(pose.translation().y(), WithinAbs(0.35, kTolerance));
        REQUIRE_THAT(pose.translation().z(), WithinAbs(0.4, kTolerance));
    }

    SECTION("Rotation matrix is valid") {
        // Check that rotation matrix is orthonormal
        Eigen::Matrix3d R = pose.linear();
        Eigen::Matrix3d RtR = R.transpose() * R;

        REQUIRE_THAT(RtR(0, 0), WithinAbs(1.0, kTolerance));
        REQUIRE_THAT(RtR(1, 1), WithinAbs(1.0, kTolerance));
        REQUIRE_THAT(RtR(2, 2), WithinAbs(1.0, kTolerance));
    }
}

// =============================================================================
// 3D Path Tests
// =============================================================================

TEST_CASE("PathGeometry 3D diagonal path", "[path_geometry]") {
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0),
        makeWaypoint(1.0, 1.0, 1.0)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    // Length should be sqrt(3)
    double expected_length = std::sqrt(3.0);
    REQUIRE_THAT(path.totalLength(), WithinAbs(expected_length, kTolerance));

    SECTION("Midpoint is correct") {
        auto point = path.evaluate(expected_length / 2.0);

        REQUIRE_THAT(point.position.x(), WithinAbs(0.5, kTolerance));
        REQUIRE_THAT(point.position.y(), WithinAbs(0.5, kTolerance));
        REQUIRE_THAT(point.position.z(), WithinAbs(0.5, kTolerance));
    }
}

// =============================================================================
// Blend Arc Tests
// =============================================================================

TEST_CASE("PathGeometry 90-degree corner with blend arc", "[path_geometry][blend]") {
    // L-shaped path: (0,0,0) -> (1,0,0) -> (1,1,0) with blend at corner
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0, 0.0),   // Start, no blend
        makeWaypoint(1.0, 0.0, 0.0, 0.1),   // Corner with blend_factor=0.1
        makeWaypoint(1.0, 1.0, 0.0, 0.0)    // End, no blend
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Path has correct number of segments") {
        // Should have: linear, arc, linear = 3 segments
        REQUIRE(path.numSegments() == 3);
    }

    SECTION("First segment is linear") {
        const auto& seg = path.segment(0);
        REQUIRE(seg.type == PathSegment::Type::Linear);
    }

    SECTION("Second segment is arc") {
        const auto& seg = path.segment(1);
        REQUIRE(seg.type == PathSegment::Type::Arc);
        REQUIRE(seg.arc_angle > 0.0);
        REQUIRE(seg.radius > 0.0);
    }

    SECTION("Third segment is linear") {
        const auto& seg = path.segment(2);
        REQUIRE(seg.type == PathSegment::Type::Linear);
    }

    SECTION("Total length is less than without blend") {
        // Without blend: 1.0 + 1.0 = 2.0
        // With blend: straight sections are shorter, arc adds some length
        // Arc replaces a corner, so total should be slightly less than 2.0
        REQUIRE(path.totalLength() < 2.0);
        REQUIRE(path.totalLength() > 1.5);  // But not too short
    }

    SECTION("Path is continuous at segment boundaries") {
        const auto& seg1 = path.segment(0);
        const auto& seg2 = path.segment(1);
        const auto& seg3 = path.segment(2);

        // End of seg1 should equal start of seg2
        REQUIRE_THAT((seg1.end_pos - seg2.start_pos).norm(), WithinAbs(0.0, kTolerance));

        // End of seg2 should equal start of seg3
        REQUIRE_THAT((seg2.end_pos - seg3.start_pos).norm(), WithinAbs(0.0, kTolerance));
    }
}

TEST_CASE("PathGeometry arc evaluation", "[path_geometry][blend]") {
    // Simple 90-degree corner with known geometry
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0, 0.0),
        makeWaypoint(1.0, 0.0, 0.0, 0.1),  // blend_factor = 0.1
        makeWaypoint(1.0, 1.0, 0.0, 0.0)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Arc midpoint is equidistant from endpoints") {
        const auto& arc = path.segment(1);

        // Evaluate at arc midpoint
        double arc_mid_s = arc.start_distance + arc.length / 2.0;
        auto mid_point = path.evaluate(arc_mid_s);

        // Arc start and end
        auto start_point = path.evaluate(arc.start_distance);
        auto end_point = path.evaluate(arc.start_distance + arc.length);

        // Distance from center to midpoint should equal radius
        double dist_to_center = (mid_point.position - arc.center).norm();
        REQUIRE_THAT(dist_to_center, WithinAbs(arc.radius, kTolerance));

        // Distance from center to start should equal radius
        double start_to_center = (start_point.position - arc.center).norm();
        REQUIRE_THAT(start_to_center, WithinAbs(arc.radius, kTolerance));

        // Distance from center to end should equal radius
        double end_to_center = (end_point.position - arc.center).norm();
        REQUIRE_THAT(end_to_center, WithinAbs(arc.radius, kTolerance));
    }

    SECTION("Arc midpoint is offset from the corner") {
        const auto& arc = path.segment(1);
        double arc_mid_s = arc.start_distance + arc.length / 2.0;
        auto mid_point = path.evaluate(arc_mid_s);

        // For a 90-degree corner at (1,0,0), the arc midpoint should be
        // somewhere in the quadrant approaching the corner but not at it
        REQUIRE(mid_point.position.x() < 1.0);  // Not at the corner x
        REQUIRE(mid_point.position.y() > 0.0);  // Started moving toward y
    }
}

TEST_CASE("PathGeometry no blend at endpoints", "[path_geometry][blend]") {
    // Blend factors at first and last waypoints should be ignored
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0, 0.5),   // blend_factor ignored (first)
        makeWaypoint(1.0, 0.0, 0.0, 0.0),   // no blend
        makeWaypoint(1.0, 1.0, 0.0, 0.5)    // blend_factor ignored (last)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Only linear segments (no arcs at endpoints)") {
        REQUIRE(path.numSegments() == 2);
        REQUIRE(path.segment(0).type == PathSegment::Type::Linear);
        REQUIRE(path.segment(1).type == PathSegment::Type::Linear);
    }

    SECTION("Path length is sum of segment lengths") {
        REQUIRE_THAT(path.totalLength(), WithinAbs(2.0, kTolerance));
    }
}

TEST_CASE("PathGeometry zero blend factor", "[path_geometry][blend]") {
    // Explicit zero blend factor should not create arc
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0),
        makeWaypoint(1.0, 0.0, 0.0, 0.0),  // explicit zero blend
        makeWaypoint(1.0, 1.0, 0.0)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Only linear segments") {
        REQUIRE(path.numSegments() == 2);
        REQUIRE(path.segment(0).type == PathSegment::Type::Linear);
        REQUIRE(path.segment(1).type == PathSegment::Type::Linear);
    }
}

TEST_CASE("PathGeometry multiple blend arcs", "[path_geometry][blend]") {
    // Zigzag path with blends at each interior corner
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0, 0.0),
        makeWaypoint(1.0, 0.0, 0.0, 0.05),  // blend
        makeWaypoint(1.0, 1.0, 0.0, 0.05),  // blend
        makeWaypoint(2.0, 1.0, 0.0, 0.0)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Has correct segment structure") {
        // linear, arc, linear, arc, linear = 5 segments
        REQUIRE(path.numSegments() == 5);
        REQUIRE(path.segment(0).type == PathSegment::Type::Linear);
        REQUIRE(path.segment(1).type == PathSegment::Type::Arc);
        REQUIRE(path.segment(2).type == PathSegment::Type::Linear);
        REQUIRE(path.segment(3).type == PathSegment::Type::Arc);
        REQUIRE(path.segment(4).type == PathSegment::Type::Linear);
    }

    SECTION("Path is continuous throughout") {
        for (size_t i = 0; i + 1 < path.numSegments(); ++i) {
            const auto& seg1 = path.segment(i);
            const auto& seg2 = path.segment(i + 1);
            REQUIRE_THAT((seg1.end_pos - seg2.start_pos).norm(), WithinAbs(0.0, kTolerance));
        }
    }
}

TEST_CASE("PathGeometry blend arc clamping", "[path_geometry][blend]") {
    // Large blend factor should be clamped to not exceed half of shortest segment
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.0, 0.0, 0.0),
        makeWaypoint(0.2, 0.0, 0.0, 0.5),  // blend=0.5, but segment is only 0.2
        makeWaypoint(0.2, 1.0, 0.0)
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Arc is created despite large blend factor") {
        REQUIRE(path.numSegments() == 3);
        REQUIRE(path.segment(1).type == PathSegment::Type::Arc);
    }

    SECTION("Arc does not extend beyond segment midpoints") {
        const auto& arc = path.segment(1);
        // Arc start should be no earlier than midpoint of first segment
        // Arc end should be no later than midpoint of last segment
        double seg1_length = 0.2;  // (0,0,0) to (0.2,0,0)
        REQUIRE(arc.start_pos.x() >= seg1_length * 0.5 - kTolerance);
    }
}

TEST_CASE("PathGeometry 3D diagonal path with blend arc", "[path_geometry][blend][3d]") {
    // Real user waypoints that showed discontinuity issue
    // WP3 -> WP4 -> WP5 with blend at WP4
    std::vector<SequenceWaypoint> waypoints = {
        makeWaypoint(0.074, -0.744, 0.403, 0.0),   // WP3: no blend
        makeWaypoint(0.071, -0.734, 0.508, 0.01),  // WP4: blend=0.01
        makeWaypoint(0.208, -0.565, 0.589, 0.0)    // WP5: no blend
    };

    auto path = PathGeometry::fromWaypoints(waypoints);

    SECTION("Path has correct segment structure") {
        // Should have: linear, arc, linear = 3 segments
        REQUIRE(path.numSegments() == 3);
        REQUIRE(path.segment(0).type == PathSegment::Type::Linear);
        REQUIRE(path.segment(1).type == PathSegment::Type::Arc);
        REQUIRE(path.segment(2).type == PathSegment::Type::Linear);
    }

    SECTION("Arc geometry is valid") {
        const auto& arc = path.segment(1);

        // Print arc details for debugging
        INFO("Arc center: (" << arc.center.x() << ", " << arc.center.y() << ", " << arc.center.z() << ")");
        INFO("Arc start: (" << arc.start_pos.x() << ", " << arc.start_pos.y() << ", " << arc.start_pos.z() << ")");
        INFO("Arc end: (" << arc.end_pos.x() << ", " << arc.end_pos.y() << ", " << arc.end_pos.z() << ")");
        INFO("Arc radius: " << arc.radius);
        INFO("Arc angle: " << arc.arc_angle << " rad (" << (arc.arc_angle * 180.0 / kPi) << " deg)");
        INFO("Arc normal: (" << arc.normal.x() << ", " << arc.normal.y() << ", " << arc.normal.z() << ")");
        INFO("Arc length: " << arc.length);

        // Distance from center to start should equal radius
        double start_to_center = (arc.start_pos - arc.center).norm();
        INFO("Distance start->center: " << start_to_center);
        REQUIRE_THAT(start_to_center, WithinAbs(arc.radius, kTolerance));

        // Distance from center to end should equal radius
        double end_to_center = (arc.end_pos - arc.center).norm();
        INFO("Distance end->center: " << end_to_center);
        REQUIRE_THAT(end_to_center, WithinAbs(arc.radius, kTolerance));
    }

    SECTION("Path is continuous at segment boundaries") {
        const auto& seg1 = path.segment(0);
        const auto& seg2 = path.segment(1);
        const auto& seg3 = path.segment(2);

        INFO("Seg1 end: (" << seg1.end_pos.x() << ", " << seg1.end_pos.y() << ", " << seg1.end_pos.z() << ")");
        INFO("Seg2 start: (" << seg2.start_pos.x() << ", " << seg2.start_pos.y() << ", " << seg2.start_pos.z() << ")");
        INFO("Seg2 end: (" << seg2.end_pos.x() << ", " << seg2.end_pos.y() << ", " << seg2.end_pos.z() << ")");
        INFO("Seg3 start: (" << seg3.start_pos.x() << ", " << seg3.start_pos.y() << ", " << seg3.start_pos.z() << ")");

        // End of seg1 should equal start of seg2
        double gap1 = (seg1.end_pos - seg2.start_pos).norm();
        INFO("Gap between seg1 and seg2: " << gap1);
        REQUIRE_THAT(gap1, WithinAbs(0.0, kTolerance));

        // End of seg2 should equal start of seg3
        double gap2 = (seg2.end_pos - seg3.start_pos).norm();
        INFO("Gap between seg2 and seg3: " << gap2);
        REQUIRE_THAT(gap2, WithinAbs(0.0, kTolerance));
    }

    SECTION("Arc evaluation produces points on the arc") {
        const auto& arc = path.segment(1);

        // Sample several points along the arc
        for (int i = 0; i <= 10; ++i) {
            double t = static_cast<double>(i) / 10.0;
            double s = arc.start_distance + t * arc.length;
            auto point = path.evaluate(s);

            // Each point should be at arc.radius distance from center
            double dist = (point.position - arc.center).norm();
            INFO("Arc t=" << t << ": pos=(" << point.position.x() << ", "
                 << point.position.y() << ", " << point.position.z() << "), dist_to_center=" << dist);
            REQUIRE_THAT(dist, WithinAbs(arc.radius, kTolerance));
        }
    }

    SECTION("Rodrigues rotation reaches arc end") {
        const auto& arc = path.segment(1);

        // Manually compute where Rodrigues rotation should end up
        Eigen::Vector3d r = arc.start_pos - arc.center;
        double cos_a = std::cos(arc.arc_angle);
        double sin_a = std::sin(arc.arc_angle);

        Eigen::Vector3d r_rot = r * cos_a
                              + arc.normal.cross(r) * sin_a
                              + arc.normal * (arc.normal.dot(r)) * (1.0 - cos_a);

        Eigen::Vector3d computed_end = arc.center + r_rot;
        Eigen::Vector3d expected_end = arc.end_pos;

        INFO("Computed end from Rodrigues: (" << computed_end.x() << ", " << computed_end.y() << ", " << computed_end.z() << ")");
        INFO("Expected end (arc.end_pos): (" << expected_end.x() << ", " << expected_end.y() << ", " << expected_end.z() << ")");

        double error = (computed_end - expected_end).norm();
        INFO("Error: " << error);
        REQUIRE_THAT(error, WithinAbs(0.0, kTolerance));
    }
}
