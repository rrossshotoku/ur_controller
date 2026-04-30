/// @file path_geometry.hpp
/// @brief Distance-parameterized geometric path through waypoints

#pragma once

#include "ur_controller/trajectory/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace ur_controller {
namespace trajectory {

/// @brief A point on the path
struct PathPoint {
    Eigen::Vector3d position{0, 0, 0};
    Eigen::Quaterniond orientation{1, 0, 0, 0};
    size_t segment_index{0};       ///< Which segment this point is in
    double segment_progress{0.0};  ///< 0-1 progress within segment

    /// @brief Convert to Isometry3d pose
    [[nodiscard]] Eigen::Isometry3d toPose() const {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = position;
        pose.linear() = orientation.toRotationMatrix();
        return pose;
    }
};

/// @brief A segment of the path (linear or arc)
struct PathSegment {
    enum class Type { Linear, Arc };
    Type type{Type::Linear};

    // Common fields
    Eigen::Vector3d start_pos{0, 0, 0};
    Eigen::Vector3d end_pos{0, 0, 0};
    Eigen::Quaterniond start_orient{1, 0, 0, 0};
    Eigen::Quaterniond end_orient{1, 0, 0, 0};

    // Arc-specific fields (only valid when type == Arc)
    Eigen::Vector3d center{0, 0, 0};       ///< Arc center point
    Eigen::Vector3d normal{0, 0, 1};       ///< Normal to arc plane
    double radius{0.0};                     ///< Arc radius
    double arc_angle{0.0};                  ///< Total arc angle (radians)

    double length{0.0};            ///< Path length of this segment
    double start_distance{0.0};    ///< Cumulative distance at segment start

    /// @brief Get end distance of this segment
    [[nodiscard]] double endDistance() const { return start_distance + length; }
};

/// @brief Distance-parameterized geometric path
///
/// Represents the geometric path through waypoints with optional blend arcs.
/// The path can be queried by distance parameter s to get the pose at that
/// point along the path. This separates geometry from velocity profile.
///
/// Usage:
/// @code
///   auto path = PathGeometry::fromWaypoints(waypoints);
///   double total = path.totalLength();
///   PathPoint point = path.evaluate(0.5 * total);  // Midpoint
/// @endcode
class PathGeometry {
public:
    /// @brief Default constructor (empty path)
    PathGeometry() = default;

    /// @brief Build path from sequence waypoints
    /// @param waypoints Waypoints with positions, orientations, and blend factors
    /// @return PathGeometry object
    [[nodiscard]] static PathGeometry fromWaypoints(
        const std::vector<SequenceWaypoint>& waypoints);

    /// @brief Evaluate path at distance s from start
    /// @param s Distance along path in meters (0 to totalLength)
    /// @return PathPoint with position, orientation, and segment info
    [[nodiscard]] PathPoint evaluate(double s) const;

    /// @brief Total path length in meters
    [[nodiscard]] double totalLength() const { return total_length_; }

    /// @brief Number of segments in the path
    [[nodiscard]] size_t numSegments() const { return segments_.size(); }

    /// @brief Check if path is empty
    [[nodiscard]] bool empty() const { return segments_.empty(); }

    /// @brief Get segment by index
    /// @param i Segment index (0 to numSegments-1)
    /// @return Reference to segment
    [[nodiscard]] const PathSegment& segment(size_t i) const { return segments_.at(i); }

    /// @brief Get all segments
    [[nodiscard]] const std::vector<PathSegment>& segments() const { return segments_; }

    /// @brief Get cumulative distances at each waypoint
    /// @return Vector of distances (for UI waypoint markers)
    [[nodiscard]] const std::vector<double>& waypointDistances() const {
        return waypoint_distances_;
    }

    /// @brief Get number of waypoints
    [[nodiscard]] size_t numWaypoints() const { return waypoint_distances_.size(); }

private:
    std::vector<PathSegment> segments_;
    std::vector<double> waypoint_distances_;  ///< Distance at each waypoint
    double total_length_{0.0};

    /// @brief Find which segment contains distance s
    /// @param s Distance along path
    /// @return Segment index
    [[nodiscard]] size_t findSegment(double s) const;

    /// @brief Evaluate a linear segment
    /// @param seg The segment
    /// @param local_s Distance within segment (0 to seg.length)
    /// @return PathPoint
    [[nodiscard]] PathPoint evaluateLinear(const PathSegment& seg, double local_s) const;

    /// @brief Evaluate an arc segment
    /// @param seg The segment
    /// @param local_s Distance within segment (0 to seg.length)
    /// @return PathPoint
    [[nodiscard]] PathPoint evaluateArc(const PathSegment& seg, double local_s) const;

    /// @brief Compute blend arc geometry between three waypoints
    /// @param prev Previous waypoint position
    /// @param current Current waypoint (where blend occurs)
    /// @param next Next waypoint position
    /// @param blend_factor Distance from vertex to arc apex
    /// @param out_segment Output: arc segment to populate
    /// @return True if arc was computed, false if blend_factor is 0 or invalid
    [[nodiscard]] static bool computeBlendArc(
        const Eigen::Vector3d& prev,
        const SequenceWaypoint& current,
        const Eigen::Vector3d& next,
        PathSegment& out_segment);
};

}  // namespace trajectory
}  // namespace ur_controller
