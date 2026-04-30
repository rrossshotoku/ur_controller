/// @file path_geometry.cpp
/// @brief Implementation of distance-parameterized geometric path

#include "ur_controller/trajectory/path_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace ur_controller {
namespace trajectory {

// =============================================================================
// Construction from Waypoints
// =============================================================================

PathGeometry PathGeometry::fromWaypoints(
    const std::vector<SequenceWaypoint>& waypoints) {

    PathGeometry path;

    if (waypoints.size() < 2) {
        // Need at least 2 waypoints for a path
        if (waypoints.size() == 1) {
            path.waypoint_distances_.push_back(0.0);
        }
        return path;
    }

    // Pre-compute blend arcs for interior waypoints (indices 1 to N-2)
    // Store arc info indexed by waypoint index
    std::vector<std::optional<PathSegment>> blend_arcs(waypoints.size());

    for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
        PathSegment arc_seg;
        if (computeBlendArc(waypoints[i - 1].position,
                           waypoints[i],
                           waypoints[i + 1].position,
                           arc_seg)) {
            blend_arcs[i] = arc_seg;
        }
    }

    double cumulative_distance = 0.0;
    path.waypoint_distances_.push_back(0.0);  // First waypoint at distance 0

    // Track current position (may be offset from waypoint due to blend arc)
    Eigen::Vector3d current_pos = waypoints[0].position;
    Eigen::Quaterniond current_orient = waypoints[0].orientation;

    // Build segments
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        const auto& wp_next = waypoints[i + 1];

        // Determine where the linear segment ends:
        // - If next waypoint has a blend arc, end at arc start
        // - Otherwise, end at the waypoint itself
        Eigen::Vector3d linear_end;
        Eigen::Quaterniond linear_end_orient;
        bool next_has_arc = blend_arcs[i + 1].has_value();

        if (next_has_arc) {
            // End linear segment at arc start
            linear_end = blend_arcs[i + 1]->start_pos;
            linear_end_orient = blend_arcs[i + 1]->start_orient;
        } else {
            // End at waypoint
            linear_end = wp_next.position;
            linear_end_orient = wp_next.orientation;
        }

        // Create linear segment from current_pos to linear_end
        double linear_length = (linear_end - current_pos).norm();
        if (linear_length > 1e-9) {
            PathSegment lin_seg;
            lin_seg.type = PathSegment::Type::Linear;
            lin_seg.start_pos = current_pos;
            lin_seg.end_pos = linear_end;
            lin_seg.start_orient = current_orient;
            lin_seg.end_orient = linear_end_orient;
            lin_seg.length = linear_length;
            lin_seg.start_distance = cumulative_distance;

            path.segments_.push_back(lin_seg);
            cumulative_distance += linear_length;
        }

        // If next waypoint has a blend arc, add the arc segment
        if (next_has_arc) {
            PathSegment arc_seg = *blend_arcs[i + 1];
            arc_seg.start_distance = cumulative_distance;

            // Interpolate orientation through arc
            // Arc connects incoming segment to outgoing, so orient stays at waypoint orientation
            arc_seg.start_orient = wp_next.orientation;
            arc_seg.end_orient = wp_next.orientation;

            path.segments_.push_back(arc_seg);
            cumulative_distance += arc_seg.length;

            // Update current position to arc end
            current_pos = arc_seg.end_pos;
            current_orient = arc_seg.end_orient;
        } else {
            // Move to waypoint position
            current_pos = linear_end;
            current_orient = linear_end_orient;
        }

        // Record waypoint distance (at the vertex, which is inside the arc if blended)
        // For blend arcs, the waypoint is at the apex of the arc
        path.waypoint_distances_.push_back(cumulative_distance);
    }

    path.total_length_ = cumulative_distance;
    return path;
}

// =============================================================================
// Path Evaluation
// =============================================================================

PathPoint PathGeometry::evaluate(double s) const {
    PathPoint result;

    if (segments_.empty()) {
        return result;
    }

    // Clamp s to valid range
    s = std::clamp(s, 0.0, total_length_);

    // Find the segment containing this distance
    size_t seg_idx = findSegment(s);
    const PathSegment& seg = segments_[seg_idx];

    // Compute local distance within segment
    double local_s = s - seg.start_distance;

    // Evaluate based on segment type
    if (seg.type == PathSegment::Type::Linear) {
        result = evaluateLinear(seg, local_s);
    } else {
        result = evaluateArc(seg, local_s);
    }

    result.segment_index = seg_idx;
    result.segment_progress = (seg.length > 0) ? (local_s / seg.length) : 0.0;

    return result;
}

size_t PathGeometry::findSegment(double s) const {
    if (segments_.empty()) {
        return 0;
    }

    // Linear search (sufficient for typical path sizes < 20 segments)
    // Could use binary search for very long paths
    for (size_t i = 0; i < segments_.size(); ++i) {
        if (s <= segments_[i].endDistance()) {
            return i;
        }
    }

    // s is beyond end - return last segment
    return segments_.size() - 1;
}

PathPoint PathGeometry::evaluateLinear(const PathSegment& seg, double local_s) const {
    PathPoint result;

    if (seg.length < 1e-9) {
        // Zero-length segment
        result.position = seg.start_pos;
        result.orientation = seg.start_orient;
        return result;
    }

    // Parameter t in [0, 1]
    double t = local_s / seg.length;
    t = std::clamp(t, 0.0, 1.0);

    // Linear interpolation for position
    result.position = seg.start_pos + t * (seg.end_pos - seg.start_pos);

    // SLERP for orientation
    result.orientation = seg.start_orient.slerp(t, seg.end_orient);

    return result;
}

PathPoint PathGeometry::evaluateArc(const PathSegment& seg, double local_s) const {
    PathPoint result;

    if (seg.length < 1e-9 || seg.arc_angle < 1e-9) {
        // Degenerate arc
        result.position = seg.start_pos;
        result.orientation = seg.start_orient;
        return result;
    }

    // Parameter t in [0, 1]
    double t = local_s / seg.length;
    t = std::clamp(t, 0.0, 1.0);

    // Angle swept at this point
    double angle = t * seg.arc_angle;

    // Rodrigues' rotation formula to rotate start_pos around center
    // Vector from center to start
    Eigen::Vector3d r = seg.start_pos - seg.center;

    // Rotate r around normal by angle
    // Using Rodrigues: r_rot = r*cos(θ) + (n×r)*sin(θ) + n*(n·r)*(1-cos(θ))
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);

    Eigen::Vector3d r_rot = r * cos_a
                          + seg.normal.cross(r) * sin_a
                          + seg.normal * (seg.normal.dot(r)) * (1.0 - cos_a);

    result.position = seg.center + r_rot;

    // SLERP for orientation
    result.orientation = seg.start_orient.slerp(t, seg.end_orient);

    return result;
}

// =============================================================================
// Blend Arc Computation (for Step 2)
// =============================================================================

bool PathGeometry::computeBlendArc(
    const Eigen::Vector3d& prev,
    const SequenceWaypoint& current,
    const Eigen::Vector3d& next,
    PathSegment& out_segment) {

    double blend_factor = current.blend_factor;

    if (blend_factor < 1e-6) {
        // No blending requested
        return false;
    }

    // Direction vectors
    Eigen::Vector3d dir_in = (current.position - prev).normalized();
    Eigen::Vector3d dir_out = (next - current.position).normalized();

    // Check for parallel or anti-parallel directions
    double dot = dir_in.dot(dir_out);
    if (std::abs(dot) > 0.9999) {
        // Nearly straight or 180° turn - no arc needed/possible
        return false;
    }

    // Angle between directions (turning angle)
    double turning_angle = std::acos(std::clamp(dot, -1.0, 1.0));

    // Arc angle is supplementary to turning angle
    double half_angle = (M_PI - turning_angle) / 2.0;

    // From blend_factor (h = distance from vertex to arc apex):
    // Using geometry of inscribed arc:
    //   d = distance from vertex to center (along bisector)
    //   r = arc radius
    //   h = d - r (apex is on bisector at distance h from vertex)
    //   r = d * sin(half_angle)  [from right triangle]
    //   h = d - d*sin(half_angle) = d*(1 - sin(half_angle))
    //   d = h / (1 - sin(half_angle))
    //   r = h * sin(half_angle) / (1 - sin(half_angle))
    double sin_half = std::sin(half_angle);
    if (std::abs(1.0 - sin_half) < 1e-9) {
        return false;
    }

    double arc_radius = blend_factor * sin_half / (1.0 - sin_half);

    // Blend radius = distance from vertex to tangent points
    // From right triangle: blend_radius = d * cos(half_angle)
    // And d = r / sin(half_angle)
    // So blend_radius = r * cos(half_angle) / sin(half_angle) = r / tan(half_angle)
    double tan_half = std::tan(half_angle);
    if (std::abs(tan_half) < 1e-9) {
        return false;
    }
    double blend_radius = arc_radius / tan_half;

    // Check if blend radius exceeds half of either segment
    double dist_to_prev = (current.position - prev).norm();
    double dist_to_next = (next - current.position).norm();
    double max_blend = std::min(dist_to_prev, dist_to_next) * 0.5;

    if (blend_radius > max_blend) {
        // Clamp and recompute arc_radius to maintain tangency
        // From geometry: tan(half_angle) = arc_radius / blend_radius
        blend_radius = max_blend;
        arc_radius = blend_radius * std::tan(half_angle);
    }

    // Arc start and end points (tangent points)
    Eigen::Vector3d arc_start = current.position - dir_in * blend_radius;
    Eigen::Vector3d arc_end = current.position + dir_out * blend_radius;

    // Arc center: along bisector from vertex
    Eigen::Vector3d bisector = (dir_out - dir_in).normalized();
    double center_dist = arc_radius / std::sin(half_angle);
    Eigen::Vector3d center = current.position + bisector * center_dist;

    // Normal to arc plane
    Eigen::Vector3d normal = dir_in.cross(dir_out);
    if (normal.norm() < 1e-9) {
        return false;
    }
    normal.normalize();

    // Arc angle equals the turning angle (angle between direction vectors)
    // NOT π - turning_angle as was previously computed
    double arc_angle = turning_angle;
    double arc_length = arc_radius * arc_angle;

    // Verify rotation direction: rotating (arc_start - center) by arc_angle
    // around normal should reach (arc_end - center). If not, flip the normal.
    Eigen::Vector3d r_start = arc_start - center;
    Eigen::Vector3d r_end = arc_end - center;

    double cos_a = std::cos(arc_angle);
    double sin_a = std::sin(arc_angle);
    Eigen::Vector3d r_rotated = r_start * cos_a
                              + normal.cross(r_start) * sin_a
                              + normal * (normal.dot(r_start)) * (1.0 - cos_a);

    // Check if rotation went the wrong direction
    double error = (r_rotated - r_end).norm();

    if (error > arc_radius * 0.01) {
        // Rotation didn't reach the expected endpoint - flip normal
        normal = -normal;
    }

    // Populate output segment
    out_segment.type = PathSegment::Type::Arc;
    out_segment.start_pos = arc_start;
    out_segment.end_pos = arc_end;
    out_segment.center = center;
    out_segment.normal = normal;
    out_segment.radius = arc_radius;
    out_segment.arc_angle = arc_angle;
    out_segment.length = arc_length;
    out_segment.start_orient = current.orientation;
    out_segment.end_orient = current.orientation;

    return true;
}

}  // namespace trajectory
}  // namespace ur_controller
