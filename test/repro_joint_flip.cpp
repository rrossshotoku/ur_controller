/// @file repro_joint_flip.cpp
/// @brief Reproduces the joint-2 flip the user reported.
///
/// Uses the exact entry/WP1/WP2 joints from the user's log:
///   entry: [-1.942, -2.510, -1.298, -0.991, 1.592, -0.373] rad
///   WP1  : [-111.29, -143.79, -74.40, -56.77, 91.24, -21.34] deg
///   WP2  : [-91.17, -133.14, -94.51, -46.57, 92.86, -1.28]   deg
///
/// Plans the trajectory and prints sample joints, looking for any joint that
/// jumps by > 10 deg between consecutive samples or whose total range exceeds
/// the expected delta.

#include "ur_controller/kinematics/ur_kinematics.hpp"
#include "ur_controller/trajectory/planner.hpp"
#include "ur_controller/trajectory/types.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>

using namespace ur_controller;

namespace {
constexpr double kDeg = M_PI / 180.0;
constexpr double kRad = 180.0 / M_PI;

kinematics::JointVector deg6(double a, double b, double c, double d, double e, double f) {
    kinematics::JointVector q;
    q << a*kDeg, b*kDeg, c*kDeg, d*kDeg, e*kDeg, f*kDeg;
    return q;
}
}  // namespace

int main() {
    kinematics::URKinematics kin(kinematics::URModel::UR5e);

    kinematics::JointVector entry_joints;
    entry_joints << -1.942, -2.510, -1.298, -0.991, 1.592, -0.373;

    auto wp1_joints = deg6(-111.29, -143.79, -74.40, -56.77, 91.24, -21.34);
    auto wp2_joints = deg6(-91.17,  -133.14, -94.51, -46.57, 92.86, -1.28);

    auto wp1_pose = kin.forward(wp1_joints);
    auto wp2_pose = kin.forward(wp2_joints);

    std::printf("Entry joints (deg):  ");
    for (int i = 0; i < 6; ++i) std::printf("%8.2f ", entry_joints[i]*kRad);
    std::printf("\n");

    std::printf("WP1   joints (deg):  ");
    for (int i = 0; i < 6; ++i) std::printf("%8.2f ", wp1_joints[i]*kRad);
    std::printf("\n");

    std::printf("WP2   joints (deg):  ");
    for (int i = 0; i < 6; ++i) std::printf("%8.2f ", wp2_joints[i]*kRad);
    std::printf("\n");

    auto fk_diff_pos = (wp1_pose.translation() - kin.forward(entry_joints).translation()).norm();
    std::printf("FK(entry) vs WP1 pose distance: %.6f m\n", fk_diff_pos);

    // Build waypoints with both pose AND saved joints
    trajectory::Waypoint wp1;
    wp1.position = wp1_pose.translation();
    wp1.orientation = Eigen::Quaterniond(wp1_pose.rotation()).normalized();
    wp1.segment_time = 0.0;
    wp1.joints = wp1_joints;

    trajectory::Waypoint wp2;
    wp2.position = wp2_pose.translation();
    wp2.orientation = Eigen::Quaterniond(wp2_pose.rotation()).normalized();
    wp2.segment_time = 5.0;
    wp2.joints = wp2_joints;

    std::vector<trajectory::Waypoint> waypoints = {wp1, wp2};

    trajectory::TrajectoryPlanner planner(kin);

    // Plan with entry_joints as start joints (matches plan2 path)
    auto traj = planner.plan(waypoints, entry_joints);

    std::printf("\nPlan valid=%d  samples=%zu  duration=%.3fs\n",
                traj.valid ? 1 : 0, traj.samples.size(), traj.total_duration);

    if (!traj.valid || traj.samples.empty()) {
        for (const auto& m : traj.messages) std::printf("  msg: %s\n", m.message.c_str());
        return 1;
    }

    // First/last sample
    const auto& s0 = traj.samples.front();
    const auto& sN = traj.samples.back();
    std::printf("\nSample 0     (deg): ");
    for (int i = 0; i < 6; ++i) std::printf("%8.2f ", s0.joints[i]*kRad);
    std::printf("\n");
    std::printf("Sample %zu (deg): ", traj.samples.size()-1);
    for (int i = 0; i < 6; ++i) std::printf("%8.2f ", sN.joints[i]*kRad);
    std::printf("\n");

    // Per-joint min/max + max single-step jump
    double jmin[6], jmax[6], jmax_step[6];
    int    jmax_step_idx[6];
    for (int j = 0; j < 6; ++j) {
        jmin[j] = jmax[j] = traj.samples[0].joints[j];
        jmax_step[j] = 0.0;
        jmax_step_idx[j] = 0;
    }
    for (size_t i = 0; i < traj.samples.size(); ++i) {
        const auto& q = traj.samples[i].joints;
        for (int j = 0; j < 6; ++j) {
            jmin[j] = std::min(jmin[j], q[j]);
            jmax[j] = std::max(jmax[j], q[j]);
            if (i > 0) {
                double step = std::abs(q[j] - traj.samples[i-1].joints[j]);
                if (step > jmax_step[j]) {
                    jmax_step[j] = step;
                    jmax_step_idx[j] = static_cast<int>(i);
                }
            }
        }
    }

    std::printf("\nPer-joint statistics (degrees):\n");
    std::printf("%4s  %10s %10s %10s %12s %10s\n",
                "J", "min", "max", "range", "max_step", "@sample");
    for (int j = 0; j < 6; ++j) {
        std::printf("J%d   %10.2f %10.2f %10.2f %12.4f %10d\n",
                    j+1, jmin[j]*kRad, jmax[j]*kRad, (jmax[j]-jmin[j])*kRad,
                    jmax_step[j]*kRad, jmax_step_idx[j]);
    }

    // Flag any joint whose total range exceeds expected delta + 30°
    std::printf("\nJoint range check (vs expected wp1->wp2 delta + 30deg slack):\n");
    bool any_bad = false;
    for (int j = 0; j < 6; ++j) {
        double expected = std::abs(wp2_joints[j] - wp1_joints[j]);
        double actual_range = jmax[j] - jmin[j];
        bool bad = (actual_range - expected) > (30.0 * kDeg);
        if (bad) any_bad = true;
        std::printf("  J%d expected=%.2fdeg  actual_range=%.2fdeg  %s\n",
                    j+1, expected*kRad, actual_range*kRad, bad ? "*** BAD ***" : "ok");
    }
    return any_bad ? 2 : 0;
}
