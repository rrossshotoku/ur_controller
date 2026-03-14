// Analyze IK issues by comparing FK output with IK solutions
#include "ur_controller/kinematics/ur_kinematics.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace ur_controller::kinematics;

constexpr double kPi = 3.14159265358979323846;

void printPose(const std::string& name, const Eigen::Isometry3d& pose) {
    std::cout << name << ":\n";
    std::cout << "  Position: [" << pose.translation().transpose() << "]\n";
    Eigen::Quaterniond q(pose.rotation());
    std::cout << "  Quaternion: [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]\n";
}

int main() {
    URKinematics kin(URModel::UR5e);
    std::cout << std::fixed << std::setprecision(6);

    // Test configuration
    JointVector q_input;
    q_input << 0.3, -0.8, 1.2, -0.5, 0.7, -0.2;

    std::cout << "=== IK Analysis ===\n\n";
    std::cout << "Input joints: " << q_input.transpose() << "\n\n";

    // Compute FK
    auto target_pose = kin.forward(q_input);
    printPose("Target pose from FK", target_pose);

    // Get IK solutions
    auto solutions = kin.inverse(target_pose);
    std::cout << "\nIK found " << solutions.size() << " solutions:\n\n";

    for (size_t i = 0; i < solutions.size(); ++i) {
        const auto& sol = solutions[i];
        auto recovered_pose = kin.forward(sol);

        double pos_error = (recovered_pose.translation() - target_pose.translation()).norm();

        // Orientation error using angle between rotations
        Eigen::AngleAxisd rot_diff(target_pose.rotation().transpose() * recovered_pose.rotation());
        double rot_error = std::abs(rot_diff.angle());

        std::cout << "Solution " << i << ": " << sol.transpose() << "\n";
        std::cout << "  Position error: " << pos_error << " m\n";
        std::cout << "  Rotation error: " << rot_error << " rad\n";

        if (pos_error < 0.001 && rot_error < 0.001) {
            std::cout << "  *** GOOD SOLUTION ***\n";
        }
        std::cout << "\n";
    }

    // Check if original input is among solutions
    std::cout << "Checking if original input matches any solution:\n";
    bool found = false;
    for (size_t i = 0; i < solutions.size(); ++i) {
        JointVector diff = solutions[i] - q_input;
        for (int j = 0; j < 6; ++j) {
            diff[j] = std::fmod(diff[j] + kPi, 2*kPi) - kPi;  // Wrap to [-pi, pi]
        }
        if (diff.norm() < 0.01) {
            std::cout << "  Solution " << i << " matches input!\n";
            found = true;
        }
    }
    if (!found) {
        std::cout << "  No solution matches the original input joints.\n";
    }

    return 0;
}
