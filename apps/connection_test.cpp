/// @file connection_test.cpp
/// @brief Simple test application to verify URSim connectivity via RTDE
///
/// This application connects to a UR robot (or URSim) and reads basic state
/// information to verify the communication link is working correctly.

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/dashboard_client.h>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {

constexpr int kDefaultRtdePort = 30004;
constexpr int kDefaultDashboardPort = 29999;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/// @brief Print joint positions in a formatted table
void printJointPositions(const std::vector<double>& q) {
    std::cout << "\nJoint Positions:\n";
    std::cout << std::string(50, '-') << "\n";
    std::cout << std::setw(10) << "Joint"
              << std::setw(15) << "Radians"
              << std::setw(15) << "Degrees" << "\n";
    std::cout << std::string(50, '-') << "\n";

    const std::array<const char*, 6> joint_names = {
        "Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"
    };

    for (size_t i = 0; i < q.size() && i < 6; ++i) {
        std::cout << std::setw(10) << joint_names[i]
                  << std::setw(15) << std::fixed << std::setprecision(4) << q[i]
                  << std::setw(15) << std::fixed << std::setprecision(2)
                  << q[i] * kRadToDeg << "\n";
    }
    std::cout << std::string(50, '-') << "\n";
}

/// @brief Print TCP pose in a formatted table
void printTcpPose(const std::vector<double>& pose) {
    std::cout << "\nTCP Pose (Base Frame):\n";
    std::cout << std::string(40, '-') << "\n";

    if (pose.size() >= 6) {
        std::cout << "Position:\n";
        std::cout << "  X: " << std::setw(10) << std::fixed << std::setprecision(4)
                  << pose[0] << " m\n";
        std::cout << "  Y: " << std::setw(10) << std::fixed << std::setprecision(4)
                  << pose[1] << " m\n";
        std::cout << "  Z: " << std::setw(10) << std::fixed << std::setprecision(4)
                  << pose[2] << " m\n";

        std::cout << "Orientation (axis-angle):\n";
        std::cout << "  Rx: " << std::setw(10) << std::fixed << std::setprecision(4)
                  << pose[3] << " rad\n";
        std::cout << "  Ry: " << std::setw(10) << std::fixed << std::setprecision(4)
                  << pose[4] << " rad\n";
        std::cout << "  Rz: " << std::setw(10) << std::fixed << std::setprecision(4)
                  << pose[5] << " rad\n";
    }
    std::cout << std::string(40, '-') << "\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    // Parse command line arguments
    CLI::App app{"UR Robot Connection Test"};

    std::string robot_ip = "ursim";  // Default for Docker network
    bool continuous = false;
    int rate_hz = 10;

    app.add_option("-i,--ip", robot_ip, "Robot IP address or hostname")
        ->default_val("ursim");
    app.add_flag("-c,--continuous", continuous,
        "Continuously print robot state");
    app.add_option("-r,--rate", rate_hz, "Update rate in Hz (with --continuous)")
        ->default_val(10)
        ->check(CLI::Range(1, 500));

    CLI11_PARSE(app, argc, argv);

    spdlog::info("UR Connection Test");
    spdlog::info("==================");
    spdlog::info("Target: {}:{}", robot_ip, kDefaultRtdePort);

    // Try dashboard connection first
    spdlog::info("Connecting to Dashboard server...");
    try {
        ur_rtde::DashboardClient dashboard(robot_ip, kDefaultDashboardPort);
        dashboard.connect();

        spdlog::info("Dashboard connected successfully");

        // Get robot information
        std::string robot_mode = dashboard.robotmode();
        std::string safety_status = dashboard.safetystatus();
        std::string program_state = dashboard.programState();

        std::cout << "\nRobot Information:\n";
        std::cout << std::string(40, '-') << "\n";
        std::cout << "  Robot Mode:    " << robot_mode << "\n";
        std::cout << "  Safety Status: " << safety_status << "\n";
        std::cout << "  Program State: " << program_state << "\n";
        std::cout << std::string(40, '-') << "\n";

        dashboard.disconnect();
    } catch (const std::exception& e) {
        spdlog::warn("Dashboard connection failed: {}", e.what());
        spdlog::info("Continuing with RTDE connection...");
    }

    // Connect via RTDE
    spdlog::info("Connecting to RTDE interface...");

    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive;

    try {
        rtde_receive = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip);
        spdlog::info("RTDE connected successfully");
    } catch (const std::exception& e) {
        spdlog::error("Failed to connect to RTDE: {}", e.what());
        spdlog::error("Make sure URSim is running and the robot is in Remote Control mode");
        return 1;
    }

    // Read and display robot state
    auto print_state = [&rtde_receive]() {
        auto q = rtde_receive->getActualQ();
        auto tcp_pose = rtde_receive->getActualTCPPose();
        auto tcp_speed = rtde_receive->getActualTCPSpeed();
        double speed_scaling = rtde_receive->getSpeedScaling();

        printJointPositions(q);
        printTcpPose(tcp_pose);

        std::cout << "\nTCP Speed: [";
        for (size_t i = 0; i < tcp_speed.size(); ++i) {
            std::cout << std::fixed << std::setprecision(4) << tcp_speed[i];
            if (i < tcp_speed.size() - 1) std::cout << ", ";
        }
        std::cout << "]\n";

        std::cout << "Speed Scaling: " << std::fixed << std::setprecision(1)
                  << speed_scaling * 100.0 << "%\n";
    };

    if (continuous) {
        spdlog::info("Continuous mode - press Ctrl+C to exit");
        auto period = std::chrono::milliseconds(1000 / rate_hz);

        while (true) {
            // Clear screen (ANSI escape code)
            std::cout << "\033[2J\033[H";
            std::cout << "UR Robot State (updating at " << rate_hz << " Hz)\n";
            std::cout << "Press Ctrl+C to exit\n";

            print_state();

            std::this_thread::sleep_for(period);
        }
    } else {
        print_state();
    }

    spdlog::info("Connection test completed successfully");
    return 0;
}
