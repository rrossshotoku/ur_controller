#pragma once

/// @file robot_state_manager.hpp
/// @brief Thread-safe robot state container and RTDE interface wrapper

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/dashboard_client.h>

#include <array>
#include <mutex>
#include <memory>
#include <string>
#include <chrono>

namespace ur_controller {
namespace webui {

/// @brief Thread-safe robot state container
struct RobotState {
    std::array<double, 6> joint_positions{};
    std::array<double, 6> joint_velocities{};
    std::array<double, 6> tcp_pose{};
    std::array<double, 6> tcp_speed{};
    double speed_scaling = 0.0;
    std::string robot_mode = "Unknown";
    std::string safety_status = "Unknown";
    bool connected = false;
    bool control_available = false;
    int64_t timestamp_ms = 0;
};

/// @brief Manages robot connection and state
class RobotStateManager {
public:
    explicit RobotStateManager(const std::string& robot_ip);
    ~RobotStateManager();

    // Non-copyable
    RobotStateManager(const RobotStateManager&) = delete;
    RobotStateManager& operator=(const RobotStateManager&) = delete;

    /// @brief Connect to robot
    /// @return true if connection successful
    bool connect();

    /// @brief Disconnect from robot
    void disconnect();

    /// @brief Check if connected
    [[nodiscard]] bool isConnected() const;

    /// @brief Update state from robot (call periodically)
    void updateState();

    /// @brief Get current state (thread-safe copy)
    [[nodiscard]] RobotState getState() const;

    /// @brief Send servoJ command
    /// @param target_q Target joint positions in radians
    /// @param lookahead_time Lookahead time (0.03-0.2)
    /// @param gain Proportional gain (100-2000)
    /// @return true if command sent successfully
    bool servoJ(const std::array<double, 6>& target_q,
                double lookahead_time = 0.1,
                double gain = 300.0);

    /// @brief Stop robot motion
    /// @param deceleration Deceleration in rad/s^2
    /// @return true if command sent successfully
    bool stopJ(double deceleration = 2.0);

    /// @brief Check if control is available
    [[nodiscard]] bool hasControl() const;

    /// @brief Try to enable control interface (retry connection)
    /// @return true if control is now available
    bool tryEnableControl();

    /// @brief Get robot IP address
    [[nodiscard]] const std::string& robotIp() const { return robot_ip_; }

private:
    std::string robot_ip_;
    mutable std::mutex state_mutex_;
    mutable std::mutex control_mutex_;
    RobotState current_state_;

    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
    std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
    std::unique_ptr<ur_rtde::DashboardClient> dashboard_;

    int64_t getCurrentTimeMs() const;
};

}  // namespace webui
}  // namespace ur_controller
