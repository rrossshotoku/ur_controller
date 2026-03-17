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

    /// @brief Send speedL command for TCP velocity control
    /// @param tcp_velocity TCP velocity [vx, vy, vz, wx, wy, wz] in m/s and rad/s
    /// @param acceleration Tool acceleration in m/s^2
    /// @param time Time to apply velocity (0 = until new command, >0 = duration)
    /// @return true if command sent successfully
    bool speedL(const std::array<double, 6>& tcp_velocity,
                double acceleration = 0.5,
                double time = 0.0);

    /// @brief Send speedJ command for joint velocity control
    /// @param joint_velocity Joint velocities in rad/s
    /// @param acceleration Joint acceleration in rad/s^2
    /// @param time Time to apply velocity (0 = until new command, >0 = duration)
    /// @return true if command sent successfully
    bool speedJ(const std::array<double, 6>& joint_velocity,
                double acceleration = 1.0,
                double time = 0.0);

    /// @brief Stop linear motion (exits speedL mode)
    /// @param deceleration Tool deceleration in m/s^2
    /// @return true if command sent successfully
    bool stopL(double deceleration = 0.5);

    /// @brief Stop and release control script (allows switching between control modes)
    /// @return true if command sent successfully
    bool speedStop();

    /// @brief Stop servo mode (exits servoJ mode for switching to speedL)
    /// @return true if command sent successfully
    bool servoStop();

    /// @brief Move to target joint positions (asynchronous)
    /// @param target_q Target joint positions in radians
    /// @param speed Joint speed in rad/s
    /// @param acceleration Joint acceleration in rad/s^2
    /// @return true if command sent successfully
    bool moveJ(const std::array<double, 6>& target_q,
               double speed = 1.0,
               double acceleration = 1.4);

    /// @brief Move linearly to target TCP pose (asynchronous)
    /// @param target_pose Target pose [x, y, z, rx, ry, rz] in meters and axis-angle
    /// @param speed Linear speed in m/s
    /// @param acceleration Linear acceleration in m/s^2
    /// @return true if command sent successfully
    bool moveL(const std::array<double, 6>& target_pose,
               double speed = 0.25,
               double acceleration = 1.2);

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
