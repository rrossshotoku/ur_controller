/// @file robot_state_manager.cpp
/// @brief Implementation of thread-safe robot state manager

#include "ur_controller/webui/robot_state_manager.hpp"

#include <spdlog/spdlog.h>

namespace ur_controller {
namespace webui {

RobotStateManager::RobotStateManager(const std::string& robot_ip)
    : robot_ip_(robot_ip) {}

RobotStateManager::~RobotStateManager() {
    disconnect();
}

bool RobotStateManager::connect() {
    std::lock_guard<std::mutex> lock(control_mutex_);

    spdlog::info("Connecting to robot at {}", robot_ip_);

    try {
        // Connect to dashboard
        dashboard_ = std::make_unique<ur_rtde::DashboardClient>(robot_ip_);
        dashboard_->connect();

        // Connect to RTDE receive
        rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_);

        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            current_state_.connected = true;
        }

        // Try to connect RTDE control (may fail if remote control not enabled)
        try {
            rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
            {
                std::lock_guard<std::mutex> state_lock(state_mutex_);
                current_state_.control_available = true;
            }
            spdlog::info("Connected to robot with control interface");
        } catch (const std::exception& e) {
            rtde_control_.reset();
            {
                std::lock_guard<std::mutex> state_lock(state_mutex_);
                current_state_.control_available = false;
            }
            spdlog::warn("Control interface not available: {}", e.what());
        }

        return true;

    } catch (const std::exception& e) {
        spdlog::error("Connection failed: {}", e.what());
        disconnect();
        return false;
    }
}

void RobotStateManager::disconnect() {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (rtde_control_) {
        try {
            rtde_control_->stopScript();
        } catch (...) {}
        rtde_control_.reset();
    }

    if (dashboard_) {
        try {
            dashboard_->disconnect();
        } catch (...) {}
        dashboard_.reset();
    }

    rtde_receive_.reset();

    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        current_state_.connected = false;
        current_state_.control_available = false;
    }

    spdlog::info("Disconnected from robot");
}

bool RobotStateManager::isConnected() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_.connected;
}

void RobotStateManager::updateState() {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (!rtde_receive_) {
        return;
    }

    try {
        // Get joint positions
        auto q = rtde_receive_->getActualQ();
        auto qd = rtde_receive_->getActualQd();
        auto tcp = rtde_receive_->getActualTCPPose();
        auto tcp_speed = rtde_receive_->getActualTCPSpeed();
        auto speed_scaling = rtde_receive_->getSpeedScaling();

        // Get dashboard info
        std::string robot_mode = "Unknown";
        std::string safety_status = "Unknown";
        if (dashboard_ && dashboard_->isConnected()) {
            try {
                robot_mode = dashboard_->robotmode();
                safety_status = dashboard_->safetystatus();
            } catch (...) {}
        }

        // Update state atomically
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            for (size_t i = 0; i < 6 && i < q.size(); ++i) {
                current_state_.joint_positions[i] = q[i];
            }
            for (size_t i = 0; i < 6 && i < qd.size(); ++i) {
                current_state_.joint_velocities[i] = qd[i];
            }
            for (size_t i = 0; i < 6 && i < tcp.size(); ++i) {
                current_state_.tcp_pose[i] = tcp[i];
            }
            for (size_t i = 0; i < 6 && i < tcp_speed.size(); ++i) {
                current_state_.tcp_speed[i] = tcp_speed[i];
            }
            current_state_.speed_scaling = speed_scaling;
            current_state_.robot_mode = robot_mode;
            current_state_.safety_status = safety_status;
            current_state_.timestamp_ms = getCurrentTimeMs();
        }

    } catch (const std::exception& e) {
        spdlog::warn("Failed to update robot state: {}", e.what());
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        current_state_.connected = false;
    }
}

RobotState RobotStateManager::getState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

bool RobotStateManager::servoJ(const std::array<double, 6>& target_q,
                                double lookahead_time,
                                double gain) {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (!rtde_control_) {
        return false;
    }

    try {
        // Switch to trajectory mode if needed
        if (control_mode_ != ControlMode::Trajectory) {
            control_mode_ = ControlMode::Trajectory;
        }

        std::vector<double> q(target_q.begin(), target_q.end());
        // servoJ(q, velocity, acceleration, time, lookahead_time, gain)
        rtde_control_->servoJ(q, 0, 0, 0.008, lookahead_time, gain);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("servoJ failed: {}", e.what());
        return false;
    }
}

bool RobotStateManager::stopJ(double deceleration) {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (!rtde_control_) {
        return false;
    }

    try {
        rtde_control_->stopJ(deceleration);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("stopJ failed: {}", e.what());
        return false;
    }
}

bool RobotStateManager::speedL(const std::array<double, 6>& tcp_velocity,
                                double acceleration,
                                double time) {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (!rtde_control_) {
        return false;
    }

    // Don't interrupt trajectory - trajectory takes priority
    // User must stop trajectory before jogging
    if (control_mode_ == ControlMode::Trajectory) {
        return false;
    }

    try {
        // Switch to jogging mode if needed
        if (control_mode_ != ControlMode::Jogging) {
            control_mode_ = ControlMode::Jogging;
        }

        std::vector<double> vel(tcp_velocity.begin(), tcp_velocity.end());
        // time parameter: how long to apply velocity (0 = until new command)
        // Using a small positive time ensures smooth motion when streaming
        rtde_control_->speedL(vel, acceleration, time);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("speedL failed: {}", e.what());
        return false;
    }
}

bool RobotStateManager::speedJ(const std::array<double, 6>& joint_velocity,
                                double acceleration,
                                double time) {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (!rtde_control_) {
        return false;
    }

    // Don't interrupt trajectory - trajectory takes priority
    // User must stop trajectory before jogging
    if (control_mode_ == ControlMode::Trajectory) {
        return false;
    }

    try {
        // Switch to jogging mode if needed
        if (control_mode_ != ControlMode::Jogging) {
            control_mode_ = ControlMode::Jogging;
        }

        std::vector<double> vel(joint_velocity.begin(), joint_velocity.end());
        rtde_control_->speedJ(vel, acceleration, time);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("speedJ failed: {}", e.what());
        return false;
    }
}

bool RobotStateManager::stopL(double deceleration) {
    std::lock_guard<std::mutex> lock(control_mutex_);

    // Don't send stopL during trajectory - would conflict with servoJ
    if (control_mode_ == ControlMode::Trajectory) {
        return false;
    }

    if (!rtde_control_) {
        return false;
    }

    try {
        rtde_control_->stopL(deceleration);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("stopL failed: {}", e.what());
        return false;
    }
}

bool RobotStateManager::speedStop() {
    std::lock_guard<std::mutex> lock(control_mutex_);

    // Don't send speedStop during trajectory - would conflict with servoJ
    if (control_mode_ == ControlMode::Trajectory) {
        return false;
    }

    if (!rtde_control_) {
        control_mode_ = ControlMode::Idle;
        return false;
    }

    try {
        // Stop linear motion - use speedStop which is designed for speedL
        rtde_control_->speedStop();
        control_mode_ = ControlMode::Idle;
        return true;
    } catch (const std::exception& e) {
        spdlog::error("speedStop failed: {}", e.what());
        control_mode_ = ControlMode::Idle;
        return false;
    }
}

bool RobotStateManager::servoStop() {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (!rtde_control_) {
        control_mode_ = ControlMode::Idle;
        return false;
    }

    try {
        // Stop servo mode - use servoStop which is designed for servoJ
        rtde_control_->servoStop();
        control_mode_ = ControlMode::Idle;
        return true;
    } catch (const std::exception& e) {
        spdlog::error("servoStop failed: {}", e.what());
        control_mode_ = ControlMode::Idle;
        return false;
    }
}

bool RobotStateManager::moveJ(const std::array<double, 6>& target_q,
                               double speed,
                               double acceleration) {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (!rtde_control_) {
        return false;
    }

    try {
        // Convert array to vector for ur_rtde
        std::vector<double> q(target_q.begin(), target_q.end());
        // async=true so it returns immediately (can be stopped with stopJ)
        rtde_control_->moveJ(q, speed, acceleration, true);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("moveJ failed: {}", e.what());
        return false;
    }
}

bool RobotStateManager::moveL(const std::array<double, 6>& target_pose,
                               double speed,
                               double acceleration) {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (!rtde_control_) {
        return false;
    }

    try {
        // Convert array to vector for ur_rtde
        std::vector<double> pose(target_pose.begin(), target_pose.end());
        // async=true so it returns immediately (can be stopped with stopL)
        rtde_control_->moveL(pose, speed, acceleration, true);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("moveL failed: {}", e.what());
        return false;
    }
}

bool RobotStateManager::hasControl() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_.control_available;
}

bool RobotStateManager::tryEnableControl() {
    std::lock_guard<std::mutex> lock(control_mutex_);

    // Already have control
    if (rtde_control_) {
        return true;
    }

    // Try to connect control interface
    try {
        rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            current_state_.control_available = true;
        }
        spdlog::info("Control interface now available");
        return true;
    } catch (const std::exception& e) {
        rtde_control_.reset();
        spdlog::warn("Control interface still not available: {}", e.what());
        return false;
    }
}

int64_t RobotStateManager::getCurrentTimeMs() const {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

bool RobotStateManager::acquireControl(ControlMode mode) {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (!rtde_control_) {
        return false;
    }

    // Already in requested mode
    if (control_mode_ == mode) {
        return true;
    }

    // Stop current mode before switching
    try {
        if (control_mode_ == ControlMode::Jogging) {
            spdlog::info("Stopping jog mode to switch to {}",
                mode == ControlMode::Trajectory ? "trajectory" : "idle");
            rtde_control_->speedStop();
        } else if (control_mode_ == ControlMode::Trajectory) {
            spdlog::info("Stopping trajectory mode to switch to {}",
                mode == ControlMode::Jogging ? "jogging" : "idle");
            rtde_control_->servoStop();
        }

        control_mode_ = mode;
        return true;

    } catch (const std::exception& e) {
        spdlog::error("Failed to switch control mode: {}", e.what());
        return false;
    }
}

bool RobotStateManager::releaseControl() {
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (!rtde_control_) {
        control_mode_ = ControlMode::Idle;
        return true;
    }

    try {
        if (control_mode_ == ControlMode::Jogging) {
            rtde_control_->speedStop();
        } else if (control_mode_ == ControlMode::Trajectory) {
            rtde_control_->servoStop();
        }

        control_mode_ = ControlMode::Idle;
        return true;

    } catch (const std::exception& e) {
        spdlog::error("Failed to release control: {}", e.what());
        control_mode_ = ControlMode::Idle;  // Reset anyway
        return false;
    }
}

ControlMode RobotStateManager::getControlMode() const {
    std::lock_guard<std::mutex> lock(control_mutex_);
    return control_mode_;
}

}  // namespace webui
}  // namespace ur_controller
