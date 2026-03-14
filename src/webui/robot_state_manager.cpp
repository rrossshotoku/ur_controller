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

}  // namespace webui
}  // namespace ur_controller
