#pragma once

/// @file web_server.hpp
/// @brief HTTP/WebSocket server for robot control

#include "robot_state_manager.hpp"
#include "websocket_broadcaster.hpp"
#include "ur_controller/kinematics/ur_kinematics.hpp"
#include "ur_controller/trajectory/planner.hpp"
#include "ur_controller/trajectory/axis_blend_planner.hpp"
#include "ur_controller/trajectory/executor.hpp"
#include "ur_controller/trajectory/path_geometry.hpp"
#include "ur_controller/trajectory/velocity_first/planner.hpp"

#include <crow.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>
#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <optional>
#include <deque>
#include <mutex>

namespace ur_controller {
namespace webui {

/// @brief Configuration for the web server
struct WebServerConfig {
    std::string robot_ip = "ursim";
    uint16_t http_port = 8080;
    std::string static_path = "./webui";
    double state_update_rate_hz = 50.0;
};

/// @brief A log entry for the UI
struct LogEntry {
    std::string timestamp;
    std::string level;
    std::string message;
};

/// @brief Thread-safe log buffer for UI display
class LogBuffer {
public:
    static constexpr size_t kMaxEntries = 200;

    void add(const std::string& timestamp, const std::string& level, const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        entries_.push_back({timestamp, level, message});
        while (entries_.size() > kMaxEntries) {
            entries_.pop_front();
        }
        new_entries_++;
    }

    std::vector<LogEntry> getNew() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<LogEntry> result(entries_.begin(), entries_.end());
        new_entries_ = 0;
        return result;
    }

    std::vector<LogEntry> getRecent(size_t count = 50) {
        std::lock_guard<std::mutex> lock(mutex_);
        size_t start = entries_.size() > count ? entries_.size() - count : 0;
        return std::vector<LogEntry>(entries_.begin() + static_cast<std::ptrdiff_t>(start), entries_.end());
    }

    bool hasNew() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return new_entries_ > 0;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        entries_.clear();
        new_entries_ = 0;
    }

private:
    mutable std::mutex mutex_;
    std::deque<LogEntry> entries_;
    size_t new_entries_{0};
};

/// @brief Custom spdlog sink that captures logs to a buffer
template<typename Mutex>
class WebUISink : public spdlog::sinks::base_sink<Mutex> {
public:
    explicit WebUISink(std::shared_ptr<LogBuffer> buffer) : buffer_(buffer) {}

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        // Format timestamp
        auto time = std::chrono::system_clock::to_time_t(msg.time);
        std::tm tm_time{};
#ifdef _WIN32
        localtime_s(&tm_time, &time);
#else
        localtime_r(&time, &tm_time);
#endif
        char timestamp[32];
        std::strftime(timestamp, sizeof(timestamp), "%H:%M:%S", &tm_time);

        // Get level string
        std::string level = spdlog::level::to_string_view(msg.level).data();

        // Get message
        std::string message(msg.payload.data(), msg.payload.size());

        buffer_->add(timestamp, level, message);
    }

    void flush_() override {}

private:
    std::shared_ptr<LogBuffer> buffer_;
};

/// @brief HTTP/WebSocket server for robot control
class WebServer {
public:
    explicit WebServer(const WebServerConfig& config);
    ~WebServer();

    // Non-copyable
    WebServer(const WebServer&) = delete;
    WebServer& operator=(const WebServer&) = delete;

    /// @brief Start the server (blocks until stopped)
    void run();

    /// @brief Stop the server
    void stop();

    /// @brief Check if server is running
    [[nodiscard]] bool isRunning() const { return running_.load(); }

private:
    void setupRoutes();
    void setupWebSocket();
    void setupTrajectoryRoutes();
    void startStateUpdateThread();
    void stateUpdateLoop();

    WebServerConfig config_;
    crow::SimpleApp app_;
    std::unique_ptr<RobotStateManager> state_manager_;
    std::unique_ptr<WebSocketBroadcaster> ws_broadcaster_;

    // Kinematics and trajectory
    std::unique_ptr<kinematics::URKinematics> kinematics_;
    std::unique_ptr<trajectory::TrajectoryPlanner> trajectory_planner_;
    std::unique_ptr<trajectory::AxisBlendPlanner> axis_blend_planner_;
    std::unique_ptr<trajectory::velocity_first::VelocityFirstPlanner> velocity_first_planner_;
    std::unique_ptr<trajectory::TrajectoryExecutor> trajectory_executor_;
    std::optional<trajectory::PlannedTrajectory> current_trajectory_;
    std::vector<trajectory::Waypoint> current_waypoints_;
    std::vector<trajectory::velocity_first::TimedWaypoint> current_timed_waypoints_;

    std::atomic<bool> running_{false};
    std::thread state_update_thread_;

    // Log buffer for UI
    std::shared_ptr<LogBuffer> log_buffer_;
    void setupLogging();
};

}  // namespace webui
}  // namespace ur_controller
