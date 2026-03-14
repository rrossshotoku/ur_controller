#pragma once

/// @file web_server.hpp
/// @brief HTTP/WebSocket server for robot control

#include "robot_state_manager.hpp"
#include "websocket_broadcaster.hpp"
#include "ur_controller/kinematics/ur_kinematics.hpp"
#include "ur_controller/trajectory/planner.hpp"
#include "ur_controller/trajectory/executor.hpp"

#include <crow.h>
#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <optional>

namespace ur_controller {
namespace webui {

/// @brief Configuration for the web server
struct WebServerConfig {
    std::string robot_ip = "ursim";
    uint16_t http_port = 8080;
    std::string static_path = "./webui";
    double state_update_rate_hz = 50.0;
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
    std::unique_ptr<trajectory::TrajectoryExecutor> trajectory_executor_;
    std::optional<trajectory::PlannedTrajectory> current_trajectory_;
    std::vector<trajectory::Waypoint> current_waypoints_;

    std::atomic<bool> running_{false};
    std::thread state_update_thread_;
};

}  // namespace webui
}  // namespace ur_controller
