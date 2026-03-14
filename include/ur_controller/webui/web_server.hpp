#pragma once

/// @file web_server.hpp
/// @brief HTTP/WebSocket server for robot control

#include "robot_state_manager.hpp"
#include "websocket_broadcaster.hpp"

#include <crow.h>
#include <memory>
#include <string>
#include <atomic>
#include <thread>

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
    void startStateUpdateThread();
    void stateUpdateLoop();

    WebServerConfig config_;
    crow::SimpleApp app_;
    std::unique_ptr<RobotStateManager> state_manager_;
    std::unique_ptr<WebSocketBroadcaster> ws_broadcaster_;

    std::atomic<bool> running_{false};
    std::thread state_update_thread_;
};

}  // namespace webui
}  // namespace ur_controller
