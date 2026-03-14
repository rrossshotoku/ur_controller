#pragma once

/// @file websocket_broadcaster.hpp
/// @brief Manages WebSocket connections and broadcasts state updates

#include "robot_state_manager.hpp"

#include <crow.h>
#include <unordered_set>
#include <mutex>
#include <string>

namespace ur_controller {
namespace webui {

/// @brief Manages WebSocket connections and broadcasts state updates
class WebSocketBroadcaster {
public:
    WebSocketBroadcaster();
    ~WebSocketBroadcaster();

    // Non-copyable
    WebSocketBroadcaster(const WebSocketBroadcaster&) = delete;
    WebSocketBroadcaster& operator=(const WebSocketBroadcaster&) = delete;

    /// @brief Add a new WebSocket connection
    void addConnection(crow::websocket::connection* conn);

    /// @brief Remove a WebSocket connection
    void removeConnection(crow::websocket::connection* conn);

    /// @brief Broadcast state to all connected clients
    void broadcast(const RobotState& state);

    /// @brief Get number of connected clients
    [[nodiscard]] size_t connectionCount() const;

private:
    /// @brief Serialize robot state to JSON string
    [[nodiscard]] std::string serializeState(const RobotState& state) const;

    mutable std::mutex connections_mutex_;
    std::unordered_set<crow::websocket::connection*> connections_;
};

}  // namespace webui
}  // namespace ur_controller
