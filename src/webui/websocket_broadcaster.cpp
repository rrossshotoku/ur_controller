/// @file websocket_broadcaster.cpp
/// @brief Implementation of WebSocket broadcaster

#include "ur_controller/webui/websocket_broadcaster.hpp"

#include <spdlog/spdlog.h>
#include <sstream>
#include <iomanip>

namespace ur_controller {
namespace webui {

WebSocketBroadcaster::WebSocketBroadcaster() = default;

WebSocketBroadcaster::~WebSocketBroadcaster() = default;

void WebSocketBroadcaster::addConnection(crow::websocket::connection* conn) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    connections_.insert(conn);
    spdlog::info("WebSocket client connected. Total clients: {}", connections_.size());
}

void WebSocketBroadcaster::removeConnection(crow::websocket::connection* conn) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    connections_.erase(conn);
    spdlog::info("WebSocket client disconnected. Total clients: {}", connections_.size());
}

void WebSocketBroadcaster::broadcast(const RobotState& state) {
    std::string message = serializeState(state);

    std::lock_guard<std::mutex> lock(connections_mutex_);
    for (auto* conn : connections_) {
        try {
            conn->send_text(message);
        } catch (const std::exception& e) {
            spdlog::warn("Failed to send to WebSocket client: {}", e.what());
        }
    }
}

size_t WebSocketBroadcaster::connectionCount() const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    return connections_.size();
}

std::string WebSocketBroadcaster::serializeState(const RobotState& state) const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);

    ss << "{";
    ss << "\"type\":\"state\",";
    ss << "\"timestamp\":" << state.timestamp_ms << ",";
    ss << "\"connected\":" << (state.connected ? "true" : "false") << ",";
    ss << "\"control_available\":" << (state.control_available ? "true" : "false") << ",";

    // Joint positions
    ss << "\"joints\":[";
    for (size_t i = 0; i < 6; ++i) {
        if (i > 0) ss << ",";
        ss << state.joint_positions[i];
    }
    ss << "],";

    // Joint velocities
    ss << "\"joint_velocities\":[";
    for (size_t i = 0; i < 6; ++i) {
        if (i > 0) ss << ",";
        ss << state.joint_velocities[i];
    }
    ss << "],";

    // TCP pose
    ss << "\"tcp_pose\":{";
    ss << "\"x\":" << state.tcp_pose[0] << ",";
    ss << "\"y\":" << state.tcp_pose[1] << ",";
    ss << "\"z\":" << state.tcp_pose[2] << ",";
    ss << "\"rx\":" << state.tcp_pose[3] << ",";
    ss << "\"ry\":" << state.tcp_pose[4] << ",";
    ss << "\"rz\":" << state.tcp_pose[5];
    ss << "},";

    // TCP speed
    ss << "\"tcp_speed\":{";
    ss << "\"vx\":" << state.tcp_speed[0] << ",";
    ss << "\"vy\":" << state.tcp_speed[1] << ",";
    ss << "\"vz\":" << state.tcp_speed[2] << ",";
    ss << "\"wx\":" << state.tcp_speed[3] << ",";
    ss << "\"wy\":" << state.tcp_speed[4] << ",";
    ss << "\"wz\":" << state.tcp_speed[5];
    ss << "},";

    ss << "\"speed_scaling\":" << state.speed_scaling << ",";
    ss << "\"robot_mode\":\"" << state.robot_mode << "\",";
    ss << "\"safety_status\":\"" << state.safety_status << "\"";
    ss << "}";

    return ss.str();
}

}  // namespace webui
}  // namespace ur_controller
