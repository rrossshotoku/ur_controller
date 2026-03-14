/// @file web_server.cpp
/// @brief Implementation of HTTP/WebSocket server

#include "ur_controller/webui/web_server.hpp"

#include <spdlog/spdlog.h>
#include <chrono>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace ur_controller {
namespace webui {

WebServer::WebServer(const WebServerConfig& config)
    : config_(config)
    , state_manager_(std::make_unique<RobotStateManager>(config.robot_ip))
    , ws_broadcaster_(std::make_unique<WebSocketBroadcaster>()) {

    setupRoutes();
    setupWebSocket();
}

WebServer::~WebServer() {
    stop();
}

void WebServer::run() {
    running_ = true;

    // Connect to robot
    if (!state_manager_->connect()) {
        spdlog::warn("Could not connect to robot at startup");
    }

    // Start state update thread
    startStateUpdateThread();

    spdlog::info("Starting web server on port {}", config_.http_port);
    spdlog::info("Open http://localhost:{} in your browser", config_.http_port);

    // Run Crow (blocks until stopped)
    app_.port(config_.http_port).multithreaded().run();

    running_ = false;

    // Wait for state update thread
    if (state_update_thread_.joinable()) {
        state_update_thread_.join();
    }
}

void WebServer::stop() {
    running_ = false;
    app_.stop();
    state_manager_->disconnect();
}

void WebServer::setupRoutes() {
    // Serve index.html at root
    CROW_ROUTE(app_, "/")
    ([this]() {
        std::string path = config_.static_path + "/index.html";
        std::ifstream file(path);
        if (!file.is_open()) {
            return crow::response(404, "index.html not found at " + path);
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        auto resp = crow::response(buffer.str());
        resp.set_header("Content-Type", "text/html");
        return resp;
    });

    // Serve CSS files
    CROW_ROUTE(app_, "/css/<string>")
    ([this](const std::string& filename) {
        std::string full_path = config_.static_path + "/css/" + filename;
        std::ifstream file(full_path);
        if (!file.is_open()) {
            return crow::response(404, "File not found: " + filename);
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        auto resp = crow::response(buffer.str());
        resp.set_header("Content-Type", "text/css");
        return resp;
    });

    // Serve JS files
    CROW_ROUTE(app_, "/js/<string>")
    ([this](const std::string& filename) {
        std::string full_path = config_.static_path + "/js/" + filename;
        std::ifstream file(full_path);
        if (!file.is_open()) {
            return crow::response(404, "File not found: " + filename);
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        auto resp = crow::response(buffer.str());
        resp.set_header("Content-Type", "application/javascript");
        return resp;
    });

    // Serve URDF files
    CROW_ROUTE(app_, "/urdf/<string>")
    ([this](const std::string& filename) {
        std::string full_path = config_.static_path + "/urdf/" + filename;
        std::ifstream file(full_path);
        if (!file.is_open()) {
            return crow::response(404, "File not found: " + filename);
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        auto resp = crow::response(buffer.str());
        resp.set_header("Content-Type", "application/xml");
        return resp;
    });

    // Serve mesh files (DAE) - nested path: /urdf/meshes/ur5e/visual/file.dae
    CROW_ROUTE(app_, "/urdf/meshes/<string>/<string>/<string>")
    ([this](const std::string& robot, const std::string& type, const std::string& filename) {
        std::string full_path = config_.static_path + "/urdf/meshes/" + robot + "/" + type + "/" + filename;
        std::ifstream file(full_path, std::ios::binary);
        if (!file.is_open()) {
            spdlog::warn("Mesh file not found: {}", full_path);
            return crow::response(404, "File not found: " + filename);
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        auto resp = crow::response(buffer.str());
        resp.set_header("Content-Type", "model/vnd.collada+xml");
        return resp;
    });

    // API: Get robot state
    CROW_ROUTE(app_, "/api/state")
    ([this]() {
        auto state = state_manager_->getState();
        crow::json::wvalue result;
        result["connected"] = state.connected;
        result["control_available"] = state.control_available;
        result["timestamp"] = state.timestamp_ms;

        crow::json::wvalue joints;
        for (size_t i = 0; i < 6; ++i) {
            joints[i] = state.joint_positions[i];
        }
        result["joints"] = std::move(joints);

        crow::json::wvalue tcp;
        tcp["x"] = state.tcp_pose[0];
        tcp["y"] = state.tcp_pose[1];
        tcp["z"] = state.tcp_pose[2];
        tcp["rx"] = state.tcp_pose[3];
        tcp["ry"] = state.tcp_pose[4];
        tcp["rz"] = state.tcp_pose[5];
        result["tcp_pose"] = std::move(tcp);

        result["speed_scaling"] = state.speed_scaling;
        result["robot_mode"] = state.robot_mode;
        result["safety_status"] = state.safety_status;

        return crow::response(result);
    });

    // API: Connect to robot
    CROW_ROUTE(app_, "/api/connect").methods("POST"_method)
    ([this](const crow::request& req) {
        auto body = crow::json::load(req.body);
        crow::json::wvalue result;

        std::string ip = config_.robot_ip;
        if (body && body.has("ip")) {
            ip = body["ip"].s();
        }

        // Reconnect with new IP if different
        state_manager_->disconnect();
        state_manager_ = std::make_unique<RobotStateManager>(ip);

        if (state_manager_->connect()) {
            result["success"] = true;
            result["message"] = "Connected to " + ip;
            result["control_available"] = state_manager_->hasControl();
        } else {
            result["success"] = false;
            result["message"] = "Failed to connect to " + ip;
        }

        return crow::response(result);
    });

    // API: Disconnect from robot
    CROW_ROUTE(app_, "/api/disconnect").methods("POST"_method)
    ([this]() {
        state_manager_->disconnect();
        crow::json::wvalue result;
        result["success"] = true;
        result["message"] = "Disconnected";
        return crow::response(result);
    });

    // API: Try to enable control (retry control interface connection)
    CROW_ROUTE(app_, "/api/enable-control").methods("POST"_method)
    ([this]() {
        crow::json::wvalue result;
        if (state_manager_->tryEnableControl()) {
            result["success"] = true;
            result["message"] = "Control enabled";
        } else {
            result["success"] = false;
            result["message"] = "Could not enable control - ensure Remote Control is enabled on pendant";
        }
        return crow::response(result);
    });

    // API: ServoJ command
    CROW_ROUTE(app_, "/api/servoj").methods("POST"_method)
    ([this](const crow::request& req) {
        crow::json::wvalue result;

        if (!state_manager_->hasControl()) {
            result["success"] = false;
            result["message"] = "Control not available";
            return crow::response(400, result);
        }

        auto body = crow::json::load(req.body);
        if (!body || !body.has("joints")) {
            result["success"] = false;
            result["message"] = "Missing 'joints' array";
            return crow::response(400, result);
        }

        std::array<double, 6> target_q{};
        auto joints = body["joints"];
        for (size_t i = 0; i < 6 && i < joints.size(); ++i) {
            target_q[i] = joints[i].d();
        }

        double lookahead = body.has("lookahead") ? body["lookahead"].d() : 0.1;
        double gain = body.has("gain") ? body["gain"].d() : 300.0;

        if (state_manager_->servoJ(target_q, lookahead, gain)) {
            result["success"] = true;
        } else {
            result["success"] = false;
            result["message"] = "servoJ command failed";
        }

        return crow::response(result);
    });

    // API: Stop robot
    CROW_ROUTE(app_, "/api/stop").methods("POST"_method)
    ([this]() {
        crow::json::wvalue result;

        if (state_manager_->stopJ()) {
            result["success"] = true;
            result["message"] = "Stop command sent";
        } else {
            result["success"] = false;
            result["message"] = "Stop command failed";
        }

        return crow::response(result);
    });
}

void WebServer::setupWebSocket() {
    CROW_ROUTE(app_, "/ws/state")
        .websocket()
        .onopen([this](crow::websocket::connection& conn) {
            ws_broadcaster_->addConnection(&conn);
        })
        .onclose([this](crow::websocket::connection& conn, const std::string& reason) {
            ws_broadcaster_->removeConnection(&conn);
            (void)reason;  // Unused
        })
        .onmessage([this](crow::websocket::connection& conn, const std::string& data, bool is_binary) {
            (void)conn;
            (void)is_binary;

            // Handle incoming commands from WebSocket
            auto body = crow::json::load(data);
            if (!body) return;

            std::string type = body.has("type") ? std::string(body["type"].s()) : "";

            if (type == "servoj" && body.has("joints")) {
                std::array<double, 6> target_q{};
                auto joints = body["joints"];
                for (size_t i = 0; i < 6 && i < joints.size(); ++i) {
                    target_q[i] = joints[i].d();
                }
                state_manager_->servoJ(target_q);
            } else if (type == "stop") {
                state_manager_->stopJ();
            }
        });
}

void WebServer::startStateUpdateThread() {
    state_update_thread_ = std::thread([this]() {
        stateUpdateLoop();
    });
}

void WebServer::stateUpdateLoop() {
    using namespace std::chrono;

    auto interval = milliseconds(static_cast<int>(1000.0 / config_.state_update_rate_hz));

    while (running_) {
        auto start = steady_clock::now();

        // Update state from robot
        if (state_manager_->isConnected()) {
            state_manager_->updateState();

            // Broadcast to WebSocket clients
            if (ws_broadcaster_->connectionCount() > 0) {
                auto state = state_manager_->getState();
                ws_broadcaster_->broadcast(state);
            }
        }

        // Sleep for remaining time
        auto elapsed = steady_clock::now() - start;
        auto sleep_time = interval - duration_cast<milliseconds>(elapsed);
        if (sleep_time > milliseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
        }
    }
}

}  // namespace webui
}  // namespace ur_controller
