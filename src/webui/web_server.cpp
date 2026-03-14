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
    , ws_broadcaster_(std::make_unique<WebSocketBroadcaster>())
    , kinematics_(std::make_unique<kinematics::URKinematics>(kinematics::URModel::UR5e))
    , trajectory_planner_(std::make_unique<trajectory::TrajectoryPlanner>(*kinematics_))
    , trajectory_executor_(std::make_unique<trajectory::TrajectoryExecutor>()) {

    // Set up trajectory executor to use servoJ
    trajectory_executor_->setCommandCallback(
        [this](const kinematics::JointVector& joints, double time) {
            (void)time;  // Unused
            if (state_manager_->hasControl()) {
                std::array<double, 6> q;
                for (int i = 0; i < 6; ++i) {
                    q[static_cast<size_t>(i)] = joints[i];
                }
                state_manager_->servoJ(q);
            }
        });

    setupRoutes();
    setupWebSocket();
    setupTrajectoryRoutes();
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

void WebServer::setupTrajectoryRoutes() {
    // API: Plan trajectory from waypoints
    CROW_ROUTE(app_, "/api/trajectory/plan").methods("POST"_method)
    ([this](const crow::request& req) {
        crow::json::wvalue result;

        auto body = crow::json::load(req.body);
        if (!body || !body.has("waypoints")) {
            result["success"] = false;
            result["message"] = "Missing 'waypoints' array";
            return crow::response(400, result);
        }

        // Parse waypoints
        std::vector<trajectory::Waypoint> waypoints;
        auto wp_array = body["waypoints"];
        for (size_t i = 0; i < wp_array.size(); ++i) {
            auto& wp_json = wp_array[i];
            trajectory::Waypoint wp;

            // Position
            if (wp_json.has("position")) {
                auto& pos = wp_json["position"];
                wp.position.x() = pos.has("x") ? pos["x"].d() : 0.0;
                wp.position.y() = pos.has("y") ? pos["y"].d() : 0.0;
                wp.position.z() = pos.has("z") ? pos["z"].d() : 0.0;
            }

            // Orientation (quaternion)
            if (wp_json.has("orientation")) {
                auto& ori = wp_json["orientation"];
                double w = ori.has("w") ? ori["w"].d() : 1.0;
                double x = ori.has("x") ? ori["x"].d() : 0.0;
                double y = ori.has("y") ? ori["y"].d() : 0.0;
                double z = ori.has("z") ? ori["z"].d() : 0.0;
                wp.orientation = Eigen::Quaterniond(w, x, y, z).normalized();
            }

            // Optional parameters
            if (wp_json.has("blend_radius")) {
                wp.blend_radius = wp_json["blend_radius"].d();
            }
            if (wp_json.has("segment_time")) {
                wp.segment_time = wp_json["segment_time"].d();
            }
            if (wp_json.has("pause_time")) {
                wp.pause_time = wp_json["pause_time"].d();
            }

            waypoints.push_back(wp);
        }

        // Parse optional config
        if (body.has("config")) {
            auto& cfg = body["config"];
            trajectory::TrajectoryConfig config;
            if (cfg.has("max_linear_velocity")) {
                config.max_linear_velocity = cfg["max_linear_velocity"].d();
            }
            if (cfg.has("max_linear_acceleration")) {
                config.max_linear_acceleration = cfg["max_linear_acceleration"].d();
            }
            if (cfg.has("max_joint_velocity")) {
                config.max_joint_velocity = cfg["max_joint_velocity"].d();
            }
            trajectory_planner_->setConfig(config);
        }

        // Plan trajectory
        auto planned = trajectory_planner_->plan(waypoints);

        // Store for later execution
        current_waypoints_ = waypoints;
        current_trajectory_ = planned;

        // Build response
        result["success"] = planned.valid;
        result["valid"] = planned.valid;
        result["duration"] = planned.total_duration;
        result["num_samples"] = planned.samples.size();

        // Validation messages
        crow::json::wvalue messages;
        for (size_t i = 0; i < planned.messages.size(); ++i) {
            crow::json::wvalue msg;
            msg["severity"] = planned.messages[i].severity == trajectory::ValidationSeverity::Error ? "error" :
                              planned.messages[i].severity == trajectory::ValidationSeverity::Warning ? "warning" : "info";
            msg["message"] = planned.messages[i].message;
            if (planned.messages[i].waypoint_index.has_value()) {
                msg["waypoint"] = *planned.messages[i].waypoint_index;
            }
            messages[i] = std::move(msg);
        }
        result["messages"] = std::move(messages);

        // Generate visualization data
        if (planned.valid) {
            auto viz = trajectory_planner_->generateVisualization(planned, waypoints);

            crow::json::wvalue viz_json;

            // Path points
            crow::json::wvalue path_points;
            for (size_t i = 0; i < viz.path_points.size(); ++i) {
                crow::json::wvalue point;
                point["x"] = viz.path_points[i].x();
                point["y"] = viz.path_points[i].y();
                point["z"] = viz.path_points[i].z();
                point["time"] = viz.path_times[i];
                path_points[i] = std::move(point);
            }
            viz_json["path"] = std::move(path_points);

            // Timing data for graphs
            crow::json::wvalue timing;
            for (size_t i = 0; i < viz.times.size(); ++i) {
                crow::json::wvalue point;
                point["time"] = viz.times[i];
                point["speed"] = viz.speeds[i];
                point["acceleration"] = i < viz.accelerations.size() ? viz.accelerations[i] : 0.0;
                timing[i] = std::move(point);
            }
            viz_json["timing"] = std::move(timing);

            // Waypoint markers
            crow::json::wvalue wp_markers;
            for (size_t i = 0; i < viz.waypoints.size(); ++i) {
                crow::json::wvalue marker;
                marker["index"] = viz.waypoints[i].index;
                marker["x"] = viz.waypoints[i].position.x();
                marker["y"] = viz.waypoints[i].position.y();
                marker["z"] = viz.waypoints[i].position.z();
                marker["time"] = viz.waypoints[i].time;
                marker["blend_radius"] = viz.waypoints[i].blend_radius;
                marker["has_pause"] = viz.waypoints[i].has_pause;
                wp_markers[i] = std::move(marker);
            }
            viz_json["waypoints"] = std::move(wp_markers);

            // Summary
            viz_json["total_duration"] = viz.total_duration;
            viz_json["total_distance"] = viz.total_distance;
            viz_json["max_speed"] = viz.max_speed;

            result["visualization"] = std::move(viz_json);
        }

        return crow::response(result);
    });

    // API: Execute planned trajectory
    CROW_ROUTE(app_, "/api/trajectory/execute").methods("POST"_method)
    ([this]() {
        crow::json::wvalue result;

        if (!current_trajectory_.has_value() || !current_trajectory_->valid) {
            result["success"] = false;
            result["message"] = "No valid trajectory planned";
            return crow::response(400, result);
        }

        if (!state_manager_->hasControl()) {
            result["success"] = false;
            result["message"] = "Control not available";
            return crow::response(400, result);
        }

        // Load and start execution
        if (!trajectory_executor_->load(*current_trajectory_)) {
            result["success"] = false;
            result["message"] = "Failed to load trajectory";
            return crow::response(500, result);
        }

        if (!trajectory_executor_->start()) {
            result["success"] = false;
            result["message"] = "Failed to start execution";
            return crow::response(500, result);
        }

        result["success"] = true;
        result["message"] = "Trajectory execution started";
        result["duration"] = current_trajectory_->total_duration;

        return crow::response(result);
    });

    // API: Get trajectory execution status
    CROW_ROUTE(app_, "/api/trajectory/status")
    ([this]() {
        auto progress = trajectory_executor_->progress();

        crow::json::wvalue result;
        result["state"] = trajectory::executorStateToString(progress.state);
        result["current_time"] = progress.current_time;
        result["total_duration"] = progress.total_duration;
        result["progress_percent"] = progress.progress_percent;
        result["current_speed"] = progress.current_speed;

        crow::json::wvalue joints;
        for (int i = 0; i < 6; ++i) {
            joints[static_cast<size_t>(i)] = progress.current_joints[i];
        }
        result["joints"] = std::move(joints);

        crow::json::wvalue tcp;
        tcp["x"] = progress.current_pose.translation().x();
        tcp["y"] = progress.current_pose.translation().y();
        tcp["z"] = progress.current_pose.translation().z();
        result["tcp"] = std::move(tcp);

        return crow::response(result);
    });

    // API: Pause trajectory execution
    CROW_ROUTE(app_, "/api/trajectory/pause").methods("POST"_method)
    ([this]() {
        trajectory_executor_->pause();

        crow::json::wvalue result;
        result["success"] = true;
        result["state"] = trajectory::executorStateToString(trajectory_executor_->state());

        return crow::response(result);
    });

    // API: Resume trajectory execution
    CROW_ROUTE(app_, "/api/trajectory/resume").methods("POST"_method)
    ([this]() {
        trajectory_executor_->resume();

        crow::json::wvalue result;
        result["success"] = true;
        result["state"] = trajectory::executorStateToString(trajectory_executor_->state());

        return crow::response(result);
    });

    // API: Stop trajectory execution
    CROW_ROUTE(app_, "/api/trajectory/stop").methods("POST"_method)
    ([this]() {
        trajectory_executor_->stop();
        state_manager_->stopJ();

        crow::json::wvalue result;
        result["success"] = true;
        result["message"] = "Trajectory stopped";

        return crow::response(result);
    });

    // API: Get current robot pose as waypoint
    CROW_ROUTE(app_, "/api/trajectory/current-pose")
    ([this]() {
        auto state = state_manager_->getState();

        crow::json::wvalue result;

        // Get TCP pose from state
        crow::json::wvalue position;
        position["x"] = state.tcp_pose[0];
        position["y"] = state.tcp_pose[1];
        position["z"] = state.tcp_pose[2];
        result["position"] = std::move(position);

        // Convert axis-angle to quaternion
        double rx = state.tcp_pose[3];
        double ry = state.tcp_pose[4];
        double rz = state.tcp_pose[5];
        double angle = std::sqrt(rx*rx + ry*ry + rz*rz);
        Eigen::Quaterniond q;
        if (angle > 1e-6) {
            Eigen::Vector3d axis(rx/angle, ry/angle, rz/angle);
            q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
        } else {
            q = Eigen::Quaterniond::Identity();
        }

        crow::json::wvalue orientation;
        orientation["w"] = q.w();
        orientation["x"] = q.x();
        orientation["y"] = q.y();
        orientation["z"] = q.z();
        result["orientation"] = std::move(orientation);

        crow::json::wvalue joints;
        for (size_t i = 0; i < 6; ++i) {
            joints[i] = state.joint_positions[i];
        }
        result["joints"] = std::move(joints);

        return crow::response(result);
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
