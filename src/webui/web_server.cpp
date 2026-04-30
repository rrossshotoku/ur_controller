/// @file web_server.cpp
/// @brief Implementation of HTTP/WebSocket server

#include "ur_controller/webui/web_server.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
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
    , axis_blend_planner_(std::make_unique<trajectory::AxisBlendPlanner>(*kinematics_))
    , velocity_first_planner_(std::make_unique<trajectory::velocity_first::VelocityFirstPlanner>(*kinematics_))
    , trajectory_executor_(std::make_unique<trajectory::TrajectoryExecutor>())
    , log_buffer_(std::make_shared<LogBuffer>()) {

    // Set up logging to capture to UI
    setupLogging();

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

    // Release servo control when trajectory completes or stops
    trajectory_executor_->setStateCallback(
        [this](trajectory::ExecutorState new_state) {
            if (new_state == trajectory::ExecutorState::Idle) {
                // Trajectory completed or stopped - release servo mode
                // so jog controls (speedL/speedJ) can work
                state_manager_->servoStop();
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

    // Serve robot viewer popup
    CROW_ROUTE(app_, "/robot-viewer-popup.html")
    ([this]() {
        std::string path = config_.static_path + "/robot-viewer-popup.html";
        std::ifstream file(path);
        if (!file.is_open()) {
            return crow::response(404, "robot-viewer-popup.html not found");
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        auto resp = crow::response(buffer.str());
        resp.set_header("Content-Type", "text/html");
        return resp;
    });

    // Serve chart popup
    CROW_ROUTE(app_, "/chart-popup.html")
    ([this]() {
        std::string path = config_.static_path + "/chart-popup.html";
        std::ifstream file(path);
        if (!file.is_open()) {
            return crow::response(404, "chart-popup.html not found");
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

    // API: Get recent logs for terminal display
    CROW_ROUTE(app_, "/api/logs")
    ([this]() {
        auto logs = log_buffer_->getRecent(100);

        crow::json::wvalue result;
        crow::json::wvalue entries;
        for (size_t i = 0; i < logs.size(); ++i) {
            crow::json::wvalue entry;
            entry["timestamp"] = logs[i].timestamp;
            entry["level"] = logs[i].level;
            entry["message"] = logs[i].message;
            entries[i] = std::move(entry);
        }
        result["logs"] = std::move(entries);
        result["count"] = logs.size();

        return crow::response(result);
    });

    // API: Clear logs
    CROW_ROUTE(app_, "/api/logs/clear").methods("POST"_method)
    ([this]() {
        log_buffer_->clear();

        crow::json::wvalue result;
        result["success"] = true;
        result["message"] = "Logs cleared";

        return crow::response(result);
    });

    // API: Move to joint positions (async move, can be stopped)
    CROW_ROUTE(app_, "/api/move-to-joints").methods("POST"_method)
    ([this](const crow::request& req) {
        crow::json::wvalue result;

        auto body = crow::json::load(req.body);
        if (!body || !body.has("joints")) {
            result["success"] = false;
            result["message"] = "Missing 'joints' array";
            return crow::response(400, result);
        }

        auto joints_json = body["joints"];
        if (joints_json.size() < 6) {
            result["success"] = false;
            result["message"] = "joints array must have 6 elements";
            return crow::response(400, result);
        }

        std::array<double, 6> target_joints;
        for (size_t i = 0; i < 6; ++i) {
            target_joints[i] = joints_json[i].d();
        }

        double speed = body.has("speed") ? body["speed"].d() : 0.5;
        double acceleration = body.has("acceleration") ? body["acceleration"].d() : 1.0;

        if (state_manager_->moveJ(target_joints, speed, acceleration)) {
            result["success"] = true;
            result["message"] = "Move started";
        } else {
            result["success"] = false;
            result["message"] = "Move command failed";
        }

        return crow::response(result);
    });

    // API: Move linearly to TCP pose (async move, can be stopped)
    CROW_ROUTE(app_, "/api/move-to-pose").methods("POST"_method)
    ([this](const crow::request& req) {
        crow::json::wvalue result;

        auto body = crow::json::load(req.body);
        if (!body) {
            result["success"] = false;
            result["message"] = "Invalid JSON body";
            return crow::response(400, result);
        }

        // Parse position
        if (!body.has("position")) {
            result["success"] = false;
            result["message"] = "Missing 'position' object";
            return crow::response(400, result);
        }
        auto& pos = body["position"];
        double x = pos.has("x") ? pos["x"].d() : 0.0;
        double y = pos.has("y") ? pos["y"].d() : 0.0;
        double z = pos.has("z") ? pos["z"].d() : 0.0;

        // Parse orientation (quaternion) and convert to axis-angle
        double rx = 0.0, ry = 0.0, rz = 0.0;
        if (body.has("orientation")) {
            auto& ori = body["orientation"];
            double qw = ori.has("w") ? ori["w"].d() : 1.0;
            double qx = ori.has("x") ? ori["x"].d() : 0.0;
            double qy = ori.has("y") ? ori["y"].d() : 0.0;
            double qz = ori.has("z") ? ori["z"].d() : 0.0;

            // Convert quaternion to axis-angle
            Eigen::Quaterniond q(qw, qx, qy, qz);
            q.normalize();
            Eigen::AngleAxisd aa(q);
            Eigen::Vector3d axis = aa.axis() * aa.angle();
            rx = axis.x();
            ry = axis.y();
            rz = axis.z();
        } else {
            // Use current orientation if not specified
            auto state = state_manager_->getState();
            rx = state.tcp_pose[3];
            ry = state.tcp_pose[4];
            rz = state.tcp_pose[5];
        }

        std::array<double, 6> target_pose = {x, y, z, rx, ry, rz};

        double speed = body.has("speed") ? body["speed"].d() : 0.1;
        double acceleration = body.has("acceleration") ? body["acceleration"].d() : 0.5;

        if (state_manager_->moveL(target_pose, speed, acceleration)) {
            result["success"] = true;
            result["message"] = "Linear move started";
        } else {
            result["success"] = false;
            result["message"] = "Move command failed";
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

            if (type == "speedj" && body.has("velocity")) {
                // Joint velocity control for joint jogging
                std::array<double, 6> joint_vel{};
                auto velocity = body["velocity"];
                for (size_t i = 0; i < 6 && i < velocity.size(); ++i) {
                    joint_vel[i] = velocity[i].d();
                }
                double accel = body.has("acceleration") ? body["acceleration"].d() : 1.0;
                // Use time=0.2s to ensure smooth motion when streaming commands
                state_manager_->speedJ(joint_vel, accel, 0.2);
            } else if (type == "speedl" && body.has("velocity")) {
                // TCP velocity control for jogging
                std::array<double, 6> tcp_vel{};
                auto velocity = body["velocity"];
                for (size_t i = 0; i < 3 && i < velocity.size(); ++i) {
                    tcp_vel[i] = velocity[i].d();
                }
                // Angular velocity (optional)
                if (body.has("angular_velocity")) {
                    auto angular = body["angular_velocity"];
                    for (size_t i = 0; i < 3 && i < angular.size(); ++i) {
                        tcp_vel[i + 3] = angular[i].d();
                    }
                }
                double accel = body.has("acceleration") ? body["acceleration"].d() : 0.5;
                // Use time=0.2s to ensure smooth motion when streaming commands
                state_manager_->speedL(tcp_vel, accel, 0.2);
            } else if (type == "stop") {
                // Universal stop for all velocity modes
                state_manager_->speedStop();
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
            if (wp_json.has("blend_factor")) {
                wp.blend_factor = wp_json["blend_factor"].d();
            }
            if (wp_json.has("segment_time")) {
                wp.segment_time = wp_json["segment_time"].d();
            }
            if (wp_json.has("pause_time")) {
                wp.pause_time = wp_json["pause_time"].d();
            }

            // Parse saved joint positions (captured when waypoint was taught)
            if (wp_json.has("joints")) {
                auto& joints_json = wp_json["joints"];
                if (joints_json.size() == 6) {
                    kinematics::JointVector joints;
                    for (size_t j = 0; j < 6; ++j) {
                        joints[static_cast<Eigen::Index>(j)] = joints_json[j].d();
                    }
                    wp.joints = joints;
                    spdlog::info("  WP{} has saved joints: [{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}]°",
                        i + 1, joints[0]*180/M_PI, joints[1]*180/M_PI, joints[2]*180/M_PI,
                        joints[3]*180/M_PI, joints[4]*180/M_PI, joints[5]*180/M_PI);
                }
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
                point["jerk"] = i < viz.jerks.size() ? viz.jerks[i] : 0.0;
                timing[i] = std::move(point);
            }
            viz_json["timing"] = std::move(timing);

            // Segment times for waypoint markers on graph
            crow::json::wvalue seg_times;
            for (size_t i = 0; i < viz.segment_times.size(); ++i) {
                seg_times[i] = viz.segment_times[i];
            }
            viz_json["segment_times"] = std::move(seg_times);

            // Waypoint markers
            crow::json::wvalue wp_markers;
            for (size_t i = 0; i < viz.waypoints.size(); ++i) {
                crow::json::wvalue marker;
                marker["index"] = viz.waypoints[i].index;
                marker["x"] = viz.waypoints[i].position.x();
                marker["y"] = viz.waypoints[i].position.y();
                marker["z"] = viz.waypoints[i].position.z();
                marker["time"] = viz.waypoints[i].time;
                marker["blend_factor"] = viz.waypoints[i].blend_factor;
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

    // API: Plan trajectory with new format (setup poses + sequences)
    CROW_ROUTE(app_, "/api/trajectory/plan2").methods("POST"_method)
    ([this](const crow::request& req) {
        crow::json::wvalue result;

        auto body = crow::json::load(req.body);
        if (!body || !body.has("elements")) {
            result["success"] = false;
            result["message"] = "Missing 'elements' array";
            return crow::response(400, result);
        }

        // Get current robot joints for planning
        auto robot_state = state_manager_->getState();
        kinematics::JointVector start_joints;
        for (int i = 0; i < 6; ++i) {
            start_joints[i] = robot_state.joint_positions[static_cast<size_t>(i)];
        }

        // Parse trajectory elements
        trajectory::Trajectory traj;
        if (body.has("name")) {
            traj.name = std::string(body["name"].s());
        }

        auto elements = body["elements"];
        for (size_t i = 0; i < elements.size(); ++i) {
            auto& elem_json = elements[i];

            if (!elem_json.has("type")) {
                result["success"] = false;
                result["message"] = "Element " + std::to_string(i + 1) + " missing 'type'";
                return crow::response(400, result);
            }

            std::string type = std::string(elem_json["type"].s());

            if (type == "setup_pose") {
                // Parse setup pose
                if (!elem_json.has("joints")) {
                    result["success"] = false;
                    result["message"] = "Setup pose " + std::to_string(i + 1) + " missing 'joints'";
                    return crow::response(400, result);
                }

                trajectory::SetupPose sp;
                auto joints = elem_json["joints"];
                for (size_t j = 0; j < 6 && j < joints.size(); ++j) {
                    sp.joints[static_cast<Eigen::Index>(j)] = joints[j].d();
                }
                if (elem_json.has("name")) {
                    sp.name = std::string(elem_json["name"].s());
                }
                if (elem_json.has("move_time")) {
                    sp.move_time = elem_json["move_time"].d();
                }

                traj.addSetupPose(sp);

            } else if (type == "sequence") {
                // Parse sequence
                if (!elem_json.has("waypoints")) {
                    result["success"] = false;
                    result["message"] = "Sequence " + std::to_string(i + 1) + " missing 'waypoints'";
                    return crow::response(400, result);
                }

                trajectory::Sequence seq;
                if (elem_json.has("name")) {
                    seq.name = std::string(elem_json["name"].s());
                }

                // Parse entry joints if provided (captured when first waypoint was added)
                if (elem_json.has("entry_joints")) {
                    auto entry_j = elem_json["entry_joints"];
                    spdlog::info("Sequence has entry_joints with {} elements", entry_j.size());
                    if (entry_j.size() >= 6) {
                        kinematics::JointVector ej;
                        for (size_t j = 0; j < 6; ++j) {
                            ej[static_cast<Eigen::Index>(j)] = entry_j[j].d();
                        }
                        seq.entry_joints = ej;
                        spdlog::info("Parsed entry_joints: [{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]",
                            ej[0], ej[1], ej[2], ej[3], ej[4], ej[5]);
                    }
                } else {
                    spdlog::warn("Sequence does NOT have entry_joints - will use current robot position");
                }

                auto waypoints = elem_json["waypoints"];
                for (size_t w = 0; w < waypoints.size(); ++w) {
                    auto& wp_json = waypoints[w];
                    trajectory::SequenceWaypoint wp;

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
                        double qw = ori.has("w") ? ori["w"].d() : 1.0;
                        double qx = ori.has("x") ? ori["x"].d() : 0.0;
                        double qy = ori.has("y") ? ori["y"].d() : 0.0;
                        double qz = ori.has("z") ? ori["z"].d() : 0.0;
                        wp.orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
                    }

                    // Optional parameters
                    if (wp_json.has("blend_factor")) {
                        wp.blend_factor = wp_json["blend_factor"].d();
                    }
                    if (wp_json.has("segment_time")) {
                        wp.segment_time = wp_json["segment_time"].d();
                    }
                    if (wp_json.has("pause_time")) {
                        wp.pause_time = wp_json["pause_time"].d();
                    }

                    // Parse saved joint positions (captured when waypoint was taught)
                    if (wp_json.has("joints")) {
                        auto& joints_json = wp_json["joints"];
                        if (joints_json.size() == 6) {
                            kinematics::JointVector joints;
                            for (size_t j = 0; j < 6; ++j) {
                                joints[static_cast<Eigen::Index>(j)] = joints_json[j].d();
                            }
                            wp.joints = joints;
                            spdlog::info("  Seq WP{} has saved joints: [{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}]°",
                                w + 1, joints[0]*180/M_PI, joints[1]*180/M_PI, joints[2]*180/M_PI,
                                joints[3]*180/M_PI, joints[4]*180/M_PI, joints[5]*180/M_PI);
                        }
                    }

                    seq.waypoints.push_back(wp);
                }

                traj.addSequence(seq);

            } else {
                result["success"] = false;
                result["message"] = "Unknown element type: " + type;
                return crow::response(400, result);
            }
        }

        // Parse planning method (geometric | axis_blend) — defaults to Geometric
        trajectory::PlanningMethod method = trajectory::PlanningMethod::Geometric;
        if (body.has("planning_method")) {
            method = trajectory::planningMethodFromString(
                std::string(body["planning_method"].s()));
        } else if (body.has("config")) {
            auto& cfg_pm = body["config"];
            if (cfg_pm.has("planning_method")) {
                method = trajectory::planningMethodFromString(
                    std::string(cfg_pm["planning_method"].s()));
            }
        }
        spdlog::info("plan2: planning_method = {}",
                     trajectory::planningMethodToString(method));

        // Update config on whichever planner we'll use
        if (body.has("config")) {
            auto& cfg = body["config"];
            trajectory::TrajectoryConfig config =
                (method == trajectory::PlanningMethod::AxisBlend)
                    ? axis_blend_planner_->config()
                    : trajectory_planner_->config();
            if (cfg.has("max_linear_velocity")) {
                config.max_linear_velocity = cfg["max_linear_velocity"].d();
            }
            if (cfg.has("max_linear_acceleration")) {
                config.max_linear_acceleration = cfg["max_linear_acceleration"].d();
            }
            if (cfg.has("max_linear_jerk")) {
                config.max_linear_jerk = cfg["max_linear_jerk"].d();
            }
            if (cfg.has("max_joint_velocity")) {
                config.max_joint_velocity = cfg["max_joint_velocity"].d();
            }
            config.planning_method = method;
            if (method == trajectory::PlanningMethod::AxisBlend) {
                axis_blend_planner_->setConfig(config);
            } else {
                trajectory_planner_->setConfig(config);
            }
        }

        // Build geometric path from sequence waypoints for visualization
        // NOTE: No velocity planning yet - just geometry
        std::vector<trajectory::SequenceWaypoint> all_waypoints;
        for (const auto& elem : traj.elements) {
            if (elem.type == trajectory::TrajectoryElementType::Sequence) {
                for (const auto& wp : elem.sequence.waypoints) {
                    all_waypoints.push_back(wp);
                }
            }
        }

        // Generate visualization using PathGeometry only
        crow::json::wvalue viz_json;
        bool path_valid = false;
        double total_length = 0.0;

        if (all_waypoints.size() >= 2) {
            trajectory::PathGeometry path_geom = trajectory::PathGeometry::fromWaypoints(all_waypoints);
            path_valid = true;
            total_length = path_geom.totalLength();

            // Sample path at ~200 points for visualization
            crow::json::wvalue path_points;
            constexpr size_t kNumVizPoints = 200;

            for (size_t i = 0; i <= kNumVizPoints; ++i) {
                double s = (total_length * static_cast<double>(i)) / static_cast<double>(kNumVizPoints);
                auto point = path_geom.evaluate(s);

                crow::json::wvalue pt;
                pt["x"] = point.position.x();
                pt["y"] = point.position.y();
                pt["z"] = point.position.z();
                pt["s"] = s;
                path_points[i] = std::move(pt);
            }
            viz_json["path"] = std::move(path_points);

            // Waypoint markers at their distances
            crow::json::wvalue wp_markers;
            const auto& wp_distances = path_geom.waypointDistances();
            for (size_t i = 0; i < wp_distances.size(); ++i) {
                auto point = path_geom.evaluate(wp_distances[i]);
                crow::json::wvalue marker;
                marker["x"] = point.position.x();
                marker["y"] = point.position.y();
                marker["z"] = point.position.z();
                marker["distance"] = wp_distances[i];
                marker["index"] = i;
                wp_markers[i] = std::move(marker);
            }
            viz_json["waypoints"] = std::move(wp_markers);
            viz_json["total_length"] = total_length;

            spdlog::info("PathGeometry: {} waypoints, {} segments, {:.3f}m total length",
                all_waypoints.size(), path_geom.numSegments(), total_length);
        } else if (all_waypoints.size() == 1) {
            // Single waypoint - show just the waypoint marker
            path_valid = true;
            crow::json::wvalue wp_markers;
            crow::json::wvalue marker;
            marker["x"] = all_waypoints[0].position.x();
            marker["y"] = all_waypoints[0].position.y();
            marker["z"] = all_waypoints[0].position.z();
            marker["distance"] = 0.0;
            marker["index"] = 0;
            wp_markers[0] = std::move(marker);
            viz_json["waypoints"] = std::move(wp_markers);
            viz_json["total_length"] = 0.0;

            spdlog::info("PathGeometry: single waypoint at ({:.3f}, {:.3f}, {:.3f})",
                all_waypoints[0].position.x(), all_waypoints[0].position.y(), all_waypoints[0].position.z());
        }

        // Run velocity planning via TrajectoryPlanner (S-curve profile) and store
        // the planned trajectory so the Execute endpoint can run it.
        trajectory::PlannedTrajectory planned;
        if (path_valid && all_waypoints.size() >= 2) {
            std::vector<trajectory::Waypoint> planner_waypoints;
            planner_waypoints.reserve(all_waypoints.size());
            for (const auto& sw : all_waypoints) {
                trajectory::Waypoint pw;
                pw.position = sw.position;
                pw.orientation = sw.orientation;
                pw.blend_factor = sw.blend_factor;
                pw.segment_time = sw.segment_time;
                pw.pause_time = sw.pause_time;
                pw.joints = sw.joints;
                planner_waypoints.push_back(pw);
            }

            // Find the first sequence's entry_joints (if any) to use as the
            // planner's start joints — this avoids re-deriving start joints via
            // numerical IK and prevents drift to a neighbouring solution branch.
            std::optional<kinematics::JointVector> start_joints_opt;
            for (const auto& elem : traj.elements) {
                if (elem.type == trajectory::TrajectoryElementType::Sequence &&
                    elem.sequence.entry_joints.has_value()) {
                    start_joints_opt = elem.sequence.entry_joints;
                    break;
                }
            }

            if (method == trajectory::PlanningMethod::AxisBlend) {
                if (start_joints_opt.has_value()) {
                    spdlog::info("plan2: AxisBlendPlanner with sequence entry_joints");
                    planned = axis_blend_planner_->plan(planner_waypoints, *start_joints_opt);
                } else {
                    spdlog::info("plan2: AxisBlendPlanner deriving start joints from first waypoint");
                    planned = axis_blend_planner_->plan(planner_waypoints);
                }
            } else {
                if (start_joints_opt.has_value()) {
                    spdlog::info("plan2: TrajectoryPlanner (Geometric) with sequence entry_joints");
                    planned = trajectory_planner_->plan(planner_waypoints, *start_joints_opt);
                } else {
                    spdlog::info("plan2: TrajectoryPlanner (Geometric) deriving start joints from first waypoint");
                    planned = trajectory_planner_->plan(planner_waypoints);
                }
            }
            current_waypoints_ = planner_waypoints;
            current_trajectory_ = planned;

            // For AxisBlend, replace the displayed path with the actual sampled
            // trajectory so the 3D viewer shows what the robot will trace, not
            // the geometric circular-arc approximation.
            if (method == trajectory::PlanningMethod::AxisBlend &&
                !planned.samples.empty()) {
                constexpr size_t kNumVizPoints = 200;
                const size_t total = planned.samples.size();
                const size_t stride =
                    std::max<size_t>(1, total / kNumVizPoints);

                crow::json::wvalue path_points;
                size_t out_idx = 0;
                double cumulative = 0.0;
                Eigen::Vector3d prev_pos = planned.samples.front().pose.translation();
                for (size_t i = 0; i < total; i += stride) {
                    Eigen::Vector3d p = planned.samples[i].pose.translation();
                    cumulative += (p - prev_pos).norm();
                    prev_pos = p;

                    crow::json::wvalue pt;
                    pt["x"] = p.x();
                    pt["y"] = p.y();
                    pt["z"] = p.z();
                    pt["s"] = cumulative;
                    pt["t"] = planned.samples[i].time;
                    path_points[out_idx++] = std::move(pt);
                }
                // Always include the final sample so the path closes at the end
                if (total > 0 && (total - 1) % stride != 0) {
                    Eigen::Vector3d p = planned.samples.back().pose.translation();
                    cumulative += (p - prev_pos).norm();

                    crow::json::wvalue pt;
                    pt["x"] = p.x();
                    pt["y"] = p.y();
                    pt["z"] = p.z();
                    pt["s"] = cumulative;
                    pt["t"] = planned.samples.back().time;
                    path_points[out_idx++] = std::move(pt);
                }
                viz_json["path"] = std::move(path_points);
                viz_json["total_length"] = cumulative;
                total_length = cumulative;
                spdlog::info("plan2: AxisBlend viz path: {} points, length {:.3f}m",
                             out_idx, cumulative);
            }

            // Build timing series (speed / accel / jerk vs time) for the chart.
            // Works for both Geometric and AxisBlend planners since they both
            // populate planned.samples with time + speed.
            if (!planned.samples.empty()) {
                constexpr size_t kTimingPoints = 200;
                const size_t total = planned.samples.size();
                const size_t stride = std::max<size_t>(1, total / kTimingPoints);
                const double dt = planned.sample_period > 0.0
                                  ? planned.sample_period
                                  : 1.0 / config_.state_update_rate_hz;

                crow::json::wvalue timing;
                size_t out_idx = 0;
                double prev_speed = planned.samples.front().speed;
                double prev_accel = 0.0;
                double prev_t = planned.samples.front().time;

                for (size_t i = 0; i < total; i += stride) {
                    const auto& s = planned.samples[i];
                    double dts = std::max(s.time - prev_t, dt);
                    double accel = (s.speed - prev_speed) / dts;
                    double jerk = (accel - prev_accel) / dts;

                    crow::json::wvalue point;
                    point["time"] = s.time;
                    point["speed"] = s.speed;
                    point["acceleration"] = accel;
                    point["jerk"] = jerk;
                    timing[out_idx++] = std::move(point);

                    prev_speed = s.speed;
                    prev_accel = accel;
                    prev_t = s.time;
                }
                viz_json["timing"] = std::move(timing);

                // Segment-boundary times (cumulative segment_time of waypoints)
                crow::json::wvalue seg_times;
                double cumulative_t = 0.0;
                seg_times[0] = 0.0;
                size_t seg_idx = 1;
                for (size_t i = 1; i < all_waypoints.size(); ++i) {
                    double T = all_waypoints[i].segment_time;
                    if (T <= 0.0) {
                        // Mirror autoSegmentTime fallback
                        Eigen::Vector3d a = all_waypoints[i - 1].position;
                        Eigen::Vector3d b = all_waypoints[i].position;
                        double v = std::max(
                            (method == trajectory::PlanningMethod::AxisBlend
                                 ? axis_blend_planner_->config().max_linear_velocity
                                 : trajectory_planner_->config().max_linear_velocity),
                            1e-3);
                        T = std::max((b - a).norm() / v, 1e-3);
                    }
                    cumulative_t += T;
                    seg_times[seg_idx++] = cumulative_t;
                }
                viz_json["segment_times"] = std::move(seg_times);
            }
        }

        // Build response
        result["success"] = planned.valid;
        result["valid"] = planned.valid;
        result["num_elements"] = traj.elements.size();
        result["num_waypoints"] = all_waypoints.size();
        result["total_length"] = total_length;
        result["duration"] = planned.total_duration;
        result["num_samples"] = planned.samples.size();

        // Validation messages from planner + path info
        crow::json::wvalue messages;
        size_t msg_idx = 0;
        for (const auto& m : planned.messages) {
            crow::json::wvalue msg;
            msg["severity"] = m.severity == trajectory::ValidationSeverity::Error ? "error" :
                              m.severity == trajectory::ValidationSeverity::Warning ? "warning" : "info";
            msg["message"] = m.message;
            if (m.waypoint_index.has_value()) {
                msg["waypoint"] = *m.waypoint_index;
            }
            messages[msg_idx++] = std::move(msg);
        }
        if (!path_valid) {
            crow::json::wvalue msg;
            msg["severity"] = "warning";
            msg["message"] = "Need at least 2 waypoints to plan a trajectory";
            messages[msg_idx++] = std::move(msg);
        }
        result["messages"] = std::move(messages);

        // Include visualization
        result["visualization"] = std::move(viz_json);

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

    // API: Get trajectory config (velocity/acceleration/jerk limits)
    CROW_ROUTE(app_, "/api/trajectory/config")
    ([this]() {
        crow::json::wvalue result;
        const auto& config = trajectory_planner_->config();

        result["max_linear_velocity"] = config.max_linear_velocity;
        result["max_linear_acceleration"] = config.max_linear_acceleration;
        result["max_linear_jerk"] = config.max_linear_jerk;
        result["max_joint_velocity"] = config.max_joint_velocity;
        result["max_joint_acceleration"] = config.max_joint_acceleration;
        result["max_joint_jerk"] = config.max_joint_jerk;

        return crow::response(result);
    });

    // API: Set trajectory config
    CROW_ROUTE(app_, "/api/trajectory/config").methods("POST"_method)
    ([this](const crow::request& req) {
        crow::json::wvalue result;

        auto body = crow::json::load(req.body);
        if (!body) {
            result["success"] = false;
            result["message"] = "Invalid JSON body";
            return crow::response(400, result);
        }

        auto config = trajectory_planner_->config();

        // Update only the fields that are provided
        if (body.has("max_linear_velocity")) {
            config.max_linear_velocity = body["max_linear_velocity"].d();
        }
        if (body.has("max_linear_acceleration")) {
            config.max_linear_acceleration = body["max_linear_acceleration"].d();
        }
        if (body.has("max_linear_jerk")) {
            config.max_linear_jerk = body["max_linear_jerk"].d();
        }
        if (body.has("max_joint_velocity")) {
            config.max_joint_velocity = body["max_joint_velocity"].d();
        }
        if (body.has("max_joint_acceleration")) {
            config.max_joint_acceleration = body["max_joint_acceleration"].d();
        }
        if (body.has("max_joint_jerk")) {
            config.max_joint_jerk = body["max_joint_jerk"].d();
        }

        trajectory_planner_->setConfig(config);

        // Also update velocity-first planner config
        trajectory::velocity_first::VelocityFirstConfig vf_config;
        vf_config.max_speed = config.max_linear_velocity;
        vf_config.max_acceleration = config.max_linear_acceleration;
        vf_config.max_jerk = config.max_linear_jerk;
        vf_config.max_joint_velocity = config.max_joint_velocity;
        velocity_first_planner_->setConfig(vf_config);

        spdlog::info("Trajectory config updated: max_linear_vel={:.2f}, max_linear_accel={:.2f}, max_linear_jerk={:.2f}",
            config.max_linear_velocity, config.max_linear_acceleration, config.max_linear_jerk);

        result["success"] = true;
        result["max_linear_velocity"] = config.max_linear_velocity;
        result["max_linear_acceleration"] = config.max_linear_acceleration;
        result["max_linear_jerk"] = config.max_linear_jerk;
        result["max_joint_velocity"] = config.max_joint_velocity;
        result["max_joint_acceleration"] = config.max_joint_acceleration;
        result["max_joint_jerk"] = config.max_joint_jerk;

        return crow::response(result);
    });

    // API: Plan trajectory using velocity-first approach
    CROW_ROUTE(app_, "/api/trajectory/plan-velocity-first").methods("POST"_method)
    ([this](const crow::request& req) {
        crow::json::wvalue result;

        auto body = crow::json::load(req.body);
        if (!body || !body.has("waypoints")) {
            result["success"] = false;
            result["message"] = "Missing 'waypoints' array";
            return crow::response(400, result);
        }

        // Get current robot joints for planning
        auto robot_state = state_manager_->getState();
        kinematics::JointVector start_joints;
        for (int i = 0; i < 6; ++i) {
            start_joints[i] = robot_state.joint_positions[static_cast<size_t>(i)];
        }

        // Parse waypoints into timed waypoints
        std::vector<trajectory::velocity_first::TimedWaypoint> timed_waypoints;
        auto wp_array = body["waypoints"];
        double accumulated_time = 0.0;

        for (size_t i = 0; i < wp_array.size(); ++i) {
            auto& wp_json = wp_array[i];
            trajectory::velocity_first::TimedWaypoint wp;

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

            // Timing: first waypoint at t=0, others use segment_time
            if (i > 0 && wp_json.has("segment_time")) {
                accumulated_time += wp_json["segment_time"].d();
            }
            wp.time = accumulated_time;

            // Parse saved joint positions
            if (wp_json.has("joints")) {
                auto& joints_json = wp_json["joints"];
                if (joints_json.size() == 6) {
                    kinematics::JointVector joints;
                    for (size_t j = 0; j < 6; ++j) {
                        joints[static_cast<Eigen::Index>(j)] = joints_json[j].d();
                    }
                    wp.joints = joints;
                }
            }

            timed_waypoints.push_back(wp);
        }

        // Parse optional config
        if (body.has("config")) {
            auto& cfg = body["config"];
            trajectory::velocity_first::VelocityFirstConfig vf_config;
            if (cfg.has("max_linear_velocity")) {
                vf_config.max_speed = cfg["max_linear_velocity"].d();
            }
            if (cfg.has("max_linear_acceleration")) {
                vf_config.max_acceleration = cfg["max_linear_acceleration"].d();
            }
            if (cfg.has("max_linear_jerk")) {
                vf_config.max_jerk = cfg["max_linear_jerk"].d();
            }
            velocity_first_planner_->setConfig(vf_config);
        }

        // Plan trajectory
        auto planned = velocity_first_planner_->plan(timed_waypoints, start_joints);

        // Store for later execution
        current_timed_waypoints_ = timed_waypoints;
        current_trajectory_ = planned;

        // Build response
        result["success"] = planned.valid;
        result["valid"] = planned.valid;
        result["duration"] = planned.total_duration;
        result["num_samples"] = planned.samples.size();
        result["planner"] = "velocity_first";

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
            auto viz = velocity_first_planner_->generateVisualization(planned, timed_waypoints);

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
                point["jerk"] = i < viz.jerks.size() ? viz.jerks[i] : 0.0;
                timing[i] = std::move(point);
            }
            viz_json["timing"] = std::move(timing);

            // Segment times for waypoint markers on graph
            crow::json::wvalue seg_times;
            for (size_t i = 0; i < viz.segment_times.size(); ++i) {
                seg_times[i] = viz.segment_times[i];
            }
            viz_json["segment_times"] = std::move(seg_times);

            // Waypoint markers
            crow::json::wvalue wp_markers;
            for (size_t i = 0; i < viz.waypoints.size(); ++i) {
                crow::json::wvalue marker;
                marker["index"] = viz.waypoints[i].index;
                marker["x"] = viz.waypoints[i].position.x();
                marker["y"] = viz.waypoints[i].position.y();
                marker["z"] = viz.waypoints[i].position.z();
                marker["time"] = viz.waypoints[i].time;
                marker["blend_factor"] = viz.waypoints[i].blend_factor;
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

void WebServer::setupLogging() {
    // Create a multi-sink logger that logs to both console and our buffer
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto webui_sink = std::make_shared<WebUISink<std::mutex>>(log_buffer_);

    // Get the default logger and add our sink
    auto default_logger = spdlog::default_logger();
    default_logger->sinks().push_back(webui_sink);

    spdlog::info("Logging to Web UI terminal enabled");
}

}  // namespace webui
}  // namespace ur_controller
