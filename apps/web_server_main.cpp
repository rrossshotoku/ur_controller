/// @file web_server_main.cpp
/// @brief Entry point for the UR Controller web server

#include "ur_controller/webui/web_server.hpp"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <csignal>
#include <memory>

namespace {
std::unique_ptr<ur_controller::webui::WebServer> g_server;

void signalHandler(int signal) {
    spdlog::info("Received signal {}, shutting down...", signal);
    if (g_server) {
        g_server->stop();
    }
}
}  // namespace

int main(int argc, char* argv[]) {
    CLI::App app{"UR Controller Web Server"};

    ur_controller::webui::WebServerConfig config;

    app.add_option("-i,--ip", config.robot_ip, "Robot IP address")
        ->default_val("ursim");
    app.add_option("-p,--port", config.http_port, "HTTP server port")
        ->default_val(8080);
    app.add_option("-s,--static", config.static_path, "Path to static web files")
        ->default_val("/workspace/webui");
    app.add_option("-r,--rate", config.state_update_rate_hz, "State update rate in Hz")
        ->default_val(50.0);

    CLI11_PARSE(app, argc, argv);

    // Set up signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try {
        g_server = std::make_unique<ur_controller::webui::WebServer>(config);
        g_server->run();
    } catch (const std::exception& e) {
        spdlog::error("Server error: {}", e.what());
        return 1;
    }

    return 0;
}
