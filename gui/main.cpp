/// @file main.cpp
/// @brief Main entry point for the UR Controller GUI application

#include "robot_viewer.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/dashboard_client.h>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <array>
#include <memory>
#include <string>
#include <chrono>
#include <thread>

namespace {

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

/// @brief GLFW error callback
void glfwErrorCallback(int error, const char* description) {
    spdlog::error("GLFW Error {}: {}", error, description);
}

/// @brief Application state
struct AppState {
    // Connection
    std::string robot_ip = "ursim";
    bool connected = false;
    std::string connection_status = "Disconnected";
    std::string robot_mode = "Unknown";
    std::string safety_status = "Unknown";

    // Robot state
    std::array<double, 6> joint_positions = {0};
    std::array<double, 6> joint_velocities = {0};
    std::array<double, 6> tcp_pose = {0};
    std::array<double, 6> tcp_speed = {0};
    double speed_scaling = 0.0;

    // RTDE interfaces
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive;
    std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control;
    std::unique_ptr<ur_rtde::DashboardClient> dashboard;

    // Joint control state
    std::array<double, 6> target_positions = {0};
    bool control_enabled = false;
    bool sliders_initialized = false;

    // UI state
    bool show_demo_window = false;
    bool show_3d_view = true;
};

/// @brief Try to connect to the robot
void connectToRobot(AppState& state) {
    spdlog::info("Connecting to robot at {}", state.robot_ip);
    state.connection_status = "Connecting...";

    try {
        // Connect to dashboard
        state.dashboard = std::make_unique<ur_rtde::DashboardClient>(state.robot_ip);
        state.dashboard->connect();

        // Connect to RTDE receive
        state.rtde_receive = std::make_unique<ur_rtde::RTDEReceiveInterface>(state.robot_ip);

        state.connected = true;
        state.sliders_initialized = false;

        // Try to connect RTDE control (may fail if remote control not enabled)
        try {
            state.rtde_control = std::make_unique<ur_rtde::RTDEControlInterface>(state.robot_ip);
            state.connection_status = "Connected (Control ready)";
            spdlog::info("Connected to robot with control interface");
        } catch (const std::exception& e) {
            state.rtde_control.reset();
            state.connection_status = "Connected (No control - enable remote)";
            spdlog::warn("Control interface not available: {} - Enable remote control on pendant", e.what());
        }
    } catch (const std::exception& e) {
        state.connected = false;
        state.connection_status = std::string("Failed: ") + e.what();
        spdlog::error("Connection failed: {}", e.what());
    }
}

/// @brief Disconnect from the robot
void disconnectFromRobot(AppState& state) {
    state.control_enabled = false;
    if (state.rtde_control) {
        state.rtde_control->stopScript();
        state.rtde_control.reset();
    }
    if (state.dashboard) {
        state.dashboard->disconnect();
        state.dashboard.reset();
    }
    state.rtde_receive.reset();
    state.connected = false;
    state.connection_status = "Disconnected";
    spdlog::info("Disconnected from robot");
}

/// @brief Update robot state from RTDE
void updateRobotState(AppState& state) {
    if (!state.connected || !state.rtde_receive) {
        return;
    }

    try {
        // Get joint positions
        auto q = state.rtde_receive->getActualQ();
        for (size_t i = 0; i < 6 && i < q.size(); ++i) {
            state.joint_positions[i] = q[i];
        }

        // Get joint velocities
        auto qd = state.rtde_receive->getActualQd();
        for (size_t i = 0; i < 6 && i < qd.size(); ++i) {
            state.joint_velocities[i] = qd[i];
        }

        // Get TCP pose
        auto tcp = state.rtde_receive->getActualTCPPose();
        for (size_t i = 0; i < 6 && i < tcp.size(); ++i) {
            state.tcp_pose[i] = tcp[i];
        }

        // Get TCP speed
        auto tcp_speed = state.rtde_receive->getActualTCPSpeed();
        for (size_t i = 0; i < 6 && i < tcp_speed.size(); ++i) {
            state.tcp_speed[i] = tcp_speed[i];
        }

        // Get speed scaling
        state.speed_scaling = state.rtde_receive->getSpeedScaling();

        // Get dashboard info (less frequently)
        static int update_counter = 0;
        if (++update_counter >= 30) {  // Every ~0.5 seconds at 60fps
            update_counter = 0;
            if (state.dashboard && state.dashboard->isConnected()) {
                state.robot_mode = state.dashboard->robotmode();
                state.safety_status = state.dashboard->safetystatus();
            }
        }
    } catch (const std::exception& e) {
        spdlog::warn("Failed to update robot state: {}", e.what());
        state.connected = false;
        state.connection_status = "Connection lost";
    }
}

/// @brief Render the connection panel
void renderConnectionPanel(AppState& state) {
    ImGui::Begin("Connection");

    // IP address input
    static char ip_buffer[64];
    strncpy(ip_buffer, state.robot_ip.c_str(), sizeof(ip_buffer) - 1);
    if (ImGui::InputText("Robot IP", ip_buffer, sizeof(ip_buffer))) {
        state.robot_ip = ip_buffer;
    }

    // Connect/Disconnect buttons
    if (!state.connected) {
        if (ImGui::Button("Connect", ImVec2(120, 0))) {
            connectToRobot(state);
        }
    } else {
        if (ImGui::Button("Disconnect", ImVec2(120, 0))) {
            disconnectFromRobot(state);
        }
    }

    ImGui::SameLine();

    // Status indicator
    if (state.connected) {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "● %s", state.connection_status.c_str());
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "● %s", state.connection_status.c_str());
    }

    ImGui::Separator();

    // Robot info
    ImGui::Text("Robot Mode: %s", state.robot_mode.c_str());
    ImGui::Text("Safety: %s", state.safety_status.c_str());
    ImGui::Text("Speed Scaling: %.1f%%", state.speed_scaling * 100.0);

    ImGui::Separator();

    // Robot control buttons
    if (state.connected && state.dashboard) {
        if (ImGui::Button("Power On")) {
            try {
                state.dashboard->powerOn();
                spdlog::info("Power on command sent");
            } catch (const std::exception& e) {
                spdlog::error("Power on failed: {}", e.what());
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Brake Release")) {
            try {
                state.dashboard->brakeRelease();
                spdlog::info("Brake release command sent");
            } catch (const std::exception& e) {
                spdlog::error("Brake release failed: {}", e.what());
            }
        }
        ImGui::SameLine();
        // Check remote control status
        try {
            bool is_remote = state.dashboard->isInRemoteControl();
            if (is_remote) {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Remote: YES");
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Remote: NO");
            }
        } catch (...) {
            ImGui::Text("Remote: ?");
        }
    }

    ImGui::End();
}

/// @brief Render the joint positions panel
void renderJointPanel(AppState& state) {
    ImGui::Begin("Joint Positions");

    const std::array<const char*, 6> joint_names = {
        "Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"
    };

    ImGui::Columns(3, "joint_columns");
    ImGui::Separator();
    ImGui::Text("Joint"); ImGui::NextColumn();
    ImGui::Text("Position (deg)"); ImGui::NextColumn();
    ImGui::Text("Velocity (deg/s)"); ImGui::NextColumn();
    ImGui::Separator();

    for (size_t i = 0; i < 6; ++i) {
        ImGui::Text("%s", joint_names[i]); ImGui::NextColumn();
        ImGui::Text("%8.2f", state.joint_positions[i] * kRadToDeg); ImGui::NextColumn();
        ImGui::Text("%8.2f", state.joint_velocities[i] * kRadToDeg); ImGui::NextColumn();
    }

    ImGui::Columns(1);
    ImGui::Separator();

    ImGui::End();
}

/// @brief Render the TCP pose panel
void renderTcpPanel(AppState& state) {
    ImGui::Begin("TCP Pose");

    ImGui::Text("Position:");
    ImGui::Text("  X: %8.4f m", state.tcp_pose[0]);
    ImGui::Text("  Y: %8.4f m", state.tcp_pose[1]);
    ImGui::Text("  Z: %8.4f m", state.tcp_pose[2]);

    ImGui::Separator();

    ImGui::Text("Orientation (axis-angle):");
    ImGui::Text("  Rx: %8.4f rad", state.tcp_pose[3]);
    ImGui::Text("  Ry: %8.4f rad", state.tcp_pose[4]);
    ImGui::Text("  Rz: %8.4f rad", state.tcp_pose[5]);

    ImGui::Separator();

    ImGui::Text("Speed:");
    ImGui::Text("  Linear:  %.4f m/s",
        std::sqrt(state.tcp_speed[0]*state.tcp_speed[0] +
                  state.tcp_speed[1]*state.tcp_speed[1] +
                  state.tcp_speed[2]*state.tcp_speed[2]));
    ImGui::Text("  Angular: %.4f rad/s",
        std::sqrt(state.tcp_speed[3]*state.tcp_speed[3] +
                  state.tcp_speed[4]*state.tcp_speed[4] +
                  state.tcp_speed[5]*state.tcp_speed[5]));

    ImGui::End();
}

/// @brief Render the joint control panel with sliders
void renderJointControlPanel(AppState& state) {
    // Set initial size and position for visibility
    ImGui::SetNextWindowSize(ImVec2(450, 350), ImGuiCond_FirstUseEver);
    ImGui::Begin("Joint Control");

    if (!state.connected) {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "Connect to robot first");
        ImGui::End();
        return;
    }

    bool control_available = (state.rtde_control != nullptr);

    if (!control_available) {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f),
            "Control not available - enable Remote Control on pendant");
        state.control_enabled = false;
    }

    // Initialize sliders to current position on first connect
    if (!state.sliders_initialized) {
        for (size_t i = 0; i < 6; ++i) {
            state.target_positions[i] = state.joint_positions[i];
        }
        state.sliders_initialized = true;
    }

    // Enable/disable control (only if control interface available)
    if (control_available) {
        if (ImGui::Checkbox("Enable Control", &state.control_enabled)) {
            if (state.control_enabled) {
                // Initialize target to current position when enabling
                for (size_t i = 0; i < 6; ++i) {
                    state.target_positions[i] = state.joint_positions[i];
                }
                spdlog::info("Joint control enabled");
            } else {
                // Stop the robot when disabling
                if (state.rtde_control) {
                    state.rtde_control->stopJ(2.0);  // Deceleration of 2 rad/s^2
                }
                spdlog::info("Joint control disabled");
            }
        }
    }

    if (!state.control_enabled) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Sliders read-only");
    }

    ImGui::Separator();

    const std::array<const char*, 6> joint_names = {
        "Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"
    };

    // Joint limits in radians (approximate for UR5e)
    constexpr double kJointMin = -6.28;  // -2*pi
    constexpr double kJointMax = 6.28;   // 2*pi

    bool target_changed = false;

    // Header row
    ImGui::Text("Joint");
    ImGui::SameLine(100);
    ImGui::Text("Actual");
    ImGui::SameLine(180);
    ImGui::Text("Target Slider");
    ImGui::SameLine(400);
    ImGui::Text("Target");

    for (size_t i = 0; i < 6; ++i) {
        ImGui::PushID(static_cast<int>(i));

        // Joint name
        ImGui::Text("%s", joint_names[i]);

        // Actual position from robot (read-only display)
        ImGui::SameLine(100);
        float actual_deg = static_cast<float>(state.joint_positions[i] * kRadToDeg);
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "%7.1f", actual_deg);

        // Target slider
        float target_rad = static_cast<float>(state.target_positions[i]);
        float min_rad = static_cast<float>(kJointMin);
        float max_rad = static_cast<float>(kJointMax);

        ImGui::SameLine(180);
        ImGui::SetNextItemWidth(200);

        if (state.control_enabled) {
            if (ImGui::SliderFloat("##target", &target_rad, min_rad, max_rad, "%.2f")) {
                state.target_positions[i] = target_rad;
                target_changed = true;
            }
        } else {
            // When disabled, show current position as read-only
            float current_rad = static_cast<float>(state.joint_positions[i]);
            ImGui::SliderFloat("##target", &current_rad, min_rad, max_rad, "%.2f");
            state.target_positions[i] = current_rad;
        }

        // Target in degrees
        ImGui::SameLine(400);
        float target_deg = static_cast<float>(state.target_positions[i] * kRadToDeg);
        ImGui::Text("%7.1f", target_deg);

        ImGui::PopID();
    }

    ImGui::Separator();

    // Snap to current position button
    if (state.control_enabled) {
        if (ImGui::Button("Snap to Current")) {
            for (size_t i = 0; i < 6; ++i) {
                state.target_positions[i] = state.joint_positions[i];
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop")) {
            if (state.rtde_control) {
                state.rtde_control->stopJ(2.0);
            }
        }
    }

    // Send servoj command if control is enabled and target changed
    if (state.control_enabled && target_changed && state.rtde_control) {
        try {
            // Convert array to vector for RTDE
            std::vector<double> target_q(state.target_positions.begin(),
                                          state.target_positions.end());

            // servoJ parameters:
            // q: target joint positions
            // velocity: not used in servo mode (0)
            // acceleration: not used in servo mode (0)
            // time: time for move (lookahead time)
            // lookahead_time: smoothing (0.03-0.2)
            // gain: proportional gain (100-2000)
            state.rtde_control->servoJ(target_q, 0, 0, 0.008, 0.1, 300);
        } catch (const std::exception& e) {
            spdlog::error("servoJ failed: {}", e.what());
            state.control_enabled = false;
        }
    }

    ImGui::End();
}

/// @brief Render the test functions panel
void renderTestPanel(AppState& state) {
    ImGui::Begin("Test Functions");

    ImGui::Text("Kinematics:");
    if (ImGui::Button("Test FK")) {
        spdlog::info("FK Test - not yet implemented");
    }
    ImGui::SameLine();
    if (ImGui::Button("Test IK")) {
        spdlog::info("IK Test - not yet implemented");
    }

    ImGui::Separator();

    ImGui::Text("Trajectory:");
    if (ImGui::Button("Load Trajectory")) {
        spdlog::info("Load Trajectory - not yet implemented");
    }
    ImGui::SameLine();
    if (ImGui::Button("Execute")) {
        spdlog::info("Execute - not yet implemented");
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop")) {
        spdlog::info("Stop - not yet implemented");
    }

    ImGui::Separator();

    ImGui::Text("Collision:");
    if (ImGui::Button("Add Keep-Out Zone")) {
        spdlog::info("Add Keep-Out Zone - not yet implemented");
    }

    ImGui::Separator();

    ImGui::Checkbox("Show ImGui Demo", &state.show_demo_window);
    ImGui::Checkbox("Show 3D View", &state.show_3d_view);

    ImGui::End();
}

}  // namespace

int main(int argc, char* argv[]) {
    // Parse command line
    CLI::App app{"UR Controller GUI"};

    std::string robot_ip = "ursim";
    app.add_option("-i,--ip", robot_ip, "Robot IP address")->default_val("ursim");

    CLI11_PARSE(app, argc, argv);

    // Initialize GLFW
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) {
        spdlog::error("Failed to initialize GLFW");
        return 1;
    }

    // GL context settings - use Compatibility Profile for legacy OpenGL support
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

    // Create window
    GLFWwindow* window = glfwCreateWindow(
        kWindowWidth, kWindowHeight,
        "UR Controller", nullptr, nullptr);

    if (!window) {
        spdlog::error("Failed to create GLFW window");
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync

    // Initialize GLEW
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        spdlog::error("Failed to initialize GLEW: {}",
            reinterpret_cast<const char*>(glewGetErrorString(err)));
        return 1;
    }

    spdlog::info("OpenGL Version: {}",
        reinterpret_cast<const char*>(glGetString(GL_VERSION)));

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Setup style
    ImGui::StyleColorsDark();

    // Setup platform/renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Application state
    AppState state;
    state.robot_ip = robot_ip;

    // Robot viewer for 3D rendering
    ur_controller::RobotViewer robot_viewer;

    // Try to load robot meshes
    robot_viewer.loadMeshes("ur5e", "/workspace/meshes/ur_description/meshes");

    spdlog::info("UR Controller GUI started");

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Update robot state
        updateRobotState(state);

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Create dockspace over entire window
        ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

        // Render panels
        renderConnectionPanel(state);
        renderJointPanel(state);
        renderTcpPanel(state);
        renderJointControlPanel(state);
        renderTestPanel(state);

        // Render 3D view
        if (state.show_3d_view) {
            robot_viewer.render(state.joint_positions);
        }

        // Show demo window if enabled
        if (state.show_demo_window) {
            ImGui::ShowDemoWindow(&state.show_demo_window);
        }

        // Render
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    disconnectFromRobot(state);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
