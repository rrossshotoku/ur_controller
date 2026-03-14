#pragma once

/// @file robot_viewer.hpp
/// @brief 3D robot visualization using OpenGL with framebuffer rendering

#include "mesh_loader.hpp"

#include <array>
#include <memory>
#include <string>
#include <GL/glew.h>

namespace ur_controller {

/// @brief 3D viewer for robot visualization
///
/// Renders the UR robot to a framebuffer using loaded meshes,
/// then displays it as an ImGui image.
class RobotViewer {
public:
    RobotViewer();
    ~RobotViewer();

    // Non-copyable
    RobotViewer(const RobotViewer&) = delete;
    RobotViewer& operator=(const RobotViewer&) = delete;

    /// @brief Load robot meshes
    /// @param model Robot model name (e.g., "ur5e")
    /// @param mesh_path Path to mesh directory
    /// @return true if meshes loaded successfully
    bool loadMeshes(const std::string& model, const std::string& mesh_path);

    /// @brief Render the robot in an ImGui window
    /// @param joint_positions Current joint angles in radians
    void render(const std::array<double, 6>& joint_positions);

private:
    // Mesh loader
    std::unique_ptr<MeshLoader> mesh_loader_;
    bool use_meshes_ = false;

    // Framebuffer for offscreen rendering
    GLuint framebuffer_ = 0;
    GLuint texture_ = 0;
    GLuint depth_renderbuffer_ = 0;
    int fb_width_ = 800;
    int fb_height_ = 600;

    // Camera state
    float camera_distance_ = 1.5f;
    float camera_yaw_ = 45.0f;      // Degrees
    float camera_pitch_ = 25.0f;    // Degrees

    // Rendering state
    bool initialized_ = false;

    void initializeGL();
    void resizeFramebuffer(int width, int height);
    void renderScene(const std::array<double, 6>& joint_positions);
    void renderRobot(const std::array<double, 6>& joint_positions);
    void renderRobotMeshes(const std::array<double, 6>& joint_positions);
    void renderRobotPrimitives(const std::array<double, 6>& joint_positions);
    void renderGrid();
    void renderAxes();
    void drawCylinder(float radius, float height, int segments = 16);
    void drawBox(float sx, float sy, float sz);
};

}  // namespace ur_controller
