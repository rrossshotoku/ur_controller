/// @file robot_viewer.cpp
/// @brief Implementation of 3D robot viewer with framebuffer rendering

#include "robot_viewer.hpp"

#include <imgui.h>
#include <spdlog/spdlog.h>
#include <cmath>
#include <algorithm>

namespace ur_controller {

namespace {
constexpr float kPi = 3.14159265358979323846f;
}  // namespace

RobotViewer::RobotViewer() : mesh_loader_(std::make_unique<MeshLoader>()) {}

bool RobotViewer::loadMeshes(const std::string& model, const std::string& mesh_path) {
    if (mesh_loader_->loadRobotMeshes(model, mesh_path)) {
        use_meshes_ = true;
        spdlog::info("Robot meshes loaded successfully");
        return true;
    }
    spdlog::warn("Failed to load robot meshes, using primitives");
    use_meshes_ = false;
    return false;
}

RobotViewer::~RobotViewer() {
    if (framebuffer_ != 0) {
        glDeleteFramebuffers(1, &framebuffer_);
    }
    if (texture_ != 0) {
        glDeleteTextures(1, &texture_);
    }
    if (depth_renderbuffer_ != 0) {
        glDeleteRenderbuffers(1, &depth_renderbuffer_);
    }
}

void RobotViewer::initializeGL() {
    // Create framebuffer
    glGenFramebuffers(1, &framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

    // Create texture for color attachment
    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, fb_width_, fb_height_, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_, 0);

    // Create renderbuffer for depth
    glGenRenderbuffers(1, &depth_renderbuffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_renderbuffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, fb_width_, fb_height_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_renderbuffer_);

    // Check framebuffer completeness
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        // Handle error
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    initialized_ = true;
}

void RobotViewer::resizeFramebuffer(int width, int height) {
    if (width == fb_width_ && height == fb_height_) {
        return;
    }

    fb_width_ = width;
    fb_height_ = height;

    // Resize texture
    glBindTexture(GL_TEXTURE_2D, texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, fb_width_, fb_height_, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);

    // Resize depth buffer
    glBindRenderbuffer(GL_RENDERBUFFER, depth_renderbuffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, fb_width_, fb_height_);
}

void RobotViewer::render(const std::array<double, 6>& joint_positions) {
    ImGui::Begin("3D Robot View");

    // Camera controls
    ImGui::SliderFloat("Distance", &camera_distance_, 0.5f, 3.0f);
    ImGui::SliderFloat("Yaw", &camera_yaw_, -180.0f, 180.0f);
    ImGui::SliderFloat("Pitch", &camera_pitch_, -89.0f, 89.0f);

    // Get available size for rendering
    ImVec2 availSize = ImGui::GetContentRegionAvail();
    int width = std::max(100, static_cast<int>(availSize.x));
    int height = std::max(100, static_cast<int>(availSize.y - 10));

    // Initialize or resize framebuffer
    if (!initialized_) {
        fb_width_ = width;
        fb_height_ = height;
        initializeGL();
    } else if (width != fb_width_ || height != fb_height_) {
        resizeFramebuffer(width, height);
    }

    // Render scene to framebuffer
    renderScene(joint_positions);

    // Display the texture in ImGui
    ImGui::Image(
        static_cast<ImTextureID>(texture_),
        ImVec2(static_cast<float>(fb_width_), static_cast<float>(fb_height_)),
        ImVec2(0, 1),  // UV flipped for OpenGL
        ImVec2(1, 0)
    );

    ImGui::End();
}

void RobotViewer::renderScene(const std::array<double, 6>& joint_positions) {
    // Bind framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    glViewport(0, 0, fb_width_, fb_height_);

    // Clear
    glClearColor(0.15f, 0.15f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Set up projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = static_cast<float>(fb_width_) / static_cast<float>(fb_height_);
    float fov = 45.0f;
    float near_plane = 0.1f;
    float far_plane = 100.0f;
    float top = near_plane * std::tan(fov * kPi / 360.0f);
    float right = top * aspect;
    glFrustum(-right, right, -top, top, near_plane, far_plane);

    // Set up modelview matrix with camera
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Camera position from spherical coordinates
    float yaw_rad = camera_yaw_ * kPi / 180.0f;
    float pitch_rad = camera_pitch_ * kPi / 180.0f;
    float cam_x = camera_distance_ * std::cos(pitch_rad) * std::sin(yaw_rad);
    float cam_y = camera_distance_ * std::cos(pitch_rad) * std::cos(yaw_rad);
    float cam_z = camera_distance_ * std::sin(pitch_rad) + 0.4f;  // Look at robot center

    // Simple look-at (looking at origin with Z up)
    float fx = -cam_x, fy = -cam_y, fz = 0.4f - cam_z;
    float len = std::sqrt(fx*fx + fy*fy + fz*fz);
    if (len > 0.0001f) {
        fx /= len; fy /= len; fz /= len;
    }

    // Right vector (forward cross up, assuming Z is up)
    float rx = fy * 1.0f - fz * 0.0f;  // forward cross (0,0,1)
    float ry = fz * 0.0f - fx * 1.0f;
    float rz = fx * 0.0f - fy * 0.0f;
    len = std::sqrt(rx*rx + ry*ry + rz*rz);
    if (len > 0.0001f) {
        rx /= len; ry /= len; rz /= len;
    }

    // Recompute up (right cross forward)
    float ux = ry * fz - rz * fy;
    float uy = rz * fx - rx * fz;
    float uz = rx * fy - ry * fx;

    // Build view matrix
    GLfloat view[16] = {
        rx, ux, -fx, 0.0f,
        ry, uy, -fy, 0.0f,
        rz, uz, -fz, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    glMultMatrixf(view);
    glTranslatef(-cam_x, -cam_y, -cam_z);

    // Enable lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    GLfloat light_pos[] = {1.0f, 1.0f, 2.0f, 0.0f};
    GLfloat light_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat light_diffuse[] = {0.7f, 0.7f, 0.7f, 1.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);

    // Render scene elements
    renderGrid();
    renderAxes();
    renderRobot(joint_positions);

    // Disable lighting for next frame
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    // Unbind framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void RobotViewer::renderGrid() {
    glDisable(GL_LIGHTING);
    glColor3f(0.4f, 0.4f, 0.4f);
    glBegin(GL_LINES);
    for (int i = -10; i <= 10; ++i) {
        float p = static_cast<float>(i) * 0.1f;
        glVertex3f(p, -1.0f, 0.0f);
        glVertex3f(p, 1.0f, 0.0f);
        glVertex3f(-1.0f, p, 0.0f);
        glVertex3f(1.0f, p, 0.0f);
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

void RobotViewer::renderAxes() {
    glDisable(GL_LIGHTING);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    // X axis - Red
    glColor3f(1.0f, 0.2f, 0.2f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.15f, 0.0f, 0.0f);
    // Y axis - Green
    glColor3f(0.2f, 1.0f, 0.2f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.15f, 0.0f);
    // Z axis - Blue
    glColor3f(0.2f, 0.2f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.15f);
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void RobotViewer::drawCylinder(float radius, float height, int segments) {
    // Draw sides
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(segments);
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);
        float nx = std::cos(angle);
        float ny = std::sin(angle);
        glNormal3f(nx, ny, 0.0f);
        glVertex3f(x, y, 0.0f);
        glVertex3f(x, y, height);
    }
    glEnd();

    // Draw top cap
    glNormal3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0.0f, 0.0f, height);
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(segments);
        glVertex3f(radius * std::cos(angle), radius * std::sin(angle), height);
    }
    glEnd();

    // Draw bottom cap
    glNormal3f(0.0f, 0.0f, -1.0f);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0.0f, 0.0f, 0.0f);
    for (int i = segments; i >= 0; --i) {
        float angle = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(segments);
        glVertex3f(radius * std::cos(angle), radius * std::sin(angle), 0.0f);
    }
    glEnd();
}

void RobotViewer::drawBox(float sx, float sy, float sz) {
    float hx = sx / 2.0f;
    float hy = sy / 2.0f;
    float hz = sz / 2.0f;

    glBegin(GL_QUADS);
    // Front (+Z)
    glNormal3f(0, 0, 1);
    glVertex3f(-hx, -hy, hz);
    glVertex3f(hx, -hy, hz);
    glVertex3f(hx, hy, hz);
    glVertex3f(-hx, hy, hz);
    // Back (-Z)
    glNormal3f(0, 0, -1);
    glVertex3f(-hx, -hy, -hz);
    glVertex3f(-hx, hy, -hz);
    glVertex3f(hx, hy, -hz);
    glVertex3f(hx, -hy, -hz);
    // Top (+Y)
    glNormal3f(0, 1, 0);
    glVertex3f(-hx, hy, -hz);
    glVertex3f(-hx, hy, hz);
    glVertex3f(hx, hy, hz);
    glVertex3f(hx, hy, -hz);
    // Bottom (-Y)
    glNormal3f(0, -1, 0);
    glVertex3f(-hx, -hy, -hz);
    glVertex3f(hx, -hy, -hz);
    glVertex3f(hx, -hy, hz);
    glVertex3f(-hx, -hy, hz);
    // Right (+X)
    glNormal3f(1, 0, 0);
    glVertex3f(hx, -hy, -hz);
    glVertex3f(hx, hy, -hz);
    glVertex3f(hx, hy, hz);
    glVertex3f(hx, -hy, hz);
    // Left (-X)
    glNormal3f(-1, 0, 0);
    glVertex3f(-hx, -hy, -hz);
    glVertex3f(-hx, -hy, hz);
    glVertex3f(-hx, hy, hz);
    glVertex3f(-hx, hy, -hz);
    glEnd();
}

void RobotViewer::renderRobot(const std::array<double, 6>& joint_positions) {
    // Always use primitives for now - mesh transforms need more work
    renderRobotPrimitives(joint_positions);
}

void RobotViewer::renderRobotMeshes(const std::array<double, 6>& joint_positions) {
    // Step-by-step approach: render just base mesh first to verify
    // Then add joints one at a time

    float q1 = static_cast<float>(joint_positions[0]) * 180.0f / kPi;
    float q2 = static_cast<float>(joint_positions[1]) * 180.0f / kPi;
    float q3 = static_cast<float>(joint_positions[2]) * 180.0f / kPi;
    float q4 = static_cast<float>(joint_positions[3]) * 180.0f / kPi;
    float q5 = static_cast<float>(joint_positions[4]) * 180.0f / kPi;
    float q6 = static_cast<float>(joint_positions[5]) * 180.0f / kPi;

    glEnable(GL_LIGHTING);
    glColor3f(0.75f, 0.75f, 0.78f);

    glPushMatrix();

    // =========== BASE ===========
    // base_link -> base_link_inertia has rpy=(0, 0, pi)
    // base visual has yaw=180 degrees
    // Combined: both rotate 180 around Z, which cancels out? No, they stack.
    // Actually let's just render with the visual origin only first
    glPushMatrix();
    // Visual origin: yaw=180 (from visual_parameters.yaml)
    glRotatef(180.0f, 0.0f, 0.0f, 1.0f);
    mesh_loader_->renderLink(0);
    glPopMatrix();

    // =========== SHOULDER ===========
    // shoulder_pan_joint origin: xyz=(0, 0, 0.1625), rpy=(0, 0, 0)
    // But parent is base_link_inertia which has rpy=(0,0,pi) from base_link
    // So we need to account for that pi rotation
    glRotatef(180.0f, 0.0f, 0.0f, 1.0f);  // base_link -> base_link_inertia transform
    glTranslatef(0.0f, 0.0f, 0.1625f);
    glRotatef(q1, 0.0f, 0.0f, 1.0f);

    // shoulder visual origin: yaw=180
    glPushMatrix();
    glRotatef(180.0f, 0.0f, 0.0f, 1.0f);
    mesh_loader_->renderLink(1);
    glPopMatrix();

    // =========== UPPER ARM ===========
    // shoulder_lift_joint origin: xyz=(0, 0, 0), rpy=(pi/2, 0, 0) = roll 90
    // In OpenGL, for RPY we apply: yaw, pitch, roll (reverse order)
    // rpy=(pi/2, 0, 0) means roll=90, pitch=0, yaw=0
    // So: glRotate(0, Z), glRotate(0, Y), glRotate(90, X)
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);  // roll
    glRotatef(q2, 0.0f, 0.0f, 1.0f);     // joint rotation around Z

    // upper_arm visual origin: xyz=(0, 0, 0.138), rpy=(90, 0, -90) degrees
    // roll=90, pitch=0, yaw=-90
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 0.138f);
    glRotatef(-90.0f, 0.0f, 0.0f, 1.0f);  // yaw
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);   // roll
    mesh_loader_->renderLink(2);
    glPopMatrix();

    // =========== FOREARM ===========
    // elbow_joint origin: xyz=(-0.425, 0, 0), rpy=(0, 0, 0)
    glTranslatef(-0.425f, 0.0f, 0.0f);
    glRotatef(q3, 0.0f, 0.0f, 1.0f);

    // forearm visual origin: xyz=(0, 0, 0.007), rpy=(90, 0, -90) degrees
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, 0.007f);
    glRotatef(-90.0f, 0.0f, 0.0f, 1.0f);  // yaw
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);   // roll
    mesh_loader_->renderLink(3);
    glPopMatrix();

    // =========== WRIST 1 ===========
    // wrist_1_joint origin: xyz=(-0.3922, 0, 0.1333), rpy=(0, 0, 0)
    glTranslatef(-0.3922f, 0.0f, 0.1333f);
    glRotatef(q4, 0.0f, 0.0f, 1.0f);

    // wrist_1 visual origin: xyz=(0, 0, -0.127), rpy=(90, 0, 0) degrees
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, -0.127f);
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);   // roll
    mesh_loader_->renderLink(4);
    glPopMatrix();

    // =========== WRIST 2 ===========
    // wrist_2_joint origin: xyz=(0, -0.0997, 0), rpy=(pi/2, 0, 0)
    glTranslatef(0.0f, -0.0997f, 0.0f);
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);   // roll
    glRotatef(q5, 0.0f, 0.0f, 1.0f);

    // wrist_2 visual origin: xyz=(0, 0, -0.0997), rpy=(0, 0, 0)
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, -0.0997f);
    mesh_loader_->renderLink(5);
    glPopMatrix();

    // =========== WRIST 3 ===========
    // wrist_3_joint origin: xyz=(0, 0.0996, 0), rpy=(pi/2, pi, pi)
    // roll=pi/2=90, pitch=pi=180, yaw=pi=180
    glTranslatef(0.0f, 0.0996f, 0.0f);
    glRotatef(180.0f, 0.0f, 0.0f, 1.0f);  // yaw
    glRotatef(180.0f, 0.0f, 1.0f, 0.0f);  // pitch
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);   // roll
    glRotatef(q6, 0.0f, 0.0f, 1.0f);

    // wrist_3 visual origin: xyz=(0, 0, -0.0989), rpy=(90, 0, 0) degrees
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, -0.0989f);
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);   // roll
    mesh_loader_->renderLink(6);
    glPopMatrix();

    glPopMatrix();
}

void RobotViewer::renderRobotPrimitives(const std::array<double, 6>& joint_positions) {
    // UR5e DH parameters (modified DH convention)
    // Link lengths
    constexpr float d1 = 0.1625f;   // Base to shoulder along Z
    constexpr float a2 = 0.425f;    // Upper arm length (along X in DH)
    constexpr float a3 = 0.3922f;   // Forearm length (along X in DH)
    constexpr float d4 = 0.1333f;   // Shoulder to wrist1 offset
    constexpr float d5 = 0.0997f;   // Wrist1 to wrist2
    constexpr float d6 = 0.0996f;   // Wrist2 to flange

    constexpr float joint_radius = 0.05f;
    constexpr float link_radius = 0.04f;

    // Convert joint angles to degrees
    float q1 = static_cast<float>(joint_positions[0]) * 180.0f / kPi;
    float q2 = static_cast<float>(joint_positions[1]) * 180.0f / kPi;
    float q3 = static_cast<float>(joint_positions[2]) * 180.0f / kPi;
    float q4 = static_cast<float>(joint_positions[3]) * 180.0f / kPi;
    float q5 = static_cast<float>(joint_positions[4]) * 180.0f / kPi;
    float q6 = static_cast<float>(joint_positions[5]) * 180.0f / kPi;

    glEnable(GL_LIGHTING);
    glPushMatrix();

    // =========== Base ===========
    glColor3f(0.3f, 0.3f, 0.35f);
    drawCylinder(0.08f, d1, 24);

    // Move to shoulder height
    glTranslatef(0.0f, 0.0f, d1);

    // =========== Joint 1: Base rotation around Z ===========
    glRotatef(q1, 0.0f, 0.0f, 1.0f);

    // Shoulder joint visual (blue cylinder along Y axis)
    glColor3f(0.2f, 0.4f, 0.8f);
    glPushMatrix();
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    glTranslatef(0.0f, 0.0f, -joint_radius * 1.2f);
    drawCylinder(joint_radius, joint_radius * 2.4f, 20);
    glPopMatrix();

    // =========== Joint 2: Shoulder rotation ===========
    // UR convention: at q2=0, upper arm is horizontal
    // Rotate -90° around X to align with UR's frame 1 (arm points up at q2=-90°)
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
    // Joint 2 rotation around local Z
    glRotatef(q2, 0.0f, 0.0f, 1.0f);

    // Upper arm (orange) - extends along X in this frame
    glColor3f(0.9f, 0.6f, 0.2f);
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);  // Rotate cylinder to lie along X
    glTranslatef(0.0f, 0.0f, 0.0f);
    drawCylinder(link_radius, a2, 16);
    glPopMatrix();

    // Move along X to elbow
    glTranslatef(a2, 0.0f, 0.0f);

    // Elbow joint visual (blue)
    glColor3f(0.2f, 0.4f, 0.8f);
    glPushMatrix();
    glTranslatef(0.0f, 0.0f, -joint_radius * 0.9f);
    drawCylinder(joint_radius * 0.85f, joint_radius * 1.8f, 20);
    glPopMatrix();

    // =========== Joint 3: Elbow rotation around Z ===========
    glRotatef(q3, 0.0f, 0.0f, 1.0f);

    // Forearm (orange) - extends along X
    glColor3f(0.9f, 0.6f, 0.2f);
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    drawCylinder(link_radius * 0.85f, a3, 16);
    glPopMatrix();

    // Move along X to wrist
    glTranslatef(a3, 0.0f, 0.0f);

    // =========== Joint 4: Wrist 1 rotation around Z ===========
    glRotatef(q4, 0.0f, 0.0f, 1.0f);

    // Wrist 1 visual - cylinder offset along Z
    glColor3f(0.2f, 0.5f, 0.9f);
    glPushMatrix();
    drawCylinder(joint_radius * 0.75f, d4, 16);
    glPopMatrix();

    // Move along Z
    glTranslatef(0.0f, 0.0f, d4);

    // =========== Joint 5: Wrist 2 ===========
    // Alpha = 90° in DH, but negated for mirrored coordinate system (same reason as alpha(1))
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
    glRotatef(q5, 0.0f, 0.0f, 1.0f);

    // Wrist 2 visual
    glColor3f(0.2f, 0.5f, 0.9f);
    glPushMatrix();
    drawCylinder(joint_radius * 0.65f, d5, 16);
    glPopMatrix();

    glTranslatef(0.0f, 0.0f, d5);

    // =========== Joint 6: Wrist 3 ===========
    // Alpha = -90° in DH, but we negate because we draw links along +X instead of -X
    // (the negative a2, a3 in DH means links extend along -X, but we visualize along +X)
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    glRotatef(q6, 0.0f, 0.0f, 1.0f);

    // Tool flange (dark grey)
    glColor3f(0.3f, 0.3f, 0.35f);
    drawCylinder(joint_radius * 0.55f, d6, 16);

    // Move to tool
    glTranslatef(0.0f, 0.0f, d6);

    // Tool/camera (red box)
    glColor3f(0.8f, 0.2f, 0.2f);
    glTranslatef(0.0f, 0.0f, 0.025f);
    drawBox(0.05f, 0.05f, 0.05f);

    glPopMatrix();
}

}  // namespace ur_controller
