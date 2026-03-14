#pragma once

/// @file mesh_loader.hpp
/// @brief Mesh loading and rendering using Assimp and OpenGL

#include <GL/glew.h>
#include <string>
#include <vector>
#include <memory>
#include <array>

namespace ur_controller {

/// @brief Represents a loaded mesh with OpenGL vertex data
struct Mesh {
    std::vector<float> vertices;   // x, y, z positions
    std::vector<float> normals;    // nx, ny, nz normals
    std::vector<unsigned int> indices;

    // OpenGL handles
    GLuint vao = 0;
    GLuint vbo_vertices = 0;
    GLuint vbo_normals = 0;
    GLuint ebo = 0;
    bool uploaded = false;

    /// @brief Upload mesh data to GPU
    void upload();

    /// @brief Render the mesh using legacy OpenGL
    void render() const;

    /// @brief Clean up GPU resources
    void cleanup();
};

/// @brief Loads and manages robot meshes
class MeshLoader {
public:
    MeshLoader();
    ~MeshLoader();

    // Non-copyable
    MeshLoader(const MeshLoader&) = delete;
    MeshLoader& operator=(const MeshLoader&) = delete;

    /// @brief Load meshes for a specific robot model
    /// @param model Robot model name (e.g., "ur5e", "ur10e")
    /// @param mesh_base_path Base path to mesh directory
    /// @return true if all meshes loaded successfully
    bool loadRobotMeshes(const std::string& model, const std::string& mesh_base_path);

    /// @brief Check if meshes are loaded
    bool isLoaded() const { return loaded_; }

    /// @brief Get mesh for a specific link
    /// @param link_index 0=base, 1=shoulder, 2=upperarm, 3=forearm, 4=wrist1, 5=wrist2, 6=wrist3
    const Mesh* getMesh(size_t link_index) const;

    /// @brief Render a specific link mesh
    void renderLink(size_t link_index) const;

private:
    bool loadMesh(const std::string& filepath, Mesh& mesh);

    std::array<Mesh, 7> meshes_;  // base, shoulder, upperarm, forearm, wrist1, wrist2, wrist3
    bool loaded_ = false;
};

}  // namespace ur_controller
