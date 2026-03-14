/// @file mesh_loader.cpp
/// @brief Implementation of mesh loading using Assimp

#include "mesh_loader.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <spdlog/spdlog.h>

namespace ur_controller {

void Mesh::upload() {
    if (uploaded || vertices.empty()) {
        return;
    }

    // Generate VAO
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // Upload vertices
    glGenBuffers(1, &vbo_vertices);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
                 vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);

    // Upload normals
    if (!normals.empty()) {
        glGenBuffers(1, &vbo_normals);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_normals);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(normals.size() * sizeof(float)),
                     normals.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
    }

    // Upload indices
    if (!indices.empty()) {
        glGenBuffers(1, &ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
                     indices.data(), GL_STATIC_DRAW);
    }

    glBindVertexArray(0);
    uploaded = true;
}

void Mesh::render() const {
    if (vertices.empty()) {
        return;
    }

    // Use legacy OpenGL for compatibility with our current setup
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    glVertexPointer(3, GL_FLOAT, 0, vertices.data());
    if (!normals.empty()) {
        glNormalPointer(GL_FLOAT, 0, normals.data());
    }

    if (!indices.empty()) {
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(indices.size()),
                       GL_UNSIGNED_INT,
                       indices.data());
    } else {
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices.size() / 3));
    }

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
}

void Mesh::cleanup() {
    if (vao != 0) {
        glDeleteVertexArrays(1, &vao);
        vao = 0;
    }
    if (vbo_vertices != 0) {
        glDeleteBuffers(1, &vbo_vertices);
        vbo_vertices = 0;
    }
    if (vbo_normals != 0) {
        glDeleteBuffers(1, &vbo_normals);
        vbo_normals = 0;
    }
    if (ebo != 0) {
        glDeleteBuffers(1, &ebo);
        ebo = 0;
    }
    uploaded = false;
}

MeshLoader::MeshLoader() = default;

MeshLoader::~MeshLoader() {
    for (auto& mesh : meshes_) {
        mesh.cleanup();
    }
}

bool MeshLoader::loadRobotMeshes(const std::string& model, const std::string& mesh_base_path) {
    const std::array<std::string, 7> link_names = {
        "base", "shoulder", "upperarm", "forearm", "wrist1", "wrist2", "wrist3"
    };

    bool all_loaded = true;
    for (size_t i = 0; i < link_names.size(); ++i) {
        std::string filepath = mesh_base_path + "/" + model + "/visual/" + link_names[i] + ".dae";
        if (!loadMesh(filepath, meshes_[i])) {
            spdlog::error("Failed to load mesh: {}", filepath);
            all_loaded = false;
        } else {
            spdlog::info("Loaded mesh: {} ({} vertices, {} triangles)",
                         link_names[i],
                         meshes_[i].vertices.size() / 3,
                         meshes_[i].indices.size() / 3);
        }
    }

    loaded_ = all_loaded;
    return all_loaded;
}

bool MeshLoader::loadMesh(const std::string& filepath, Mesh& mesh) {
    Assimp::Importer importer;

    const aiScene* scene = importer.ReadFile(filepath,
        aiProcess_Triangulate |
        aiProcess_GenSmoothNormals |
        aiProcess_JoinIdenticalVertices |
        aiProcess_OptimizeMeshes);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        spdlog::error("Assimp error: {}", importer.GetErrorString());
        return false;
    }

    // Process all meshes in the scene
    mesh.vertices.clear();
    mesh.normals.clear();
    mesh.indices.clear();

    std::function<void(aiNode*, const aiMatrix4x4&)> processNode;
    processNode = [&](aiNode* node, const aiMatrix4x4& parent_transform) {
        aiMatrix4x4 transform = parent_transform * node->mTransformation;

        // Process meshes in this node
        for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
            aiMesh* ai_mesh = scene->mMeshes[node->mMeshes[i]];

            unsigned int vertex_offset = static_cast<unsigned int>(mesh.vertices.size() / 3);

            // Extract vertices and normals
            for (unsigned int j = 0; j < ai_mesh->mNumVertices; ++j) {
                // Transform vertex by node transform
                aiVector3D v = ai_mesh->mVertices[j];
                aiVector3D transformed = transform * v;

                mesh.vertices.push_back(transformed.x);
                mesh.vertices.push_back(transformed.y);
                mesh.vertices.push_back(transformed.z);

                if (ai_mesh->HasNormals()) {
                    // Transform normal (rotation only, no translation)
                    aiMatrix3x3 normal_matrix(transform);
                    aiVector3D n = normal_matrix * ai_mesh->mNormals[j];
                    n.Normalize();

                    mesh.normals.push_back(n.x);
                    mesh.normals.push_back(n.y);
                    mesh.normals.push_back(n.z);
                }
            }

            // Extract indices
            for (unsigned int j = 0; j < ai_mesh->mNumFaces; ++j) {
                aiFace& face = ai_mesh->mFaces[j];
                for (unsigned int k = 0; k < face.mNumIndices; ++k) {
                    mesh.indices.push_back(vertex_offset + face.mIndices[k]);
                }
            }
        }

        // Process child nodes
        for (unsigned int i = 0; i < node->mNumChildren; ++i) {
            processNode(node->mChildren[i], transform);
        }
    };

    processNode(scene->mRootNode, aiMatrix4x4());

    return !mesh.vertices.empty();
}

const Mesh* MeshLoader::getMesh(size_t link_index) const {
    if (link_index < meshes_.size()) {
        return &meshes_[link_index];
    }
    return nullptr;
}

void MeshLoader::renderLink(size_t link_index) const {
    if (link_index < meshes_.size()) {
        meshes_[link_index].render();
    }
}

}  // namespace ur_controller
