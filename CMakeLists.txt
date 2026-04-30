cmake_minimum_required(VERSION 3.16)
project(ur_controller
    VERSION 0.1.0
    DESCRIPTION "UR Robot Controller with Trajectory Planning"
    LANGUAGES CXX
)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Werror
    -Wconversion
    -Wsign-conversion
)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------

# Eigen3 - Linear algebra
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ur_rtde - UR robot communication
find_package(ur_rtde REQUIRED)

# yaml-cpp - Configuration parsing
find_package(yaml-cpp REQUIRED)

# spdlog - Logging
find_package(spdlog REQUIRED)

# FCL - Collision detection
find_package(fcl REQUIRED)

# Ruckig - Trajectory generation
find_package(ruckig REQUIRED)

# CLI11 - Command line parsing
find_package(CLI11 REQUIRED)

# Catch2 - Testing (optional)
find_package(Catch2 3 QUIET)

# FetchContent for downloading dependencies
include(FetchContent)

# --------------------------------------------------------------------------
# GUI Dependencies (optional)
# --------------------------------------------------------------------------

option(BUILD_GUI "Build the GUI application" ON)

if(BUILD_GUI)
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
    find_package(GLEW REQUIRED)
    find_package(assimp REQUIRED)

    # Fetch Dear ImGui (docking branch for docking support)
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG docking
    )
    FetchContent_MakeAvailable(imgui)

    # Create ImGui library
    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )
    target_link_libraries(imgui PUBLIC
        glfw
        GLEW::GLEW
        OpenGL::GL
    )
    # Suppress warnings in third-party code
    target_compile_options(imgui PRIVATE -w)
endif()

# --------------------------------------------------------------------------
# Library
# --------------------------------------------------------------------------

# Kinematics library
add_library(ur_kinematics
    src/kinematics/dh_parameters.cpp
    src/kinematics/ur_kinematics.cpp
)
target_include_directories(ur_kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ur_kinematics PUBLIC
    Eigen3::Eigen
)

# Trajectory library
add_library(ur_trajectory
    src/trajectory/types.cpp
    src/trajectory/validator.cpp
    src/trajectory/planner.cpp
    src/trajectory/axis_blend_planner.cpp
    src/trajectory/executor.cpp
    src/trajectory/path_geometry.cpp
    src/trajectory/velocity_first/planner.cpp
)
target_include_directories(ur_trajectory PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ur_trajectory PUBLIC
    ur_kinematics
    Eigen3::Eigen
    ruckig::ruckig
    spdlog::spdlog
)

# Main library (will be populated as we add modules)
add_library(ur_controller_lib INTERFACE)
target_include_directories(ur_controller_lib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ur_controller_lib INTERFACE
    ur_kinematics
    ur_trajectory
    Eigen3::Eigen
    ur_rtde::rtde
    yaml-cpp
    spdlog::spdlog
    fcl
    ruckig::ruckig
)

# --------------------------------------------------------------------------
# Applications
# --------------------------------------------------------------------------

add_subdirectory(apps)

# --------------------------------------------------------------------------
# GUI Application
# --------------------------------------------------------------------------

if(BUILD_GUI)
    add_subdirectory(gui)
endif()

# --------------------------------------------------------------------------
# Web UI Server (optional)
# --------------------------------------------------------------------------

option(BUILD_WEBUI "Build the web UI server" ON)

if(BUILD_WEBUI)
    # Crow - header-only C++ web framework with WebSocket support
    FetchContent_Declare(
        crow
        GIT_REPOSITORY https://github.com/CrowCpp/Crow.git
        GIT_TAG v1.0+5
    )
    FetchContent_MakeAvailable(crow)

    add_subdirectory(webui)
endif()

# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------

if(Catch2_FOUND)
    enable_testing()
    add_subdirectory(test)
else()
    message(STATUS "Catch2 not found - tests disabled")
endif()

# --------------------------------------------------------------------------
# Installation
# --------------------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
