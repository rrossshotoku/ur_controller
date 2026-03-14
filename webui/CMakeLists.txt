# Web UI Server CMake configuration

# Source files
set(WEBUI_SOURCES
    ${CMAKE_SOURCE_DIR}/src/webui/robot_state_manager.cpp
    ${CMAKE_SOURCE_DIR}/src/webui/websocket_broadcaster.cpp
    ${CMAKE_SOURCE_DIR}/src/webui/web_server.cpp
)

# Web server executable
add_executable(ur_web_server
    ${CMAKE_SOURCE_DIR}/apps/web_server_main.cpp
    ${WEBUI_SOURCES}
)

# Crow uses Boost.Asio for networking - we need to find Boost components
find_package(Boost REQUIRED COMPONENTS system)

target_link_libraries(ur_web_server PRIVATE
    ur_controller_lib
    Crow::Crow
    Boost::system
    CLI11::CLI11
    pthread
)

target_include_directories(ur_web_server PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Suppress warnings in third-party Crow headers
target_compile_options(ur_web_server PRIVATE
    -Wno-conversion
    -Wno-sign-conversion
)

# Copy static assets to build directory
add_custom_command(
    TARGET ur_web_server POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_BINARY_DIR}/webui
    COMMENT "Copying web UI static files"
)

# Copy robot meshes to webui static directory
add_custom_command(
    TARGET ur_web_server POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/meshes/ur_description
        ${CMAKE_BINARY_DIR}/webui/static/urdf/ur_description
    COMMENT "Copying robot meshes for 3D visualization"
)
