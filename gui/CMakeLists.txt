# GUI Application

add_executable(ur_controller_gui
    main.cpp
    robot_viewer.cpp
    mesh_loader.cpp
)

target_link_libraries(ur_controller_gui PRIVATE
    ur_controller_lib
    imgui
    CLI11::CLI11
    assimp::assimp
)

target_include_directories(ur_controller_gui PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
