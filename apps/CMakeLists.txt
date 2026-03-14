# Applications

# Connection test - verify URSim connectivity
add_executable(connection_test connection_test.cpp)
target_link_libraries(connection_test PRIVATE
    ur_controller_lib
    CLI11::CLI11
)

# Future applications will be added here:
# - trajectory_runner
# - teach_mode
