# Tests

include(Catch)

# Kinematics tests
add_executable(test_kinematics test_kinematics.cpp)
target_link_libraries(test_kinematics PRIVATE
    ur_kinematics
    Catch2::Catch2WithMain
)
catch_discover_tests(test_kinematics)

# Trajectory tests
add_executable(test_trajectory test_trajectory.cpp)
target_link_libraries(test_trajectory PRIVATE
    ur_trajectory
    ur_kinematics
    Catch2::Catch2WithMain
)
catch_discover_tests(test_trajectory)

# Placeholder test to verify the test framework works
add_executable(test_placeholder test_placeholder.cpp)
target_link_libraries(test_placeholder PRIVATE
    Catch2::Catch2WithMain
)
catch_discover_tests(test_placeholder)
