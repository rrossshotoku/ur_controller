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

# Path geometry tests
add_executable(test_path_geometry test_path_geometry.cpp)
target_link_libraries(test_path_geometry PRIVATE
    ur_trajectory
    Catch2::Catch2WithMain
)
catch_discover_tests(test_path_geometry)

# S-curve profile tests
add_executable(test_s_curve test_s_curve.cpp)
target_link_libraries(test_s_curve PRIVATE
    ur_trajectory
    Catch2::Catch2WithMain
)
catch_discover_tests(test_s_curve)

# Standalone repro: joint-2 spin issue
add_executable(repro_joint_flip repro_joint_flip.cpp)
target_link_libraries(repro_joint_flip PRIVATE
    ur_trajectory
    ur_kinematics
)

# Placeholder test to verify the test framework works
add_executable(test_placeholder test_placeholder.cpp)
target_link_libraries(test_placeholder PRIVATE
    Catch2::Catch2WithMain
)
catch_discover_tests(test_placeholder)
