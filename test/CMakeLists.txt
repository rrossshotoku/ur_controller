# Tests

include(Catch)

# Placeholder - tests will be added as modules are implemented
# Example structure:
#
# add_executable(test_kinematics test_kinematics.cpp)
# target_link_libraries(test_kinematics PRIVATE
#     ur_controller_lib
#     Catch2::Catch2WithMain
# )
# catch_discover_tests(test_kinematics)

# For now, create a minimal test to verify the test framework works
add_executable(test_placeholder test_placeholder.cpp)
target_link_libraries(test_placeholder PRIVATE
    Catch2::Catch2WithMain
)
catch_discover_tests(test_placeholder)
