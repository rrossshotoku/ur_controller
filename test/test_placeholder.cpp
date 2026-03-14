/// @file test_placeholder.cpp
/// @brief Placeholder test to verify test framework is working

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

TEST_CASE("Test framework is working", "[placeholder]") {
    REQUIRE(1 + 1 == 2);
}

TEST_CASE("Catch2 Approx works for floating point", "[placeholder]") {
    double pi = 3.14159265358979323846;
    REQUIRE(pi == Catch::Approx(3.14159).margin(0.001));
}
