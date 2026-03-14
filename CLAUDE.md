# UR Controller - Project Guidelines

## Project Overview

C++ application for controlling Universal Robots manipulators via RTDE. Runs on Raspberry Pi with real-time constraints. Features time-parameterised Cartesian trajectories, collision monitoring, and trajectory blending.

**Key documentation:** `docs/architecture.md`

---

## Build Commands

```bash
# Development build (in Docker)
docker compose -f docker/docker-compose.yml exec dev bash
cd /workspace && mkdir -p build && cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Debug
ninja

# Run tests
ctest --output-on-failure

# Run specific test
./test/test_kinematics

# Cross-compile for RPi
docker compose --profile cross run cross-arm64 bash -c "
    cd /workspace/build-arm64 && cmake .. -GNinja \
    -DCMAKE_TOOLCHAIN_FILE=/workspace/cmake/toolchain-arm64.cmake \
    && ninja
"

# Format code
find src include -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i

# Static analysis
clang-tidy src/**/*.cpp -- -I include
```

---

## C++ Coding Standards

### Language Version
- **C++17** minimum (C++20 where beneficial)
- Compile with `-Wall -Wextra -Wpedantic -Werror`

### Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| Classes/Structs | PascalCase | `TrajectoryBuilder`, `KeepOutZone` |
| Functions/Methods | camelCase | `computeBlend()`, `getActualQ()` |
| Variables | snake_case | `blend_radius`, `current_pose` |
| Member variables | snake_case with trailing underscore | `kinematics_`, `zones_` |
| Constants | kPascalCase | `kMaxJointVelocity`, `kDefaultBlendRadius` |
| Enum values | PascalCase | `ZoneStatus::Violation` |
| Namespaces | lowercase | `ur_controller`, `trajectory` |
| Files | snake_case | `ur_kinematics.hpp`, `collision_monitor.cpp` |
| Template params | PascalCase | `template<typename Scalar>` |

### File Organisation

```
// header_file.hpp
#pragma once

#include <system_headers>      // Standard library first

#include <third_party_headers> // Third-party libraries

#include "project/headers.hpp" // Project headers last

namespace ur_controller {
namespace subsystem {

// Forward declarations first
class ForwardDeclared;

// Constants
constexpr double kDefaultValue = 1.0;

// Type aliases
using JointVector = Eigen::Matrix<double, 6, 1>;

// Class declaration
class MyClass {
public:
    // Constructors, destructor, assignment
    // Public methods
    // Public static methods

protected:
    // Protected methods

private:
    // Private methods
    // Private static methods
    // Member variables (with trailing underscore)
};

}  // namespace subsystem
}  // namespace ur_controller
```

### Class Design

```cpp
class Example {
public:
    // Rule of 5: If you define any, consider all
    Example();
    ~Example();
    Example(const Example&);
    Example(Example&&) noexcept;
    Example& operator=(const Example&);
    Example& operator=(Example&&) noexcept;

    // Getters: const, no 'get' prefix for simple accessors
    [[nodiscard]] double blendRadius() const { return blend_radius_; }

    // Getters returning non-trivial: use 'get' or descriptive name
    [[nodiscard]] std::vector<JointVector> computeSolutions() const;

    // Setters: use 'set' prefix
    void setBlendRadius(double radius);

    // Methods that can fail: return std::optional, std::expected, or bool
    [[nodiscard]] std::optional<JointVector> selectBestSolution(...) const;

private:
    double blend_radius_;  // Trailing underscore for members
};
```

### Modern C++ Practices

**DO:**
```cpp
// Use auto for complex types, explicit for simple
auto it = container.find(key);
double velocity = 0.5;  // Not auto for primitives

// Use constexpr where possible
constexpr double kPi = 3.14159265358979323846;

// Use [[nodiscard]] for functions where ignoring return is likely a bug
[[nodiscard]] bool isValid() const;

// Use structured bindings
auto [position, orientation] = decomposePose(pose);

// Use std::optional for nullable returns
std::optional<JointVector> inverse(const Pose& pose) const;

// Use enum class, not plain enum
enum class ZoneStatus { Clear, Warning, Violation };

// Use nullptr, not NULL or 0
RobotInterface* robot = nullptr;

// Use range-based for loops
for (const auto& zone : zones_) { ... }

// Use emplace_back over push_back for construction
solutions.emplace_back(q1, q2, q3, q4, q5, q6);

// Use string_view for non-owning string parameters
void log(std::string_view message);
```

**DON'T:**
```cpp
// Don't use raw new/delete - use smart pointers or containers
auto* ptr = new MyClass();  // BAD
auto ptr = std::make_unique<MyClass>();  // GOOD

// Don't use C-style casts
double x = (double)intValue;  // BAD
double x = static_cast<double>(intValue);  // GOOD

// Don't use using namespace in headers
using namespace std;  // BAD in headers, discouraged in cpp

// Don't pass smart pointers unless transferring/sharing ownership
void process(std::shared_ptr<Data> data);  // BAD if just reading
void process(const Data& data);  // GOOD

// Don't use std::bind - use lambdas
auto fn = std::bind(&Class::method, this, _1);  // BAD
auto fn = [this](auto x) { return method(x); };  // GOOD
```

### Error Handling

```cpp
// Use exceptions for exceptional conditions (constructor failures, etc.)
// Use std::optional for "not found" or "no solution" cases
// Use std::expected (C++23) or custom Result<T, E> for recoverable errors

// NOT acceptable in real-time control loop:
throw std::runtime_error("...");  // No exceptions in RT loop

// Acceptable in RT loop:
if (!solution.has_value()) {
    state_ = ControllerState::Faulted;
    return;
}

// Log errors with context
spdlog::error("IK failed for pose [{}, {}, {}], segment {}",
              pose.x(), pose.y(), pose.z(), segment_index);
```

### Real-Time Code Rules

The 500Hz control loop has strict requirements:

```cpp
// IN THE CONTROL LOOP - FORBIDDEN:
// - Dynamic memory allocation (new, malloc, std::vector resize)
// - Exceptions (throw)
// - Blocking I/O
// - Unbounded loops
// - Mutex locks that might contend (use lock-free or try_lock)
// - Logging that allocates (pre-allocate log buffers)

// IN THE CONTROL LOOP - REQUIRED:
// - Pre-allocated buffers
// - Bounded iteration counts
// - Lock-free data structures for cross-thread communication
// - Time-bounded operations only

// Example: pre-allocate in constructor
class RTController {
    RTController() {
        ik_solutions_.reserve(8);  // Pre-allocate for max solutions
        command_buffer_.resize(kBufferSize);
    }

    void controlLoop() {
        ik_solutions_.clear();  // Reuse, don't reallocate
        // ...
    }

private:
    std::vector<JointVector> ik_solutions_;  // Pre-allocated
};
```

### Documentation

```cpp
/// @brief Brief description of the function.
///
/// Longer description if needed. Explain the algorithm,
/// assumptions, and any non-obvious behavior.
///
/// @param pose Target TCP pose in base frame
/// @param current_q Current joint configuration (for solution selection)
/// @return Up to 8 IK solutions, or empty if unreachable
/// @throws Never throws (real-time safe)
///
/// @note This function is called at 500Hz in the control loop.
/// @warning Solutions are not filtered for joint limits.
[[nodiscard]] std::vector<JointVector> inverse(
    const Eigen::Isometry3d& pose,
    const JointVector& current_q) const noexcept;
```

**Document:**
- All public APIs
- Non-obvious algorithms
- Performance characteristics
- Thread safety guarantees
- Real-time safety

**Don't document:**
- Obvious getters/setters
- Self-explanatory code

### Testing Requirements

- **Unit tests required** for all non-trivial functions
- **Test file naming:** `test_<module>.cpp`
- **Use Catch2** framework
- **Test naming:** Descriptive, reads like a sentence

```cpp
TEST_CASE("URKinematics forward kinematics", "[kinematics]") {
    URKinematics kin(URModel::UR5e);

    SECTION("home position returns correct TCP pose") {
        JointVector home = JointVector::Zero();
        auto pose = kin.forward(home);

        REQUIRE(pose.translation().z() == Approx(0.8915).margin(1e-4));
    }

    SECTION("inverse kinematics roundtrips with forward") {
        JointVector q_original = randomJointConfig();
        auto pose = kin.forward(q_original);
        auto solutions = kin.inverse(pose);

        REQUIRE(!solutions.empty());
        // At least one solution should match original
        bool found_match = false;
        for (const auto& q : solutions) {
            if ((q - q_original).norm() < 1e-6) {
                found_match = true;
                break;
            }
        }
        REQUIRE(found_match);
    }
}
```

### Git Commit Standards

```
<type>: <short summary>

<body - explain what and why, not how>

Co-Authored-By: Claude <noreply@anthropic.com>
```

Types: `feat`, `fix`, `refactor`, `test`, `docs`, `build`, `ci`

Examples:
- `feat: add circular blend interpolation for trajectories`
- `fix: prevent IK solution jumps near singularities`
- `refactor: extract collision primitives to separate module`
- `test: add edge case tests for blend radius validation`

---

## Architecture Principles

### Separation of Concerns

Each module has a single responsibility:
- `kinematics/` - FK, IK, Jacobian (no I/O, no state)
- `trajectory/` - Path representation and interpolation (no robot comms)
- `collision/` - Geometry and intersection tests (no trajectory knowledge)
- `control/` - Real-time loop orchestration
- `robot/` - Hardware abstraction (RTDE communication)
- `config/` - YAML parsing and validation

### Dependency Direction

```
config/ ──┐
          ▼
     ┌─────────┐
     │ control │ ◄── apps/
     └─────────┘
      │  │  │
      ▼  ▼  ▼
kinematics/  trajectory/  collision/
      │          │            │
      └──────────┴────────────┘
                 │
                 ▼
            robot/ (interface)
                 │
         ┌───────┴───────┐
         ▼               ▼
    ur_robot.cpp    mock_robot.cpp
```

- Lower modules don't depend on higher modules
- `control` orchestrates but doesn't implement algorithms
- `robot/` interface allows testing without hardware

### Interface Design

- Define abstract interfaces for testability
- Prefer composition over inheritance
- Use dependency injection

```cpp
// Good: injectable dependency
class RTController {
public:
    RTController(
        std::unique_ptr<RobotInterface> robot,  // Injected
        const URKinematics& kinematics,
        CollisionMonitor& collision_monitor);
};

// Bad: hard-coded dependency
class RTController {
public:
    RTController() : robot_(std::make_unique<URRobot>()) {}  // Not testable
};
```

---

## Performance Guidelines

### Eigen Usage

```cpp
// Use fixed-size matrices where dimensions are known
using JointVector = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Avoid temporary allocations
Eigen::Vector3d result;
result.noalias() = A * B * v;  // noalias when no aliasing

// Use Ref for function parameters to avoid copies
void process(Eigen::Ref<const Eigen::VectorXd> input);
```

### Avoid Premature Optimisation

- Write clear code first
- Profile before optimising
- Document performance-critical sections
- Benchmark with realistic data

---

## Common Patterns in This Project

### Factory for Polymorphic Creation

```cpp
std::unique_ptr<KeepOutZone> createZone(const YAML::Node& node) {
    std::string type = node["type"].as<std::string>();
    if (type == "box") return std::make_unique<BoxZone>(node);
    if (type == "sphere") return std::make_unique<SphereZone>(node);
    throw ConfigError("Unknown zone type: " + type);
}
```

### Result Type for Fallible Operations

```cpp
template<typename T>
struct Result {
    std::optional<T> value;
    std::string error;

    bool ok() const { return value.has_value(); }
    const T& operator*() const { return *value; }
};

Result<Trajectory> loadTrajectory(const std::string& path);
```

### Pre-allocated Buffers in RT Code

```cpp
class RTController {
public:
    RTController() {
        // Pre-allocate everything needed in control loop
        ik_buffer_.reserve(8);
        collision_result_ = MonitorResult{};
    }

private:
    std::vector<JointVector> ik_buffer_;  // Reused each cycle
    MonitorResult collision_result_;       // Reused each cycle
};
```

---

## What NOT to Do

- Don't add features not in `docs/architecture.md` without discussion
- Don't introduce new dependencies without justification
- Don't break the interface contracts defined in headers
- Don't add blocking operations to the control loop
- Don't use raw pointers for ownership
- Don't ignore compiler warnings
- Don't commit code without tests
- Don't use magic numbers - define named constants
