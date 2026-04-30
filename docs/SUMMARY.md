# UR Controller - Implementation Summary

## Overview

A C++17 application for controlling Universal Robots manipulators via RTDE with real-time trajectory planning and collision monitoring. Designed for Raspberry Pi deployment with a 500Hz control loop for camera-based inspection tasks.

---

## Architecture

```
┌─────────────────────────────────────────┐
│       Web Browser (localhost:8080)      │
│  - Trajectory planning UI               │
│  - 3D robot visualization (Three.js)    │
│  - Real-time state display              │
└─────────────────┬───────────────────────┘
                  │ WebSocket / HTTP
                  ▼
┌─────────────────────────────────────────┐
│            Web Server (C++)             │
│  - Crow HTTP framework                  │
│  - Robot state manager (50Hz)           │
│  - WebSocket broadcaster                │
└──────┬──────────────────────┬───────────┘
       │                      │
       ▼                      ▼
┌──────────────┐       ┌─────────────────┐
│  Kinematics  │       │   Trajectory    │
│  - FK/IK     │◄──────│   - Planner     │
│  - Jacobian  │       │   - Validator   │
│  - Config    │       │   - Executor    │
│    tracking  │       │   - PathGeometry│
└──────┬───────┘       └─────────────────┘
       │
       ▼ (500 Hz control loop)
┌─────────────────────────────────────────┐
│         RTDE Interface (ur_rtde)        │
└─────────────────┬───────────────────────┘
                  ▼
┌─────────────────────────────────────────┐
│           UR Robot / URSim              │
└─────────────────────────────────────────┘
```

---

## Directory Structure

```
UR controller/
├── include/ur_controller/     # Public API headers
│   ├── kinematics/            # FK/IK, Jacobian, singularity detection
│   ├── trajectory/            # Planner, executor, path geometry
│   └── webui/                 # Web server, state manager
├── src/                       # Implementation files
│   ├── kinematics/
│   ├── trajectory/
│   └── webui/
├── apps/                      # Executables
│   ├── web_server_main.cpp    # Main web server entry point
│   └── connection_test.cpp    # RTDE connectivity test
├── webui/                     # Frontend (HTML/CSS/JS)
│   ├── js/                    # JavaScript modules
│   ├── css/                   # Styles
│   └── urdf/                  # Robot models for 3D view
├── test/                      # Unit tests (Catch2)
├── docker/                    # Development environment
└── docs/                      # Documentation
```

---

## Core Modules

### 1. Kinematics Engine

**Location:** `include/ur_controller/kinematics/`, `src/kinematics/`

| Component | Purpose |
|-----------|---------|
| `ur_kinematics.hpp/cpp` | Analytical IK solver (up to 8 solutions in ~5μs) |
| `dh_parameters.hpp/cpp` | DH parameters for UR3e, UR5e, UR10e, UR16e, UR20 |
| `types.hpp` | JointVector, Jacobian, SingularityInfo types |

**Key Features:**
- Forward Kinematics: Joint angles → TCP pose (~1μs)
- Inverse Kinematics: TCP pose → up to 8 joint solutions (~5μs)
- Numerical IK: Damped least squares with seed tracking
- Configuration tracking: Maintains arm configuration to prevent jumps
- Singularity detection via Jacobian condition number

### 2. Trajectory System

**Location:** `include/ur_controller/trajectory/`, `src/trajectory/`

| Component | Purpose |
|-----------|---------|
| `planner.hpp/cpp` | Main trajectory planning from waypoints |
| `path_geometry.hpp/cpp` | Distance-parameterized path with blend arcs |
| `executor.hpp/cpp` | Executes pre-computed trajectories at 500Hz |
| `validator.hpp/cpp` | Validates against robot limits |
| `types.hpp/cpp` | Trajectory data structures |
| `s_curve.hpp/cpp` | S-curve velocity profiles |

**Trajectory Data Model:**
```cpp
Trajectory
├── config: TrajectoryConfig (limits, sample rate)
└── elements: vector<TrajectoryElement>
    ├── SetupPose (joint-space MoveJ)
    └── Sequence
        └── waypoints: vector<SequenceWaypoint>
            ├── position (x, y, z)
            ├── orientation (quaternion)
            └── blend_factor (corner rounding)
```

**Planning Pipeline:**
1. Parse waypoints from trajectory elements
2. Build PathGeometry (linear segments + blend arcs)
3. Generate velocity profile using Ruckig
4. Sample at 500Hz into PlannedTrajectory
5. Validate against limits
6. Execute via Executor

### 3. PathGeometry (Blend Arcs)

**Location:** `src/trajectory/path_geometry.cpp`

Distance-parameterized geometric path supporting:
- **Linear segments:** Straight-line TCP motion with SLERP orientation
- **Blend arcs:** Circular arcs inscribed at corners for smooth transitions

**Blend Arc Geometry:**
```
              dir_out
               ↗
    arc_end   /
         *---/
        /
       ( arc )  ← Inscribed circle tangent to both segments
        \
         *---\
    arc_start \
               \ dir_in
                ↘
```

- `blend_factor` = distance from vertex to arc apex
- Arc is tangent to both incoming and outgoing segments
- Uses Rodrigues' rotation formula for arc evaluation

### 4. Web Server

**Location:** `include/ur_controller/webui/`, `src/webui/`

| Component | Purpose |
|-----------|---------|
| `web_server.hpp/cpp` | HTTP/WebSocket server (Crow framework) |
| `robot_state_manager.hpp/cpp` | Tracks real-time robot state |
| `websocket_broadcaster.hpp/cpp` | Broadcasts state to clients |

**REST API Endpoints:**
| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/trajectory/plan2` | POST | Plan trajectory from waypoints |
| `/api/trajectory/execute` | POST | Execute planned trajectory |
| `/api/robot/state` | GET | Current robot state |
| `/api/robot/jog` | POST | Manual joint/TCP jogging |

**WebSocket Protocol:**
- Server broadcasts state at 50Hz
- Client receives JSON with joint positions, TCP pose, status

---

## Web UI

**Location:** `webui/`

### JavaScript Modules

| Module | Purpose |
|--------|---------|
| `app.js` | Main controller, WebSocket connection |
| `trajectory-panel.js` | Trajectory planning UI |
| `robot-viewer.js` | 3D visualization (Three.js) |
| `jog-controller.js` | Manual robot control |
| `control-panel.js` | Connection settings, E-stop |
| `terminal-controller.js` | Command interface |

### 3D Visualization
- Loads URDF robot models from `urdf/` directory
- Three.js rendering with orbit camera controls
- Real-time joint position updates at 50Hz
- Trajectory path visualization (green line)
- Waypoint markers

---

## Build System

### Dependencies

| Library | Purpose |
|---------|---------|
| Eigen3 | Linear algebra |
| ur_rtde | UR robot communication |
| Ruckig | Jerk-limited trajectory generation |
| Crow | HTTP/WebSocket server |
| spdlog | Logging |
| yaml-cpp | Configuration parsing |
| Catch2 | Unit testing |

### Build Commands

```bash
# Development build (in Docker)
docker compose -f docker/docker-compose.yml exec dev bash
cd /workspace && mkdir -p build && cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Debug
ninja

# Run tests
ctest --output-on-failure

# Run specific test
./test/test_path_geometry

# Start web server
./webui/ur_web_server --ip ursim
```

### Docker Environment

```yaml
Services:
  dev:    Ubuntu 22.04 with all build tools
  ursim:  UR robot simulator
```

---

## Key Algorithms

### Analytical Inverse Kinematics

6-DOF UR robots with spherical wrist have closed-form IK solution:
1. Solve for wrist center position
2. Compute θ1 (base rotation) - 2 solutions
3. Compute θ5, θ6 (wrist angles) - 2 solutions each
4. Compute θ2, θ3, θ4 (arm angles)
5. Total: up to 8 valid configurations

### Blend Arc Computation

Given three waypoints P₀, P₁, P₂ with blend_factor at P₁:
1. Compute direction vectors: dir_in = (P₁-P₀).normalized(), dir_out = (P₂-P₁).normalized()
2. Compute turning angle: θ = acos(dir_in · dir_out)
3. Compute arc radius from blend_factor and θ
4. Find arc center along angle bisector
5. Arc tangent points: arc_start = P₁ - dir_in × blend_radius, arc_end = P₁ + dir_out × blend_radius

### Arc Evaluation (Rodrigues' Rotation)

Given arc segment with center, normal, radius, arc_angle:
```cpp
// Evaluate position at parameter t ∈ [0,1]
double angle = t * arc_angle;
Vector3d r = start_pos - center;
Vector3d r_rot = r * cos(angle)
               + normal.cross(r) * sin(angle)
               + normal * (normal.dot(r)) * (1 - cos(angle));
position = center + r_rot;
```

---

## Real-Time Considerations

The 500Hz control loop has strict requirements:

**Forbidden in RT loop:**
- Dynamic memory allocation (new, malloc, vector resize)
- Exceptions (throw)
- Blocking I/O
- Unbounded loops
- Mutex contention

**Required:**
- Pre-allocated buffers
- Bounded iteration counts
- Lock-free data structures
- Time-bounded operations

---

## Testing

| Test File | Coverage |
|-----------|----------|
| `test_kinematics.cpp` | FK/IK roundtrip, configuration tracking, singularities |
| `test_trajectory.cpp` | Trajectory planning, waypoint validation |
| `test_path_geometry.cpp` | Path geometry, blend arcs, 3D diagonal paths |

Run tests:
```bash
cd build && ctest --output-on-failure
```

---

## Current Implementation Status

### Completed
- [x] Analytical IK solver with configuration tracking
- [x] Forward kinematics
- [x] Trajectory planning with waypoints
- [x] PathGeometry with linear segments
- [x] Blend arcs (inscribed circles) at waypoint corners
- [x] Web UI with 3D visualization
- [x] Real-time state broadcasting
- [x] Docker development environment

### In Progress
- [ ] Seven-segment velocity profiles (jerk-limited)
- [ ] Full trajectory execution with RTDE
- [ ] Collision monitoring

---

## Code Standards

Per `CLAUDE.md`:
- **C++17** minimum
- **Naming:** PascalCase (classes), camelCase (methods), snake_case (variables)
- **Members:** Trailing underscore (`member_`)
- **Constants:** kPascalCase
- **Documentation:** Doxygen-style for public APIs
- **Compiler flags:** `-Wall -Wextra -Wpedantic -Werror`
