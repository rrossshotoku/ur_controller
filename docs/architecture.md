# UR Controller Architecture Document

## 1. System Overview

### 1.1 Purpose

This application provides a trajectory controller for Universal Robots (UR) manipulators, designed to run on a Raspberry Pi. The system enables operators to define sequences of TCP (Tool Center Point) poses with precise timing, which are then streamed to the robot via the Real-Time Data Exchange (RTDE) interface.

The primary use case is camera-based inspection/scanning where:
- The end effector carries a camera
- Predictable Cartesian TCP motion is required
- Precise timing synchronisation is critical
- Keep-out zones must be monitored for safety

### 1.2 Key Features

| Feature | Description |
|---------|-------------|
| **Time-parameterised trajectories** | Segments complete in user-defined durations |
| **Cartesian space control** | TCP follows predictable paths for camera applications |
| **Trajectory blending** | Smooth corner rounding with configurable blend radius |
| **Keep-out zone monitoring** | Runtime collision detection with protective stop |
| **UR simulator support** | URSim integration for testing without hardware |
| **GUI with 3D visualisation** | ImGui interface with live robot view for development/testing |

### 1.3 High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              OPERATOR                                        │
│                                                                             │
│                    ┌─────────────────────────┐                              │
│                    │  Trajectory Definition  │                              │
│                    │  (YAML files)           │                              │
│                    └───────────┬─────────────┘                              │
└────────────────────────────────┼────────────────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          RASPBERRY PI                                        │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     UR CONTROLLER APPLICATION                        │   │
│  │                                                                      │   │
│  │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐            │   │
│  │  │  Trajectory   │  │   Collision   │  │  Kinematics   │            │   │
│  │  │  Processing   │  │   Monitor     │  │  Engine       │            │   │
│  │  └───────┬───────┘  └───────┬───────┘  └───────┬───────┘            │   │
│  │          │                  │                  │                     │   │
│  │          └──────────────────┼──────────────────┘                     │   │
│  │                             │                                        │   │
│  │                             ▼                                        │   │
│  │                    ┌───────────────┐                                 │   │
│  │                    │ RT Controller │                                 │   │
│  │                    │   (500 Hz)    │                                 │   │
│  │                    └───────┬───────┘                                 │   │
│  │                            │                                         │   │
│  └────────────────────────────┼─────────────────────────────────────────┘   │
│                               │ RTDE                                        │
└───────────────────────────────┼─────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      UR ROBOT / URSIM                                        │
│                                                                             │
│                    ┌─────────────────────────┐                              │
│                    │  6-DOF Manipulator      │                              │
│                    │  + Camera End Effector  │                              │
│                    └─────────────────────────┘                              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Trajectory System

### 2.1 Time-Parameterised Model

The trajectory system uses **time as the primary constraint**. Each segment has a predefined duration, and the system calculates the required velocities to meet this timing.

| Aspect | Specification |
|--------|---------------|
| **Primary constraint** | Segment duration (seconds) |
| **Derived values** | Velocity, acceleration |
| **Validation** | Check derived values against robot limits |
| **Execution** | Pose lookup by absolute time `t` |

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     TIME-BASED TRAJECTORY EXECUTION                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Keyframe 1         Keyframe 2         Keyframe 3         Keyframe 4      │
│       ●─────────────────●─────────────────●─────────────────●              │
│       │                 │                 │                 │              │
│       │   t = 2.0s      │   t = 1.5s      │   t = 3.0s      │              │
│       │                 │                 │                 │              │
│       ├────────────────►├────────────────►├────────────────►│              │
│                                                                             │
│   Total trajectory time = 6.5s (deterministic)                              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Keyframe Definition

Each keyframe specifies:

```cpp
struct Keyframe {
    Eigen::Vector3d position;           // TCP position (x, y, z) in metres
    Eigen::Quaterniond orientation;     // TCP orientation as quaternion

    double segment_duration;            // Time to reach from previous keyframe (seconds)
    double blend_radius;                // Corner blend radius (metres), 0 = exact

    std::optional<double> dwell_time;   // Pause duration at keyframe (requires blend=0)
};
```

### 2.3 Trajectory Blending

Blending enables smooth continuous motion through waypoints by rounding corners with circular arcs.

**Blend Radius Behaviour:**

| Blend Radius | Behaviour | Use Case |
|--------------|-----------|----------|
| 0 | Exact waypoint, stop | Precise positioning, image capture |
| Small (5mm) | Slight rounding | Near-precise with smoothing |
| Medium (20mm) | Noticeable rounding | Continuous scanning |
| Large (50mm+) | Significant corner cutting | Fast sweeping motions |

**Geometric Representation:**

```
    Blend = 0 (exact waypoint)          Blend = 0.05m (smooth corner)

           P2 ●                                P2 ○ (doesn't reach)
             /\                                  __
            /  \                                /  \
           /    \                              /    \
          /      \                            │      │
    P1 ●─┘        └─● P3                 P1 ●─┘      └─● P3

    Robot stops at P2                   Robot blends through arc
    Sharp direction change              Continuous velocity
```

**Blend Time Allocation:**

Blend regions consume time from both adjacent segments:

```
      Segment 1 (2.0s)              Segment 2 (1.5s)
    ◄─────────────────────►     ◄─────────────────────►

    ●═══════════════●━━━━━━━━━━━━━━━●═══════════════════●
   P1              blend           blend               P3
                   start            end

                   ◄──────────────►
                   Blend zone (arc)
                   Uses time from both segments
```

### 2.4 Cartesian Interpolation

Since a camera is mounted on the TCP, all interpolation occurs in Cartesian space:

**Position Interpolation:**
- Linear interpolation between keyframes on linear segments
- Circular arc interpolation through blend zones

**Orientation Interpolation:**
- SLERP (Spherical Linear Interpolation) for quaternions
- Ensures shortest path rotation with no flips

```cpp
Eigen::Quaterniond interpolateOrientation(
    const Eigen::Quaterniond& q0,
    const Eigen::Quaterniond& q1,
    double s)  // s ∈ [0, 1]
{
    // Ensure shortest path
    Eigen::Quaterniond q1_adj = q1;
    if (q0.dot(q1) < 0.0) {
        q1_adj = Eigen::Quaterniond(-q1.coeffs());
    }
    return q0.slerp(s, q1_adj);
}
```

### 2.5 Trajectory Processing Pipeline

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        TRAJECTORY PROCESSING                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐  │
│  │  Keyframes  │───▶│  Blend      │───▶│  Segment    │───▶│  Timing     │  │
│  │  (YAML)     │    │  Validation │    │  Generation │    │  Validation │  │
│  └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘  │
│                                               │                             │
│        Keyframe with blend_radius             ▼                             │
│        ┌─────────────────┐            ┌─────────────────┐                   │
│        │ position        │            │ Linear segments │                   │
│        │ orientation     │            │ Blend arcs      │                   │
│        │ blend_radius    │──────────▶ │ Dwell points    │                   │
│        │ segment_duration│            │                 │                   │
│        │ dwell_time      │            │ Time-indexed    │                   │
│        └─────────────────┘            └─────────────────┘                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.6 Timing Validation

Before execution, the system validates that required velocities and accelerations are achievable:

```cpp
struct TrajectoryLimits {
    double max_tcp_velocity;        // m/s
    double max_tcp_acceleration;    // m/s²
    double max_joint_velocity;      // rad/s
    double max_joint_acceleration;  // rad/s²
};

struct ValidationResult {
    bool valid;
    std::vector<std::string> violations;

    struct SegmentAnalysis {
        size_t segment_index;
        double required_velocity;       // m/s
        double required_acceleration;   // m/s²
        bool exceeds_limits;
    };
    std::vector<SegmentAnalysis> segment_analysis;
};
```

---

## 3. Kinematics Engine

### 3.1 Overview

The kinematics engine provides forward and inverse kinematics for UR robots. A custom analytical implementation is used for performance (closed-form IK in ~5μs vs ~100μs for numerical methods).

| Function | Input | Output |
|----------|-------|--------|
| **Forward Kinematics (FK)** | 6 joint angles (rad) | TCP pose (position + quaternion) |
| **Inverse Kinematics (IK)** | TCP pose | Up to 8 joint solutions |
| **Jacobian** | Joint angles | 6x6 matrix |
| **Singularity Check** | Joint angles | Boolean + condition number |

### 3.2 DH Parameters

UR robots use standard Denavit-Hartenberg parameters. These are model-specific:

```
UR5e example:
d1 = 0.1625m, a2 = -0.425m, a3 = -0.3922m
d4 = 0.1333m, d5 = 0.0997m, d6 = 0.0996m
```

### 3.3 Kinematics Interface

```cpp
class URKinematics {
public:
    URKinematics(URModel model);  // UR3, UR5, UR10, UR16, UR20

    // Forward kinematics: joints → TCP pose
    Eigen::Isometry3d forward(const JointVector& q) const;

    // All link transforms (for collision checking)
    std::vector<Eigen::Isometry3d> allLinkTransforms(const JointVector& q) const;

    // Inverse kinematics: TCP pose → up to 8 solutions
    std::vector<JointVector> inverse(const Eigen::Isometry3d& pose) const;

    // Jacobian at configuration
    Eigen::Matrix6d jacobian(const JointVector& q) const;

    // Singularity detection
    bool nearSingularity(const JointVector& q, double threshold) const;

private:
    DHParameters dh_;
};
```

### 3.4 IK Solution Selection

With up to 8 IK solutions, consistent selection is critical to avoid joint discontinuities:

```cpp
class IKSolutionSelector {
public:
    std::optional<JointVector> selectBest(
        const std::vector<JointVector>& solutions,
        const JointVector& current_q,
        const JointLimits& limits)
    {
        std::optional<JointVector> best;
        double best_distance = std::numeric_limits<double>::max();

        for (const auto& solution : solutions) {
            // Skip if outside joint limits
            if (!withinLimits(solution, limits))
                continue;

            // Skip if would cause large joint jump
            double distance = (solution - current_q).norm();
            if (distance > max_joint_jump_)
                continue;

            if (distance < best_distance) {
                best_distance = distance;
                best = solution;
            }
        }
        return best;
    }

private:
    double max_joint_jump_ = 0.5;  // radians per control cycle
};
```

### 3.5 Singularity Types

| Singularity | Condition | Detection |
|-------------|-----------|-----------|
| **Shoulder** | Wrist centre on J1 axis | Check wrist position radius |
| **Elbow** | Arm fully extended/folded | Check J3 near 0 or π |
| **Wrist** | J4 and J6 axes aligned | Check J5 near 0 or π |

---

## 4. Collision Monitoring

### 4.1 Overview

The collision system provides runtime monitoring of keep-out zones. If the robot enters a zone, motion is immediately stopped.

**Design Decision:** The initial implementation uses runtime monitoring with protective stop, not path planning. The robot does not automatically plan around obstacles.

### 4.2 Monitoring Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      RUNTIME COLLISION MONITORING                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│                         500Hz Control Loop                                  │
│                                ▼                                            │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐                  │
│  │ Read current │───▶│   Collision  │───▶│   Decision   │                  │
│  │ joint state  │    │   check      │    │              │                  │
│  └──────────────┘    └──────────────┘    └──────────────┘                  │
│                              │                   │                          │
│                              ▼                   ▼                          │
│                       ┌─────────────┐     ┌─────────────┐                  │
│                       │ In zone?    │     │ CLEAR:      │                  │
│                       │ Near zone?  │     │ Continue    │                  │
│                       └─────────────┘     │ trajectory  │                  │
│                              │            └─────────────┘                  │
│                              ▼                                              │
│                       ┌─────────────┐                                       │
│                       │ VIOLATION:  │                                       │
│                       │ Stop robot  │                                       │
│                       │ Alert user  │                                       │
│                       └─────────────┘                                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 4.3 Zone Status

```cpp
enum class ZoneStatus {
    Clear,          // No issues
    Warning,        // Within safety margin
    Violation       // Inside keep-out zone
};

struct MonitorResult {
    ZoneStatus status;
    std::string zone_id;           // Which zone triggered
    std::string link_name;         // Which robot link
    double distance;               // Negative = penetration depth
};
```

| Status | Trigger | Action |
|--------|---------|--------|
| **Clear** | No proximity issues | Continue trajectory |
| **Warning** | Within safety margin | Log warning, optionally reduce speed |
| **Violation** | Geometry intersection | Halt motion, set fault state |

### 4.4 Robot Geometry Model

Each robot link is modelled as a capsule (cylinder with hemispherical caps) for efficient collision checking:

```
        ┌─────┐
        │     │  ← Wrist/Flange
        │  ○  │  ← Camera (box or sphere)
        └──┬──┘
           │    ← Capsule (wrist3 to TCP)
        ┌──┴──┐
        │     │  ← Wrist joints
        └──┬──┘
           │
           │    ← Forearm capsule
           │
        ┌──┴──┐
        │     │  ← Elbow
        └──┬──┘
           │
           │    ← Upper arm capsule
           │
     ┌─────┴─────┐
     │           │  ← Shoulder
     └─────┬─────┘
           │
    ═══════╧═══════  ← Base
```

```cpp
struct CapsuleLink {
    std::string name;
    double radius;
    Eigen::Vector3d p1;     // Start point (in link frame)
    Eigen::Vector3d p2;     // End point (in link frame)
    int parent_joint;
};

struct RobotCollisionModel {
    std::vector<CapsuleLink> links;
    std::variant<Sphere, Box> end_effector;  // Camera housing

    void updatePoses(const std::vector<Eigen::Isometry3d>& link_transforms);
};
```

### 4.5 Keep-Out Zone Types

```cpp
struct KeepOutZone {
    std::string id;
    std::string frame;          // "world" or "base"
    double safety_margin;       // Extra padding (metres)
    bool enabled;
};

struct BoxZone : KeepOutZone {
    Eigen::Vector3d center;
    Eigen::Vector3d half_extents;
    Eigen::Quaterniond orientation;  // For OBB
};

struct SphereZone : KeepOutZone {
    Eigen::Vector3d center;
    double radius;
};

struct CylinderZone : KeepOutZone {
    Eigen::Vector3d base_center;
    Eigen::Vector3d axis;
    double radius;
};
```

### 4.6 Collision Monitor Interface

```cpp
class CollisionMonitor {
public:
    CollisionMonitor(
        const URKinematics& kinematics,
        const RobotCollisionModel& robot_model,
        const std::vector<KeepOutZone>& zones);

    // Call every control cycle
    MonitorResult check(const JointVector& current_q);

    // Runtime zone management
    void enableZone(const std::string& zone_id);
    void disableZone(const std::string& zone_id);

private:
    URKinematics kinematics_;
    RobotCollisionModel robot_model_;
    std::vector<KeepOutZone> zones_;
};
```

---

## 5. Real-Time Controller

### 5.1 Overview

The RT controller executes trajectories at 500Hz, streaming joint commands via RTDE while monitoring for collisions.

### 5.2 Controller States

```cpp
enum class ControllerState {
    Idle,       // Ready, no active trajectory
    Running,    // Executing trajectory
    Paused,     // Trajectory paused (can resume)
    Stopped,    // Trajectory complete
    Faulted     // Error condition (collision, IK failure) - requires reset
};
```

### 5.3 Control Loop

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         500Hz CONTROL LOOP                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  1. Calculate elapsed time since trajectory start                           │
│                         │                                                   │
│                         ▼                                                   │
│  2. Read robot state via RTDE                                               │
│     - actual_q (joint positions)                                            │
│     - actual_qd (joint velocities)                                          │
│                         │                                                   │
│                         ▼                                                   │
│  3. Collision check ─────────────────────────┐                              │
│                         │                    │                              │
│                         ▼                    ▼                              │
│                    [CLEAR]              [VIOLATION]                         │
│                         │                    │                              │
│                         ▼                    ▼                              │
│  4. Get target pose at time t         Stop, set FAULTED                     │
│     from trajectory                                                         │
│                         │                                                   │
│                         ▼                                                   │
│  5. Solve IK for target pose                                                │
│     Select best solution                                                    │
│                         │                                                   │
│                         ▼                                                   │
│  6. Send servoJ command via RTDE                                            │
│                         │                                                   │
│                         ▼                                                   │
│  7. Maintain loop timing (sleep for remainder of 2ms)                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.4 Controller Interface

```cpp
class RTController {
public:
    RTController(
        std::unique_ptr<RobotInterface> robot,
        const URKinematics& kinematics,
        CollisionMonitor& collision_monitor);

    // Trajectory execution
    void loadTrajectory(const Trajectory& trajectory);
    void start();
    void pause();
    void resume();
    void stop();
    void reset();  // Clear fault state

    // State queries
    ControllerState getState() const;
    double getElapsedTime() const;
    double getRemainingTime() const;
    JointVector getCurrentJoints() const;
    Eigen::Isometry3d getCurrentTCP() const;

private:
    void controlLoop();

    std::atomic<ControllerState> state_;
    std::unique_ptr<RobotInterface> robot_;
    URKinematics kinematics_;
    CollisionMonitor& collision_monitor_;
    TimeParameterizedTrajectory trajectory_;

    std::thread control_thread_;
    std::chrono::steady_clock::time_point trajectory_start_time_;
};
```

### 5.5 Robot Interface Abstraction

Abstract interface allows swapping between real robot and simulator:

```cpp
class RobotInterface {
public:
    virtual ~RobotInterface() = default;

    // Control commands
    virtual void servoJ(
        const JointVector& q,
        double velocity,
        double acceleration,
        double dt,
        double lookahead_time,
        double gain) = 0;

    virtual void servoStop() = 0;

    // State queries
    virtual JointVector getActualQ() = 0;
    virtual JointVector getActualQd() = 0;
    virtual Eigen::Isometry3d getActualTCPPose() = 0;
    virtual bool isConnected() = 0;
    virtual SafetyStatus getSafetyStatus() = 0;
};

// Real robot implementation
class URRobot : public RobotInterface {
    // Uses ur_rtde library
};

// Mock for testing
class MockRobot : public RobotInterface {
    // Simulates robot state, validates commands
};
```

---

## 6. Configuration

### 6.1 Trajectory Configuration

```yaml
trajectory:
  name: "inspection_scan_01"
  description: "Camera inspection path for component A"

  # Robot limits for timing validation
  limits:
    max_tcp_velocity: 0.5       # m/s
    max_tcp_acceleration: 1.0   # m/s²

  keyframes:
    - id: 1
      position: [0.4, 0.0, 0.5]
      orientation: [1.0, 0.0, 0.0, 0.0]  # quaternion (w, x, y, z)
      # First keyframe: no segment_duration (starting point)
      blend_radius: 0.0

    - id: 2
      position: [0.4, 0.2, 0.5]
      orientation: [0.707, 0.0, 0.707, 0.0]
      segment_duration: 2.0     # 2 seconds from keyframe 1
      blend_radius: 0.03        # 30mm blend radius

    - id: 3
      position: [0.3, 0.2, 0.4]
      orientation: [0.707, 0.0, 0.707, 0.0]
      segment_duration: 1.5     # 1.5 seconds from keyframe 2
      blend_radius: 0.0         # Exact point - stop here
      dwell_time: 0.5           # Pause 500ms for image capture

    - id: 4
      position: [0.3, 0.0, 0.4]
      orientation: [1.0, 0.0, 0.0, 0.0]
      segment_duration: 3.0     # 3 seconds from keyframe 3
      blend_radius: 0.0         # Exact endpoint
```

### 6.2 Keep-Out Zone Configuration

```yaml
environment:
  name: "inspection_cell"
  reference_frame: "robot_base"

  keep_out_zones:
    # Floor - prevent TCP from going below
    - id: "floor"
      type: "box"
      center: [0.0, 0.0, -0.05]
      half_extents: [2.0, 2.0, 0.05]
      safety_margin: 0.02
      enabled: true

    # Operator area
    - id: "operator_zone"
      type: "box"
      center: [-0.5, 0.0, 0.5]
      half_extents: [0.3, 0.5, 0.5]
      orientation: [1, 0, 0, 0]
      safety_margin: 0.05
      enabled: true

    # Ceiling mounted equipment
    - id: "ceiling_equipment"
      type: "cylinder"
      base_center: [0.3, 0.3, 1.5]
      axis: [0, 0, -0.5]
      radius: 0.15
      safety_margin: 0.03
      enabled: true

    # Sphere example
    - id: "sensor_housing"
      type: "sphere"
      center: [0.6, 0.1, 0.3]
      radius: 0.08
      safety_margin: 0.02
      enabled: true
```

### 6.3 Robot Configuration

```yaml
robot:
  model: "UR5e"
  ip_address: "192.168.1.100"

  # DH parameters (loaded from model, can override)
  dh_parameters:
    d1: 0.1625
    a2: -0.425
    a3: -0.3922
    d4: 0.1333
    d5: 0.0997
    d6: 0.0996

  # Joint limits
  joint_limits:
    position:
      min: [-6.283, -6.283, -3.142, -6.283, -6.283, -6.283]
      max: [6.283, 6.283, 3.142, 6.283, 6.283, 6.283]
    velocity: [3.14, 3.14, 3.14, 6.28, 6.28, 6.28]  # rad/s

  # End effector / tool
  tool:
    type: "box"
    dimensions: [0.08, 0.08, 0.12]  # Camera housing
    tcp_offset:
      position: [0, 0, 0.06]
      orientation: [1, 0, 0, 0]
```

---

## 7. Library Dependencies

### 7.1 Core Libraries

| Component | Library | Purpose |
|-----------|---------|---------|
| **UR Communication** | [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) | RTDE protocol, servoJ commands |
| **Linear Algebra** | [Eigen3](https://eigen.tuxfamily.org/) | Matrices, transforms, quaternions |
| **Collision Detection** | [FCL](https://github.com/flexible-collision-library/fcl) or [HPP-FCL](https://github.com/humanoid-path-planner/hpp-fcl) | Zone intersection checks |
| **Trajectory Generation** | [Ruckig](https://github.com/pantor/ruckig) | Time-optimal, jerk-limited profiles |
| **Configuration** | [yaml-cpp](https://github.com/jbeder/yaml-cpp) | YAML parsing |
| **Logging** | [spdlog](https://github.com/gabime/spdlog) | Fast async logging |
| **CLI** | [CLI11](https://github.com/CLIUtils/CLI11) | Command-line parsing |

### 7.2 Testing Libraries

| Component | Library | Purpose |
|-----------|---------|---------|
| **Unit Testing** | [Catch2](https://github.com/catchorg/Catch2) | Test framework |
| **Mocking** | Custom MockRobot | Robot interface mock |

### 7.3 GUI Libraries

| Component | Library | Purpose |
|-----------|---------|---------|
| **UI Framework** | [Dear ImGui](https://github.com/ocornut/imgui) | Immediate-mode GUI |
| **Windowing** | [GLFW](https://www.glfw.org/) | Window and input management |
| **OpenGL Loading** | [GLEW](http://glew.sourceforge.net/) | OpenGL extension loading |
| **3D Math** | [GLM](https://github.com/g-truc/glm) | OpenGL mathematics |
| **Plotting** | [implot](https://github.com/epezent/implot) | Trajectory visualisation (future) |

---

## 8. Development Environment

### 8.1 Overview

Development uses Docker containers on a PC for consistent builds and easy dependency management. The application is then cross-compiled or rebuilt for deployment on the Raspberry Pi.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         DEVELOPMENT (PC)                                     │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     Docker Compose                                   │   │
│  │                                                                      │   │
│  │  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐  │   │
│  │  │  ur_controller  │    │     URSim       │    │   Dev Tools     │  │   │
│  │  │  (build & run)  │◄──►│  (UR simulator) │    │  (analysis)     │  │   │
│  │  │                 │    │                 │    │                 │  │   │
│  │  │  - Build        │    │  - RTDE server  │    │  - clang-tidy   │  │   │
│  │  │  - Unit tests   │    │  - Polyscope    │    │  - clang-format │  │   │
│  │  │  - Integration  │    │  - Dashboard    │    │  - cppcheck     │  │   │
│  │  └─────────────────┘    └─────────────────┘    └─────────────────┘  │   │
│  │           │                                                          │   │
│  │           ▼                                                          │   │
│  │  ┌─────────────────────────────────────────┐                        │   │
│  │  │         Mounted Source Code             │                        │   │
│  │  │         (live editing from host IDE)    │                        │   │
│  │  └─────────────────────────────────────────┘                        │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    │ Cross-compile or
                                    │ rebuild natively
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         DEPLOYMENT (Raspberry Pi)                            │
│                                                                             │
│  ┌─────────────────┐         ┌─────────────────┐                           │
│  │  ur_controller  │────────►│   UR Robot      │                           │
│  │  (native ARM64) │  RTDE   │   (real HW)     │                           │
│  │                 │         │                 │                           │
│  │  - PREEMPT_RT   │         │                 │                           │
│  │  - Optimised    │         │                 │                           │
│  └─────────────────┘         └─────────────────┘                           │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 8.2 Build Strategy

| Stage | Environment | Purpose |
|-------|-------------|---------|
| **Development** | Docker (x86_64 Linux) | Fast iteration, IDE integration |
| **Unit Testing** | Docker (x86_64 Linux) | Test kinematics, trajectory, collision |
| **Integration Testing** | Docker + URSim | Test RTDE communication |
| **Deployment Build** | Cross-compile or native RPi | ARM64 binary for target |
| **RT Testing** | Raspberry Pi | Verify real-time performance |

### 8.3 Docker Compose Configuration

```yaml
# docker/docker-compose.yml
version: '3.8'

services:
  # Main development container
  dev:
    build:
      context: .
      dockerfile: Dockerfile.dev
    volumes:
      - ../:/workspace                    # Mount source code
      - build-cache:/workspace/build      # Persist build artifacts
    network_mode: host                    # Easy access to URSim ports
    stdin_open: true
    tty: true
    cap_add:
      - SYS_PTRACE                        # For debugging
    security_opt:
      - seccomp:unconfined                # For debugging

  # UR Simulator
  ursim:
    image: universalrobots/ursim_e-series
    environment:
      - ROBOT_MODEL=UR5
    ports:
      - "30001:30001"   # Primary interface
      - "30002:30002"   # Secondary interface
      - "30003:30003"   # Real-time interface
      - "30004:30004"   # RTDE
      - "29999:29999"   # Dashboard server
      - "6080:6080"     # Web VNC (Polyscope GUI)
    volumes:
      - ursim-programs:/ursim/programs

  # Cross-compilation for ARM64 (Raspberry Pi)
  cross-arm64:
    build:
      context: .
      dockerfile: Dockerfile.cross-arm64
    volumes:
      - ../:/workspace
      - cross-build-cache:/workspace/build-arm64
    profiles:
      - cross   # Only start with: docker compose --profile cross up

volumes:
  build-cache:
  cross-build-cache:
  ursim-programs:
```

### 8.4 Development Dockerfile

```dockerfile
# docker/Dockerfile.dev
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install build tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    gdb \
    valgrind \
    clang \
    clang-tidy \
    clang-format \
    cppcheck \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libfcl-dev \
    libyaml-cpp-dev \
    libspdlog-dev \
    libboost-system-dev \
    libboost-thread-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ur_rtde from source
RUN git clone https://gitlab.com/sdurobotics/ur_rtde.git /tmp/ur_rtde \
    && cd /tmp/ur_rtde \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/ur_rtde

# Install Ruckig from source
RUN git clone https://github.com/pantor/ruckig.git /tmp/ruckig \
    && cd /tmp/ruckig \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/ruckig

# Install Catch2
RUN git clone https://github.com/catchorg/Catch2.git /tmp/catch2 \
    && cd /tmp/catch2 \
    && mkdir build && cd build \
    && cmake .. -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) \
    && make install \
    && rm -rf /tmp/catch2

WORKDIR /workspace

# Default command: bash shell
CMD ["/bin/bash"]
```

### 8.5 Cross-Compilation Dockerfile

```dockerfile
# docker/Dockerfile.cross-arm64
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install cross-compilation toolchain
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    crossbuild-essential-arm64 \
    qemu-user-static \
    && rm -rf /var/lib/apt/lists/*

# Set up cross-compilation environment
ENV CROSS_COMPILE=aarch64-linux-gnu-
ENV CC=aarch64-linux-gnu-gcc
ENV CXX=aarch64-linux-gnu-g++
ENV CMAKE_TOOLCHAIN_FILE=/workspace/cmake/toolchain-arm64.cmake

WORKDIR /workspace

CMD ["/bin/bash"]
```

### 8.6 CMake Toolchain File for ARM64

```cmake
# cmake/toolchain-arm64.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```

### 8.7 Development Workflow

**Daily Development:**

```bash
# Start development environment
cd docker
docker compose up -d dev ursim

# Enter development container
docker compose exec dev bash

# Inside container: build and test
cd /workspace
mkdir -p build && cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Debug
ninja
ctest --output-on-failure

# Access URSim Polyscope GUI
# Open browser: http://localhost:6080
```

**Cross-Compile for Raspberry Pi:**

```bash
# Build ARM64 binary
docker compose --profile cross run cross-arm64 bash -c "
    cd /workspace
    mkdir -p build-arm64 && cd build-arm64
    cmake .. -GNinja \
        -DCMAKE_TOOLCHAIN_FILE=/workspace/cmake/toolchain-arm64.cmake \
        -DCMAKE_BUILD_TYPE=Release
    ninja
"

# Copy to Raspberry Pi
scp build-arm64/apps/trajectory_runner pi@raspberrypi:/home/pi/ur_controller/
```

**Alternative: Build Natively on RPi:**

```bash
# SSH to Raspberry Pi
ssh pi@raspberrypi

# Clone and build
git clone <repo> ur_controller
cd ur_controller
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

### 8.8 IDE Integration

The source code is mounted into Docker, so you can edit with your preferred IDE on the host:

| IDE | Setup |
|-----|-------|
| **VS Code** | Use Remote-Containers extension, or edit locally with Docker build |
| **CLion** | Configure remote toolchain pointing to Docker container |
| **vim/neovim** | Edit on host, build via `docker compose exec` |

**VS Code devcontainer.json:**

```json
{
    "name": "UR Controller Dev",
    "dockerComposeFile": "../docker/docker-compose.yml",
    "service": "dev",
    "workspaceFolder": "/workspace",
    "extensions": [
        "ms-vscode.cpptools",
        "ms-vscode.cmake-tools",
        "xaver.clang-format"
    ],
    "settings": {
        "cmake.configureOnOpen": true,
        "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools"
    }
}
```

### 8.9 Continuous Integration

```yaml
# .github/workflows/ci.yml (example)
name: CI

on: [push, pull_request]

jobs:
  build-and-test:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v3

      - name: Build dev container
        run: docker compose -f docker/docker-compose.yml build dev

      - name: Build and test
        run: |
          docker compose -f docker/docker-compose.yml run dev bash -c "
            mkdir -p build && cd build
            cmake .. -GNinja -DCMAKE_BUILD_TYPE=Debug
            ninja
            ctest --output-on-failure
          "

  cross-compile:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v3

      - name: Cross-compile for ARM64
        run: |
          docker compose -f docker/docker-compose.yml --profile cross build cross-arm64
          docker compose -f docker/docker-compose.yml --profile cross run cross-arm64 bash -c "
            mkdir -p build-arm64 && cd build-arm64
            cmake .. -GNinja -DCMAKE_TOOLCHAIN_FILE=/workspace/cmake/toolchain-arm64.cmake
            ninja
          "
```

---

## 9. Project Structure

```
ur_controller/
├── CMakeLists.txt
├── include/ur_controller/
│   ├── kinematics/
│   │   ├── ur_kinematics.hpp
│   │   ├── dh_parameters.hpp
│   │   └── types.hpp                  # JointVector, Pose types
│   ├── collision/
│   │   ├── collision_monitor.hpp
│   │   ├── keep_out_zone.hpp
│   │   └── robot_geometry.hpp
│   ├── trajectory/
│   │   ├── keyframe.hpp
│   │   ├── trajectory.hpp
│   │   ├── trajectory_builder.hpp
│   │   └── interpolator.hpp
│   ├── control/
│   │   ├── rt_controller.hpp
│   │   └── robot_interface.hpp
│   ├── robot/
│   │   ├── ur_robot.hpp               # Real robot (ur_rtde)
│   │   └── mock_robot.hpp             # Mock for testing
│   └── config/
│       └── config_loader.hpp
├── src/
│   ├── kinematics/
│   ├── collision/
│   ├── trajectory/
│   ├── control/
│   ├── robot/
│   └── config/
├── apps/
│   ├── trajectory_runner.cpp          # Main execution application
│   ├── teach_mode.cpp                 # Record keyframes
│   └── connection_test.cpp            # Test URSim connectivity
├── gui/
│   ├── CMakeLists.txt
│   ├── main.cpp                       # ImGui application entry point
│   ├── robot_viewer.hpp               # 3D robot visualization
│   └── robot_viewer.cpp
├── config/
│   ├── robots/
│   │   ├── ur3e.yaml
│   │   ├── ur5e.yaml
│   │   └── ur10e.yaml
│   ├── environments/
│   │   └── default_zones.yaml
│   └── trajectories/
│       └── example.yaml
├── test/
│   ├── test_kinematics.cpp
│   ├── test_collision.cpp
│   ├── test_trajectory.cpp
│   └── test_controller.cpp
└── docker/
    └── ursim/
        └── docker-compose.yml
```

---

## 10. Simulator Setup

### 10.1 URSim Docker Configuration

```yaml
# docker/ursim/docker-compose.yml
version: '3'
services:
  ursim:
    image: universalrobots/ursim_e-series
    environment:
      - ROBOT_MODEL=UR5
    ports:
      - "30001:30001"  # Primary interface
      - "30002:30002"  # Secondary interface
      - "30003:30003"  # Real-time interface
      - "30004:30004"  # RTDE
      - "29999:29999"  # Dashboard server
      - "6080:6080"    # Web VNC (Polyscope GUI)
    volumes:
      - ./programs:/ursim/programs
```

### 10.2 Mock Robot for Unit Testing

```cpp
class MockRobot : public RobotInterface {
public:
    void servoJ(const JointVector& q, ...) override {
        // Validate command limits
        validateJointLimits(q);

        // Simulate motion
        current_q_ = q;
        current_tcp_ = kinematics_.forward(q);

        // Record for assertions
        command_history_.push_back({q, getCurrentTime()});
    }

    JointVector getActualQ() override { return current_q_; }

    // Test helpers
    const std::vector<Command>& getCommandHistory() const;
    void setCurrentState(const JointVector& q);
    void injectFault(SafetyStatus status);

private:
    URKinematics kinematics_;
    JointVector current_q_;
    Eigen::Isometry3d current_tcp_;
    std::vector<Command> command_history_;
};
```

---

## 11. Computational Considerations

### 11.1 Timing Budget (Raspberry Pi)

| Operation | Typical Time | Notes |
|-----------|--------------|-------|
| FK computation | ~1 μs | Very fast |
| Analytical IK | ~5 μs | 8 solutions |
| Collision check (simple) | ~50-100 μs | Few primitives |
| Collision check (complex) | ~200-500 μs | Many zones |
| RTDE round-trip | ~500 μs | Network dependent |

**500Hz loop budget: 2ms**

Typical iteration: FK + IK + collision + RTDE ≈ 0.6-1.2ms (sufficient margin)

### 11.2 Real-Time Considerations

- Use `PREEMPT_RT` Linux kernel patch on Raspberry Pi for deterministic timing
- Pin control thread to dedicated CPU core
- Use memory-locked allocations to prevent page faults
- Avoid dynamic allocation in control loop

---

## 12. GUI Application

### 12.1 Overview

The GUI provides a development and testing interface for the UR Controller. It uses **Dear ImGui** for the user interface and **OpenGL** for 3D robot visualization.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  UR Controller                                                       [─][□][×]│
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────┐  ┌──────────────────────────────────┐ │
│  │                                 │  │ Connection                       │ │
│  │      3D Robot View              │  │ Robot IP: [ursim        ]        │ │
│  │                                 │  │ [Connect]  ● Connected           │ │
│  │      - Live joint positions     │  │ Mode: RUNNING                    │ │
│  │      - Camera orbit controls    │  │ Safety: NORMAL                   │ │
│  │      - Grid and axes            │  └──────────────────────────────────┘ │
│  │                                 │  ┌──────────────────────────────────┐ │
│  │                                 │  │ Joint Positions                  │ │
│  │                                 │  │ Base:     -91.71°                │ │
│  └─────────────────────────────────┘  │ Shoulder: -98.96°                │ │
│                                       │ Elbow:   -126.22°                │ │
│  ┌─────────────────────────────────┐  │ Wrist1:  -46.29°                 │ │
│  │ Test Functions                  │  │ Wrist2:   91.39°                 │ │
│  │ [FK Test] [IK Test]             │  │ Wrist3:   -1.78°                 │ │
│  │ [Load Trajectory] [Execute]    │  └──────────────────────────────────┘ │
│  │ [Add Keep-Out Zone]            │                                       │
│  └─────────────────────────────────┘                                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 12.2 Technology Stack

| Component | Library | Purpose |
|-----------|---------|---------|
| **UI Framework** | [Dear ImGui](https://github.com/ocornut/imgui) | Immediate-mode GUI |
| **Windowing** | [GLFW](https://www.glfw.org/) | Window creation and input |
| **Graphics** | OpenGL 3.3 | 3D rendering |
| **Extension Loading** | [GLEW](http://glew.sourceforge.net/) | OpenGL extension management |

### 12.3 GUI Panels

| Panel | Purpose |
|-------|---------|
| **Connection** | Robot IP configuration, connect/disconnect, status display |
| **Joint Positions** | Live display of joint angles and velocities |
| **TCP Pose** | Current tool position and orientation |
| **Test Functions** | Buttons to test modules as they're developed |
| **3D Robot View** | Visual representation of robot with camera controls |

### 12.4 3D Robot Viewer

The robot viewer renders a simplified model of the UR robot using OpenGL primitives:

- **Links**: Cylinders representing arm segments
- **Joints**: Cylinders at rotation points
- **End effector**: Box representing camera/tool
- **Grid**: Reference grid on the floor plane
- **Axes**: RGB axes showing world coordinate frame

Camera controls:
- **Distance**: Zoom in/out
- **Yaw**: Orbit horizontally around robot
- **Pitch**: Orbit vertically around robot

### 12.5 Adding Test Functions

As modules are implemented, add test buttons to the GUI:

```cpp
// In renderTestPanel():
ImGui::Text("Kinematics:");
if (ImGui::Button("Test FK")) {
    // Call kinematics module
    auto pose = kinematics.forward(current_joints);
    spdlog::info("FK result: [{}, {}, {}]", pose.x(), pose.y(), pose.z());
}
```

### 12.6 Running the GUI

**From Docker (requires X11 forwarding):**
```bash
# Install VcXsrv on Windows, run with "Disable access control"
export DISPLAY=host.docker.internal:0.0
./gui/ur_controller_gui --ip ursim
```

**Native Windows build (alternative):**
```powershell
# Build natively on Windows with MSVC and vcpkg for dependencies
cmake -B build -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
cmake --build build
./build/gui/ur_controller_gui.exe --ip localhost
```

---

## 13. Future Enhancements

The following features are out of scope for the initial implementation but may be added later:

| Feature | Description |
|---------|-------------|
| **Path planning** | Automatic trajectory planning around obstacles (OMPL integration) |
| **Pre-validation** | Check entire trajectory for collisions before execution |
| **Trajectory replanning** | Dynamic obstacle avoidance during execution |
| **URDF model loading** | Load proper robot meshes instead of primitives |
| **Mesh collision objects** | Complex geometry for keep-out zones |
| **Force/torque monitoring** | Contact detection via F/T sensor |

---

## Appendix A: Glossary

| Term | Definition |
|------|------------|
| **TCP** | Tool Centre Point - the reference point at the end of the robot tool |
| **FK** | Forward Kinematics - computing TCP pose from joint angles |
| **IK** | Inverse Kinematics - computing joint angles from TCP pose |
| **RTDE** | Real-Time Data Exchange - UR's real-time communication protocol |
| **DH Parameters** | Denavit-Hartenberg parameters defining robot geometry |
| **SLERP** | Spherical Linear Interpolation - smooth quaternion interpolation |
| **Blend radius** | Distance from waypoint where corner rounding begins |
| **servoJ** | UR command for streaming joint positions in real-time |

---

## Appendix B: Reference Documents

- [UR RTDE Guide](https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/)
- [UR DH Parameters](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)
- [ur_rtde Library Documentation](https://sdurobotics.gitlab.io/ur_rtde/)
- [FCL Documentation](https://flexible-collision-library.github.io/)
- [Ruckig Documentation](https://docs.ruckig.com/)
