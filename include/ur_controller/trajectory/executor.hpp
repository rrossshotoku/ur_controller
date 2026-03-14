/// @file executor.hpp
/// @brief Real-time trajectory execution at 500 Hz

#pragma once

#include "ur_controller/trajectory/types.hpp"
#include "ur_controller/kinematics/types.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace ur_controller {
namespace trajectory {

/// @brief Callback for trajectory state changes
using StateCallback = std::function<void(ExecutorState)>;

/// @brief Callback for streaming joint commands
/// @param joints Joint positions to command
/// @param time Current trajectory time
using CommandCallback = std::function<void(const JointVector& joints, double time)>;

/// @brief Progress information for trajectory execution
struct ExecutorProgress {
    ExecutorState state{ExecutorState::Idle};
    double current_time{0.0};           ///< Current time in trajectory
    double total_duration{0.0};         ///< Total trajectory duration
    double progress_percent{0.0};       ///< Progress as percentage (0-100)
    size_t current_sample_index{0};     ///< Current sample index
    size_t total_samples{0};            ///< Total number of samples
    JointVector current_joints;         ///< Current joint positions
    Eigen::Isometry3d current_pose;     ///< Current TCP pose
    double current_speed{0.0};          ///< Current TCP speed (m/s)
};

/// @brief Configuration for the trajectory executor
struct ExecutorConfig {
    double control_rate{500.0};         ///< Control loop rate in Hz
    double lookahead_time{0.1};         ///< Look-ahead time for servoj (seconds)
    double gain{300.0};                 ///< servoj gain parameter
    bool dry_run{false};                ///< If true, don't send commands to robot
};

/// @brief Real-time trajectory executor for 500 Hz playback
///
/// The executor plays back pre-computed trajectories by sampling at 500 Hz
/// and streaming joint positions via the provided command callback.
///
/// Thread safety:
/// - load(), start(), pause(), resume(), stop() are thread-safe
/// - Command callback is called from executor thread
/// - State callback is called from executor thread
///
/// @note This class spawns its own thread for timing-critical execution.
class TrajectoryExecutor {
public:
    /// @brief Construct executor with default configuration
    TrajectoryExecutor();

    /// @brief Construct executor with custom configuration
    explicit TrajectoryExecutor(const ExecutorConfig& config);

    /// @brief Destructor - stops execution and joins thread
    ~TrajectoryExecutor();

    // Non-copyable, non-movable (owns thread)
    TrajectoryExecutor(const TrajectoryExecutor&) = delete;
    TrajectoryExecutor& operator=(const TrajectoryExecutor&) = delete;
    TrajectoryExecutor(TrajectoryExecutor&&) = delete;
    TrajectoryExecutor& operator=(TrajectoryExecutor&&) = delete;

    /// @brief Load a trajectory for execution
    ///
    /// Transitions from Idle to Ready state.
    /// Cannot be called while executing.
    ///
    /// @param trajectory Pre-computed trajectory to execute
    /// @return true if trajectory was loaded successfully
    bool load(const PlannedTrajectory& trajectory);

    /// @brief Start trajectory execution
    ///
    /// Transitions from Ready to Running state.
    /// The command callback will be called at control_rate Hz.
    ///
    /// @return true if execution started successfully
    bool start();

    /// @brief Pause trajectory execution
    ///
    /// Transitions from Running to Paused state.
    /// The robot will hold at the current position.
    void pause();

    /// @brief Resume trajectory execution
    ///
    /// Transitions from Paused to Running state.
    void resume();

    /// @brief Stop trajectory execution
    ///
    /// Transitions to Idle state from any state.
    /// The trajectory must be reloaded before starting again.
    void stop();

    /// @brief Get current executor state
    [[nodiscard]] ExecutorState state() const;

    /// @brief Get current execution progress
    [[nodiscard]] ExecutorProgress progress() const;

    /// @brief Check if executor has a loaded trajectory
    [[nodiscard]] bool hasTrajectory() const;

    /// @brief Check if executor is currently running
    [[nodiscard]] bool isRunning() const;

    /// @brief Set callback for joint commands
    ///
    /// This callback is invoked at control_rate Hz with the joint
    /// positions to send to the robot.
    void setCommandCallback(CommandCallback callback);

    /// @brief Set callback for state changes
    void setStateCallback(StateCallback callback);

    /// @brief Get executor configuration
    [[nodiscard]] const ExecutorConfig& config() const { return config_; }

    /// @brief Set executor configuration (only when Idle)
    bool setConfig(const ExecutorConfig& config);

private:
    /// @brief Main execution loop (runs in separate thread)
    void executionLoop();

    /// @brief Transition to a new state
    void setState(ExecutorState new_state);

    /// @brief Get interpolated sample at time t
    [[nodiscard]] TrajectorySample sampleAt(double t) const;

    ExecutorConfig config_;

    // Thread-safe state
    mutable std::mutex mutex_;
    std::atomic<ExecutorState> state_{ExecutorState::Idle};
    std::atomic<bool> should_stop_{false};
    std::atomic<bool> should_pause_{false};

    // Trajectory data (protected by mutex_)
    std::unique_ptr<PlannedTrajectory> trajectory_;
    double current_time_{0.0};
    size_t current_index_{0};

    // Callbacks
    CommandCallback command_callback_;
    StateCallback state_callback_;

    // Execution thread
    std::thread execution_thread_;
    std::atomic<bool> thread_running_{false};
};

}  // namespace trajectory
}  // namespace ur_controller
