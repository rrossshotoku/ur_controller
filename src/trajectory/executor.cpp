/// @file executor.cpp
/// @brief Real-time trajectory executor implementation

#include "ur_controller/trajectory/executor.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <algorithm>
#include <cmath>

namespace ur_controller {
namespace trajectory {

TrajectoryExecutor::TrajectoryExecutor()
    : config_{} {}

TrajectoryExecutor::TrajectoryExecutor(const ExecutorConfig& config)
    : config_(config) {}

TrajectoryExecutor::~TrajectoryExecutor() {
    stop();
    if (execution_thread_.joinable()) {
        execution_thread_.join();
    }
}

bool TrajectoryExecutor::load(const PlannedTrajectory& trajectory) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ != ExecutorState::Idle) {
        return false;
    }

    if (!trajectory.valid || trajectory.samples.empty()) {
        return false;
    }

    trajectory_ = std::make_unique<PlannedTrajectory>(trajectory);
    current_time_ = 0.0;
    current_index_ = 0;

    // Diagnostic dump: per-joint range and max single-step jump for the loaded trajectory
    if (!trajectory_->samples.empty()) {
        const double kRad = 180.0 / 3.14159265358979323846;
        double jmin[6], jmax[6], jmax_step[6];
        size_t jmax_step_idx[6];
        for (int j = 0; j < 6; ++j) {
            jmin[j] = jmax[j] = trajectory_->samples[0].joints[j];
            jmax_step[j] = 0.0;
            jmax_step_idx[j] = 0;
        }
        for (size_t i = 0; i < trajectory_->samples.size(); ++i) {
            const auto& q = trajectory_->samples[i].joints;
            for (int j = 0; j < 6; ++j) {
                if (q[j] < jmin[j]) jmin[j] = q[j];
                if (q[j] > jmax[j]) jmax[j] = q[j];
                if (i > 0) {
                    double step = std::abs(q[j] - trajectory_->samples[i - 1].joints[j]);
                    if (step > jmax_step[j]) {
                        jmax_step[j] = step;
                        jmax_step_idx[j] = i;
                    }
                }
            }
        }
        const auto& s0 = trajectory_->samples.front().joints;
        const auto& sN = trajectory_->samples.back().joints;
        spdlog::info("Executor::load: {} samples, duration={:.3f}s",
                     trajectory_->samples.size(), trajectory_->total_duration);
        spdlog::info("  sample[0]    deg: [{:8.2f}, {:8.2f}, {:8.2f}, {:8.2f}, {:8.2f}, {:8.2f}]",
                     s0[0]*kRad, s0[1]*kRad, s0[2]*kRad, s0[3]*kRad, s0[4]*kRad, s0[5]*kRad);
        spdlog::info("  sample[last] deg: [{:8.2f}, {:8.2f}, {:8.2f}, {:8.2f}, {:8.2f}, {:8.2f}]",
                     sN[0]*kRad, sN[1]*kRad, sN[2]*kRad, sN[3]*kRad, sN[4]*kRad, sN[5]*kRad);
        for (int j = 0; j < 6; ++j) {
            spdlog::info("  J{} range=[{:8.2f}, {:8.2f}] = {:7.2f}deg  max_step={:7.4f}deg @ sample {}",
                         j+1, jmin[j]*kRad, jmax[j]*kRad,
                         (jmax[j]-jmin[j])*kRad, jmax_step[j]*kRad, jmax_step_idx[j]);
        }
    }

    setState(ExecutorState::Ready);
    return true;
}

bool TrajectoryExecutor::start() {
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (state_ != ExecutorState::Ready && state_ != ExecutorState::Paused) {
            return false;
        }

        if (!trajectory_) {
            return false;
        }

        should_stop_ = false;
        should_pause_ = false;
    }

    // Start or resume execution thread
    if (!thread_running_) {
        if (execution_thread_.joinable()) {
            execution_thread_.join();
        }
        thread_running_ = true;
        execution_thread_ = std::thread(&TrajectoryExecutor::executionLoop, this);
    } else {
        // Resume from pause
        should_pause_ = false;
    }

    setState(ExecutorState::Running);
    return true;
}

void TrajectoryExecutor::pause() {
    if (state_ == ExecutorState::Running) {
        should_pause_ = true;
        setState(ExecutorState::Paused);
    }
}

void TrajectoryExecutor::resume() {
    if (state_ == ExecutorState::Paused) {
        should_pause_ = false;
        setState(ExecutorState::Running);
    }
}

void TrajectoryExecutor::stop() {
    should_stop_ = true;
    should_pause_ = false;

    // Wait for thread to finish
    if (execution_thread_.joinable()) {
        execution_thread_.join();
    }
    thread_running_ = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        trajectory_.reset();
        current_time_ = 0.0;
        current_index_ = 0;
    }

    setState(ExecutorState::Idle);
}

ExecutorState TrajectoryExecutor::state() const {
    return state_.load();
}

ExecutorProgress TrajectoryExecutor::progress() const {
    std::lock_guard<std::mutex> lock(mutex_);

    ExecutorProgress prog;
    prog.state = state_.load();
    prog.current_time = current_time_;

    if (trajectory_) {
        prog.total_duration = trajectory_->total_duration;
        prog.total_samples = trajectory_->samples.size();
        prog.current_sample_index = current_index_;

        if (prog.total_duration > 0.0) {
            prog.progress_percent = (current_time_ / prog.total_duration) * 100.0;
            prog.progress_percent = std::clamp(prog.progress_percent, 0.0, 100.0);
        }

        // Get current sample data
        if (!trajectory_->samples.empty() && current_index_ < trajectory_->samples.size()) {
            const auto& sample = trajectory_->samples[current_index_];
            prog.current_joints = sample.joints;
            prog.current_pose = sample.pose;
            prog.current_speed = sample.speed;
        }
    }

    return prog;
}

bool TrajectoryExecutor::hasTrajectory() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return trajectory_ != nullptr;
}

bool TrajectoryExecutor::isRunning() const {
    return state_ == ExecutorState::Running;
}

void TrajectoryExecutor::setCommandCallback(CommandCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    command_callback_ = std::move(callback);
}

void TrajectoryExecutor::setStateCallback(StateCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_callback_ = std::move(callback);
}

bool TrajectoryExecutor::setConfig(const ExecutorConfig& config) {
    if (state_ != ExecutorState::Idle) {
        return false;
    }
    config_ = config;
    return true;
}

void TrajectoryExecutor::executionLoop() {
    using namespace std::chrono;

    const auto period = duration<double>(1.0 / config_.control_rate);
    auto next_time = steady_clock::now();

    while (!should_stop_) {
        // Wait for next control period
        std::this_thread::sleep_until(next_time);
        next_time += duration_cast<steady_clock::duration>(period);

        // Check for pause
        if (should_pause_) {
            // Hold current position while paused
            if (command_callback_ && !config_.dry_run) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (trajectory_ && current_index_ < trajectory_->samples.size()) {
                    command_callback_(trajectory_->samples[current_index_].joints,
                                      current_time_);
                }
            }
            continue;
        }

        // Check if we're done
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (!trajectory_ || current_time_ >= trajectory_->total_duration) {
                // Trajectory complete
                setState(ExecutorState::Idle);
                trajectory_.reset();
                break;
            }

            // Get current sample
            TrajectorySample sample = sampleAt(current_time_);

            // Send command
            if (command_callback_ && !config_.dry_run) {
                command_callback_(sample.joints, current_time_);
            }

            // Advance time
            current_time_ += 1.0 / config_.control_rate;

            // Update index for progress tracking
            current_index_ = static_cast<size_t>(
                current_time_ / trajectory_->sample_period);
            current_index_ = std::min(current_index_, trajectory_->samples.size() - 1);
        }
    }

    thread_running_ = false;
}

void TrajectoryExecutor::setState(ExecutorState new_state) {
    ExecutorState old_state = state_.exchange(new_state);

    if (old_state != new_state && state_callback_) {
        state_callback_(new_state);
    }
}

TrajectorySample TrajectoryExecutor::sampleAt(double t) const {
    // Use the PlannedTrajectory's sampleAt method
    if (trajectory_) {
        return trajectory_->sampleAt(t);
    }
    return TrajectorySample{};
}

}  // namespace trajectory
}  // namespace ur_controller
