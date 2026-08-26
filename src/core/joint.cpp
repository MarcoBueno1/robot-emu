// src/core/joint.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/core/joint.hpp"

#include <algorithm>

namespace robot::core {

namespace {
// Proportional gain used to convert a position error into a desired velocity
// setpoint. This is intentionally simple (Phase 1 scope) — a real trajectory
// planner (S-curve/trapezoidal) is Phase 4's job.
constexpr double kPositionGain = 8.0;
}  // namespace

Joint::Joint(JointLimits limits) noexcept : limits_(limits) {}

void Joint::enable() noexcept {
    enabled_ = true;
}

void Joint::disable() noexcept {
    enabled_ = false;
}

void Joint::home() noexcept {
    state_.position     = 0.0;
    state_.velocity      = 0.0;
    state_.acceleration  = 0.0;
    target_position_     = 0.0;
    target_velocity_     = 0.0;
    homed_                = true;
}

std::expected<void, JointError> Joint::setTargetPosition(double position) noexcept {
    if (!enabled_) {
        return std::unexpected(JointError::JointDisabled);
    }
    if (position < limits_.min_position || position > limits_.max_position) {
        return std::unexpected(JointError::PositionOutOfRange);
    }
    target_position_ = position;
    return {};
}

std::expected<void, JointError> Joint::setTargetVelocity(double velocity) noexcept {
    if (!enabled_) {
        return std::unexpected(JointError::JointDisabled);
    }
    if (velocity < -limits_.max_velocity || velocity > limits_.max_velocity) {
        return std::unexpected(JointError::VelocityOutOfRange);
    }
    target_velocity_ = velocity;
    return {};
}

void Joint::update(std::chrono::nanoseconds dt) noexcept {
    if (!enabled_ || faulted_) {
        return;
    }

    const double dt_s = std::chrono::duration<double>(dt).count();
    if (dt_s <= 0.0) {
        return;
    }

    // Saturated proportional controller: convert position error into a
    // velocity setpoint, clamp to max_velocity, then ramp actual velocity
    // toward that setpoint limited by max_acceleration.
    const double position_error = target_position_ - state_.position;
    double desired_velocity = kPositionGain * position_error;
    desired_velocity = std::clamp(desired_velocity, -limits_.max_velocity, limits_.max_velocity);

    const double max_delta_v = limits_.max_acceleration * dt_s;
    const double velocity_error = desired_velocity - state_.velocity;
    const double delta_v = std::clamp(velocity_error, -max_delta_v, max_delta_v);

    state_.acceleration = delta_v / dt_s;
    state_.velocity += delta_v;
    state_.velocity = std::clamp(state_.velocity, -limits_.max_velocity, limits_.max_velocity);

    state_.position += state_.velocity * dt_s;

    clampToLimits();
}

void Joint::clampToLimits() noexcept {
    if (state_.position > limits_.max_position) {
        state_.position = limits_.max_position;
        state_.velocity = std::min(state_.velocity, 0.0);
    } else if (state_.position < limits_.min_position) {
        state_.position = limits_.min_position;
        state_.velocity = std::max(state_.velocity, 0.0);
    }
}

}
