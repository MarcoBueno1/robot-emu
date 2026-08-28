// include/robot/motion/trapezoidal_trajectory.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <chrono>
#include <expected>
#include "robot/core/joint_error.hpp"
#include "robot/core/joint_limits.hpp"
#include "robot/motion/trajectory_sample.hpp"

namespace robot::motion {

/// @brief A precomputed, closed-form trapezoidal (accelerate / cruise /
///        decelerate) or triangular (accelerate / decelerate, no cruise)
///        motion profile between two positions.
///
/// This is the "trajectory planner" box from docs/architecture.md section
/// 3.5's motion pipeline: given a start, a target, and velocity/
/// acceleration bounds, it answers "where will this joint be at time t"
/// exactly and upfront, rather than reacting per-cycle to the current
/// error the way Joint::update() (Phase 1) does. It is not wired into
/// Joint — see docs/task-briefs/phase-04-motion.md's Non-Goals for why.
///
/// A TrapezoidalTrajectory is an immutable value once constructed: all
/// profile-wide quantities (phase durations, distances) are computed once
/// in create(), so sample() is O(1) with no recomputation and no heap
/// allocation.
class TrapezoidalTrajectory {
public:
    /// @brief Builds a trajectory from start_position to target_position,
    ///        bounded by limits.max_velocity/max_acceleration.
    /// @param start_position Starting position.
    /// @param target_position Target position.
    /// @param limits Must satisfy limits.validate() (positive
    ///        max_velocity/max_acceleration); both positions must fall
    ///        within [limits.min_position, limits.max_position].
    /// @return A valid trajectory, or robot::core::JointError::InvalidConfiguration
    ///         (non-positive velocity/acceleration bound, same check as
    ///         JointLimits::validate()) or
    ///         robot::core::JointError::PositionOutOfRange (either position
    ///         outside the limits, same check as Joint::setTargetPosition()).
    [[nodiscard]] static std::expected<TrapezoidalTrajectory, robot::core::JointError> create(
        double start_position, double target_position, const robot::core::JointLimits& limits) noexcept;

    /// @brief Total duration of this trajectory. Zero if start_position ==
    ///        target_position.
    [[nodiscard]] std::chrono::nanoseconds duration() const noexcept;

    /// @brief The state at time t.
    ///
    /// t is clamped into [0, duration()] first: t <= 0 returns the start
    /// state (position = start_position, velocity = 0, acceleration = the
    /// initial acceleration this trajectory begins with); t >=
    /// duration() returns exactly {target_position, 0.0, 0.0} — no
    /// floating-point residue at the endpoint.
    [[nodiscard]] TrajectorySample sample(std::chrono::nanoseconds t) const noexcept;

private:
    TrapezoidalTrajectory(double start_position, double target_position, double sign,
                           double max_acceleration, double accel_duration_s,
                           double cruise_duration_s, double cruise_velocity,
                           double accel_distance) noexcept;

    double start_position_;
    double target_position_;
    double sign_;               // +1, -1, or 0 (zero-distance trajectory)
    double max_acceleration_;   // magnitude, always >= 0
    double accel_duration_s_;   // duration of the accelerate phase, seconds
    double cruise_duration_s_;  // duration of the cruise phase, seconds (0 for triangular)
    double cruise_velocity_;    // magnitude reached at the end of the accelerate phase
    double accel_distance_;     // magnitude of distance covered during the accelerate phase
};

}  // namespace robot::motion
