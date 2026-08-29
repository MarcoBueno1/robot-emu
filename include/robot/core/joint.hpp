// include/robot/core/joint.hpp
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
#include "robot/core/joint_state.hpp"

namespace robot::core {

/// @brief A single robot joint: physical state (position, velocity,
///        acceleration, torque) plus configurable safety limits.
///
/// Joint owns no clock and does no I/O — it is advanced purely by explicit
/// calls to update(dt), making it deterministic and easy to unit test.
/// Internally, update() uses a simple saturated proportional controller
/// toward the current target; a real trajectory planner (S-curve/
/// trapezoidal) is out of scope here and lands in Phase 4 (Motion).
///
/// Thread-safety: none — Joint does no internal synchronization by design
/// (see docs/task-briefs/phase-01-core.md, "Thread model"). A single owner
/// thread is expected to call its mutating methods.
class Joint {
public:
    /// @brief Constructs a joint with the given limits.
    ///
    /// Starts disabled, not homed, not faulted, at position/velocity/
    /// acceleration/torque all zero.
    /// @param limits Position/velocity/acceleration bounds for this joint.
    ///        Not validated here — validate the enclosing RobotConfig (or
    ///        the JointLimits directly) before constructing, if needed.
    explicit Joint(JointLimits limits) noexcept;

    // --- State reads (no side effects) ---

    /// @brief Current position, in radians.
    [[nodiscard]] double position() const noexcept     { return state_.position; }
    /// @brief Current velocity, in radians/second.
    [[nodiscard]] double velocity() const noexcept     { return state_.velocity; }
    /// @brief Current acceleration, in radians/second^2.
    [[nodiscard]] double acceleration() const noexcept { return state_.acceleration; }
    /// @brief Current torque, in N·m (placeholder until Phase 5 — Virtual Hardware).
    [[nodiscard]] double torque() const noexcept       { return state_.torque; }

    /// @brief The position/velocity/acceleration limits this joint was
    ///        constructed with. Added in Phase 8 for robot::safety::LimitMonitor
    ///        — Phase 1 never needed to read a limit back, only enforce it.
    [[nodiscard]] const JointLimits& limits() const noexcept { return limits_; }

    /// @brief Whether the joint is enabled. A disabled joint ignores its
    ///        target and does not move on update().
    [[nodiscard]] bool enabled() const noexcept { return enabled_; }
    /// @brief Whether the joint has been homed (see home()).
    [[nodiscard]] bool homed() const noexcept   { return homed_; }
    /// @brief Whether the joint is in a fault state. A faulted joint does
    ///        not move on update() (fault handling proper is Phase 8's job).
    [[nodiscard]] bool faulted() const noexcept { return faulted_; }

    // --- Commands ---

    /// @brief Enables the joint, allowing update() to move it toward its target.
    void enable() noexcept;
    /// @brief Disables the joint. A subsequent update() will not move it,
    ///        even if a target was set before disabling.
    void disable() noexcept;

    /// @brief Homes the joint.
    ///
    /// Synchronous and deterministic in Phase 1 (no real homing trajectory
    /// yet — that's expanded in Phase 4/5): resets position, velocity,
    /// acceleration, and both targets to zero, and sets homed() to true.
    void home() noexcept;

    /// @brief Sets a new target position for the joint to move toward.
    /// @param position Target position, in radians.
    /// @return Success, or JointError::JointDisabled if the joint is not
    ///         enabled, or JointError::PositionOutOfRange if position falls
    ///         outside this joint's limits. On failure, state is unchanged.
    [[nodiscard]] std::expected<void, JointError> setTargetPosition(double position) noexcept;

    /// @brief Sets a new target velocity for the joint to move toward.
    /// @param velocity Target velocity, in radians/second.
    /// @return Success, or JointError::JointDisabled if the joint is not
    ///         enabled, or JointError::VelocityOutOfRange if the magnitude
    ///         exceeds max_velocity. On failure, state is unchanged.
    [[nodiscard]] std::expected<void, JointError> setTargetVelocity(double velocity) noexcept;

    /// @brief Advances the joint's physical state by dt.
    ///
    /// No-op if the joint is disabled or faulted. Otherwise, ramps velocity
    /// toward a setpoint derived from the position error (clamped to
    /// max_velocity), itself ramped toward by at most max_acceleration * dt,
    /// then integrates position and clamps it to [min_position, max_position].
    /// Performs no heap allocation and reads no clock — dt is always explicit.
    /// @param dt Time step to advance.
    void update(std::chrono::nanoseconds dt) noexcept;

private:
    JointLimits limits_;
    JointState  state_;

    double target_position_ = 0.0;
    double target_velocity_ = 0.0;

    bool enabled_ = false;
    bool homed_   = false;
    bool faulted_ = false;

    /// @brief Clamps state_.position to [min_position, max_position],
    ///        zeroing velocity in the direction of the violated bound.
    ///        Called internally by update() — not exposed publicly.
    void clampToLimits() noexcept;
};

}
