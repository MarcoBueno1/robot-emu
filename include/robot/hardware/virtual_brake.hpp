// include/robot/hardware/virtual_brake.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::hardware {

/// @brief State of a VirtualBrake.
enum class BrakeState {
    Engaged,   ///< Holding — the fail-safe default.
    Released,  ///< Free to move.
};

/// @brief Simulates a normally-engaged ("fail-safe") holding brake.
///
/// Unpowered/unconfigured, a real fail-safe brake holds — that's the whole
/// point of the design (power loss must not let a joint fall/drift).
/// engage()/release() are unconditional: no precondition, no failure mode,
/// because Phase 8's safety system must be able to engage every brake
/// robot-wide regardless of any other component's state.
class VirtualBrake {
public:
    /// @brief Constructs a brake starting in BrakeState::Engaged.
    VirtualBrake() noexcept;

    /// @brief Current state.
    [[nodiscard]] BrakeState state() const noexcept;

    /// @brief Convenience for state() == BrakeState::Engaged.
    [[nodiscard]] bool isEngaged() const noexcept;

    /// @brief Engages the brake. Idempotent — safe to call while already engaged.
    void engage() noexcept;

    /// @brief Releases the brake. Idempotent — safe to call while already released.
    void release() noexcept;

private:
    BrakeState state_ = BrakeState::Engaged;
};

}  // namespace robot::hardware
