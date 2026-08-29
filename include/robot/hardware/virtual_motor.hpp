// include/robot/hardware/virtual_motor.hpp
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
#include "robot/hardware/hardware_error.hpp"

namespace robot::hardware {

/// @brief Simulates a motor's first-order electrical/mechanical lag
///        between a commanded torque and the actual torque it produces.
///
/// Real motors don't reach a new torque instantly. update() advances
/// torque() toward commandedTorque() using the exact analytic solution of
/// a first-order lag (torque_new = commanded + (torque_old - commanded) *
/// exp(-dt/tau)) — exact for any dt, including large ones, unlike a naive
/// per-step (Euler) approximation.
class VirtualMotor {
public:
    /// @brief Constructs a motor with the given torque limit and response lag.
    /// @param max_torque Maximum |torque| this motor can be commanded to.
    ///        Must be > 0.
    /// @param time_constant The lag's time constant (tau): the time to
    ///        close ~63.2% (1 - 1/e) of the gap to a new commanded torque.
    ///        Must be > 0.
    /// @return A valid motor, or HardwareError::InvalidConfiguration if
    ///         either parameter is not positive.
    [[nodiscard]] static std::expected<VirtualMotor, HardwareError> create(
        double max_torque, std::chrono::nanoseconds time_constant) noexcept;

    /// @brief Current actual torque. Starts at 0.
    [[nodiscard]] double torque() const noexcept;

    /// @brief Current commanded torque (setpoint). Starts at 0.
    [[nodiscard]] double commandedTorque() const noexcept;

    /// @brief Sets a new commanded torque for update() to approach.
    /// @param torque New setpoint.
    /// @return Success, or HardwareError::OutOfRange if |torque| exceeds
    ///         max_torque — commandedTorque() is unchanged on failure.
    [[nodiscard]] std::expected<void, HardwareError> setCommandedTorque(double torque) noexcept;

    /// @brief Advances torque() toward commandedTorque() by dt, using the
    ///        exact first-order lag solution described on the class.
    /// @param dt Time step to advance.
    void update(std::chrono::nanoseconds dt) noexcept;

private:
    VirtualMotor(double max_torque, double time_constant_s) noexcept;

    double max_torque_;
    double time_constant_s_;
    double torque_ = 0.0;
    double commanded_torque_ = 0.0;
};

}  // namespace robot::hardware
