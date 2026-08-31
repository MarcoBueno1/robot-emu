// include/robot/sensors/temperature_sensor.hpp
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

namespace robot::sensors {

/// @brief Simulates a temperature reading with first-order thermal lag.
///
/// Reuses the same exact-analytic-solution technique
/// robot::hardware::VirtualMotor established in Phase 5 for its torque
/// response — exact for any dt, not a per-step (Euler) approximation.
class TemperatureSensor {
public:
    /// @param ambientCelsius Starting temperature, and the value celsius()
    ///        approaches if update() is always called with
    ///        targetCelsius == ambientCelsius.
    /// @param timeConstant Same meaning as VirtualMotor's tau — time to
    ///        close ~63.2% of the gap to a new target. Must be > 0.
    /// @param maxSafeCelsius Threshold isOverTemperature() compares against.
    /// @return A valid sensor, or HardwareError::InvalidConfiguration if
    ///         timeConstant is not positive.
    [[nodiscard]] static std::expected<TemperatureSensor, robot::hardware::HardwareError> create(
        double ambientCelsius, std::chrono::nanoseconds timeConstant, double maxSafeCelsius) noexcept;

    /// @brief Current reading. Starts at ambientCelsius.
    [[nodiscard]] double celsius() const noexcept;

    /// @brief Whether celsius() exceeds maxSafeCelsius.
    [[nodiscard]] bool isOverTemperature() const noexcept;

    /// @brief Advances celsius() toward targetCelsius using the exact
    ///        first-order lag solution described on the class.
    ///
    /// Unlike VirtualMotor (which separates setCommandedTorque() from
    /// update()), the target is passed directly to update() each call —
    /// deliberate: a real heat source fluctuates continuously in a way
    /// that doesn't need a persistent "commanded" concept.
    /// @param dt Time step to advance.
    /// @param targetCelsius The temperature celsius() is heading toward.
    void update(std::chrono::nanoseconds dt, double targetCelsius) noexcept;

private:
    TemperatureSensor(double celsius, double timeConstantS, double maxSafeCelsius) noexcept;

    double celsius_;
    double timeConstantS_;
    double maxSafeCelsius_;
};

}  // namespace robot::sensors
