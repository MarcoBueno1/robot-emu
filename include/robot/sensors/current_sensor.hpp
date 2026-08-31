// include/robot/sensors/current_sensor.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <expected>
#include "robot/hardware/hardware_error.hpp"

namespace robot::sensors {

/// @brief Stateless overcurrent classifier.
class CurrentSensor {
public:
    /// @param maxSafeAmps Must be > 0.
    /// @return A valid sensor, or HardwareError::InvalidConfiguration if
    ///         maxSafeAmps is not positive.
    [[nodiscard]] static std::expected<CurrentSensor, robot::hardware::HardwareError> create(
        double maxSafeAmps) noexcept;

    /// @return |amps| > maxSafeAmps — magnitude-based, since current can
    ///         be negative (e.g. regenerative braking direction) without
    ///         that alone indicating an overcurrent condition.
    [[nodiscard]] bool isOverCurrent(double amps) const noexcept;

private:
    explicit CurrentSensor(double maxSafeAmps) noexcept;

    double maxSafeAmps_;
};

}  // namespace robot::sensors
