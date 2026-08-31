// src/sensors/current_sensor.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/sensors/current_sensor.hpp"

#include <cmath>

namespace robot::sensors {

using robot::hardware::HardwareError;

std::expected<CurrentSensor, HardwareError> CurrentSensor::create(double maxSafeAmps) noexcept {
    if (maxSafeAmps <= 0.0) {
        return std::unexpected(HardwareError::InvalidConfiguration);
    }
    return CurrentSensor(maxSafeAmps);
}

CurrentSensor::CurrentSensor(double maxSafeAmps) noexcept : maxSafeAmps_(maxSafeAmps) {}

bool CurrentSensor::isOverCurrent(double amps) const noexcept {
    return std::abs(amps) > maxSafeAmps_;
}

}  // namespace robot::sensors
