// src/sensors/proximity_sensor.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/sensors/proximity_sensor.hpp"

namespace robot::sensors {

using robot::hardware::HardwareError;

std::expected<ProximitySensor, HardwareError> ProximitySensor::create(double triggerDistance) noexcept {
    if (triggerDistance <= 0.0) {
        return std::unexpected(HardwareError::InvalidConfiguration);
    }
    return ProximitySensor(triggerDistance);
}

ProximitySensor::ProximitySensor(double triggerDistance) noexcept : triggerDistance_(triggerDistance) {}

bool ProximitySensor::isTriggered(double distance) const noexcept {
    return distance <= triggerDistance_;
}

}  // namespace robot::sensors
