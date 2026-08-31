// include/robot/sensors/proximity_sensor.hpp
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

/// @brief Stateless digital proximity classifier.
class ProximitySensor {
public:
    /// @param triggerDistance Must be > 0.
    /// @return A valid sensor, or HardwareError::InvalidConfiguration if
    ///         triggerDistance is not positive.
    [[nodiscard]] static std::expected<ProximitySensor, robot::hardware::HardwareError> create(
        double triggerDistance) noexcept;

    /// @return distance <= triggerDistance. Callers are expected to pass
    ///         a non-negative distance; this class does not separately
    ///         validate that at the call site (a negative reading would
    ///         simply always be treated as triggered).
    [[nodiscard]] bool isTriggered(double distance) const noexcept;

private:
    explicit ProximitySensor(double triggerDistance) noexcept;

    double triggerDistance_;
};

}  // namespace robot::sensors
