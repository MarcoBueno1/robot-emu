// include/robot/hardware/digital_io.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <cstddef>
#include <expected>
#include <vector>
#include "robot/hardware/hardware_error.hpp"

namespace robot::hardware {

/// @brief A fixed-size, bounds-checked array of digital (boolean) I/O channels.
///
/// Raw, index-addressed — no semantic meaning attached to a channel index
/// (e.g. "channel 3 = gripper solenoid" is a configuration-layer concern,
/// not this type's). channel_count == 0 is a valid, if useless,
/// configuration: every read()/write() on it simply returns OutOfRange.
class DigitalIO {
public:
    /// @brief Constructs channel_count channels, all initially false.
    explicit DigitalIO(std::size_t channel_count) noexcept;

    /// @brief Number of channels.
    [[nodiscard]] std::size_t channelCount() const noexcept;

    /// @brief Reads a channel's current value.
    /// @return The value, or HardwareError::OutOfRange if channel >= channelCount().
    [[nodiscard]] std::expected<bool, HardwareError> read(std::size_t channel) const noexcept;

    /// @brief Writes a channel's value.
    /// @return Success, or HardwareError::OutOfRange if channel >= channelCount().
    [[nodiscard]] std::expected<void, HardwareError> write(std::size_t channel, bool value) noexcept;

private:
    std::vector<bool> channels_;
};

}  // namespace robot::hardware
