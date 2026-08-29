// include/robot/hardware/analog_io.hpp
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

/// @brief A fixed-size, bounds-checked array of analog (double) I/O channels.
///
/// Raw, index-addressed, like DigitalIO — see its documentation for why no
/// semantic meaning is attached to a channel index here.
class AnalogIO {
public:
    /// @brief Constructs channel_count channels, all initially min_value.
    /// @param channel_count Number of channels.
    /// @param min_value Minimum value any channel may hold.
    /// @param max_value Maximum value any channel may hold. Must be > min_value.
    /// @return A valid AnalogIO, or HardwareError::InvalidConfiguration if
    ///         min_value >= max_value.
    [[nodiscard]] static std::expected<AnalogIO, HardwareError> create(
        std::size_t channel_count, double min_value, double max_value) noexcept;

    /// @brief Number of channels.
    [[nodiscard]] std::size_t channelCount() const noexcept;

    /// @brief Reads a channel's current value.
    /// @return The value, or HardwareError::OutOfRange if channel >= channelCount().
    [[nodiscard]] std::expected<double, HardwareError> read(std::size_t channel) const noexcept;

    /// @brief Writes a channel's value.
    /// @return Success, or HardwareError::OutOfRange if channel >=
    ///         channelCount(), or if value falls outside
    ///         [min_value, max_value] — either way, the channel's prior
    ///         value is left unchanged on failure.
    [[nodiscard]] std::expected<void, HardwareError> write(std::size_t channel, double value) noexcept;

private:
    AnalogIO(std::size_t channel_count, double min_value, double max_value) noexcept;

    double min_value_;
    double max_value_;
    std::vector<double> channels_;
};

}  // namespace robot::hardware
