// include/robot/cli/move_joint_payload.hpp
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
#include <cstdint>
#include <expected>
#include <span>
#include <vector>
#include "robot/cli/cli_error.hpp"

namespace robot::cli {

/// @brief The payload of a MoveJoint request.
struct MoveJointPayload {
    std::uint8_t jointIndex;
    double targetRadians;
};

/// @brief Serializes payload to bytes.
///
/// Wire layout: u8 joint index, f64 target (big-endian IEEE-754 bit
/// pattern, same convention as StatusPayload's doubles).
[[nodiscard]] std::vector<std::byte> encodeMoveJointPayload(const MoveJointPayload& payload);

/// @brief Parses bytes produced by encodeMoveJointPayload().
/// @return The decoded MoveJointPayload, or CliError::TruncatedPayload if
///         bytes is shorter than the fixed 9-byte layout requires.
[[nodiscard]] std::expected<MoveJointPayload, CliError> decodeMoveJointPayload(std::span<const std::byte> bytes);

}  // namespace robot::cli
