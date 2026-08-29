// include/robot/cli/status_payload.hpp
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
#include <span>
#include <string>
#include <vector>
#include "robot/cli/cli_error.hpp"

namespace robot::cli {

/// @brief One joint's state, as reported in a Status response.
struct JointStatus {
    double positionRadians;
    double velocityRadiansPerSecond;
};

/// @brief The data carried by a successful GetStatus response, after the
///        1-byte ResponseStatus (see response.hpp).
///
/// controllerStateName is plain text (e.g. "Ready") rather than
/// robot::controller::ControllerState — robot_cli does not depend on
/// robot_controller (see the phase brief's Context for why).
struct StatusPayload {
    std::string robotName;
    std::string controllerStateName;
    std::vector<JointStatus> joints;
};

/// @brief Serializes status to bytes.
///
/// Wire layout (big-endian, same convention as robot::protocol::FrameCodec):
/// u16 name length + name bytes, u16 state-name length + state-name bytes,
/// u16 joint count, then per joint: f64 position + f64 velocity (each a
/// big-endian IEEE-754 bit pattern).
[[nodiscard]] std::vector<std::byte> encodeStatusPayload(const StatusPayload& status);

/// @brief Parses bytes produced by encodeStatusPayload().
/// @return The decoded StatusPayload, or CliError::TruncatedPayload if
///         bytes is shorter than its declared lengths require.
[[nodiscard]] std::expected<StatusPayload, CliError> decodeStatusPayload(std::span<const std::byte> bytes);

}  // namespace robot::cli
