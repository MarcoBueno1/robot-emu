// include/robot/protocol/command_type.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <cstdint>
#include <string_view>

namespace robot::protocol {

/// @brief The fourteen named commands from docs/architecture.md section 3.7.
///
/// Backed by std::uint16_t to match the wire format's TYPE field exactly.
/// 0 is deliberately unused — an all-zero header is never a valid
/// CommandType, which helps distinguish a garbled/truncated frame from a
/// real one during decode.
enum class CommandType : std::uint16_t {
    Connect       = 1,
    GetStatus     = 2,
    Enable        = 3,
    Disable       = 4,
    Home          = 5,
    MoveJoint     = 6,
    MoveLinear    = 7,
    Stop          = 8,
    EmergencyStop = 9,
    GetPosition   = 10,
    GetIO         = 11,
    SetIO         = 12,
    ResetFault    = 13,
    InjectFault   = 14,
};

/// @brief Whether raw corresponds to one of CommandType's enumerators.
///
/// Used by FrameCodec::decode() to reject a wire TYPE value that doesn't
/// map to any known command, rather than silently reinterpreting it.
[[nodiscard]] bool isKnownCommandType(std::uint16_t raw) noexcept;

/// @brief Human-readable name for a CommandType, e.g. for logging.
[[nodiscard]] std::string_view toString(CommandType type) noexcept;

}  // namespace robot::protocol
