// include/robot/protocol/protocol_error.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::protocol {

/// @brief Errors returned by FrameCodec::decode().
///
/// Used exclusively with std::expected — no exceptions on the control
/// path (see CONTRIBUTING.md and docs/architecture.md section 3.15).
enum class ProtocolError {
    /// Not enough bytes yet to decode a full frame — not a malformed-frame
    /// error. The caller (typically a TCP receive loop) should buffer more
    /// bytes and retry decode() later.
    Incomplete,
    /// The header's MAGIC field did not match kProtocolMagic.
    InvalidMagic,
    /// The header's VERSION field did not match kProtocolVersion.
    UnsupportedVersion,
    /// The header's TYPE field did not match any robot::protocol::CommandType.
    UnknownCommandType,
    /// The header's LENGTH field exceeds FrameCodec::maxPayloadSize.
    PayloadTooLarge,
};

}  // namespace robot::protocol
