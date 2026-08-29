// include/robot/protocol/frame_codec.hpp
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
#include <expected>
#include <span>
#include <vector>
#include "robot/protocol/frame.hpp"
#include "robot/protocol/protocol_error.hpp"

namespace robot::protocol {

/// @brief Fixed magic constant every frame's MAGIC field must equal. ASCII "ROBO".
inline constexpr std::uint32_t kProtocolMagic = 0x524F424F;

/// @brief The only protocol version this phase encodes/decodes. See
///        docs/task-briefs/phase-06-protocol.md Non-Goals for why
///        multi-version support isn't implemented yet.
inline constexpr std::uint8_t kProtocolVersion = 1;

/// @brief Pure encode/decode of the wire format from
///        docs/architecture.md section 3.7: MAGIC(u32) | VERSION(u8) |
///        FLAGS(u8) | TYPE(u16) | LENGTH(u32) | PAYLOAD(LENGTH bytes),
///        all multi-byte fields big-endian. No I/O — this class knows
///        nothing about sockets; see TcpConnection for that layer.
class FrameCodec {
public:
    /// @brief Size of the fixed header, in bytes: 4+1+1+2+4.
    static constexpr std::size_t headerSize = 12;

    /// @brief Safety cap on LENGTH — a decode() whose header claims a
    ///        larger payload is rejected before any payload bytes are read.
    static constexpr std::uint32_t maxPayloadSize = 1'048'576;  // 1 MiB

    /// @brief Serializes frame to wire bytes, always using kProtocolMagic
    ///        and kProtocolVersion.
    [[nodiscard]] static std::vector<std::byte> encode(const Frame& frame);

    /// @brief Result of a successful decode(): the parsed Frame, plus how
    ///        many bytes of the input it actually consumed.
    struct DecodeResult {
        Frame frame;
        /// Exactly headerSize + frame.payload.size(). Bytes in the input
        /// span beyond this offset are untouched and may be the start of
        /// a subsequent frame — this is what makes decode() usable
        /// directly against a streaming socket receive buffer.
        std::size_t bytesConsumed;
    };

    /// @brief Decodes exactly one frame from the front of bytes.
    /// @param bytes Input buffer. May contain trailing bytes past one
    ///        frame — see DecodeResult::bytesConsumed.
    /// @return A DecodeResult, or a ProtocolError. ProtocolError::Incomplete
    ///         means "not a malformed frame, just not enough bytes yet" —
    ///         every other value means the input is genuinely invalid.
    [[nodiscard]] static std::expected<DecodeResult, ProtocolError> decode(std::span<const std::byte> bytes);
};

}  // namespace robot::protocol
