// src/protocol/frame_codec.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/protocol/frame_codec.hpp"

#include "robot/protocol/command_type.hpp"

namespace robot::protocol {

// --- CommandType helpers ---

bool isKnownCommandType(std::uint16_t raw) noexcept {
    switch (static_cast<CommandType>(raw)) {
        case CommandType::Connect:
        case CommandType::GetStatus:
        case CommandType::Enable:
        case CommandType::Disable:
        case CommandType::Home:
        case CommandType::MoveJoint:
        case CommandType::MoveLinear:
        case CommandType::Stop:
        case CommandType::EmergencyStop:
        case CommandType::GetPosition:
        case CommandType::GetIO:
        case CommandType::SetIO:
        case CommandType::ResetFault:
        case CommandType::InjectFault:
            return true;
    }
    return false;
}

std::string_view toString(CommandType type) noexcept {
    switch (type) {
        case CommandType::Connect:       return "Connect";
        case CommandType::GetStatus:     return "GetStatus";
        case CommandType::Enable:        return "Enable";
        case CommandType::Disable:       return "Disable";
        case CommandType::Home:          return "Home";
        case CommandType::MoveJoint:     return "MoveJoint";
        case CommandType::MoveLinear:    return "MoveLinear";
        case CommandType::Stop:          return "Stop";
        case CommandType::EmergencyStop: return "EmergencyStop";
        case CommandType::GetPosition:   return "GetPosition";
        case CommandType::GetIO:         return "GetIO";
        case CommandType::SetIO:         return "SetIO";
        case CommandType::ResetFault:    return "ResetFault";
        case CommandType::InjectFault:   return "InjectFault";
    }
    return "Unknown";
}

// --- Big-endian byte-order helpers (no OS headers — see brief section 8) ---

namespace {

void writeU16BE(std::vector<std::byte>& out, std::uint16_t value) {
    out.push_back(static_cast<std::byte>((value >> 8) & 0xFF));
    out.push_back(static_cast<std::byte>(value & 0xFF));
}

void writeU32BE(std::vector<std::byte>& out, std::uint32_t value) {
    out.push_back(static_cast<std::byte>((value >> 24) & 0xFF));
    out.push_back(static_cast<std::byte>((value >> 16) & 0xFF));
    out.push_back(static_cast<std::byte>((value >> 8) & 0xFF));
    out.push_back(static_cast<std::byte>(value & 0xFF));
}

[[nodiscard]] std::uint16_t readU16BE(std::span<const std::byte> bytes) noexcept {
    return static_cast<std::uint16_t>((std::to_integer<std::uint16_t>(bytes[0]) << 8) |
                                       std::to_integer<std::uint16_t>(bytes[1]));
}

[[nodiscard]] std::uint32_t readU32BE(std::span<const std::byte> bytes) noexcept {
    return (std::to_integer<std::uint32_t>(bytes[0]) << 24) |
           (std::to_integer<std::uint32_t>(bytes[1]) << 16) |
           (std::to_integer<std::uint32_t>(bytes[2]) << 8) |
           std::to_integer<std::uint32_t>(bytes[3]);
}

}  // namespace

// --- FrameCodec ---

std::vector<std::byte> FrameCodec::encode(const Frame& frame) {
    std::vector<std::byte> out;
    out.reserve(headerSize + frame.payload.size());

    writeU32BE(out, kProtocolMagic);
    out.push_back(static_cast<std::byte>(kProtocolVersion));
    out.push_back(static_cast<std::byte>(frame.flags));
    writeU16BE(out, static_cast<std::uint16_t>(frame.type));
    writeU32BE(out, static_cast<std::uint32_t>(frame.payload.size()));

    out.insert(out.end(), frame.payload.begin(), frame.payload.end());
    return out;
}

std::expected<FrameCodec::DecodeResult, ProtocolError> FrameCodec::decode(std::span<const std::byte> bytes) {
    if (bytes.size() < headerSize) {
        return std::unexpected(ProtocolError::Incomplete);
    }

    const auto magic = readU32BE(bytes.subspan(0, 4));
    if (magic != kProtocolMagic) {
        return std::unexpected(ProtocolError::InvalidMagic);
    }

    const auto version = std::to_integer<std::uint8_t>(bytes[4]);
    if (version != kProtocolVersion) {
        return std::unexpected(ProtocolError::UnsupportedVersion);
    }

    const auto flags = std::to_integer<std::uint8_t>(bytes[5]);
    const auto rawType = readU16BE(bytes.subspan(6, 2));
    if (!isKnownCommandType(rawType)) {
        return std::unexpected(ProtocolError::UnknownCommandType);
    }

    const auto length = readU32BE(bytes.subspan(8, 4));
    if (length > maxPayloadSize) {
        return std::unexpected(ProtocolError::PayloadTooLarge);
    }

    if (bytes.size() < headerSize + length) {
        return std::unexpected(ProtocolError::Incomplete);
    }

    Frame frame;
    frame.flags = flags;
    frame.type = static_cast<CommandType>(rawType);
    frame.payload.assign(bytes.begin() + static_cast<std::ptrdiff_t>(headerSize),
                          bytes.begin() + static_cast<std::ptrdiff_t>(headerSize + length));

    return DecodeResult{std::move(frame), headerSize + length};
}

}  // namespace robot::protocol
