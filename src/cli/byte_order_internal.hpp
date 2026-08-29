// src/cli/byte_order_internal.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
// Private implementation helper — not installed under include/, not part of
// robot_cli's public interface. Shared by status_payload.cpp and
// move_joint_payload.cpp to avoid duplicating the same handful of lines
// twice; deliberately NOT added to robot_protocol/FrameCodec, since command
// payload formats are robot_cli's concern, not the framing layer's (see
// docs/task-briefs/phase-06-protocol.md's Non-Goals).
#pragma once
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace robot::cli::internal {

inline void writeU16BE(std::vector<std::byte>& out, std::uint16_t value) {
    out.push_back(static_cast<std::byte>((value >> 8) & 0xFF));
    out.push_back(static_cast<std::byte>(value & 0xFF));
}

inline std::uint16_t readU16BE(std::span<const std::byte> bytes) noexcept {
    return static_cast<std::uint16_t>((std::to_integer<std::uint16_t>(bytes[0]) << 8) |
                                       std::to_integer<std::uint16_t>(bytes[1]));
}

inline void writeF64BE(std::vector<std::byte>& out, double value) {
    const auto bits = std::bit_cast<std::uint64_t>(value);
    for (int shift = 56; shift >= 0; shift -= 8) {
        out.push_back(static_cast<std::byte>((bits >> shift) & 0xFF));
    }
}

inline double readF64BE(std::span<const std::byte> bytes) noexcept {
    std::uint64_t bits = 0;
    for (int i = 0; i < 8; ++i) {
        bits = (bits << 8) | std::to_integer<std::uint64_t>(bytes[static_cast<std::size_t>(i)]);
    }
    return std::bit_cast<double>(bits);
}

}  // namespace robot::cli::internal
