// include/robot/websocket/base64.hpp
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
#include <span>
#include <string>

namespace robot::websocket {

/// @brief Standard (RFC 4648) Base64 encoding, with padding.
///
/// Exists for the same reason sha1() does — the WebSocket handshake needs
/// it, and no dependency this project permits provides it.
[[nodiscard]] std::string base64Encode(std::span<const std::uint8_t> input);

}  // namespace robot::websocket
