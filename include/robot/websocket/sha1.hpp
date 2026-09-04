// include/robot/websocket/sha1.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <array>
#include <cstdint>
#include <string_view>

namespace robot::websocket {

/// @brief Computes the SHA-1 digest of input.
///
/// SHA-1 is not (and should not be) used anywhere for security in this
/// codebase — it exists solely because the RFC 6455 WebSocket handshake
/// mandates it for computing Sec-WebSocket-Accept, and no SHA-1
/// implementation exists in the standard library or in any dependency
/// this project permits (docs/architecture.md section 3.2).
/// @return The 20-byte digest.
[[nodiscard]] std::array<std::uint8_t, 20> sha1(std::string_view input) noexcept;

}  // namespace robot::websocket
