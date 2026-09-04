// include/robot/websocket/websocket_error.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::websocket {

/// @brief Errors returned by WebSocketConnection.
///
/// Used exclusively with std::expected — no exceptions on the control
/// path (see CONTRIBUTING.md and docs/architecture.md section 3.15).
enum class WebSocketError {
    HandshakeFailed,   ///< The client's HTTP upgrade request was missing/malformed.
    TransportFailure,  ///< The underlying TCP connection failed.
    ConnectionClosed,  ///< The client sent a close frame, or the connection dropped.
};

}  // namespace robot::websocket
