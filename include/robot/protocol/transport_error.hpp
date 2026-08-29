// include/robot/protocol/transport_error.hpp
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

/// @brief Errors returned by TcpListener/TcpConnection.
///
/// Used exclusively with std::expected — no exceptions on the control
/// path (see CONTRIBUTING.md and docs/architecture.md section 3.15).
enum class TransportError {
    BindFailed,
    ListenFailed,
    AcceptFailed,
    ConnectFailed,
    SendFailed,
    ReceiveFailed,
    /// The peer closed the connection (a receive() that reads 0 bytes).
    ConnectionClosed,
};

}  // namespace robot::protocol
