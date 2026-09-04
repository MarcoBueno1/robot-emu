// include/robot/websocket/websocket_connection.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <expected>
#include <string_view>
#include <vector>
#include "robot/protocol/tcp_connection.hpp"
#include "robot/websocket/websocket_error.hpp"

namespace robot::websocket {

/// @brief A minimal, server-side subset of RFC 6455 built directly on
///        robot::protocol::TcpConnection.
///
/// Implements exactly what a modern browser's WebSocket client needs from
/// this side: the opening HTTP-upgrade handshake, and unmasked
/// server-to-client text frames. Does not implement binary frames,
/// fragmentation, ping/pong keepalive, or full validation of every
/// client-frame edge case — see
/// docs/task-briefs/phase-12-visualization.md Non-Goals.
class WebSocketConnection {
public:
    WebSocketConnection(const WebSocketConnection&) = delete;
    WebSocketConnection& operator=(const WebSocketConnection&) = delete;
    WebSocketConnection(WebSocketConnection&&) noexcept = default;
    WebSocketConnection& operator=(WebSocketConnection&&) noexcept = default;

    /// @brief Performs the server-side RFC 6455 opening handshake on an
    ///        already-accepted TCP connection.
    ///
    /// Reads the client's HTTP upgrade request, computes
    /// Sec-WebSocket-Accept = base64(sha1(Sec-WebSocket-Key + the RFC 6455
    /// GUID "258EAFA5-E914-47DA-95CA-C5AB0DC85B11")), and writes the HTTP
    /// 101 response.
    /// @return A ready-to-use connection, or WebSocketError::HandshakeFailed
    ///         if the request is missing/malformed, or
    ///         WebSocketError::TransportFailure on a lower-level I/O error.
    [[nodiscard]] static std::expected<WebSocketConnection, WebSocketError> accept(
        robot::protocol::TcpConnection connection);

    /// @brief Sends text as one unmasked, unfragmented WebSocket text frame.
    [[nodiscard]] std::expected<void, WebSocketError> sendText(std::string_view text);

    /// @brief Blocks for one incoming frame from the client.
    ///
    /// Only used to detect a close frame; any other frame's payload is
    /// read (unmasking it correctly, since client frames are always
    /// masked per RFC 6455) and discarded — this viewer never needs the
    /// browser to send it anything beyond opening/closing the connection.
    /// @return Success if a non-close frame was read and discarded, or
    ///         WebSocketError::ConnectionClosed if it was a close frame
    ///         (or the underlying transport closed), or
    ///         WebSocketError::TransportFailure on a lower-level I/O error.
    [[nodiscard]] std::expected<void, WebSocketError> pollForClose();

private:
    explicit WebSocketConnection(robot::protocol::TcpConnection connection) noexcept;

    // Reads exactly count bytes, correctly carrying over any excess bytes
    // a single underlying recv() call returned beyond what was needed —
    // without this, bytes belonging to a later logical read (e.g. a close
    // frame's mask key, read right after its 2-byte header) could be
    // silently consumed and discarded by an earlier read that only needed
    // part of what the OS handed back in one call.
    [[nodiscard]] std::expected<std::vector<std::byte>, WebSocketError> readExactBuffered(std::size_t count);

    robot::protocol::TcpConnection connection_;
    std::vector<std::byte> residual_;
};

}  // namespace robot::websocket
