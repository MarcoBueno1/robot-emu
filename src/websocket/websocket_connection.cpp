// src/websocket/websocket_connection.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/websocket/websocket_connection.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <vector>

#include "robot/websocket/base64.hpp"
#include "robot/websocket/sha1.hpp"

namespace robot::websocket {

namespace {

constexpr std::string_view kRfc6455Guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

// Reads bytes off connection, appending to buffer, until buffer contains
// needle. Returns the offset right after needle ends.
[[nodiscard]] std::expected<std::size_t, WebSocketError> readUntil(robot::protocol::TcpConnection& connection,
                                                                     std::string& buffer, std::string_view needle) {
    std::array<std::byte, 512> chunk{};
    while (true) {
        if (auto pos = buffer.find(needle); pos != std::string::npos) {
            return pos + needle.size();
        }
        auto n = connection.receive(chunk);
        if (!n.has_value()) {
            return std::unexpected(WebSocketError::TransportFailure);
        }
        for (std::size_t i = 0; i < *n; ++i) {
            buffer.push_back(static_cast<char>(chunk[i]));
        }
        if (buffer.size() > 16384) {
            // A real HTTP upgrade request is small; anything this large
            // without the header terminator is malformed, not just slow.
            return std::unexpected(WebSocketError::HandshakeFailed);
        }
    }
}

// Case-insensitive search for "headerName: value\r\n" within request,
// returning the trimmed value.
[[nodiscard]] std::expected<std::string, WebSocketError> findHeaderValue(std::string_view request,
                                                                           std::string_view headerName) {
    std::string lowerRequest(request);
    std::transform(lowerRequest.begin(), lowerRequest.end(), lowerRequest.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::string lowerName(headerName);
    std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    auto pos = lowerRequest.find(lowerName);
    if (pos == std::string::npos) {
        return std::unexpected(WebSocketError::HandshakeFailed);
    }
    auto colon = request.find(':', pos);
    if (colon == std::string_view::npos) {
        return std::unexpected(WebSocketError::HandshakeFailed);
    }
    auto lineEnd = request.find("\r\n", colon);
    if (lineEnd == std::string_view::npos) {
        return std::unexpected(WebSocketError::HandshakeFailed);
    }

    std::string_view value = request.substr(colon + 1, lineEnd - colon - 1);
    auto first = value.find_first_not_of(" \t");
    auto last = value.find_last_not_of(" \t");
    if (first == std::string_view::npos) {
        return std::unexpected(WebSocketError::HandshakeFailed);
    }
    return std::string(value.substr(first, last - first + 1));
}

}  // namespace

std::expected<std::vector<std::byte>, WebSocketError> WebSocketConnection::readExactBuffered(std::size_t count) {
    std::array<std::byte, 512> chunk{};
    while (residual_.size() < count) {
        auto n = connection_.receive(chunk);
        if (!n.has_value()) {
            return std::unexpected(WebSocketError::ConnectionClosed);
        }
        // Append everything the OS handed back, even if more than needed
        // right now — the excess stays in residual_ for the *next* call,
        // rather than being discarded (the bug this member replaced a
        // free-function version of this logic specifically to fix).
        residual_.insert(residual_.end(), chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(*n));
    }
    std::vector<std::byte> out(residual_.begin(), residual_.begin() + static_cast<std::ptrdiff_t>(count));
    residual_.erase(residual_.begin(), residual_.begin() + static_cast<std::ptrdiff_t>(count));
    return out;
}

WebSocketConnection::WebSocketConnection(robot::protocol::TcpConnection connection) noexcept
    : connection_(std::move(connection)) {}

std::expected<WebSocketConnection, WebSocketError> WebSocketConnection::accept(
    robot::protocol::TcpConnection connection) {
    std::string requestBuffer;
    auto headerEnd = readUntil(connection, requestBuffer, "\r\n\r\n");
    if (!headerEnd.has_value()) {
        return std::unexpected(headerEnd.error());
    }

    auto key = findHeaderValue(requestBuffer, "Sec-WebSocket-Key");
    if (!key.has_value()) {
        return std::unexpected(key.error());
    }

    const auto digest = sha1(*key + std::string(kRfc6455Guid));
    const auto acceptValue = base64Encode(std::span<const std::uint8_t>(digest.data(), digest.size()));

    std::string response = "HTTP/1.1 101 Switching Protocols\r\n";
    response += "Upgrade: websocket\r\n";
    response += "Connection: Upgrade\r\n";
    response += "Sec-WebSocket-Accept: " + acceptValue + "\r\n";
    response += "\r\n";

    std::vector<std::byte> responseBytes(response.size());
    for (std::size_t i = 0; i < response.size(); ++i) {
        responseBytes[i] = static_cast<std::byte>(response[i]);
    }
    if (auto sent = connection.send(responseBytes); !sent.has_value()) {
        return std::unexpected(WebSocketError::TransportFailure);
    }

    WebSocketConnection ws(std::move(connection));
    // Carry over any bytes readUntil() already consumed past the header
    // terminator (e.g. if a client sent its first frame immediately,
    // arriving coalesced with the handshake request in one recv()) — these
    // belong to the WebSocket framing layer, not the discarded HTTP parse.
    if (requestBuffer.size() > *headerEnd) {
        std::string_view leftover(requestBuffer.data() + *headerEnd, requestBuffer.size() - *headerEnd);
        for (char c : leftover) {
            ws.residual_.push_back(static_cast<std::byte>(c));
        }
    }

    return ws;
}

std::expected<void, WebSocketError> WebSocketConnection::sendText(std::string_view text) {
    std::vector<std::byte> frame;
    frame.push_back(std::byte{0x81});  // FIN=1, opcode=0x1 (text)

    const std::size_t len = text.size();
    if (len <= 125) {
        frame.push_back(static_cast<std::byte>(len));
    } else if (len <= 0xFFFF) {
        frame.push_back(std::byte{126});
        frame.push_back(static_cast<std::byte>((len >> 8) & 0xFF));
        frame.push_back(static_cast<std::byte>(len & 0xFF));
    } else {
        frame.push_back(std::byte{127});
        for (int shift = 56; shift >= 0; shift -= 8) {
            frame.push_back(static_cast<std::byte>((static_cast<std::uint64_t>(len) >> shift) & 0xFF));
        }
    }

    for (char c : text) {
        frame.push_back(static_cast<std::byte>(c));
    }

    if (auto sent = connection_.send(frame); !sent.has_value()) {
        return std::unexpected(WebSocketError::TransportFailure);
    }
    return {};
}

std::expected<void, WebSocketError> WebSocketConnection::pollForClose() {
    auto header = readExactBuffered(2);
    if (!header.has_value()) {
        return std::unexpected(header.error());
    }

    const auto byte0 = std::to_integer<std::uint8_t>((*header)[0]);
    const auto byte1 = std::to_integer<std::uint8_t>((*header)[1]);
    const int opcode = byte0 & 0x0F;
    const bool masked = (byte1 & 0x80) != 0;
    std::uint64_t payloadLen = byte1 & 0x7F;

    if (payloadLen == 126) {
        auto ext = readExactBuffered(2);
        if (!ext.has_value()) return std::unexpected(ext.error());
        payloadLen = (std::to_integer<std::uint64_t>((*ext)[0]) << 8) | std::to_integer<std::uint64_t>((*ext)[1]);
    } else if (payloadLen == 127) {
        auto ext = readExactBuffered(8);
        if (!ext.has_value()) return std::unexpected(ext.error());
        payloadLen = 0;
        for (int i = 0; i < 8; ++i) {
            payloadLen = (payloadLen << 8) | std::to_integer<std::uint64_t>((*ext)[static_cast<std::size_t>(i)]);
        }
    }

    std::array<std::byte, 4> maskKey{};
    if (masked) {
        auto keyBytes = readExactBuffered(4);
        if (!keyBytes.has_value()) return std::unexpected(keyBytes.error());
        maskKey = {(*keyBytes)[0], (*keyBytes)[1], (*keyBytes)[2], (*keyBytes)[3]};
    }

    if (payloadLen > 0) {
        auto payload = readExactBuffered(static_cast<std::size_t>(payloadLen));
        if (!payload.has_value()) {
            return std::unexpected(payload.error());
        }
        if (masked) {
            // Unmask (required by RFC 6455 to correctly advance past the
            // frame, even though this minimal implementation discards the
            // content) — XOR each payload byte with maskKey[i % 4].
            for (std::size_t i = 0; i < payload->size(); ++i) {
                (*payload)[i] = static_cast<std::byte>(std::to_integer<std::uint8_t>((*payload)[i]) ^
                                                         std::to_integer<std::uint8_t>(maskKey[i % 4]));
            }
        }
        // Payload intentionally discarded beyond unmasking — see class docs.
    }

    if (opcode == 0x8) {  // close
        return std::unexpected(WebSocketError::ConnectionClosed);
    }
    return {};
}

}  // namespace robot::websocket
