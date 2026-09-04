// tests/websocket/websocket_connection_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include <thread>
#include "robot/protocol/tcp_listener.hpp"
#include "robot/websocket/websocket_connection.hpp"

using namespace robot::websocket;
using robot::protocol::TcpConnection;
using robot::protocol::TcpListener;

namespace {

std::vector<std::byte> toBytes(std::string_view s) {
    std::vector<std::byte> out(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) out[i] = static_cast<std::byte>(s[i]);
    return out;
}

std::string readAvailable(TcpConnection& connection, std::size_t maxBytes = 4096) {
    std::vector<std::byte> buffer(maxBytes);
    auto n = connection.receive(buffer);
    if (!n.has_value()) return "";
    std::string out;
    for (std::size_t i = 0; i < *n; ++i) out.push_back(static_cast<char>(buffer[i]));
    return out;
}

// The RFC 6455 spec's own worked example key/accept pair — reusing it here
// is an independent cross-check beyond this project's own sha1/base64 tests.
constexpr std::string_view kSpecKey = "dGhlIHNhbXBsZSBub25jZQ==";
constexpr std::string_view kSpecAccept = "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=";

std::string buildUpgradeRequest(std::string_view key) {
    std::string req = "GET / HTTP/1.1\r\n";
    req += "Host: localhost\r\n";
    req += "Upgrade: websocket\r\n";
    req += "Connection: Upgrade\r\n";
    req += "Sec-WebSocket-Key: ";
    req += key;
    req += "\r\n";
    req += "Sec-WebSocket-Version: 13\r\n";
    req += "\r\n";
    return req;
}

}  // namespace

TEST(WebSocketConnection, HandshakeComputesCorrectAcceptValue) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    std::jthread serverThread([&]() {
        auto raw = listener->accept();
        ASSERT_TRUE(raw.has_value());
        auto ws = WebSocketConnection::accept(std::move(raw.value()));
        ASSERT_TRUE(ws.has_value());
    });

    auto client = TcpConnection::connectTo("127.0.0.1", port);
    ASSERT_TRUE(client.has_value());
    ASSERT_TRUE(client->send(toBytes(buildUpgradeRequest(kSpecKey))).has_value());

    std::string serverResponse = readAvailable(*client);
    serverThread.join();

    EXPECT_NE(serverResponse.find("101"), std::string::npos) << serverResponse;
    EXPECT_NE(serverResponse.find(std::string("Sec-WebSocket-Accept: ") + std::string(kSpecAccept)),
              std::string::npos)
        << serverResponse;
}

TEST(WebSocketConnection, HandshakeFailsWithoutKeyHeader) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    bool handshakeSucceeded = true;
    std::jthread serverThread([&]() {
        auto raw = listener->accept();
        ASSERT_TRUE(raw.has_value());
        auto ws = WebSocketConnection::accept(std::move(raw.value()));
        handshakeSucceeded = ws.has_value();
    });

    auto client = TcpConnection::connectTo("127.0.0.1", port);
    ASSERT_TRUE(client.has_value());
    std::string badRequest = "GET / HTTP/1.1\r\nHost: localhost\r\n\r\n";  // no Sec-WebSocket-Key
    ASSERT_TRUE(client->send(toBytes(badRequest)).has_value());

    serverThread.join();
    EXPECT_FALSE(handshakeSucceeded);
}

TEST(WebSocketConnection, SendTextProducesWellFormedUnmaskedFrame) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    std::jthread serverThread([&]() {
        auto raw = listener->accept();
        ASSERT_TRUE(raw.has_value());
        auto ws = WebSocketConnection::accept(std::move(raw.value()));
        ASSERT_TRUE(ws.has_value());
        ASSERT_TRUE(ws->sendText("hello").has_value());
    });

    auto client = TcpConnection::connectTo("127.0.0.1", port);
    ASSERT_TRUE(client.has_value());
    ASSERT_TRUE(client->send(toBytes(buildUpgradeRequest(kSpecKey))).has_value());

    // The HTTP 101 response and the subsequent sendText() frame may arrive
    // coalesced in a single receive() call (ordinary TCP stream behavior)
    // — accumulate into one running buffer rather than assuming a second,
    // separate receive() call will find the frame's bytes.
    std::string buffer = readAvailable(*client);
    auto headerEnd = buffer.find("\r\n\r\n");
    ASSERT_NE(headerEnd, std::string::npos);
    headerEnd += 4;

    while (buffer.size() - headerEnd < 7) {  // 2-byte frame header + 5-byte "hello"
        buffer += readAvailable(*client);
    }

    serverThread.join();

    const auto byte0 = static_cast<std::uint8_t>(buffer[headerEnd]);
    const auto byte1 = static_cast<std::uint8_t>(buffer[headerEnd + 1]);
    EXPECT_EQ(byte0, 0x81);      // FIN=1, opcode=text
    EXPECT_EQ(byte1 & 0x80, 0);  // server frames must be unmasked
    EXPECT_EQ(byte1 & 0x7F, 5);  // "hello" is 5 bytes

    std::string payload = buffer.substr(headerEnd + 2, 5);
    EXPECT_EQ(payload, "hello");
}

TEST(WebSocketConnection, PollForCloseDetectsClientCloseFrame) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    std::optional<WebSocketError> result;
    std::jthread serverThread([&]() {
        auto raw = listener->accept();
        ASSERT_TRUE(raw.has_value());
        auto ws = WebSocketConnection::accept(std::move(raw.value()));
        ASSERT_TRUE(ws.has_value());
        auto poll = ws->pollForClose();
        ASSERT_FALSE(poll.has_value());
        result = poll.error();
    });

    auto client = TcpConnection::connectTo("127.0.0.1", port);
    ASSERT_TRUE(client.has_value());
    ASSERT_TRUE(client->send(toBytes(buildUpgradeRequest(kSpecKey))).has_value());
    readAvailable(*client);  // consume the HTTP 101 response

    // A masked close frame with an empty payload: FIN=1, opcode=0x8 (close);
    // masked=1, length=0; then the 4-byte mask key (required whenever the
    // mask bit is set, even with zero payload bytes to actually unmask).
    std::vector<std::byte> closeFrame = {std::byte{0x88}, std::byte{0x80}, std::byte{0x00}, std::byte{0x00},
                                          std::byte{0x00}, std::byte{0x00}};
    ASSERT_TRUE(client->send(closeFrame).has_value());

    serverThread.join();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, WebSocketError::ConnectionClosed);
}
