// tests/protocol/tcp_test.cpp
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
#include "robot/protocol/frame_codec.hpp"
#include "robot/protocol/tcp_connection.hpp"
#include "robot/protocol/tcp_listener.hpp"

using namespace robot::protocol;

TEST(TcpListener, CreateWithPortZeroBindsToRealEphemeralPort) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    EXPECT_NE(listener->port(), 0);
}

TEST(Tcp, ConnectAcceptSendReceiveRoundTrip) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    Frame sentFrame{.flags = 0, .type = CommandType::MoveJoint, .payload = {}};
    for (int i = 0; i < 8; ++i) {
        sentFrame.payload.push_back(static_cast<std::byte>(i));
    }
    auto encoded = FrameCodec::encode(sentFrame);

    std::optional<std::vector<std::byte>> receivedOnServer;
    std::jthread serverThread([&]() {
        auto connection = listener->accept();
        ASSERT_TRUE(connection.has_value());

        std::vector<std::byte> received;
        std::array<std::byte, 64> buffer{};
        while (received.size() < encoded.size()) {
            auto n = connection->receive(buffer);
            ASSERT_TRUE(n.has_value());
            received.insert(received.end(), buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(*n));
        }
        receivedOnServer = std::move(received);
    });

    auto client = TcpConnection::connectTo("127.0.0.1", port);
    ASSERT_TRUE(client.has_value());
    ASSERT_TRUE(client->send(encoded).has_value());

    serverThread.join();

    ASSERT_TRUE(receivedOnServer.has_value());
    EXPECT_EQ(*receivedOnServer, encoded);

    auto decoded = FrameCodec::decode(*receivedOnServer);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->frame.type, CommandType::MoveJoint);
    EXPECT_EQ(decoded->frame.payload, sentFrame.payload);
}

TEST(Tcp, ReceiveReturnsConnectionClosedAfterPeerCloses) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    std::jthread serverThread([&]() {
        auto connection = TcpConnection::connectTo("127.0.0.1", port);
        ASSERT_TRUE(connection.has_value());
        // connection destructs here at the end of the lambda, closing the socket.
    });

    auto accepted = listener->accept();
    ASSERT_TRUE(accepted.has_value());
    serverThread.join();  // ensure the client side has already closed

    std::array<std::byte, 16> buffer{};
    auto result = accepted->receive(buffer);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), TransportError::ConnectionClosed);
}

TEST(Tcp, ReceiveIntoSmallerBufferReturnsShortRead) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    std::vector<std::byte> payload(100, std::byte{0x42});

    std::jthread serverThread([&]() {
        auto client = TcpConnection::connectTo("127.0.0.1", port);
        ASSERT_TRUE(client.has_value());
        ASSERT_TRUE(client->send(payload).has_value());
    });

    auto accepted = listener->accept();
    ASSERT_TRUE(accepted.has_value());

    std::array<std::byte, 10> smallBuffer{};  // smaller than the 100-byte payload
    auto n = accepted->receive(smallBuffer);

    ASSERT_TRUE(n.has_value());
    EXPECT_LE(*n, smallBuffer.size());

    serverThread.join();
}
