// tests/cli/client_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
// Client is tested end-to-end against a minimal, test-only fake server built
// directly on robot::protocol::TcpListener/TcpConnection — NOT a real robot
// controller implementation. See docs/task-briefs/phase-07-cli.md Context.
#include <gtest/gtest.h>
#include <numbers>
#include <thread>
#include "robot/cli/client.hpp"
#include "robot/cli/move_joint_payload.hpp"
#include "robot/protocol/frame_codec.hpp"
#include "robot/protocol/tcp_listener.hpp"

using namespace robot::cli;
using robot::protocol::CommandType;
using robot::protocol::Frame;
using robot::protocol::FrameCodec;
using robot::protocol::TcpConnection;
using robot::protocol::TcpListener;

namespace {

// Reads from connection until FrameCodec can decode one full frame.
Frame receiveOneFrame(TcpConnection& connection) {
    std::vector<std::byte> received;
    std::array<std::byte, 4096> chunk{};
    while (true) {
        auto decoded = FrameCodec::decode(received);
        if (decoded.has_value()) {
            return decoded->frame;
        }
        auto n = connection.receive(chunk);
        if (!n.has_value()) {
            ADD_FAILURE() << "fake server: receive() failed while waiting for a request frame";
            return Frame{};
        }
        received.insert(received.end(), chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(*n));
    }
}

void sendResponse(TcpConnection& connection, CommandType type, ResponseStatus status,
                   std::span<const std::byte> extraPayload = {}) {
    Frame response;
    response.type = type;
    response.payload.push_back(static_cast<std::byte>(status));
    response.payload.insert(response.payload.end(), extraPayload.begin(), extraPayload.end());

    ASSERT_TRUE(connection.send(FrameCodec::encode(response)).has_value());
}

}  // namespace

TEST(Client, StatusCommandReturnsDecodedStatusPayload) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    StatusPayload serverStatus{.robotName = "RE-6AXIS", .controllerStateName = "Ready", .joints = {{1.0, 0.0}}};

    std::jthread serverThread([&]() {
        auto connection = listener->accept();
        ASSERT_TRUE(connection.has_value());
        auto request = receiveOneFrame(*connection);
        ASSERT_EQ(request.type, CommandType::GetStatus);

        auto encodedStatus = encodeStatusPayload(serverStatus);
        sendResponse(*connection, CommandType::GetStatus, ResponseStatus::Ok, encodedStatus);
    });

    auto client = Client::connect("127.0.0.1", port);
    ASSERT_TRUE(client.has_value());

    auto outcome = client->execute(Command{.kind = CommandKind::Status});

    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->status, ResponseStatus::Ok);
    ASSERT_TRUE(outcome->statusPayload.has_value());
    EXPECT_EQ(outcome->statusPayload->robotName, "RE-6AXIS");
    ASSERT_EQ(outcome->statusPayload->joints.size(), 1u);
    EXPECT_DOUBLE_EQ(outcome->statusPayload->joints[0].positionRadians, 1.0);
}

TEST(Client, ActionCommandReturnsOkWithNoStatusPayload) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    std::jthread serverThread([&]() {
        auto connection = listener->accept();
        ASSERT_TRUE(connection.has_value());
        auto request = receiveOneFrame(*connection);
        ASSERT_EQ(request.type, CommandType::Enable);
        sendResponse(*connection, CommandType::Enable, ResponseStatus::Ok);
    });

    auto client = Client::connect("127.0.0.1", port);
    ASSERT_TRUE(client.has_value());

    auto outcome = client->execute(Command{.kind = CommandKind::Enable});

    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->status, ResponseStatus::Ok);
    EXPECT_FALSE(outcome->statusPayload.has_value());
}

TEST(Client, ServerErrorResponseSurfacesAsServerReportedError) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    std::jthread serverThread([&]() {
        auto connection = listener->accept();
        ASSERT_TRUE(connection.has_value());
        receiveOneFrame(*connection);
        sendResponse(*connection, CommandType::Enable, ResponseStatus::Error);
    });

    auto client = Client::connect("127.0.0.1", port);
    ASSERT_TRUE(client.has_value());

    auto outcome = client->execute(Command{.kind = CommandKind::Enable});

    ASSERT_FALSE(outcome.has_value());
    EXPECT_EQ(outcome.error(), CliError::ServerReportedError);
}

TEST(Client, MismatchedResponseTypeSurfacesAsUnexpectedResponseType) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    std::jthread serverThread([&]() {
        auto connection = listener->accept();
        ASSERT_TRUE(connection.has_value());
        receiveOneFrame(*connection);
        // Responds with the wrong CommandType on purpose.
        sendResponse(*connection, CommandType::GetStatus, ResponseStatus::Ok, {});
    });

    auto client = Client::connect("127.0.0.1", port);
    ASSERT_TRUE(client.has_value());

    auto outcome = client->execute(Command{.kind = CommandKind::Enable});

    ASSERT_FALSE(outcome.has_value());
    EXPECT_EQ(outcome.error(), CliError::UnexpectedResponseType);
}

TEST(Client, MoveJointCommandSendsCorrectPayload) {
    auto listener = TcpListener::create(0);
    ASSERT_TRUE(listener.has_value());
    const std::uint16_t port = listener->port();

    MoveJointPayload receivedPayload{};
    std::jthread serverThread([&]() {
        auto connection = listener->accept();
        ASSERT_TRUE(connection.has_value());
        auto request = receiveOneFrame(*connection);
        ASSERT_EQ(request.type, CommandType::MoveJoint);
        auto decoded = decodeMoveJointPayload(request.payload);
        ASSERT_TRUE(decoded.has_value());
        receivedPayload = decoded.value();
        sendResponse(*connection, CommandType::MoveJoint, ResponseStatus::Ok);
    });

    auto client = Client::connect("127.0.0.1", port);
    ASSERT_TRUE(client.has_value());

    auto outcome = client->execute(Command{.kind = CommandKind::MoveJoint, .jointIndex = 2, .targetDegrees = 90.0});
    ASSERT_TRUE(outcome.has_value());

    serverThread.join();
    EXPECT_EQ(receivedPayload.jointIndex, 2);
    EXPECT_NEAR(receivedPayload.targetRadians, std::numbers::pi / 2.0, 1e-9);
}
