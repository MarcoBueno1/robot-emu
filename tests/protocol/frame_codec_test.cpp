// tests/protocol/frame_codec_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/protocol/frame_codec.hpp"

using namespace robot::protocol;

namespace {

std::vector<std::byte> bytesOf(std::initializer_list<int> values) {
    std::vector<std::byte> out;
    out.reserve(values.size());
    for (int v : values) {
        out.push_back(static_cast<std::byte>(v));
    }
    return out;
}

}  // namespace

// --- Round-trip ---

TEST(FrameCodec, RoundTripsEveryCommandTypeWithEmptyPayload) {
    for (auto type : {CommandType::Connect, CommandType::GetStatus, CommandType::Enable, CommandType::Disable,
                       CommandType::Home, CommandType::MoveJoint, CommandType::MoveLinear, CommandType::Stop,
                       CommandType::EmergencyStop, CommandType::GetPosition, CommandType::GetIO,
                       CommandType::SetIO, CommandType::ResetFault, CommandType::InjectFault}) {
        Frame frame{.flags = 0, .type = type, .payload = {}};

        auto encoded = FrameCodec::encode(frame);
        auto result = FrameCodec::decode(encoded);

        ASSERT_TRUE(result.has_value()) << "Failed for " << toString(type);
        EXPECT_EQ(result->frame.type, type);
        EXPECT_TRUE(result->frame.payload.empty());
        EXPECT_EQ(result->bytesConsumed, encoded.size());
    }
}

TEST(FrameCodec, RoundTripsNonEmptyPayload) {
    std::vector<std::byte> payload = bytesOf({1, 2, 3, 4, 5, 255, 0, 127});
    Frame frame{.flags = 0b0000'0001, .type = CommandType::MoveJoint, .payload = payload};

    auto encoded = FrameCodec::encode(frame);
    auto result = FrameCodec::decode(encoded);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->frame.flags, 0b0000'0001);
    EXPECT_EQ(result->frame.type, CommandType::MoveJoint);
    EXPECT_EQ(result->frame.payload, payload);
    EXPECT_EQ(result->bytesConsumed, FrameCodec::headerSize + payload.size());
}

TEST(FrameCodec, EncodedSizeMatchesHeaderPlusPayload) {
    std::vector<std::byte> payload = bytesOf({9, 9, 9});
    Frame frame{.flags = 0, .type = CommandType::GetStatus, .payload = payload};

    auto encoded = FrameCodec::encode(frame);

    EXPECT_EQ(encoded.size(), FrameCodec::headerSize + payload.size());
}

// --- Incomplete ---

TEST(FrameCodec, DecodeFewerThanHeaderSizeBytesIsIncomplete) {
    std::vector<std::byte> tooShort(FrameCodec::headerSize - 1, std::byte{0});

    auto result = FrameCodec::decode(tooShort);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ProtocolError::Incomplete);
}

TEST(FrameCodec, DecodeCompleteHeaderButIncompletePayloadIsIncomplete) {
    Frame frame{.flags = 0, .type = CommandType::MoveJoint, .payload = bytesOf({1, 2, 3, 4, 5})};
    auto encoded = FrameCodec::encode(frame);
    std::span<const std::byte> truncated(encoded.data(), encoded.size() - 2);  // header + partial payload

    auto result = FrameCodec::decode(truncated);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ProtocolError::Incomplete);
}

TEST(FrameCodec, DecodeEmptyBufferIsIncomplete) {
    auto result = FrameCodec::decode(std::span<const std::byte>{});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ProtocolError::Incomplete);
}

// --- Malformed frames (hand-built, not via encode()) ---

TEST(FrameCodec, DecodeRejectsWrongMagic) {
    // Wrong magic (0x00000000), otherwise well-formed header, no payload.
    auto bytes = bytesOf({0x00, 0x00, 0x00, 0x00,  // MAGIC (wrong)
                           0x01,                    // VERSION
                           0x00,                    // FLAGS
                           0x00, 0x06,               // TYPE = MoveJoint (6)
                           0x00, 0x00, 0x00, 0x00}); // LENGTH = 0

    auto result = FrameCodec::decode(bytes);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ProtocolError::InvalidMagic);
}

TEST(FrameCodec, DecodeRejectsWrongVersion) {
    auto bytes = bytesOf({0x52, 0x4F, 0x42, 0x4F,  // MAGIC ("ROBO")
                           0x02,                    // VERSION (unsupported)
                           0x00,                    // FLAGS
                           0x00, 0x06,               // TYPE = MoveJoint
                           0x00, 0x00, 0x00, 0x00}); // LENGTH = 0

    auto result = FrameCodec::decode(bytes);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ProtocolError::UnsupportedVersion);
}

TEST(FrameCodec, DecodeRejectsUnknownCommandType) {
    auto bytes = bytesOf({0x52, 0x4F, 0x42, 0x4F,  // MAGIC
                           0x01,                    // VERSION
                           0x00,                    // FLAGS
                           0xFF, 0xFF,               // TYPE = 65535 (unknown)
                           0x00, 0x00, 0x00, 0x00}); // LENGTH = 0

    auto result = FrameCodec::decode(bytes);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ProtocolError::UnknownCommandType);
}

TEST(FrameCodec, DecodeRejectsPayloadTooLarge) {
    auto bytes = bytesOf({0x52, 0x4F, 0x42, 0x4F,  // MAGIC
                           0x01,                    // VERSION
                           0x00,                    // FLAGS
                           0x00, 0x06,               // TYPE = MoveJoint
                           0xFF, 0xFF, 0xFF, 0xFF}); // LENGTH = huge

    auto result = FrameCodec::decode(bytes);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ProtocolError::PayloadTooLarge);
}

// --- Streaming: bytesConsumed + trailing bytes ---

TEST(FrameCodec, DecodeIgnoresTrailingBytesAndReportsBytesConsumed) {
    Frame frame{.flags = 0, .type = CommandType::Home, .payload = {}};
    auto encoded = FrameCodec::encode(frame);
    encoded.push_back(std::byte{0xAB});  // trailing garbage / start of a next frame

    auto result = FrameCodec::decode(encoded);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->bytesConsumed, FrameCodec::headerSize);
    EXPECT_LT(result->bytesConsumed, encoded.size());
}

TEST(FrameCodec, DecodesTwoConcatenatedFramesSequentially) {
    Frame first{.flags = 0, .type = CommandType::Enable, .payload = {}};
    Frame second{.flags = 0, .type = CommandType::Disable, .payload = bytesOf({7, 8})};

    auto encodedFirst = FrameCodec::encode(first);
    auto encodedSecond = FrameCodec::encode(second);

    std::vector<std::byte> combined = encodedFirst;
    combined.insert(combined.end(), encodedSecond.begin(), encodedSecond.end());

    auto firstResult = FrameCodec::decode(combined);
    ASSERT_TRUE(firstResult.has_value());
    EXPECT_EQ(firstResult->frame.type, CommandType::Enable);
    EXPECT_EQ(firstResult->bytesConsumed, encodedFirst.size());

    std::span<const std::byte> remaining(combined.data() + firstResult->bytesConsumed,
                                          combined.size() - firstResult->bytesConsumed);
    auto secondResult = FrameCodec::decode(remaining);
    ASSERT_TRUE(secondResult.has_value());
    EXPECT_EQ(secondResult->frame.type, CommandType::Disable);
    EXPECT_EQ(secondResult->frame.payload, bytesOf({7, 8}));
}

// --- CommandType helpers ---

TEST(CommandType, IsKnownCommandTypeRejectsZeroAndUnmappedValues) {
    EXPECT_FALSE(isKnownCommandType(0));
    EXPECT_FALSE(isKnownCommandType(65535));
    EXPECT_TRUE(isKnownCommandType(static_cast<std::uint16_t>(CommandType::MoveJoint)));
}

TEST(CommandType, ToStringCoversEveryValue) {
    for (auto type : {CommandType::Connect, CommandType::GetStatus, CommandType::Enable, CommandType::Disable,
                       CommandType::Home, CommandType::MoveJoint, CommandType::MoveLinear, CommandType::Stop,
                       CommandType::EmergencyStop, CommandType::GetPosition, CommandType::GetIO,
                       CommandType::SetIO, CommandType::ResetFault, CommandType::InjectFault}) {
        EXPECT_FALSE(toString(type).empty());
    }
    EXPECT_EQ(toString(CommandType::MoveJoint), "MoveJoint");
}
