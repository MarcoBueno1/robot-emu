// tests/websocket/base64_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/websocket/base64.hpp"

using namespace robot::websocket;

namespace {
std::string encodeString(std::string_view text) {
    return base64Encode(
        std::span<const std::uint8_t>(reinterpret_cast<const std::uint8_t*>(text.data()), text.size()));
}
}  // namespace

// RFC 4648 section 10's own worked examples.
TEST(Base64, EncodesEmptyInput) {
    EXPECT_EQ(encodeString(""), "");
}

TEST(Base64, EncodesF) {
    EXPECT_EQ(encodeString("f"), "Zg==");
}

TEST(Base64, EncodesFo) {
    EXPECT_EQ(encodeString("fo"), "Zm8=");
}

TEST(Base64, EncodesFoo) {
    EXPECT_EQ(encodeString("foo"), "Zm9v");
}

TEST(Base64, EncodesFoob) {
    EXPECT_EQ(encodeString("foob"), "Zm9vYg==");
}

TEST(Base64, EncodesFooba) {
    EXPECT_EQ(encodeString("fooba"), "Zm9vYmE=");
}

TEST(Base64, EncodesFoobar) {
    EXPECT_EQ(encodeString("foobar"), "Zm9vYmFy");
}

TEST(Base64, EncodesTwentyByteInputWithoutTruncation) {
    // A 20-byte input (matching sha1()'s digest size) always encodes to a
    // 28-character string (with one trailing '=' — 20 % 3 == 2).
    std::vector<std::uint8_t> input(20, 0xAB);
    auto encoded = base64Encode(input);
    EXPECT_EQ(encoded.size(), 28u);
    EXPECT_EQ(encoded.back(), '=');
}
