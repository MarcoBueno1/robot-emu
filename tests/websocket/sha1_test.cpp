// tests/websocket/sha1_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>
#include "robot/websocket/sha1.hpp"

using namespace robot::websocket;

namespace {
std::string toHex(const std::array<std::uint8_t, 20>& digest) {
    std::ostringstream oss;
    for (auto byte : digest) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    return oss.str();
}
}  // namespace

TEST(Sha1, MatchesKnownDigestForEmptyString) {
    // Standard published SHA-1 reference value for the empty input.
    EXPECT_EQ(toHex(sha1("")), "da39a3ee5e6b4b0d3255bfef95601890afd80709");
}

TEST(Sha1, MatchesKnownDigestForAbc) {
    // Standard published SHA-1 reference value for "abc".
    EXPECT_EQ(toHex(sha1("abc")), "a9993e364706816aba3e25717850c26c9cd0d89d");
}

TEST(Sha1, MatchesKnownDigestForLongerMessage) {
    // Standard published SHA-1 reference value for this 56-character message
    // — exercises the padding boundary (56 bytes is exactly where a single
    // 64-byte block's data can't also fit the 8-byte length field, forcing
    // a second block).
    EXPECT_EQ(toHex(sha1("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")),
              "84983e441c3bd26ebaae4aa1f95129e5e54670f1");
}

TEST(Sha1, DifferentInputsProduceDifferentDigests) {
    EXPECT_NE(toHex(sha1("hello")), toHex(sha1("world")));
}

TEST(Sha1, ProducesTwentyByteDigest) {
    EXPECT_EQ(sha1("anything").size(), 20u);
}
