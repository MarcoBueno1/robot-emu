// src/websocket/sha1.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/websocket/sha1.hpp"

#include <vector>

namespace robot::websocket {

namespace {

[[nodiscard]] std::uint32_t rotateLeft(std::uint32_t value, int bits) noexcept {
    return (value << bits) | (value >> (32 - bits));
}

}  // namespace

std::array<std::uint8_t, 20> sha1(std::string_view input) noexcept {
    std::uint32_t h0 = 0x67452301;
    std::uint32_t h1 = 0xEFCDAB89;
    std::uint32_t h2 = 0x98BADCFE;
    std::uint32_t h3 = 0x10325476;
    std::uint32_t h4 = 0xC3D2E1F0;

    // --- Pre-processing: pad the message ---
    std::vector<std::uint8_t> message(input.begin(), input.end());
    const std::uint64_t originalBitLength = static_cast<std::uint64_t>(message.size()) * 8;

    message.push_back(0x80);
    while (message.size() % 64 != 56) {
        message.push_back(0x00);
    }
    for (int shift = 56; shift >= 0; shift -= 8) {
        message.push_back(static_cast<std::uint8_t>((originalBitLength >> shift) & 0xFF));
    }

    // --- Process each 512-bit (64-byte) chunk ---
    for (std::size_t chunkStart = 0; chunkStart < message.size(); chunkStart += 64) {
        std::array<std::uint32_t, 80> w{};
        for (int i = 0; i < 16; ++i) {
            const std::size_t offset = chunkStart + static_cast<std::size_t>(i) * 4;
            w[static_cast<std::size_t>(i)] = (static_cast<std::uint32_t>(message[offset]) << 24) |
                                              (static_cast<std::uint32_t>(message[offset + 1]) << 16) |
                                              (static_cast<std::uint32_t>(message[offset + 2]) << 8) |
                                              static_cast<std::uint32_t>(message[offset + 3]);
        }
        for (int i = 16; i < 80; ++i) {
            w[static_cast<std::size_t>(i)] = rotateLeft(
                w[static_cast<std::size_t>(i - 3)] ^ w[static_cast<std::size_t>(i - 8)] ^
                    w[static_cast<std::size_t>(i - 14)] ^ w[static_cast<std::size_t>(i - 16)],
                1);
        }

        std::uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;

        for (int i = 0; i < 80; ++i) {
            std::uint32_t f;
            std::uint32_t k;
            if (i < 20) {
                f = (b & c) | ((~b) & d);
                k = 0x5A827999;
            } else if (i < 40) {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1;
            } else if (i < 60) {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDC;
            } else {
                f = b ^ c ^ d;
                k = 0xCA62C1D6;
            }

            const std::uint32_t temp = rotateLeft(a, 5) + f + e + k + w[static_cast<std::size_t>(i)];
            e = d;
            d = c;
            c = rotateLeft(b, 30);
            b = a;
            a = temp;
        }

        h0 += a;
        h1 += b;
        h2 += c;
        h3 += d;
        h4 += e;
    }

    std::array<std::uint8_t, 20> digest{};
    std::uint32_t values[5] = {h0, h1, h2, h3, h4};
    for (int i = 0; i < 5; ++i) {
        digest[static_cast<std::size_t>(i) * 4 + 0] = static_cast<std::uint8_t>((values[i] >> 24) & 0xFF);
        digest[static_cast<std::size_t>(i) * 4 + 1] = static_cast<std::uint8_t>((values[i] >> 16) & 0xFF);
        digest[static_cast<std::size_t>(i) * 4 + 2] = static_cast<std::uint8_t>((values[i] >> 8) & 0xFF);
        digest[static_cast<std::size_t>(i) * 4 + 3] = static_cast<std::uint8_t>(values[i] & 0xFF);
    }
    return digest;
}

}  // namespace robot::websocket
