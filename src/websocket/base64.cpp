// src/websocket/base64.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/websocket/base64.hpp"

namespace robot::websocket {

namespace {
constexpr std::string_view kAlphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
}

std::string base64Encode(std::span<const std::uint8_t> input) {
    std::string out;
    out.reserve(((input.size() + 2) / 3) * 4);

    std::size_t i = 0;
    while (i + 3 <= input.size()) {
        const std::uint32_t triple =
            (static_cast<std::uint32_t>(input[i]) << 16) | (static_cast<std::uint32_t>(input[i + 1]) << 8) |
            static_cast<std::uint32_t>(input[i + 2]);
        out.push_back(kAlphabet[(triple >> 18) & 0x3F]);
        out.push_back(kAlphabet[(triple >> 12) & 0x3F]);
        out.push_back(kAlphabet[(triple >> 6) & 0x3F]);
        out.push_back(kAlphabet[triple & 0x3F]);
        i += 3;
    }

    const std::size_t remaining = input.size() - i;
    if (remaining == 1) {
        const std::uint32_t value = static_cast<std::uint32_t>(input[i]) << 16;
        out.push_back(kAlphabet[(value >> 18) & 0x3F]);
        out.push_back(kAlphabet[(value >> 12) & 0x3F]);
        out.push_back('=');
        out.push_back('=');
    } else if (remaining == 2) {
        const std::uint32_t value =
            (static_cast<std::uint32_t>(input[i]) << 16) | (static_cast<std::uint32_t>(input[i + 1]) << 8);
        out.push_back(kAlphabet[(value >> 18) & 0x3F]);
        out.push_back(kAlphabet[(value >> 12) & 0x3F]);
        out.push_back(kAlphabet[(value >> 6) & 0x3F]);
        out.push_back('=');
    }

    return out;
}

}  // namespace robot::websocket
