// tests/hardware/io_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/hardware/analog_io.hpp"
#include "robot/hardware/digital_io.hpp"

using namespace robot::hardware;

// --- DigitalIO ---

TEST(DigitalIO, AllChannelsStartFalse) {
    DigitalIO io(4);
    for (std::size_t i = 0; i < io.channelCount(); ++i) {
        EXPECT_EQ(io.read(i).value(), false);
    }
}

TEST(DigitalIO, WriteThenReadReturnsWrittenValue) {
    DigitalIO io(4);
    ASSERT_TRUE(io.write(2, true).has_value());
    EXPECT_EQ(io.read(2).value(), true);
    EXPECT_EQ(io.read(0).value(), false);
}

TEST(DigitalIO, ReadOutOfRangeChannelFails) {
    DigitalIO io(4);
    auto result = io.read(10);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::OutOfRange);
}

TEST(DigitalIO, WriteOutOfRangeChannelFails) {
    DigitalIO io(4);
    auto result = io.write(10, true);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::OutOfRange);
}

TEST(DigitalIO, ZeroChannelsIsValidButAlwaysOutOfRange) {
    DigitalIO io(0);
    EXPECT_EQ(io.channelCount(), 0u);
    EXPECT_FALSE(io.read(0).has_value());
}

// --- AnalogIO ---

TEST(AnalogIO, RejectsMinNotLessThanMax) {
    auto result = AnalogIO::create(4, 10.0, 10.0);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::InvalidConfiguration);
}

TEST(AnalogIO, AllChannelsStartAtMinValue) {
    auto io = AnalogIO::create(3, -10.0, 10.0).value();
    for (std::size_t i = 0; i < io.channelCount(); ++i) {
        EXPECT_DOUBLE_EQ(io.read(i).value(), -10.0);
    }
}

TEST(AnalogIO, WriteThenReadReturnsWrittenValue) {
    auto io = AnalogIO::create(3, -10.0, 10.0).value();
    ASSERT_TRUE(io.write(1, 5.0).has_value());
    EXPECT_DOUBLE_EQ(io.read(1).value(), 5.0);
}

TEST(AnalogIO, WriteRejectsValueOutsideRange) {
    auto io = AnalogIO::create(3, -10.0, 10.0).value();
    auto result = io.write(1, 50.0);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::OutOfRange);
    EXPECT_DOUBLE_EQ(io.read(1).value(), -10.0)  // unchanged
        << "A rejected write must not modify the channel's prior value";
}

TEST(AnalogIO, ReadAndWriteOutOfRangeChannelFail) {
    auto io = AnalogIO::create(3, -10.0, 10.0).value();
    EXPECT_FALSE(io.read(10).has_value());
    EXPECT_FALSE(io.write(10, 0.0).has_value());
}
