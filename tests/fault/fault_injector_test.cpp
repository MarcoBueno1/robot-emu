// tests/fault/fault_injector_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include <algorithm>
#include "robot/fault/fault_injector.hpp"

using namespace robot::fault;

TEST(FaultInjector, InjectCallsRegisteredHandlerWithJointIndex) {
    FaultInjector injector;
    std::optional<std::size_t> received;
    int callCount = 0;

    injector.onFault(FaultType::EncoderFailure, [&](std::optional<std::size_t> jointIndex) {
        received = jointIndex;
        ++callCount;
    });

    injector.inject(FaultType::EncoderFailure, 2);

    EXPECT_EQ(callCount, 1);
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(*received, 2u);
}

TEST(FaultInjector, InjectPassesNulloptForRobotWideFault) {
    FaultInjector injector;
    std::optional<std::size_t> received = 999;  // sentinel, should be overwritten to nullopt

    injector.onFault(FaultType::PowerFailure, [&](std::optional<std::size_t> jointIndex) { received = jointIndex; });

    injector.inject(FaultType::PowerFailure);

    EXPECT_FALSE(received.has_value());
}

TEST(FaultInjector, InjectWithNoHandlerDoesNotCrash) {
    FaultInjector injector;
    injector.inject(FaultType::MotorFailure, 0);
    EXPECT_TRUE(injector.isActive(FaultType::MotorFailure, 0));
}

TEST(FaultInjector, OnFaultCalledTwiceReplacesHandler) {
    FaultInjector injector;
    int firstCalls = 0;
    int secondCalls = 0;

    injector.onFault(FaultType::BrakeFailure, [&](std::optional<std::size_t>) { ++firstCalls; });
    injector.onFault(FaultType::BrakeFailure, [&](std::optional<std::size_t>) { ++secondCalls; });

    injector.inject(FaultType::BrakeFailure, 0);

    EXPECT_EQ(firstCalls, 0);
    EXPECT_EQ(secondCalls, 1);
}

TEST(FaultInjector, DistinguishesFaultsByJointIndex) {
    FaultInjector injector;
    injector.inject(FaultType::EncoderFailure, 0);

    EXPECT_TRUE(injector.isActive(FaultType::EncoderFailure, 0));
    EXPECT_FALSE(injector.isActive(FaultType::EncoderFailure, 1));
}

TEST(FaultInjector, DistinguishesRobotWideFromJointScopedFaultOfSameType) {
    FaultInjector injector;
    injector.inject(FaultType::OverTemperature, 3);

    EXPECT_TRUE(injector.isActive(FaultType::OverTemperature, 3));
    EXPECT_FALSE(injector.isActive(FaultType::OverTemperature));  // robot-wide (nullopt) is distinct
}

TEST(FaultInjector, InjectingSamePairTwiceDoesNotDuplicateButStillCallsHandler) {
    FaultInjector injector;
    int callCount = 0;
    injector.onFault(FaultType::LimitSwitch, [&](std::optional<std::size_t>) { ++callCount; });

    injector.inject(FaultType::LimitSwitch, 1);
    injector.inject(FaultType::LimitSwitch, 1);

    EXPECT_EQ(callCount, 2);
    auto faults = injector.activeFaults();
    EXPECT_EQ(std::count_if(faults.begin(), faults.end(),
                             [](const ActiveFault& f) { return f.type == FaultType::LimitSwitch; }),
              1);
}

TEST(FaultInjector, ClearOnInactiveFaultIsHarmlessNoOp) {
    FaultInjector injector;
    injector.clear(FaultType::CommunicationTimeout, 0);
    EXPECT_TRUE(injector.activeFaults().empty());
}

TEST(FaultInjector, ClearRemovesOnlyTheMatchingFault) {
    FaultInjector injector;
    injector.inject(FaultType::PositionError, 0);
    injector.inject(FaultType::PositionError, 1);

    injector.clear(FaultType::PositionError, 0);

    EXPECT_FALSE(injector.isActive(FaultType::PositionError, 0));
    EXPECT_TRUE(injector.isActive(FaultType::PositionError, 1));
}

TEST(FaultInjector, ClearAllEmptiesActiveFaultsButKeepsHandlers) {
    FaultInjector injector;
    int callCount = 0;
    injector.onFault(FaultType::VelocityError, [&](std::optional<std::size_t>) { ++callCount; });
    injector.inject(FaultType::VelocityError, 0);
    injector.inject(FaultType::EmergencyStop);

    injector.clearAll();

    EXPECT_TRUE(injector.activeFaults().empty());

    injector.inject(FaultType::VelocityError, 0);
    EXPECT_EQ(callCount, 2) << "clearAll() must not remove registered handlers";
}

TEST(FaultInjector, ActiveFaultsReflectsInjectionOrder) {
    FaultInjector injector;
    injector.inject(FaultType::EncoderFailure, 0);
    injector.inject(FaultType::MotorFailure, 1);
    injector.inject(FaultType::PowerFailure);

    auto faults = injector.activeFaults();

    ASSERT_EQ(faults.size(), 3u);
    EXPECT_EQ(faults[0].type, FaultType::EncoderFailure);
    EXPECT_EQ(faults[1].type, FaultType::MotorFailure);
    EXPECT_EQ(faults[2].type, FaultType::PowerFailure);
}

TEST(FaultInjector, ToStringCoversEveryFaultType) {
    for (auto type : {FaultType::EncoderFailure, FaultType::MotorFailure, FaultType::OverCurrent,
                       FaultType::OverTemperature, FaultType::CommunicationTimeout, FaultType::PositionError,
                       FaultType::VelocityError, FaultType::BrakeFailure, FaultType::LimitSwitch,
                       FaultType::EmergencyStop, FaultType::PowerFailure}) {
        EXPECT_FALSE(toString(type).empty());
    }
    EXPECT_EQ(toString(FaultType::EncoderFailure), "EncoderFailure");
    EXPECT_EQ(toString(FaultType::PowerFailure), "PowerFailure");
}
