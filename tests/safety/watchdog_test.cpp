// tests/safety/watchdog_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
// Watchdog is the second component in this codebase (after
// robot::runtime::ControlLoop in Phase 3) whose tests inherently depend on
// real timing. Kept short (tens of milliseconds) with generous, tolerant
// assertions — same approach Phase 3 used, for the same reason.
#include <gtest/gtest.h>
#include <atomic>
#include <thread>
#include "robot/safety/watchdog.hpp"

using namespace robot::safety;

TEST(Watchdog, NotTrippedWhileHeartbeatKeepsAdvancing) {
    std::atomic<std::uint64_t> counter{0};
    Watchdog watchdog([&] { return counter.load(); }, std::chrono::milliseconds(50), std::chrono::milliseconds(5));

    ASSERT_TRUE(watchdog.start().has_value());

    for (int i = 0; i < 8; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        counter.fetch_add(1);
    }

    EXPECT_FALSE(watchdog.tripped());
    ASSERT_TRUE(watchdog.stop().has_value());
}

TEST(Watchdog, TripsWhenHeartbeatStalls) {
    std::atomic<std::uint64_t> counter{0};
    Watchdog watchdog([&] { return counter.load(); }, std::chrono::milliseconds(20), std::chrono::milliseconds(5));

    ASSERT_TRUE(watchdog.start().has_value());
    // Never advance counter — the watchdog should trip after ~20ms.
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    EXPECT_TRUE(watchdog.tripped());
    ASSERT_TRUE(watchdog.stop().has_value());
}

TEST(Watchdog, ResetClearsTrippedAndDoesNotImmediatelyRetrip) {
    std::atomic<std::uint64_t> counter{0};
    Watchdog watchdog([&] { return counter.load(); }, std::chrono::milliseconds(30), std::chrono::milliseconds(5));

    ASSERT_TRUE(watchdog.start().has_value());
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    ASSERT_TRUE(watchdog.tripped());

    watchdog.reset();
    EXPECT_FALSE(watchdog.tripped());

    // Well under the 30ms timeout — should still be false.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_FALSE(watchdog.tripped());

    ASSERT_TRUE(watchdog.stop().has_value());
}

TEST(Watchdog, StartFailsWhenAlreadyRunning) {
    Watchdog watchdog([] { return std::uint64_t{0}; }, std::chrono::milliseconds(50), std::chrono::milliseconds(5));

    ASSERT_TRUE(watchdog.start().has_value());
    auto second = watchdog.start();

    ASSERT_FALSE(second.has_value());
    EXPECT_EQ(second.error(), WatchdogError::AlreadyRunning);

    ASSERT_TRUE(watchdog.stop().has_value());
}

TEST(Watchdog, StopFailsWhenNotRunning) {
    Watchdog watchdog([] { return std::uint64_t{0}; }, std::chrono::milliseconds(50), std::chrono::milliseconds(5));

    auto result = watchdog.stop();

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), WatchdogError::NotRunning);
}

TEST(Watchdog, IsRunningReflectsLifecycle) {
    Watchdog watchdog([] { return std::uint64_t{0}; }, std::chrono::milliseconds(50), std::chrono::milliseconds(5));

    EXPECT_FALSE(watchdog.isRunning());
    ASSERT_TRUE(watchdog.start().has_value());
    EXPECT_TRUE(watchdog.isRunning());
    ASSERT_TRUE(watchdog.stop().has_value());
    EXPECT_FALSE(watchdog.isRunning());
}

TEST(Watchdog, DestructorStopsStillRunningThread) {
    {
        Watchdog watchdog([] { return std::uint64_t{0}; }, std::chrono::milliseconds(50),
                           std::chrono::milliseconds(5));
        ASSERT_TRUE(watchdog.start().has_value());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // watchdog destructs here without an explicit stop() call — under
        // AddressSanitizer/UBSan (enabled in this project's Debug build),
        // any lifetime or data-race bug here would be caught.
    }
    SUCCEED();
}
