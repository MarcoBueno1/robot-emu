// tests/cli/command_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/cli/command.hpp"

using namespace robot::cli;

namespace {
std::expected<ParsedInvocation, CliError> parse(std::initializer_list<std::string_view> args) {
    std::vector<std::string_view> v(args);
    return parseArgs(v);
}
}  // namespace

TEST(ParseArgs, ParsesStatusCommand) {
    auto result = parse({"127.0.0.1:9000", "status"});
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->host, "127.0.0.1");
    EXPECT_EQ(result->port, 9000);
    EXPECT_EQ(result->command.kind, CommandKind::Status);
}

TEST(ParseArgs, ParsesEachSimpleCommandKind) {
    struct Case {
        std::string_view name;
        CommandKind kind;
    };
    for (auto [name, kind] : {Case{"status", CommandKind::Status}, Case{"enable", CommandKind::Enable},
                              Case{"disable", CommandKind::Disable}, Case{"home", CommandKind::Home},
                              Case{"stop", CommandKind::Stop}, Case{"estop", CommandKind::EmergencyStop}}) {
        auto result = parse({"host:1", name});
        ASSERT_TRUE(result.has_value()) << name;
        EXPECT_EQ(result->command.kind, kind) << name;
    }
}

TEST(ParseArgs, ParsesMoveJointWithArguments) {
    auto result = parse({"host:1", "move-joint", "3", "90.5"});
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->command.kind, CommandKind::MoveJoint);
    EXPECT_EQ(result->command.jointIndex, 3);
    EXPECT_DOUBLE_EQ(result->command.targetDegrees, 90.5);
}

TEST(ParseArgs, ParsesMoveJointWithNegativeDegrees) {
    auto result = parse({"host:1", "move-joint", "0", "-45.0"});
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->command.targetDegrees, -45.0);
}

TEST(ParseArgs, RejectsMissingHostPort) {
    auto result = parse({});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::InvalidHostPort);
}

TEST(ParseArgs, RejectsHostPortWithoutColon) {
    auto result = parse({"127.0.0.1", "status"});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::InvalidHostPort);
}

TEST(ParseArgs, RejectsHostPortWithNonNumericPort) {
    auto result = parse({"127.0.0.1:abc", "status"});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::InvalidHostPort);
}

TEST(ParseArgs, RejectsMissingCommand) {
    auto result = parse({"host:1"});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::MissingCommand);
}

TEST(ParseArgs, RejectsUnknownCommand) {
    auto result = parse({"host:1", "fly"});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::UnknownCommand);
}

TEST(ParseArgs, RejectsMoveJointWithTooFewArguments) {
    auto result = parse({"host:1", "move-joint", "3"});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::MissingArgument);
}

TEST(ParseArgs, RejectsMoveJointWithNonNumericDegrees) {
    auto result = parse({"host:1", "move-joint", "3", "abc"});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::InvalidArgument);
}

TEST(ParseArgs, RejectsMoveJointWithNonNumericJointIndex) {
    auto result = parse({"host:1", "move-joint", "abc", "90"});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::InvalidArgument);
}
