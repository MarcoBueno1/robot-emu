// apps/robotctl/main.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
// Thin executable: argv -> robot::cli::parseArgs() -> Client::connect() ->
// Client::execute() -> print. All actual logic lives in robot_cli, where
// it's covered by robot_cli_tests — nothing here is testable-but-untested.
#include <iostream>
#include <string>
#include <vector>
#include "robot/cli/client.hpp"
#include "robot/cli/command.hpp"
#include "robot/cli/format.hpp"

namespace {

void printUsage() {
    std::cerr << "Usage: robotctl <host>:<port> <command> [args...]\n"
                 "Commands: status | enable | disable | home | move-joint <joint> <degrees> | stop | estop\n";
}

}  // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> owned(argv + 1, argv + argc);
    std::vector<std::string_view> args(owned.begin(), owned.end());

    auto parsed = robot::cli::parseArgs(args);
    if (!parsed.has_value()) {
        printUsage();
        return 1;
    }

    auto client = robot::cli::Client::connect(parsed->host, parsed->port);
    if (!client.has_value()) {
        std::cerr << "robotctl: failed to connect to " << parsed->host << ":" << parsed->port << "\n";
        return 1;
    }

    auto outcome = client->execute(parsed->command);
    if (!outcome.has_value()) {
        std::cerr << "robotctl: command failed\n";
        return 1;
    }

    if (parsed->command.kind == robot::cli::CommandKind::Status && outcome->statusPayload.has_value()) {
        std::cout << robot::cli::formatStatusTable(outcome->statusPayload.value());
    } else {
        std::cout << "OK\n";
    }

    return 0;
}
