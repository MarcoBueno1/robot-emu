# -----------------------------------------------------------------------------
# Robot Emulator Framework
# -----------------------------------------------------------------------------
# Author:   Marco Bueno
# Email:    bueno.marco@gmail.com
# LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
# License:  MIT (see LICENSE file at project root)
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.25)
project(robot-emulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROBOT_EMULATOR_BUILD_TESTS "Build unit tests" ON)
option(ROBOT_EMULATOR_ENABLE_SANITIZERS "Enable ASan/UBSan in Debug builds" ON)
option(ROBOT_EMULATOR_EXPERIMENTAL_CXX26 "Enable experimental C++26 features (GCC 15)" OFF)

if(ROBOT_EMULATOR_EXPERIMENTAL_CXX26)
    set(CMAKE_CXX_STANDARD 26)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(Threads REQUIRED)

if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND ROBOT_EMULATOR_ENABLE_SANITIZERS)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

# --- robot_core: domain library, no external dependencies ---
add_library(robot_core STATIC
    src/core/joint.cpp
    src/core/joint_limits.cpp
    src/core/robot.cpp
    src/core/robot_config.cpp
)

target_include_directories(robot_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(robot_core PUBLIC cxx_std_23)

# --- robot_controller: controller state machine, no dependency on robot_core ---
add_library(robot_controller STATIC
    src/controller/controller_state_machine.cpp
)

target_include_directories(robot_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(robot_controller PUBLIC cxx_std_23)

# --- robot_runtime: control loop, depends on both robot_core and robot_controller ---
add_library(robot_runtime STATIC
    src/runtime/control_loop.cpp
)

target_include_directories(robot_runtime PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(robot_runtime PUBLIC
    robot_core
    robot_controller
    Threads::Threads
)

target_compile_features(robot_runtime PUBLIC cxx_std_23)

# --- robot_motion: trajectory planning, depends only on robot_core ---
add_library(robot_motion STATIC
    src/motion/trapezoidal_trajectory.cpp
)

target_include_directories(robot_motion PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(robot_motion PUBLIC
    robot_core
)

target_compile_features(robot_motion PUBLIC cxx_std_23)

# --- robot_hardware: virtual hardware, no dependency on any other robot_* target ---
add_library(robot_hardware STATIC
    src/hardware/virtual_motor.cpp
    src/hardware/virtual_encoder.cpp
    src/hardware/virtual_brake.cpp
    src/hardware/digital_io.cpp
    src/hardware/analog_io.cpp
)

target_include_directories(robot_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(robot_hardware PUBLIC cxx_std_23)

# --- robot_protocol: binary framing + TCP transport, no dependency on any other robot_* target ---
add_library(robot_protocol STATIC
    src/protocol/frame_codec.cpp
    src/protocol/tcp_connection.cpp
    src/protocol/tcp_listener.cpp
)

target_include_directories(robot_protocol PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(robot_protocol PUBLIC
    Threads::Threads
)

target_compile_features(robot_protocol PUBLIC cxx_std_23)

# --- robot_cli: robotctl's logic, depends only on robot_protocol ---
add_library(robot_cli STATIC
    src/cli/command.cpp
    src/cli/status_payload.cpp
    src/cli/move_joint_payload.cpp
    src/cli/client.cpp
    src/cli/format.cpp
)

target_include_directories(robot_cli PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(robot_cli PUBLIC
    robot_protocol
)

target_compile_features(robot_cli PUBLIC cxx_std_23)

# --- robotctl: the actual CLI executable ---
add_executable(robotctl
    apps/robotctl/main.cpp
)

target_link_libraries(robotctl PRIVATE
    robot_cli
)

# --- robot_safety: limits, E-stop, watchdog — combines robot_core, robot_controller, robot_hardware ---
add_library(robot_safety STATIC
    src/safety/limit_monitor.cpp
    src/safety/emergency_stop_controller.cpp
    src/safety/watchdog.cpp
)

target_include_directories(robot_safety PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(robot_safety PUBLIC
    robot_core
    robot_controller
    robot_hardware
    Threads::Threads
)

target_compile_features(robot_safety PUBLIC cxx_std_23)

# --- robot_sensors: encoder/temperature/current/proximity, depends only on robot_hardware ---
add_library(robot_sensors STATIC
    src/sensors/encoder_sensor.cpp
    src/sensors/temperature_sensor.cpp
    src/sensors/current_sensor.cpp
    src/sensors/proximity_sensor.cpp
)

target_include_directories(robot_sensors PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(robot_sensors PUBLIC
    robot_hardware
)

target_compile_features(robot_sensors PUBLIC cxx_std_23)

# --- robot_fault: fault injection registry, no dependency on any other robot_* target ---
add_library(robot_fault STATIC
    src/fault/fault_type.cpp
    src/fault/fault_injector.cpp
)

target_include_directories(robot_fault PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_features(robot_fault PUBLIC cxx_std_23)

# --- Tests ---
if(ROBOT_EMULATOR_BUILD_TESTS)
    enable_testing()

    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
    )
    # Prevents GoogleTest's own config from overriding the project's flags
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(robot_core_tests
        tests/core/joint_test.cpp
        tests/core/robot_test.cpp
    )

    target_link_libraries(robot_core_tests PRIVATE
        robot_core
        GTest::gtest_main
    )

    include(GoogleTest)
    gtest_discover_tests(robot_core_tests)

    add_executable(robot_controller_tests
        tests/controller/controller_state_machine_test.cpp
    )

    target_link_libraries(robot_controller_tests PRIVATE
        robot_controller
        GTest::gtest_main
    )

    gtest_discover_tests(robot_controller_tests)

    add_executable(robot_runtime_tests
        tests/runtime/control_loop_test.cpp
    )

    target_link_libraries(robot_runtime_tests PRIVATE
        robot_runtime
        GTest::gtest_main
    )

    gtest_discover_tests(robot_runtime_tests)

    add_executable(robot_motion_tests
        tests/motion/trapezoidal_trajectory_test.cpp
    )

    target_link_libraries(robot_motion_tests PRIVATE
        robot_motion
        GTest::gtest_main
    )

    gtest_discover_tests(robot_motion_tests)

    add_executable(robot_hardware_tests
        tests/hardware/virtual_motor_test.cpp
        tests/hardware/virtual_encoder_test.cpp
        tests/hardware/virtual_brake_test.cpp
        tests/hardware/io_test.cpp
    )

    target_link_libraries(robot_hardware_tests PRIVATE
        robot_hardware
        GTest::gtest_main
    )

    gtest_discover_tests(robot_hardware_tests)

    add_executable(robot_protocol_tests
        tests/protocol/frame_codec_test.cpp
        tests/protocol/tcp_test.cpp
    )

    target_link_libraries(robot_protocol_tests PRIVATE
        robot_protocol
        GTest::gtest_main
    )

    gtest_discover_tests(robot_protocol_tests)

    add_executable(robot_cli_tests
        tests/cli/command_test.cpp
        tests/cli/payload_test.cpp
        tests/cli/format_test.cpp
        tests/cli/client_test.cpp
    )

    target_link_libraries(robot_cli_tests PRIVATE
        robot_cli
        GTest::gtest_main
    )

    gtest_discover_tests(robot_cli_tests)

    add_executable(robot_safety_tests
        tests/safety/limit_monitor_test.cpp
        tests/safety/emergency_stop_controller_test.cpp
        tests/safety/watchdog_test.cpp
    )

    target_link_libraries(robot_safety_tests PRIVATE
        robot_safety
        GTest::gtest_main
    )

    gtest_discover_tests(robot_safety_tests)

    add_executable(robot_sensors_tests
        tests/sensors/encoder_sensor_test.cpp
        tests/sensors/temperature_sensor_test.cpp
        tests/sensors/current_sensor_test.cpp
        tests/sensors/proximity_sensor_test.cpp
    )

    target_link_libraries(robot_sensors_tests PRIVATE
        robot_sensors
        GTest::gtest_main
    )

    gtest_discover_tests(robot_sensors_tests)

    add_executable(robot_fault_tests
        tests/fault/fault_injector_test.cpp
    )

    target_link_libraries(robot_fault_tests PRIVATE
        robot_fault
        GTest::gtest_main
    )

    gtest_discover_tests(robot_fault_tests)
endif()

# --- Documentation (optional: only registered if Doxygen is installed) ---
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen -> docs/generated/html/index.html"
        VERBATIM
    )
endif()
