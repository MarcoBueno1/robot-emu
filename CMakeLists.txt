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
