# bridges/ros2_bridge/CMakeLists.txt
# -----------------------------------------------------------------------------
# Robot Emulator Framework
# -----------------------------------------------------------------------------
# Author:   Marco Bueno
# Email:    bueno.marco@gmail.com
# LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
# License:  MIT (see LICENSE file at project root)
# -----------------------------------------------------------------------------
# NOT build-verified — see docs/task-briefs/phase-13-ros2-bridge.md's
# warning. This is a standard ament_cmake package CMakeLists.txt, added via
# a single add_subdirectory() call from the repository root only when
# -DROBOT_EMULATOR_BUILD_ROS2_BRIDGE=ON, exactly as
# docs/architecture.md section 3.16 specifies.
cmake_minimum_required(VERSION 3.16)
project(robot_emulator_ros2)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(robot_emulator_ros2 SHARED
    src/hardware_interface.cpp
)

target_include_directories(robot_emulator_ros2 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_emulator_ros2 PUBLIC
    robot_cli
)

ament_target_dependencies(robot_emulator_ros2 PUBLIC
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface robot_emulator_ros2.xml)

install(
    TARGETS robot_emulator_ros2
    EXPORT export_robot_emulator_ros2
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_robot_emulator_ros2 HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib)

ament_package()
