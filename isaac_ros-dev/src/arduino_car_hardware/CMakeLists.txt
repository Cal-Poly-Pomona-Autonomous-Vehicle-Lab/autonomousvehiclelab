cmake_minimum_required(VERSION 3.5)
project(arduino_car_hardware)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add source files for the hardware library
add_library(arduino_car_hardware
    src/arduino_car_hardware.cpp
    src/arduino_comms.cpp
)

# Link dependencies to the library
ament_target_dependencies(arduino_car_hardware
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
    geometry_msgs
)

# Add the robot node as an executable
add_executable(arduino_car_node
    src/robot.cpp
)

# Link dependencies to the executable
ament_target_dependencies(arduino_car_node
    rclcpp
    controller_manager
    hardware_interface
    geometry_msgs
)

# Link the hardware library to the executable
target_link_libraries(arduino_car_node arduino_car_hardware)

# Export plugins for ros2_control
pluginlib_export_plugin_description_file(hardware_interface share/arduino_car_hardware_plugins.xml)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)

# Install plugin XML files
install(
    FILES
        share/arduino_car_hardware_plugins.xml
    DESTINATION share/${PROJECT_NAME}
)

# Install the robot executable
install(
    TARGETS arduino_car_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
