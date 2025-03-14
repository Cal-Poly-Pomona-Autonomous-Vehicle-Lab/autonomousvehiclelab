cmake_minimum_required(VERSION 3.8)
project(arduino_car_hardware)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Add include directories (header files)
include_directories(
  include
)

# Add shared library target
add_library(${PROJECT_NAME} SHARED
  src/arduino_comms.cpp
  src/carlikebot_system.cpp)

# Link required libraries
target_link_libraries(${PROJECT_NAME}
  Boost::system
)

# Add ament target dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  Boost
)

# Export plugin description for ROS 2 control
pluginlib_export_plugin_description_file(hardware_interface share/arduino_car_hardware_plugins.xml)

# Install header files (from include/)
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the shared library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin description XML
install(
  FILES share/arduino_car_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export include directories and libraries
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib Boost)

# Finalize package
ament_package()
