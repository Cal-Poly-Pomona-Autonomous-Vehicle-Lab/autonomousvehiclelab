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

# Add shared library target
add_library(${PROJECT_NAME} SHARED
  src/arduino_comms.cpp
  src/carlikebot_system.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)


# Add include directories (header files)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Add ament target dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  Boost
)

# Link required libraries
target_link_libraries(${PROJECT_NAME}
  Boost::system
)

# Export plugin description for ROS 2 control
pluginlib_export_plugin_description_file(hardware_interface arduino_car_hardware_plugins.xml)

# Install header files (ensures headers are installed properly)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install the shared library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# # Install plugin description XML
# install(
#   FILES arduino_car_hardware_plugins.xml
#   DESTINATION share/${PROJECT_NAME}
# )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib Boost)

# Finalize package
ament_package()
