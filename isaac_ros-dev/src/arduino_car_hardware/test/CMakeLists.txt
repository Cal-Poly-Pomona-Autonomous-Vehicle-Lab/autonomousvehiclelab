cmake_minimum_required(VERSION 3.10)
project(ArduinoCarHardwareTest)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

include_directories(
  ../include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

set(SOURCE_FILES
  ../src/arduino_comms.cpp
  ../src/carlikebot_system.cpp
)

set(TEST_FILE
  ../test/test_load_carlikebot_system.cpp
)

add_executable(ArduinoCarHardwareTest ${SOURCE_FILES} ${TEST_FILE})

target_link_libraries(ArduinoCarHardwareTest
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  Threads::Threads
  GTest::GTest
  GTest::Main
)

enable_testing()
add_test(NAME ArduinoCarHardwareTest COMMAND ArduinoCarHardwareTest)
