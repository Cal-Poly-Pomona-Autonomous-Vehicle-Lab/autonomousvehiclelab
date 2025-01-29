#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include "arduino_car_hardware.h"

int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create an executor for managing node execution
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto node = std::make_shared<rclcpp::Node>("arduino_car_node");

    // Declare and get parameters
    node->declare_parameter<std::string>("device", "/dev/ttyACM0");
    node->declare_parameter<int>("baud_rate", 115200);

    std::string device;
    int baud_rate;
    node->get_parameter("device", device);
    node->get_parameter("baud_rate", baud_rate);

    // Create the hardware interface
    auto hardware = std::make_shared<ArduinoCarHardware>();

    // Configure hardware interface
    hardware_interface::HardwareInfo hw_info;
    hw_info.hardware_parameters["serial_device"] = device;
    hw_info.hardware_parameters["baud_rate"] = std::to_string(baud_rate);
    hw_info.hardware_parameters["timeout_ms"] = "100";

    if (hardware->on_init(hw_info) != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize ArduinoCarHardware.");
        rclcpp::shutdown();
        return 1;
    }

    // Create the controller manager
    auto resource_manager = std::make_unique<hardware_interface::ResourceManager>();
    controller_manager::ControllerManager cm(
        std::move(resource_manager), executor, "arduino_controller_manager", node->get_fully_qualified_name());

    // Add the node to the executor
    executor->add_node(node);

    // Main control loop at 10 Hz
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        hardware->read();  // Read hardware state
        cm.update(hardware->get_time(), hardware->get_period()); //Change this
        hardware->write(); // Send commands to hardware

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
