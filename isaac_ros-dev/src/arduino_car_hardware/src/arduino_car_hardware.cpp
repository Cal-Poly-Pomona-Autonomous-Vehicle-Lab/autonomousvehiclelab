#include "arduino_car_hardware/include/arduino_car_hardware.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <rclcpp/logging.hpp>
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

hardware_interface::CallbackReturn ArduinoCarHardware::on_init(const hardware_interface::HardwareInfo &info) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"), "Initializing hardware interface...");

    // Retrieve hardware parameters
    const std::string port = info.hardware_parameters.at("serial_device");
    const int baud_rate = std::stoi(info.hardware_parameters.at("baud_rate"));
    const int timeout_ms = std::stoi(info.hardware_parameters.at("timeout_ms"));

    try {
        // Initialize Arduino communication
        arduino_comms_(port, baud_rate, timeout_ms);

        if (!arduino_comms_.is_connected_) {
            RCLCPP_ERROR(rclcpp::get_logger("ArduinoCarHardware"),
                         "Failed to connect to Arduino on port: %s", port.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"),
                    "Successfully connected to Arduino on port: %s", port.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoCarHardware"),
                     "Exception during Arduino setup: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoCarHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back("wheel", hardware_interface::HW_IF_VELOCITY, &velocity_state_);
    state_interfaces.emplace_back("steering", hardware_interface::HW_IF_POSITION, &steering_state_);
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoCarHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back("wheel", hardware_interface::HW_IF_VELOCITY, &velocity_command_);
    command_interfaces.emplace_back("steering", hardware_interface::HW_IF_POSITION, &steering_command_);
    return command_interfaces;
}

hardware_interface::CallbackReturn ArduinoCarHardware::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"), "Configuring hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoCarHardware::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"), "Activating hardware interface...");
    arduino_comms_.setMotorValues(0.0, 512.0);  // Default state
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoCarHardware::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"), "Deactivating hardware interface...");
    arduino_comms_.setMotorValues(0.0, 0.0);  // Stop motors
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoCarHardware::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"), "Cleaning up hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoCarHardware::on_shutdown(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoCarHardware"), "Shutting down hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoCarHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
    try {
        // Read sensor data from Arduino
        arduino_comms_.getVelocityAndSteerValues();

        // Update state variables
        velocity_state_ = (arduino_comms_.left_wheel_vel + arduino_comms_.right_wheel_vel) / 2.0;
        steering_state_ = arduino_comms_.steering_angle;

        RCLCPP_DEBUG(rclcpp::get_logger("ArduinoCarHardware"),
                     "Read values - Velocity: %f, Steering: %f", velocity_state_, steering_state_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoCarHardware"),
                     "Error reading from Arduino: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoCarHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
    try {
        // Write commands to Arduino
        arduino_comms_.setMotorValues(velocity_command_, steering_command_);

        RCLCPP_DEBUG(rclcpp::get_logger("ArduinoCarHardware"),
                     "Sent commands - Velocity: %f, Steering: %f", velocity_command_, steering_command_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArduinoCarHardware"),
                     "Error writing to Arduino: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}


PLUGINLIB_EXPORT_CLASS(
    ArduinoCarHardware,
    hardware_interface::SystemInterface
)
