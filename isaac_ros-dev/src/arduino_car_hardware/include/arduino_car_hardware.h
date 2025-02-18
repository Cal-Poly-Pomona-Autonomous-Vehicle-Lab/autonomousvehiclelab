#ifndef ARDUINO_CAR_HARDWARE_H
#define ARDUINO_CAR_HARDWARE_H



#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "arduino_comms.h"

class ArduinoCarHardware : public hardware_interface::SystemInterface {
public:
    // Lifecycle methods
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override;

    // ROS 2 hardware interface methods
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

    // Command setters
    void set_velocity_command(double velocity) { velocity_command_ = velocity; };
    void set_steering_command(double steering) { steering_command_ = steering; };

private:
    ArduinoComms arduino_comms_;
    double velocity_command_ = 0.0;  // Command: forward velocity
    double steering_command_ = 0.0; // Command: steering angle
    double velocity_state_ = 0.0;   // State: current velocity
    double steering_state_ = 0.0;  // State: current steering angle
};

#endif // ARDUINO_CAR_HARDWARE_H
