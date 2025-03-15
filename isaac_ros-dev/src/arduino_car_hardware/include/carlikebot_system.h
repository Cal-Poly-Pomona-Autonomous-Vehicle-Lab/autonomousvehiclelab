#ifndef BICDRIVE_ARDUINO_CARLIKEBOT_SYSTEM_H
#define BICDRIVE_ARDUINO_CARLIKEBOT_SYSTEM_H

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arduino_comms.h"
#include "steering.h"
#include "traction.h"

namespace bicdrive_arduino
{

// Joint value representation
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

// Joint container
struct Joint
{
  explicit Joint(const std::string & name) : joint_name(name)
  {
    state = JointValue();
    command = JointValue();
  }

  Joint() = default;

  std::string joint_name;
  JointValue state;
  JointValue command;
};

class CarlikeBotSystemHardware : public hardware_interface::SystemInterface
{
  struct Config
  {
    std::string rear_wheel_name = "";
    std::string front_wheel_name = "";
    float loop_rate = 0.0;
    std::string device = "";
    int baud_rate = 0;
    int timeout_ms = 0;
  };

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CarlikeBotSystemHardware);

  // System interface overrides
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Public setters for testing (allows unit tests to inject values)
  void set_steering_pos(double pos) { steering_.pos = pos; }
  void set_traction_cmd(double cmd) { traction_.cmd = cmd; }

  // Public getters for testing and inspection
  double get_steering_pos() const { return steering_.pos; }
  double get_traction_cmd() const { return traction_.cmd; }

private:
  ArduinoComms comms_;  // Communication class with Arduino
  Config cfg_;          // Configuration for hardware connection

  Steering steering_;  // Steering control (front wheel)
  Traction traction_;  // Traction control (rear wheel)
};

}  // namespace bicdrive_arduino

#endif  // BICDRIVE_ARDUINO_CARLIKEBOT_SYSTEM_H
