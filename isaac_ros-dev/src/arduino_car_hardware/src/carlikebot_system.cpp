// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "carlikebot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arduino_car_hardware
{
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.rear_wheel_name = info_.hardware_parameters["rear_wheel_name"];
  cfg_.front_wheel_name = info_.hardware_parameters["front_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  traction_.setup(cfg_.rear_wheel_name);
  steering_.setup(cfg_.front_wheel_name);

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CarlikeBotSystemHardware"),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' is a steering joint.",
        joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' is a drive joint.",
        joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      steering_.name, hardware_interface::HW_IF_POSITION, &steering_.pos));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      traction_.name, hardware_interface::HW_IF_VELOCITY, &traction_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        traction_.name, hardware_interface::HW_IF_POSITION, &traction_.pos));
    
    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "State interfaces exported");

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    steering_.name, hardware_interface::HW_IF_POSITION,
    &steering_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    traction_.name, hardware_interface::HW_IF_VELOCITY,
    &traction_.cmd));

  RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Command interfaces exported");

  return command_interfaces;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Configuring ...please wait...");
  if (!comms_.connected())
  {
    comms_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  }
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Activating ...please wait...");

  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{ 
  std::vector<double> values;
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  values = comms_.getVelocityAndSteerValues();
  steering_.pos = values[1];
  traction_.vel = values[0];

  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Reading Velocity:%f, Steer:%f",traction_.vel,steering_.pos);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type arduino_car_hardware ::CarlikeBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  
  double steering_val = steering_.cmd;
  double velocity = traction_.cmd;
  

  if (steering_val >= 1){
    steering_val = 1;
}

  if (steering_val <= -1){
    steering_val = -1;
  }


  if (velocity > 0.3){
    velocity = 0.3;
  }

  if (velocity < -0.3){
    velocity = -0.3;
  }

  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Writing Velocity:%f, Steer:%f",traction_.cmd,steering_.cmd);
  comms_.setMotorValues(velocity,steering_val);
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Wrote Velocity:%f, Steer:%f",velocity,steering_val);

  // RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Wrote Velocity:%f, Steer:%f",traction_.cmd,steering_.cmd);

  return hardware_interface::return_type::OK;
}

} 

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arduino_car_hardware::CarlikeBotSystemHardware, hardware_interface::SystemInterface)

