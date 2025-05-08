#include "diffdrive_hardware/diffdrive_hardware.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_hardware
{

hardware_interface::CallbackReturn DiffDriveHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize parameters
  hw_start_sec_ = std::stod(info_.hardware_parameters["hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["hw_stop_duration_sec"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
  encoder_ticks_per_rev_ = std::stod(info_.hardware_parameters["encoder_ticks_per_rev"]);
  serial_port_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  timeout_ms_ = std::stoi(info_.hardware_parameters["timeout_ms"]);

  // Initialize encoder pins
  encoder_pins_[0] = 3;  // ENCODER_L_A
  encoder_pins_[1] = 5;  // ENCODER_L_B
  encoder_pins_[2] = 2;  // ENCODER_R_A
  encoder_pins_[3] = 4;  // ENCODER_R_B

  // Initialize motor pins
  motor_pins_[0] = 9;   // ENA
  motor_pins_[1] = 10;  // IN1
  motor_pins_[2] = 11;  // IN2
  motor_pins_[3] = 8;   // IN3
  motor_pins_[4] = 7;   // IN4
  motor_pins_[5] = 6;   // ENB

  // Initialize state arrays
  hw_positions_[0] = 0.0;
  hw_positions_[1] = 0.0;
  hw_velocities_[0] = 0.0;
  hw_velocities_[1] = 0.0;
  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;

  // Register state interfaces
  for (uint i = 0; i < info_.joints.size(); i++) {
    hardware_interface::ComponentInfo joint = info_.joints[i];
    
    // Register state interfaces
    state_interfaces_.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces_.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));

    // Register command interfaces
    command_interfaces_.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Initialize Arduino communication
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Clean up Arduino communication
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Start Arduino communication
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO: Stop Arduino communication
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Read encoder values from Arduino
  // For now, just update the state with the last command
  for (uint i = 0; i < hw_commands_.size(); i++) {
    hw_velocities_[i] = hw_commands_[i];
    hw_positions_[i] += hw_velocities_[i] * 0.01;  // Simple integration
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Send motor commands to Arduino
  // For now, just store the commands
  for (uint i = 0; i < hw_commands_.size(); i++) {
    // Store the command
    hw_commands_[i] = hw_commands_[i];
  }

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_hardware::DiffDriveHardware, hardware_interface::SystemInterface) 