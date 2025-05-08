#ifndef DIFFDRIVE_HARDWARE_HPP_
#define DIFFDRIVE_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace diffdrive_hardware
{
class DiffDriveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

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

private:
  // Parameters for the DiffBot
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_positions_[2];
  double hw_velocities_[2];
  double hw_commands_[2];

  // Store the command for the robot
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  // Wheel properties
  double wheel_radius_;
  double wheel_separation_;
  double encoder_ticks_per_rev_;

  // Arduino communication
  std::string serial_port_;
  int baud_rate_;
  int timeout_ms_;
  int encoder_pins_[4];  // [ENCODER_L_A, ENCODER_L_B, ENCODER_R_A, ENCODER_R_B]
  int motor_pins_[6];    // [ENA, IN1, IN2, IN3, IN4, ENB]

  // ROS2 publishers and subscribers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Helper functions
  void updateOdom();
  void publishOdom();
  void publishJointStates();
};

}  // namespace diffdrive_hardware

#endif  // DIFFDRIVE_HARDWARE_HPP_ 