#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "s7_client.h"

namespace agv_project
{
class PlcSystemHardware : public hardware_interface::SystemInterface
{
public:
  PlcSystemHardware() = default;
  RCLCPP_SHARED_PTR_DEFINITIONS(PlcSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  TSnap7Client *client_;
  std::string plc_ip_ = "192.168.0.1";

  double wheel_speed_command_ = 0.0;
  double steering_angle_command_ = 0.0;
  double wheel_speed_state_ = 0.0;
  double steering_angle_state_ = 0.0;
  double wheel_position_state_ = 0.0;
  double yaw_ = 0.0;
  int32_t last_pulse_ = 0;
};
} // namespace agv_project
