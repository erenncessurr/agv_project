#include "agv_project/plc_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstring>
#include "pluginlib/class_list_macros.hpp"
#include "s7_client.h"


namespace agv_project
{
hardware_interface::CallbackReturn PlcSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  client_ = new TSnap7Client();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PlcSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (client_->ConnectTo(plc_ip_.c_str(), 0, 1) == 0) {
    RCLCPP_INFO(rclcpp::get_logger("PlcSystemHardware"), "Connected to PLC at %s", plc_ip_.c_str());
    return CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("PlcSystemHardware"), "Failed to connect to PLC");
    return CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn PlcSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  client_->Disconnect();
  return CallbackReturn::SUCCESS;
}

float reverse_bytes_to_float(const uint8_t *bytes)
{
  uint8_t reversed[4] = {bytes[3], bytes[2], bytes[1], bytes[0]};
  float value;
  std::memcpy(&value, reversed, sizeof(float));
  return value;
}

hardware_interface::return_type PlcSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  uint8_t buf_steering[4], buf_pulse[4], buf_pot[4];

  // PLC'den veri oku
  client_->ReadArea(S7AreaDB, 1, 12, 4, S7WLByte, buf_steering);     // Yedek - kullanılmayacak
  client_->ReadArea(S7AreaDB, 1, 28, 4, S7WLByte, buf_pulse);        // Pulse (REAL)
  client_->ReadArea(S7AreaDB, 20, 16, 4, S7WLByte, buf_pot);          // Potansiyometre (REAL)

  float pulse_float = reverse_bytes_to_float(buf_pulse);
  int32_t current_pulse = static_cast<int32_t>(std::round(pulse_float));

  float steering_deg = reverse_bytes_to_float(buf_pot) + 7;
  float steering_rad = steering_deg * M_PI / 180.0;

  static bool first_read = true;
  if (first_read) {
    last_pulse_ = current_pulse;
    yaw_ = 0.0;
    first_read = false;
  }

  int32_t delta_pulse = current_pulse - last_pulse_;
  if (delta_pulse > 6000)
    delta_pulse -= 65536;
  else if (delta_pulse < -6000)
    delta_pulse += 65536;

  last_pulse_ = current_pulse;

  double pulse_distance = 0.057583;  // metre/pulse
  double delta_distance = delta_pulse * pulse_distance;

  double dt = period.seconds();
  wheel_position_state_ += delta_distance;
  wheel_speed_state_ = (dt > 0.0) ? delta_distance / dt : 0.0;

  steering_angle_state_ = steering_rad;

  //yaw hesapla
  double wheel_base = 1.015;  // metre cinsinden (aracına göre ayarla)
  double angular_velocity = (wheel_speed_state_ / wheel_base) * std::tan(steering_angle_state_);
  yaw_ += angular_velocity * dt;

  //RCLCPP_INFO(rclcpp::get_logger("PlcSystemHardware"),
  //            "pulse: %d | Δpulse: %d | v: %.3f m/s | yaw: %.3f rad | angle: %.2f°",
  //            current_pulse, delta_pulse, wheel_speed_state_, yaw_, steering_deg);

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type PlcSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Gelen velocity komutunu int16_t'e dönüştür
  int16_t plc_speed_command = static_cast<int16_t>(wheel_speed_command_ * 1000.0);
  uint8_t buf[2];
  buf[0] = (plc_speed_command >> 8) & 0xFF;
  buf[1] = plc_speed_command & 0xFF;

  // Log: hız
  // RCLCPP_INFO(rclcpp::get_logger("PlcSystemHardware"),
  //             "Speed Command (ROS): %.3f m/s → PLC: %d", wheel_speed_command_, plc_speed_command);

  if (client_->WriteArea(S7AreaDB, 1, 18, 2, S7WLByte, buf) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("PlcSystemHardware"), "Failed to write speed command");
  }

  // Gelen direksiyon komutunu int16_t'e dönüştür
  int16_t plc_steering_command = static_cast<int16_t>(steering_angle_command_ * 1000.0);
  uint8_t buf2[2];
  buf2[0] = (plc_steering_command >> 8) & 0xFF;
  buf2[1] = plc_steering_command & 0xFF;

  // // Log: direksiyon
  // RCLCPP_INFO(rclcpp::get_logger("PlcSystemHardware"),
  //             "Steering Command (ROS): %.3f rad → PLC: %d", steering_angle_command_, plc_steering_command);

  if (client_->WriteArea(S7AreaDB, 1, 20, 2, S7WLByte, buf2) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("PlcSystemHardware"), "Failed to write steering command");
  }

  return hardware_interface::return_type::OK;
}


std::vector<hardware_interface::StateInterface> PlcSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(
      "front_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_position_state_);
    state_interfaces.emplace_back(
      "front_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_speed_state_);
    state_interfaces.emplace_back(
      "steering_cylinder_joint", hardware_interface::HW_IF_VELOCITY, &steering_angle_state_);   
    state_interfaces.emplace_back(
      "steering_cylinder_joint", hardware_interface::HW_IF_POSITION, &steering_angle_state_);
  
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PlcSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
      "front_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_speed_command_);
  
    command_interfaces.emplace_back(
      "steering_cylinder_joint", hardware_interface::HW_IF_POSITION, &steering_angle_command_);
  
    return command_interfaces;
}



} // namespace agv_project

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(agv_project::PlcSystemHardware, hardware_interface::SystemInterface)
