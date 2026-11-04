#ifndef MYCAR_BASE__MYCAR_SYSTEM_HPP_
#define MYCAR_BASE__MYCAR_SYSTEM_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"

namespace mycar_base
{
class MyCarSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyCarSystem)
  ~MyCarSystem() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  bool open_serial();
  void close_serial();
  bool send_wheel_command(double left, double right);
  bool receive_reply();

  int serial_fd_{-1};
  std::string serial_port_;
  int baud_rate_{115200};
  double max_rpm_{100.0};
  double encoder_cpr_{1468.2};
  int feedback_timeout_ms_{500};
  std::array<double, 2> wheel_position_{{0.0, 0.0}};
  std::array<double, 2> wheel_velocity_{{0.0, 0.0}};
  std::array<double, 2> wheel_command_{{0.0, 0.0}};
  std::array<uint32_t, 2> counts_{};
  uint32_t sample_ms_{0};
  bool feedback_received_{false};
  std::array<uint8_t, 17> rx_{};
  std::size_t rx_size_{0};
  std::chrono::steady_clock::time_point last_feedback_time_;
};
}  // namespace mycar_base
#endif
