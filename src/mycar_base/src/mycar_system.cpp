#include "mycar_base/mycar_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const auto logger = rclcpp::get_logger("mycar_hardware");
constexpr double rpm_per_rad_s = 60.0 / (2.0 * M_PI);

double number(const hardware_interface::HardwareInfo & info, const char * key, double fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? fallback : std::stod(it->second);
}

int integer(const hardware_interface::HardwareInfo & info, const char * key, int fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? fallback : std::stoi(it->second);
}

speed_t baud_value(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: throw std::invalid_argument("Unsupported baud rate");
  }
}
}  // namespace

namespace mycar_base
{
MyCarSystem::~MyCarSystem()
{
  if (serial_fd_ >= 0) {
    send_wheel_command(0.0, 0.0);
  }
  close_serial();
}

hardware_interface::CallbackReturn MyCarSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  using hardware_interface::CallbackReturn;
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // 固定左、右顺序，避免串口字段与关节对应关系混乱。
  if (info_.joints.size() != 2 || info_.joints[0].name != "left_wheel_joint" ||
    info_.joints[1].name != "right_wheel_joint")
  {
    RCLCPP_ERROR(logger, "Expected left_wheel_joint, then right_wheel_joint");
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "velocity" ||
      joint.state_interfaces.size() != 2 || joint.state_interfaces[0].name != "position" ||
      joint.state_interfaces[1].name != "velocity")
    {
      RCLCPP_ERROR(logger, "Invalid wheel interfaces");
      return CallbackReturn::ERROR;
    }
  }
  try {
    const auto it = info_.hardware_parameters.find("serial_port");
    serial_port_ = it == info_.hardware_parameters.end() ? "/dev/ttyUSB0" : it->second;
    baud_rate_ = integer(info_, "baud_rate", 115200);
    max_rpm_ = number(info_, "max_rpm", 100.0);
    feedback_timeout_ms_ = integer(info_, "feedback_timeout_ms", 500);
    baud_value(baud_rate_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Invalid hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (!std::isfinite(max_rpm_) || max_rpm_ <= 0.0 || max_rpm_ > 100.0 ||
    feedback_timeout_ms_ <= 0)
  {
    RCLCPP_ERROR(logger, "Invalid RPM limit or timeout (max_rpm must be >0 and <=100)");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyCarSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  for (std::size_t i = 0; i < 2; ++i) {
    result.emplace_back(info_.joints[i].name, "position", &wheel_position_[i]);
    result.emplace_back(info_.joints[i].name, "velocity", &wheel_velocity_[i]);
  }
  return result;
}

std::vector<hardware_interface::CommandInterface> MyCarSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  for (std::size_t i = 0; i < 2; ++i) {
    result.emplace_back(info_.joints[i].name, "velocity", &wheel_command_[i]);
  }
  return result;
}

hardware_interface::CallbackReturn MyCarSystem::on_configure(const rclcpp_lifecycle::State &)
{
  wheel_position_.fill(0.0);
  wheel_velocity_.fill(0.0);
  wheel_command_.fill(0.0);
  sent_.fill(0.0);
  rx_size_ = 0;
  feedback_timeout_reported_ = false;
  close_serial();
  if (!open_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  last_feedback_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger, "HEX RPM chassis %s @ %d; encoder control stays on STM32", serial_port_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCarSystem::on_activate(const rclcpp_lifecycle::State &)
{
  wheel_command_.fill(0.0);
  last_feedback_time_ = std::chrono::steady_clock::now();
  return send_wheel_command(0.0, 0.0) ?
    hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn MyCarSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  wheel_command_.fill(0.0);
  send_wheel_command(0.0, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MyCarSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!receive_reply()) {
    return hardware_interface::return_type::ERROR;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto silent = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_feedback_time_);
  if (silent.count() > feedback_timeout_ms_ && !feedback_timeout_reported_) {
    RCLCPP_WARN(logger, "No valid RPM acknowledgement for %d ms", feedback_timeout_ms_);
    feedback_timeout_reported_ = true;
  }
  // 这里是指令估计，只供关节显示；open_loop 控制器不把它当编码器反馈。
  // 不从RPM确认包计算实测轮速；定位仍由激光和IMU负责。
  const bool recent = now - last_send_time_ < std::chrono::milliseconds(500);
  for (std::size_t i = 0; i < 2; ++i) {
    wheel_velocity_[i] = recent ? sent_[i] : 0.0;
    wheel_position_[i] += wheel_velocity_[i] * std::max(0.0, period.seconds());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyCarSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return send_wheel_command(wheel_command_[0], wheel_command_[1]) ?
    hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

bool MyCarSystem::open_serial()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  termios tty{};
  if (serial_fd_ < 0 || tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(logger, "Cannot open %s: %s", serial_port_.c_str(), std::strerror(errno));
    close_serial();
    return false;
  }
  cfmakeraw(&tty);
  const speed_t baud = baud_value(baud_rate_);
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);
  tty.c_cflag = (tty.c_cflag & ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS)) |
    CS8 | CLOCAL | CREAD;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger, "Cannot configure serial port: %s", std::strerror(errno));
    close_serial();
    return false;
  }
  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void MyCarSystem::close_serial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool MyCarSystem::send_wheel_command(double left, double right)
{
  if (serial_fd_ < 0 || !std::isfinite(left) || !std::isfinite(right)) {
    return false;
  }
  const std::array<double, 2> command{{left, right}};
  std::array<int16_t, 2> rpm{};
  for (std::size_t i = 0; i < 2; ++i) {
    // 单位0.1 RPM，保留低速指令；PWM和编码器PI全部由STM32处理。
    const double value = std::clamp(command[i] * rpm_per_rad_s, -max_rpm_, max_rpm_);
    rpm[i] = static_cast<int16_t>(std::lround(value * 10.0));
  }
  const uint16_t l = static_cast<uint16_t>(rpm[0]);
  const uint16_t r = static_cast<uint16_t>(rpm[1]);
  // 与当前 STM32 car_swarm 一致：大端、有符号16位、固定8字节。
  const std::array<uint8_t, 8> packet{{0xAA, 0x55,
    static_cast<uint8_t>(l >> 8), static_cast<uint8_t>(l),
    static_cast<uint8_t>(r >> 8), static_cast<uint8_t>(r), 0x0D, 0x0A}};
  std::size_t done = 0;
  while (done < packet.size()) {
    const ssize_t count = ::write(serial_fd_, packet.data() + done, packet.size() - done);
    if (count > 0) {
      done += static_cast<std::size_t>(count);
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd fd{serial_fd_, POLLOUT, 0};
      if (::poll(&fd, 1, 5) > 0 && (fd.revents & POLLOUT)) {
        continue;
      }
      RCLCPP_ERROR(logger, "Serial transmit timeout");
      return false;
    } else {
      RCLCPP_ERROR(logger, "Serial write failed: %s", std::strerror(errno));
      return false;
    }
  }
  for (std::size_t i = 0; i < 2; ++i) {
    sent_[i] = static_cast<double>(rpm[i]) * 0.1 / rpm_per_rad_s;
  }
  last_send_time_ = std::chrono::steady_clock::now();
  return true;
}

bool MyCarSystem::receive_reply()
{
  std::array<uint8_t, 128> chunk{};
  // 限制单次读取量，噪声输入也不会长期占用50Hz控制线程。
  for (unsigned pass = 0; pass < 4; ++pass) {
    const ssize_t count = ::read(serial_fd_, chunk.data(), chunk.size());
    if (count == 0 || (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
      break;
    }
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(logger, "Serial read failed: %s", std::strerror(errno));
      return false;
    }
    for (ssize_t i = 0; i < count; ++i) {
      std::move(rx_.begin() + 1, rx_.end(), rx_.begin());
      rx_.back() = chunk[i];
      rx_size_ = std::min(rx_size_ + 1, rx_.size());
      if (rx_size_ == 8 && rx_[0] == 0xAA && rx_[1] == 0x55 &&
        rx_[6] == 0x0D && rx_[7] == 0x0A)
      {
        // 回包仅确认通信；不当作实际转速或编码器计数使用。
        last_feedback_time_ = std::chrono::steady_clock::now();
        feedback_timeout_reported_ = false;
        rx_size_ = 0;
      }
    }
  }
  return true;
}
}  // namespace mycar_base

PLUGINLIB_EXPORT_CLASS(mycar_base::MyCarSystem, hardware_interface::SystemInterface)
