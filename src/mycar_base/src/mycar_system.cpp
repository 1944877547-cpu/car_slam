// 底盘硬件桥：上接ros2_control，下接STM32；轮速PI在MCU内执行。
// 使用说明见项目根目录 RTAB_MAP.md 和 cartographer.md。

#include "mycar_base/mycar_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const auto logger = rclcpp::get_logger("mycar_hardware");
constexpr double rpm_per_rad_s = 60.0 / (2.0 * M_PI);

// STM32反馈的32位字段为大端；先按无符号数解码，差分时再恢复有符号方向。
uint32_t u32(const uint8_t * p)
{
  return uint32_t(p[0]) << 24 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 8 | p[3];
}

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
// 插件卸载时尽力发送零速，再关闭串口；下位机另有独立命令超时停车。
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
    encoder_cpr_ = number(info_, "encoder_cpr", 1468.2);
    feedback_timeout_ms_ = integer(info_, "feedback_timeout_ms", 500);
    baud_value(baud_rate_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Invalid hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (!std::isfinite(max_rpm_) || max_rpm_ <= 0.0 || max_rpm_ > 100.0 ||
    !std::isfinite(encoder_cpr_) || encoder_cpr_ <= 0.0 ||
    feedback_timeout_ms_ <= 0)
  {
    RCLCPP_ERROR(logger, "Invalid RPM limit, encoder CPR or feedback timeout");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ros2_control状态接口：累计轮角度(rad)、由编码器计算的轮角速度(rad/s)。
std::vector<hardware_interface::StateInterface> MyCarSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  for (std::size_t i = 0; i < 2; ++i) {
    result.emplace_back(info_.joints[i].name, "position", &wheel_position_[i]);
    result.emplace_back(info_.joints[i].name, "velocity", &wheel_velocity_[i]);
  }
  return result;
}

// 控制器写入左右目标轮角速度(rad/s)，write()负责转为串口0.1 RPM。
std::vector<hardware_interface::CommandInterface> MyCarSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  for (std::size_t i = 0; i < 2; ++i) {
    result.emplace_back(info_.joints[i].name, "velocity", &wheel_command_[i]);
  }
  return result;
}

// 生命周期配置阶段只初始化串口和测量基线；激活阶段还要等到真实反馈。
hardware_interface::CallbackReturn MyCarSystem::on_configure(const rclcpp_lifecycle::State &)
{
  wheel_position_.fill(0.0);
  wheel_velocity_.fill(0.0);
  wheel_command_.fill(0.0);
  counts_.fill(0);
  sample_ms_ = 0;
  feedback_received_ = false;
  rx_size_ = 0;
  close_serial();
  if (!open_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  last_feedback_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger, "RPM chassis %s @ %d; real encoder feedback, %.1f counts/revolution",
    serial_port_.c_str(), baud_rate_, encoder_cpr_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCarSystem::on_activate(const rclcpp_lifecycle::State &)
{
  wheel_command_.fill(0.0);
  last_feedback_time_ = std::chrono::steady_clock::now();
  if (!send_wheel_command(0.0, 0.0)) return hardware_interface::CallbackReturn::ERROR;
  // 先收到真实编码器反馈才允许激活，防止把旧ACK固件当成轮式里程计。
  const auto deadline = last_feedback_time_ + std::chrono::milliseconds(feedback_timeout_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!receive_reply()) return hardware_interface::CallbackReturn::ERROR;
    if (feedback_received_) return hardware_interface::CallbackReturn::SUCCESS;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  RCLCPP_ERROR(logger, "No encoder feedback; update STM32 firmware and check wiring");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn MyCarSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  wheel_command_.fill(0.0);
  send_wheel_command(0.0, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCarSystem::on_error(const rclcpp_lifecycle::State &)
{
  wheel_command_.fill(0.0);
  send_wheel_command(0.0, 0.0);
  close_serial();
  // 错误后停止硬件读写，重新启动底盘才能恢复。
  return hardware_interface::CallbackReturn::ERROR;
}

// 50Hz控制周期中的read阶段：更新真实关节状态，并检查反馈是否持续有效。
hardware_interface::return_type MyCarSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!receive_reply()) {
    send_wheel_command(0.0, 0.0);
    return hardware_interface::return_type::ERROR;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto silent = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_feedback_time_);
  if (!feedback_received_ || silent.count() >= feedback_timeout_ms_) {
    wheel_velocity_.fill(0.0);
    send_wheel_command(0.0, 0.0);
    RCLCPP_ERROR(logger, "Encoder feedback timed out; stopping chassis");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

// 每周期的顺序由controller_manager组织：read -> 控制器update -> write。
hardware_interface::return_type MyCarSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return send_wheel_command(wheel_command_[0], wheel_command_[1]) ?
    hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

// POSIX串口设为非阻塞raw模式、8N1、无硬件流控；这里直接收发二进制字节。
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
  // 高速转弯达到单轮上限时，两轮同比缩放，保持指令的转弯曲率。
  const double limit = max_rpm_ / rpm_per_rad_s;
  const double peak = std::max(std::abs(left), std::abs(right));
  const double scale = peak > limit ? limit / peak : 1.0;
  std::array<int16_t, 2> rpm{};
  for (std::size_t i = 0; i < 2; ++i) {
    // 单位0.1 RPM，保留低速指令；PWM和编码器PI全部由STM32处理。
    const double value = std::clamp(command[i] * scale * rpm_per_rad_s, -max_rpm_, max_rpm_);
    rpm[i] = static_cast<int16_t>(std::lround(value * 10.0));
  }
  // 转成uint16_t保留二补码位模式，负转速不会变成绝对值；本下发帧没有校验字段。
  const uint16_t l = static_cast<uint16_t>(rpm[0]);
  const uint16_t r = static_cast<uint16_t>(rpm[1]);
  // 与当前 STM32 car_swarm 一致：大端、有符号16位、固定8字节。
  const std::array<uint8_t, 8> packet{{0xAA, 0x55,
    static_cast<uint8_t>(l >> 8), static_cast<uint8_t>(l),
    static_cast<uint8_t>(r >> 8), static_cast<uint8_t>(r), 0x0D, 0x0A}};
  // write()可能只写出部分数据；EAGAIN用poll短暂等待，EINTR重试。
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
      // 串口是字节流：用17字节滑窗识别帧，支持拆包、粘包和错位恢复。
      std::move(rx_.begin() + 1, rx_.end(), rx_.begin());
      rx_.back() = chunk[i];
      rx_size_ = std::min(rx_size_ + 1, rx_.size());
      if (rx_size_ == rx_.size() && rx_[0] == 0xAA && rx_[1] == 0x66 &&
        rx_[15] == 0x0D && rx_[16] == 0x0A)
      {
        // 上行校验为前14字节的8位累加和，不是CRC；校验失败不刷新保活时间。
        uint8_t sum = 0;
        for (std::size_t j = 0; j < 14; ++j) sum += rx_[j];
        if (sum != rx_[14]) continue;
        const uint32_t stamp = u32(rx_.data() + 2);
        const std::array<uint32_t, 2> count{{u32(rx_.data() + 6), u32(rx_.data() + 10)}};
        if (feedback_received_) {
          const uint32_t dt = stamp - sample_ms_;  // 无符号减法支持毫秒计数回绕。
          if (dt == 0) continue;                  // 重复包不能刷新反馈保活时间。
          if (dt > static_cast<uint32_t>(feedback_timeout_ms_)) {
            RCLCPP_ERROR(logger, "Encoder sample time reset or discontinuity; restart chassis");
            return false;
          }
          for (std::size_t j = 0; j < 2; ++j) {
            // 计数允许32位自然回绕；相邻采样间实际增量必须远小于2^31。
            const int32_t delta = static_cast<int32_t>(count[j] - counts_[j]);
            const double angle = delta * (2.0 * M_PI / encoder_cpr_);
            wheel_position_[j] += angle;
            wheel_velocity_[j] = angle * 1000.0 / dt;
          }
        }
        // 首包仅建立计数基线，避免把MCU上电后的历史累计值当成ROS起点位移。
        counts_ = count;
        sample_ms_ = stamp;
        feedback_received_ = true;
        last_feedback_time_ = std::chrono::steady_clock::now();
        rx_size_ = 0;
      }
    }
  }
  return true;
}
}  // namespace mycar_base

PLUGINLIB_EXPORT_CLASS(mycar_base::MyCarSystem, hardware_interface::SystemInterface)
