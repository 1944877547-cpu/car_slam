# 本文件是setup.py注册的实际驱动入口；同包根目录的同名旧文件不是此入口。
import math
import struct
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


# WIT二进制协议每帧11字节，末字节为前10字节和；载荷是小端有符号int16。
FRAME_HEADER = 0x55
FRAME_LENGTH = 11
ACCEL_FRAME = 0x51
GYRO_FRAME = 0x52
ANGLE_FRAME = 0x53
MAG_FRAME = 0x54

DEFAULT_PORT = "/dev/ttyTHS1"
DEFAULT_BAUD = 115200
DEFAULT_RATE_HZ = 20
GRAVITY = 9.8
OUTPUT_ACC_GYRO_ANGLE = 0x000E

WIT_RETURN_RATE_VALUES = {
    1: 0x03,
    2: 0x04,
    5: 0x05,
    10: 0x06,
    20: 0x07,
    50: 0x08,
    100: 0x09,
    125: 0x0A,
    200: 0x0B,
}


def checksum_ok(frame):
    return (sum(frame[:10]) & 0xFF) == frame[10]


def unpack_shorts(payload):
    return struct.unpack("<hhhh", bytes(payload))


def quaternion_from_euler(roll, pitch, yaw):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class WitParser:
    def __init__(self):
        self.buffer = bytearray()
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]

    # 按帧头、类型、长度和校验定位有效帧，噪声只丢一个字节后继续找帧。
    def push_byte(self, value):
        self.buffer.append(value)

        while self.buffer:
            if self.buffer[0] != FRAME_HEADER:
                self.buffer.pop(0)
                continue

            if len(self.buffer) < 2:
                return None

            if self.buffer[1] not in (ACCEL_FRAME, GYRO_FRAME, ANGLE_FRAME, MAG_FRAME):
                self.buffer.pop(0)
                continue

            if len(self.buffer) < FRAME_LENGTH:
                return None

            frame = bytes(self.buffer[:FRAME_LENGTH])
            if not checksum_ok(frame):
                self.buffer.pop(0)
                return None

            del self.buffer[:FRAME_LENGTH]
            self._parse_frame(frame)
            return frame[1]

        return None

    # 将厂商量程换算为ROS SI单位：加速度m/s²、角速度rad/s；角度调试话题保留度。
    def _parse_frame(self, frame):
        values = unpack_shorts(frame[2:10])
        frame_type = frame[1]

        if frame_type == ACCEL_FRAME:
            self.acceleration = [
                values[0] / 32768.0 * 16.0 * GRAVITY,
                values[1] / 32768.0 * 16.0 * GRAVITY,
                values[2] / 32768.0 * 16.0 * GRAVITY,
            ]
        elif frame_type == GYRO_FRAME:
            self.angular_velocity = [
                values[0] / 32768.0 * 2000.0 * math.pi / 180.0,
                values[1] / 32768.0 * 2000.0 * math.pi / 180.0,
                values[2] / 32768.0 * 2000.0 * math.pi / 180.0,
            ]
        elif frame_type == ANGLE_FRAME:
            self.angle_degree = [
                values[0] / 32768.0 * 180.0,
                values[1] / 32768.0 * 180.0,
                values[2] / 32768.0 * 180.0,
            ]
        elif frame_type == MAG_FRAME:
            self.magnetometer = [float(values[0]), float(values[1]), float(values[2])]


class IMUDriverNode(Node):
    def __init__(self):
        super().__init__("imu_driver_node")

        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("baud", DEFAULT_BAUD)
        self.declare_parameter("rate_hz", DEFAULT_RATE_HZ)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud").get_parameter_value().integer_value
        self.rate_hz = self.get_parameter("rate_hz").get_parameter_value().integer_value
        if self.baud_rate <= 0 or self.rate_hz <= 0:
            raise ValueError("Invalid IMU baud or rate_hz")

        self.stop_event = threading.Event()
        self.parser = WitParser()
        self.frame_times = {}
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"
        # 初始协方差，避免EKF把未知精度当作接近零噪声；可按实测噪声调整。
        # 姿态尚未验证，-1 表示不可用于融合；原始角度仍发布到 /imu/angle。
        self.imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.imu_msg.angular_velocity_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
        self.imu_msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
        self.angle_msg = Vector3Stamped()
        self.angle_msg.header.frame_id = "imu_link"

        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.angle_pub = self.create_publisher(Vector3Stamped, "imu/angle", 10)

        # 串口读取放在后台线程；ROS主线程负责spin和退出管理。
        self.driver_thread = threading.Thread(target=self.driver_loop, daemon=True)
        self.driver_thread.start()

    def destroy_node(self):
        self.stop_event.set()
        self.driver_thread.join(timeout=1.2)
        super().destroy_node()

    def driver_loop(self):
        try:
            # with保证正常退出、断连或配置失败时都会关闭串口。
            with serial.Serial(port=self.port, baudrate=self.baud_rate, timeout=0.5) as wit_serial:
                self.configure_return_rate(wit_serial)
                self.read_serial_loop(wit_serial)
        except Exception as exc:
            if not self.stop_event.is_set() and rclpy.ok():
                self.get_logger().error("IMU serial error: {}".format(exc))
                rclpy.try_shutdown()

    def configure_return_rate(self, wit_serial):
        rate_value = WIT_RETURN_RATE_VALUES.get(self.rate_hz)
        if rate_value is None:
            self.get_logger().warn(
                "Unsupported WIT return rate {} Hz; supported: {}".format(
                    self.rate_hz, sorted(WIT_RETURN_RATE_VALUES.keys())
                )
            )
            return

        self.get_logger().info(
            "Setting IMU output to accel/gyro/angle only, return rate: {} Hz".format(
                self.rate_hz
            )
        )
        wit_serial.write(bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5]))
        wit_serial.flush()
        time.sleep(0.05)

        # RSW 输出内容寄存器：0x000E = 加速度(0x51) + 角速度(0x52) + 角度(0x53)。
        # 幻尔官方说明 200Hz 时只建议选择这三类输出，内容过多会自动降频。
        wit_serial.write(
            bytes(
                [
                    0xFF,
                    0xAA,
                    0x02,
                    OUTPUT_ACC_GYRO_ANGLE & 0xFF,
                    (OUTPUT_ACC_GYRO_ANGLE >> 8) & 0xFF,
                ]
            )
        )
        wit_serial.flush()
        time.sleep(0.05)

        # RRATE 回传速率寄存器。
        wit_serial.write(bytes([0xFF, 0xAA, 0x03, rate_value, 0x00]))
        wit_serial.flush()
        time.sleep(0.05)
        wit_serial.reset_input_buffer()

    # 无硬件绝对采样时钟：使用本次读到串口数据的ROS时间，不按假定频率倒推时间戳。
    def read_serial_loop(self, wit_serial):
        stats = SerialStats()
        last_data_time = time.monotonic()

        while rclpy.ok() and not self.stop_event.is_set():
            try:
                waiting = wit_serial.in_waiting
            except Exception as exc:
                self.get_logger().error("IMU disconnected: {}".format(exc))
                rclpy.try_shutdown()
                return

            if waiting:
                data = wit_serial.read(min(waiting, 4096))
                received = self.get_clock().now()
                stats.add_bytes(data)
                last_data_time = time.monotonic()
                imu_ready = angle_ready = False
                for value in data:
                    frame_type = self.parser.push_byte(value)
                    if frame_type in (ACCEL_FRAME, GYRO_FRAME, ANGLE_FRAME):
                        # 新的加速度帧开始一组，缺帧时不能沿用上一组数据。
                        if frame_type == ACCEL_FRAME:
                            self.frame_times.clear()
                        self.frame_times[frame_type] = received
                        stats.valid_packets += 1
                        if frame_type == ANGLE_FRAME:
                            imu_ready = self.update_messages(received) or imu_ready
                            angle_ready = True
                # 无硬件采样时间，使用有效帧接收时刻；一次读到多组时发布最新完整组。
                if imu_ready:
                    self.imu_pub.publish(self.imu_msg)
                    stats.published += 1
                if angle_ready:
                    self.angle_pub.publish(self.angle_msg)
            else:
                self.log_if_waiting_for_data(last_data_time)
                time.sleep(0.001)

            if stats.ready_to_log():
                self.log_serial_stats(stats)
                stats.reset()

    def log_if_waiting_for_data(self, last_data_time):
        if time.monotonic() - last_data_time > 2.0:
            self.get_logger().warn(
                "No serial data received from {}; check baudrate and IMU output mode".format(
                    self.port
                )
            )
            self.stop_event.wait(1.0)

    def log_serial_stats(self, stats):
        if stats.valid_packets > 0:
            # self.get_logger().info(
            #     "Serial RX: {} bytes/s, valid IMU frames: {}, published: {}".format(
            #         stats.rx_bytes, stats.valid_packets, stats.published
            #     )
            # )
            return

        # self.get_logger().info(
        #     "Serial RX: {} bytes/s, valid IMU frames: {}, published: {}, raw: {}".format(
        #         stats.rx_bytes, stats.valid_packets, stats.published, stats.raw_sample_hex()
        #     )
        # )

    def update_messages(self, angle_time):
        accel_time = self.frame_times.pop(ACCEL_FRAME, None)
        gyro_time = self.frame_times.pop(GYRO_FRAME, None)
        angle_radian = [math.radians(value) for value in self.parser.angle_degree]
        qx, qy, qz, qw = quaternion_from_euler(*angle_radian)

        # EKF使用角速度：IMU消息取本组0x52的时间；缺帧时不重发上组测量。
        fresh = (accel_time is not None and gyro_time is not None and
                 0 <= (gyro_time - accel_time).nanoseconds <= 2e9 / self.rate_hz and
                 0 <= (angle_time - gyro_time).nanoseconds <= 2e9 / self.rate_hz)
        if fresh:
            self.imu_msg.header.stamp = gyro_time.to_msg()
            self.imu_msg.linear_acceleration.x = float(self.parser.acceleration[0])
            self.imu_msg.linear_acceleration.y = float(self.parser.acceleration[1])
            self.imu_msg.linear_acceleration.z = float(self.parser.acceleration[2])
            self.imu_msg.angular_velocity.x = float(self.parser.angular_velocity[0])
            self.imu_msg.angular_velocity.y = float(self.parser.angular_velocity[1])
            self.imu_msg.angular_velocity.z = float(self.parser.angular_velocity[2])
            self.imu_msg.orientation.x = float(qx)
            self.imu_msg.orientation.y = float(qy)
            self.imu_msg.orientation.z = float(qz)
            self.imu_msg.orientation.w = float(qw)

        # 只有收到真正的角度帧 0x53 时，才发布 /imu/angle。
        # 这样 PID 节点第一帧拿到的就是 IMU 当前实际角度，不会被默认 0.0 误初始化。
        self.angle_msg.header.stamp = angle_time.to_msg()
        self.angle_msg.vector.x = float(self.parser.angle_degree[0])
        self.angle_msg.vector.y = float(self.parser.angle_degree[1])
        self.angle_msg.vector.z = float(self.parser.angle_degree[2])
        return fresh


class SerialStats:
    def __init__(self):
        self.last_log_time = time.monotonic()
        self.rx_bytes = 0
        self.valid_packets = 0
        self.published = 0
        self.raw_sample = bytearray()

    def add_bytes(self, data):
        self.rx_bytes += len(data)
        if len(self.raw_sample) < 64:
            self.raw_sample.extend(data[:64 - len(self.raw_sample)])

    def ready_to_log(self):
        return time.monotonic() - self.last_log_time > 1.0

    def raw_sample_hex(self):
        return " ".join("{:02X}".format(byte) for byte in self.raw_sample)

    def reset(self):
        self.last_log_time = time.monotonic()
        self.rx_bytes = 0
        self.valid_packets = 0
        self.published = 0
        self.raw_sample = bytearray()


def main():
    rclpy.init()
    node = IMUDriverNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
