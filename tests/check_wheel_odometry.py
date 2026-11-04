#!/usr/bin/env python3
"""轮式里程计+IMU离线联调：隔离ROS域、虚拟串口，不打开真实电机设备。"""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import select
import signal
import statistics
import struct
import subprocess
import tempfile
import threading
import time
import tty

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_msgs.msg import TFMessage

CPR, RADIUS, SEPARATION = 1468.2, 0.0325, 0.125


def frame(ms, left, right):
    data = struct.pack('>BBIII', 0xAA, 0x66, ms & 0xffffffff, left & 0xffffffff, right & 0xffffffff)
    return data + bytes([sum(data) & 255, 13, 10])


class VirtualFirmware:
    """默认模型用于ROS回归；--firmware-model可接入实际固件C代码的主机模型。"""
    def __init__(self):
        self.ms = self.sent = 0
        self.target = [0, 0]
        self.rpm = [0., 0.]
        self.positions = [0., 0.]
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        while len(self.buffer) >= 8:
            if self.buffer[:2] != b'\xaa\x55' or self.buffer[6:8] != b'\r\n':
                del self.buffer[0]
                continue
            self.target = list(struct.unpack('>hh', self.buffer[2:6]))
            del self.buffer[:8]

    def step(self, gains=(.11, .105)):
        self.ms += 10
        for i in (0, 1):
            desired = self.target[i] * .1 if gains[i] else 0.
            self.rpm[i] += (desired - self.rpm[i]) * (1 - math.exp(-.01/.15))
            self.positions[i] += self.rpm[i] * CPR / 6000
        return self.state()

    def state(self):
        return {'ms': self.ms, 'enc': [round(p) for p in self.positions],
                'rpm_model': self.rpm[:], 'target': self.target[:], 'err': 0}

    def service(self):
        if self.ms - self.sent >= 20:
            self.sent = self.ms
            return frame(self.ms, *self.state()['enc'])
        return b''


class Device:
    def __init__(self, model):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)  # 固件主动反馈前关闭PTY默认回显，避免把反馈当成下发命令。
        self.port = os.ttyname(self.slave)
        os.set_blocking(self.master, False)
        self.model = model
        self.samples, self.commands, self.pending = [], [], []
        self.freeze = self.drop = False
        self.hand = (0, 0)
        self.pose = [0., 0., 0.]
        self.angular = 0.
        self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def loop(self):
        buffer = bytearray()
        previous = self.model.state()['enc']
        next_tick = time.monotonic() + .01
        try:
            while not self.stop.is_set():
                ready, _, _ = select.select([self.master], [], [], .001)
                if ready:
                    buffer.extend(os.read(self.master, 4096))
                    while len(buffer) >= 8:
                        assert buffer[:2] == b'\xaa\x55' and buffer[6:8] == b'\r\n'
                        command = bytes(buffer[:8]); del buffer[:8]
                        self.commands.append((time.monotonic(), *struct.unpack('>hh', command[2:6])))
                        self.model.feed(command[:3]); self.model.feed(command[3:])
                now = time.monotonic()
                while now >= next_tick:
                    sample = self.model.step(gains=(0, 0) if self.freeze else (.11, .105))
                    for i in (0, 1):
                        if self.hand[i]:
                            self.model.positions[i] += self.hand[i]
                            if hasattr(self.model, 'lib'):
                                self.model.model_counts[i] += self.hand[i]
                                self.model.lib.fw_encoder_move(i, self.hand[i])
                    sample = self.model.state()
                    sample['time'] = now
                    self.samples.append(sample)
                    dl, dr = [(sample['enc'][i] - previous[i]) * 2 * math.pi * RADIUS / CPR for i in (0, 1)]
                    previous = sample['enc']
                    angle = (dr-dl) / SEPARATION
                    self.pose[0] += (dl+dr) / 2 * math.cos(self.pose[2] + angle/2)
                    self.pose[1] += (dl+dr) / 2 * math.sin(self.pose[2] + angle/2)
                    self.pose[2] += angle
                    rpm = sample['rpm_model']
                    self.angular = (rpm[1]-rpm[0]) * 2 * math.pi / 60 * RADIUS / SEPARATION
                    next_tick += .01
                reply = self.model.service()
                if reply and not self.drop:
                    assert len(reply) == 17 and sum(reply[:14]) & 255 == reply[14]
                    if len(self.samples) % 26 == 0:
                        os.write(self.master, b'\x12\x99' + reply[:5])
                        self.pending.append((now+.003, reply[5:]))
                    else:
                        os.write(self.master, reply)
                while self.pending and self.pending[0][0] <= now:
                    os.write(self.master, self.pending.pop(0)[1])
        except BaseException as exc:
            self.error = repr(exc)

    def close(self):
        self.stop.set(); self.thread.join(3)
        for fd in (self.master, self.slave): os.close(fd)


def yaw(odom):
    q = odom.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware-model', type=Path, help='可选：实际固件主机模型的Python模块路径')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '144'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    folder = Path(tempfile.mkdtemp(prefix='wheel_imu_check_'))
    os.environ['ROS_LOG_DIR'] = str(folder/'ros_logs')
    model = VirtualFirmware()
    if args.firmware_model:
        spec = importlib.util.spec_from_file_location('firmware_model', args.firmware_model)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        model = module.Firmware()
    device = Device(model)
    report = {'passed': False, 'folder': str(folder), 'actual_firmware_code': bool(args.firmware_model)}
    rclpy.init(); node = rclpy.create_node('wheel_imu_test')
    imu_pub = node.create_publisher(Imu, '/imu/data', 20)
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    raw, odom, joints, transforms = [], [], [], []
    subscriptions = [node.create_subscription(Odometry, '/odom_raw', raw.append, 100),
                     node.create_subscription(Odometry, '/odom', odom.append, 100),
                     node.create_subscription(JointState, '/joint_states', joints.append, 100),
                     node.create_subscription(TFMessage, '/tf', transforms.append, 100)]
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    config = Path(get_package_share_directory('mycar_base'))
    description = subprocess.check_output(['xacro', str(config/'urdf/mycar.urdf.xacro'),
                                           'serial_port:='+device.port], text=True)
    params = folder/'robot.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': description,
                                                               'use_sim_time': False}}}))
    processes, logs = [], []
    last_imu = last_cmd = 0.
    imu_override = None

    def spin(seconds, v=None, w=0.):
        nonlocal last_imu, last_cmd
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            now = time.monotonic()
            assert device.error is None, device.error
            assert all(p.poll() is None for p in processes), 'ROS process exited; see '+str(folder)
            if now-last_imu >= .05:
                m = Imu(); m.header.stamp = node.get_clock().now().to_msg(); m.header.frame_id = 'imu_link'
                m.orientation_covariance[0] = -1.; m.linear_acceleration.z = -9.81
                m.angular_velocity.z = -(device.angular if imu_override is None else imu_override)
                for i in (0, 4, 8): m.angular_velocity_covariance[i] = .0004
                imu_pub.publish(m); last_imu = now
            if v is not None and now-last_cmd >= .04:
                m = Twist(); m.linear.x = float(v); m.angular.z = float(w)
                cmd_pub.publish(m); last_cmd = now
            rclpy.spin_once(node, timeout_sec=.005)

    try:
        for name, command in [
            ('controller', ['/opt/ros/humble/lib/controller_manager/ros2_control_node', '--ros-args',
                            '--params-file', str(config/'config/controllers.yaml'), '--params-file', str(params),
                            '-r', '/diff_drive_controller/cmd_vel_unstamped:=/cmd_vel',
                            '-r', '/diff_drive_controller/odom:=/odom_raw']),
            ('state', ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args', '--params-file', str(params)]),
            ('ekf', ['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args', '-r', '__node:=ekf_filter_node',
                     '-r', 'odometry/filtered:=/odom', '--params-file', str(config/'config/ekf.yaml')])]:
            log = (folder/(name+'.log')).open('w'); logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 20
        while not client.wait_for_service(timeout_sec=.2):
            assert device.error is None, device.error
            assert processes[0].poll() is None, 'controller startup failed; see '+str(folder)
            assert time.monotonic() < deadline, 'controller service missing'
        for name in ('joint_state_broadcaster', 'diff_drive_controller'):
            result = subprocess.run(['/opt/ros/humble/lib/controller_manager/spawner', name],
                                    capture_output=True, text=True, timeout=20)
            assert result.returncode == 0, result.stdout+result.stderr
        spin(2., 0.)
        assert raw and odom and joints and transforms, 'Missing wheel/EKF output'
        assert abs(raw[-1].pose.pose.position.x) < .001
        assert any(i.node_name == 'ekf_filter_node' for i in node.get_subscriptions_info_by_topic('/odom_raw'))
        assert not any('rf2o' in n.lower() for n in node.get_node_names())
        assert not node.get_publishers_info_by_topic('/odom_lidar')
        report['startup_without_scan_and_real_wheel_input'] = True
        spin(6., .05)
        assert abs(raw[-1].twist.twist.linear.x-.05) < .01
        assert abs(odom[-1].twist.twist.linear.x-.05) < .015
        error = math.hypot(odom[-1].pose.pose.position.x-device.pose[0], odom[-1].pose.pose.position.y-device.pose[1])
        assert error < .06, error
        report['forward_ekf_position_error_m'] = error
        spin(4., -.05)
        assert raw[-1].twist.twist.linear.x < -.03 and odom[-1].twist.twist.linear.x < -.025
        before = yaw(odom[-1]); spin(4., 0., .3)
        # 验证估计器跟踪实际运动；低速换向时电机模型未必已到目标转速。
        expected = device.angular
        assert expected > .05, expected
        assert abs(raw[-1].twist.twist.angular.z-expected) < .08, (raw[-1].twist.twist.angular.z, expected)
        assert abs(odom[-1].twist.twist.angular.z-expected) < .08, (odom[-1].twist.twist.angular.z, expected)
        assert yaw(odom[-1])-before > .3
        report['reverse_turn_and_imu_mount_transform'] = True
        report['turn_actual_and_estimated_rad_s'] = [expected, raw[-1].twist.twist.angular.z, odom[-1].twist.twist.angular.z]
        spin(1., 1., 5.)
        assert all(c[1:] == (367, 808) for c in device.commands[-10:]), device.commands[-10:]
        spin(.4, .05); stopped_after = last_cmd; spin(.85)
        zeros = [c for c in device.commands if c[0] > stopped_after and c[1:] == (0, 0)]
        assert zeros and zeros[0][0]-stopped_after < .7
        report['cmd_vel_timeout_s'] = zeros[0][0]-stopped_after
        device.freeze = True; spin(2., .05)
        p = odom[-1].pose.pose.position
        before = (p.x, p.y); spin(1.5, .05)
        assert abs(raw[-1].twist.twist.linear.x) < .002
        p = odom[-1].pose.pose.position
        assert math.hypot(p.x-before[0], p.y-before[1]) < .006
        report['nonzero_command_stationary_encoder_does_not_move_odometry'] = True
        spin(.3, 0.); device.hand = (2, 2)
        p = odom[-1].pose.pose.position
        before = (p.x, p.y); spin(2., 0.); device.hand = (0, 0)
        p = odom[-1].pose.pose.position
        assert math.hypot(p.x-before[0], p.y-before[1]) > .02
        report['manual_encoder_motion_with_zero_command'] = True
        spin(.7, 0.); before = yaw(odom[-1]); raw_before = yaw(raw[-1])
        imu_override = .25; spin(1.6, 0.); imu_override = None
        assert yaw(odom[-1])-before > .25 and abs(yaw(raw[-1])-raw_before) < .02
        report['imu_only_yaw_changes_ekf_not_wheel_odometry'] = True
        spin(.4, 0.)
        tf_ns = {t.header.stamp.sec*10**9+t.header.stamp.nanosec for msg in transforms
                 for t in msg.transforms if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link'}
        middle = odom[5:-5]
        ratio = sum(m.header.stamp.sec*10**9+m.header.stamp.nanosec in tf_ns for m in middle)/len(middle)
        assert ratio > .95, ratio
        report['odom_tf_identical_stamp_ratio'] = ratio
        tf_publishers = [p.node_name for p in node.get_publishers_info_by_topic('/tf')]
        assert 'ekf_filter_node' in tf_publishers and 'controller_manager' not in tf_publishers
        spin(.4, .05); device.drop = True; loss = time.monotonic(); spin(.85, .05)
        zeros = [c for c in device.commands if c[0] > loss and c[1:] == (0, 0)]
        assert zeros and zeros[0][0]-loss < .7, 'No stop command after encoder feedback loss'
        report['encoder_loss_stop_s'] = zeros[0][0]-loss
        hardware = node.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        assert hardware.wait_for_service(timeout_sec=2)
        request = hardware.call_async(ListHardwareComponents.Request())
        rclpy.spin_until_future_complete(node, request, timeout_sec=2)
        states = {c.name: c.state.label for c in request.result().component}
        assert states['MyCarSystem'] == 'finalized', states
        assert all(c[1:] == (0, 0) for c in device.commands if c[0] >= zeros[0][0])
        report['encoder_loss_stops_hardware'] = states
        assert all(abs(p) <= 1200 for s in device.samples for p in s.get('pwm', []))
        report['serial_commands'] = len(device.commands)
        report['samples'] = len(device.samples)
        report['passed'] = True
    except BaseException as exc:
        report['error'] = repr(exc)
        raise
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=3)
        for log in logs: log.close()
        device.close(); del subscriptions
        node.destroy_node(); rclpy.shutdown()
        (folder/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
