#!/usr/bin/env python3
"""Offline RF2O + EKF integration check. Uses a private ROS domain; no motor nodes."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import sys
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster


CLOCKWISE = '--clockwise' in sys.argv
if CLOCKWISE:
    sys.argv.remove('--clockwise')


class Fixture:
    def __init__(self):
        self.node = rclpy.create_node('laser_odom_test')
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.node.create_publisher(Imu, '/imu/data', 10)
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.raw_pub = self.node.create_publisher(Odometry, '/odom_raw', 10)
        self.laser = []
        self.odom = []
        self.transforms = []
        self.subs = [
            self.node.create_subscription(Odometry, '/odom_lidar', self.laser.append, 10),
            self.node.create_subscription(Odometry, '/odom', self.odom.append, 10),
            self.node.create_subscription(TFMessage, '/tf', self.transforms.append, 100),
        ]
        self.static = StaticTransformBroadcaster(self.node)
        self.last_scan = None
        # Asymmetric room and internal obstacles make translation and rotation observable.
        self.segments = []
        for x0, y0, x1, y1 in ((-4, -3, 5, 4), (1.6, 0.8, 2.2, 1.5), (-1.5, 1.2, -0.8, 1.7)):
            p = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            self.segments.extend(zip(p, p[1:] + p[:1]))
        self.angles = (-np.arange(720) * (2 * math.pi / 720) if CLOCKWISE else
                       np.arange(720) * (2 * math.pi / 720) - math.pi)

    def mount(self):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        # Non-zero lever arm and yaw test that velocities refer to the base, not the lidar.
        t.transform.translation.x = 0.3
        t.transform.translation.y = -0.12
        t.transform.translation.z = 0.2
        t.transform.rotation.z = math.sin(0.25 / 2)
        t.transform.rotation.w = math.cos(0.25 / 2)
        self.static.sendTransform(t)

    def scan(self, pose, valid=True):
        x, y, yaw = pose
        sx = x + math.cos(yaw)*0.3 + math.sin(yaw)*0.12
        sy = y + math.sin(yaw)*0.3 - math.cos(yaw)*0.12
        dx = np.cos(self.angles + yaw + 0.25)
        dy = np.sin(self.angles + yaw + 0.25)
        ranges = np.full(720, np.inf)
        for (ax, ay), (bx, by) in self.segments:
            vx, vy = bx-ax, by-ay
            denominator = dx*vy - dy*vx
            with np.errstate(divide='ignore', invalid='ignore'):
                distance = ((ax-sx)*vy - (ay-sy)*vx) / denominator
                along = ((ax-sx)*dy - (ay-sy)*dx) / denominator
            good = (distance > 0) & (along >= 0) & (along <= 1) & (np.abs(denominator) > 1e-10)
            ranges = np.minimum(ranges, np.where(good, distance, np.inf))
        m = LaserScan()
        now = self.node.get_clock().now()
        if CLOCKWISE:
            now -= Duration(nanoseconds=round(719 * (0.1/720) * 1e9))
        m.header.stamp = now.to_msg()
        m.header.frame_id = 'laser'
        m.angle_min = float(self.angles[0])
        m.angle_max = float(self.angles[-1]) if CLOCKWISE else math.pi
        m.angle_increment = (-1 if CLOCKWISE else 1) * 2*math.pi/720
        m.scan_time = 0.1
        m.time_increment = 0.1/720
        m.range_min = 0.15
        m.range_max = 8.0
        m.ranges = ranges.astype(np.float32).tolist() if valid else [float('inf')]*720
        self.last_scan = m
        self.scan_pub.publish(m)

    def drive(self, seconds, pose, angular=0.0, valid=True):
        start = time.monotonic()
        next_scan = next_imu = 0.0
        while time.monotonic()-start < seconds:
            elapsed = time.monotonic()-start
            if elapsed >= next_scan:
                self.scan(pose(elapsed) if callable(pose) else pose, valid)
                next_scan += 0.1
            if elapsed >= next_imu:
                imu = Imu()
                imu.header.stamp = self.node.get_clock().now().to_msg()
                imu.header.frame_id = 'base_link'
                imu.orientation_covariance[0] = -1.0
                imu.angular_velocity.z = angular
                for axis in (0, 4, 8):
                    imu.angular_velocity_covariance[axis] = 0.0004
                self.imu_pub.publish(imu)
                # Deliberately contradictory command odometry must have no effect on EKF.
                command = Twist()
                command.linear.x = 0.2
                command.angular.z = 0.5
                self.cmd_pub.publish(command)
                raw = Odometry()
                raw.header = imu.header
                raw.header.frame_id = 'odom'
                raw.child_frame_id = 'base_link'
                raw.pose.pose.orientation.w = 1.0
                raw.twist.twist.linear.x = 0.5
                raw.twist.covariance[0] = 0.0001
                self.raw_pub.publish(raw)
                next_imu += 0.05
            rclpy.spin_once(self.node, timeout_sec=0.005)


def xy(message):
    p = message.pose.pose.position
    return np.array([p.x, p.y])


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def main():
    os.environ['ROS_DOMAIN_ID'] = '93' if CLOCKWISE else '94'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    folder = Path(tempfile.mkdtemp(prefix='car_swarm_laser_check_'))
    os.environ['ROS_LOG_DIR'] = str(folder/'ros_log')
    config = Path(get_package_share_directory('mycar_base'))/'config'
    report = {'folder': str(folder), 'passed': False, 'clockwise_scan': CLOCKWISE}
    processes = []
    logs = []
    rclpy.init()
    fixture = Fixture()
    try:
        for name, command in (
            ('rf2o', ['ros2', 'run', 'rf2o_laser_odometry', 'rf2o_laser_odometry_node',
                      '--ros-args', '--params-file', str(config/'laser_odom.yaml')]),
            ('ekf', ['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
                     '-r', '__node:=ekf_filter_node', '-r', 'odometry/filtered:=/odom',
                     '--params-file', str(config/'ekf.yaml')]),
        ):
            log = (folder/(name+'.log')).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
        until = time.monotonic()+8
        while fixture.node.count_subscribers('/scan') == 0 and time.monotonic() < until:
            rclpy.spin_once(fixture.node, timeout_sec=0.1)
        assert fixture.node.count_subscribers('/scan') > 0, 'RF2O did not subscribe'
        fixture.drive(0.6, (0, 0, 0))
        assert not fixture.laser, 'RF2O initialized before TF existed'
        fixture.mount()
        fixture.drive(2.0, (0, 0, 0))
        assert len(fixture.laser) >= 5 and fixture.odom, 'No laser/EKF output'
        assert np.linalg.norm(xy(fixture.odom[-1])) < 0.02, 'Command odometry affected stationary pose'
        assert not any(i.node_name == 'ekf_filter_node' for i in fixture.node.get_subscriptions_info_by_topic('/odom_raw'))
        assert not any(i.node_name == 'rf2o_laser_odometry' for i in fixture.node.get_publishers_info_by_topic('/tf'))
        report['tf_wait_and_command_independence'] = True

        fixture.drive(4.0, lambda t: (0.08*t, 0.02*t, 0.10*t), angular=0.10)
        target = np.array([0.32, 0.08])
        fixture.drive(1.2, (0.32, 0.08, 0.4))
        laser_error = float(np.linalg.norm(xy(fixture.laser[-1])-target))
        ekf_error = float(np.linalg.norm(xy(fixture.odom[-1])-target))
        report['translation'] = {'laser_error_m': laser_error, 'ekf_error_m': ekf_error,
                                  'laser_xy': xy(fixture.laser[-1]).tolist(), 'ekf_xy': xy(fixture.odom[-1]).tolist()}
        assert laser_error < 0.05 and ekf_error < 0.08, report['translation']
        before = xy(fixture.laser[-1])
        fixture.drive(2.0, lambda t: (0.32, 0.08, 0.4+0.2*t), angular=0.2)
        fixture.drive(0.8, (0.32, 0.08, 0.8))
        rotation_drift = float(np.linalg.norm(xy(fixture.laser[-1])-before))
        report['lever_arm_rotation_drift_m'] = rotation_drift
        assert rotation_drift < 0.03, 'Laser offset was interpreted as base translation'

        duplicate = copy.deepcopy(fixture.last_scan)
        duplicate.header.stamp.sec -= 2
        for _ in range(4):
            fixture.scan_pub.publish(duplicate)
            rclpy.spin_once(fixture.node, timeout_sec=0.02)
        fixture.drive(0.3, (0.32, 0.08, 0.8), valid=False)
        fixture.drive(0.6, (0.32, 0.08, 0.8))
        assert processes[0].poll() is None
        for message in fixture.laser:
            assert np.isfinite([*xy(message), message.twist.twist.linear.x,
                                message.twist.twist.linear.y, message.twist.twist.angular.z]).all()
            assert all(message.twist.covariance[i] > 0 for i in (0, 7, 35))
        report['invalid_scan_recovery_and_covariance'] = True
        fixture.drive(1.6, (0.32, 0.08, 0.8), valid=False)
        assert processes[0].poll() is not None, 'RF2O failed to stop on scan loss'
        report['scan_loss_stops_node'] = True
        report['messages'] = {'laser': len(fixture.laser), 'ekf': len(fixture.odom)}
        report['passed'] = True
    finally:
        for process in processes:
            stop(process)
        for log in logs:
            log.close()
        fixture.node.destroy_node()
        rclpy.shutdown()
        (folder/'result.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
