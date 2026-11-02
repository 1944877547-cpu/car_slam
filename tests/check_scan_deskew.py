#!/usr/bin/env python3
"""Synthetic clockwise scans and 20 Hz upside-down IMU; no motors or USB hardware."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import StaticTransformBroadcaster


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


class Fixture:
    def __init__(self):
        self.node = rclpy.create_node('deskew_check')
        self.raw = self.node.create_publisher(LaserScan, '/scan_raw', 10)
        self.imu = self.node.create_publisher(Imu, '/imu/data', 10)
        self.odom = self.node.create_publisher(Odometry, '/odom', 10)
        self.output = []
        self.sub = self.node.create_subscription(LaserScan, '/scan', self.output.append, 10)
        self.tf = StaticTransformBroadcaster(self.node)
        self.t0 = self.node.get_clock().now().nanoseconds * 1e-9 - 1.0
        self.mount_xy = np.array([0.22, -0.08])
        self.mount_yaw = 0.2
        self.segments = []
        for x0, y0, x1, y1 in [(-4., -3., 5., 4.), (1.6, 0.8, 2.2, 1.5), (-1.5, 1.2, -0.8, 1.7)]:
            points = [(x0,y0), (x1,y0), (x1,y1), (x0,y1)]
            self.segments.extend(zip(points, points[1:] + points[:1]))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def mounts(self):
        a = TransformStamped()
        a.header.frame_id = 'base_link'
        a.child_frame_id = 'laser'
        a.header.stamp = self.node.get_clock().now().to_msg()
        a.transform.translation.x, a.transform.translation.y = map(float, self.mount_xy)
        a.transform.translation.z = 0.2
        a.transform.rotation.z = math.sin(self.mount_yaw/2)
        a.transform.rotation.w = math.cos(self.mount_yaw/2)
        b = TransformStamped()
        b.header = a.header
        b.child_frame_id = 'imu_link'
        # Actual project's upside-down IMU: roll=pi, yaw=-pi/2.
        b.transform.rotation.x = math.sqrt(0.5)
        b.transform.rotation.y = -math.sqrt(0.5)
        b.transform.rotation.w = 0.0
        self.tf.sendTransform([a, b])
        self.spin(0.25)

    def gyros(self, start, omega, offsets=(-0.05, 0, 0.05, 0.10, 0.15)):
        for t in offsets:
            m = Imu()
            m.header.stamp = stamp(start+t)
            m.header.frame_id = 'imu_link'
            m.orientation_covariance[0] = -1.0
            m.angular_velocity.z = -float(omega(t) if callable(omega) else omega)
            m.angular_velocity_covariance[0] = 0.0004
            self.imu.publish(m)
        self.spin(0.04)

    def velocity(self, t, vx, vy=0.0):
        m = Odometry()
        m.header.stamp = stamp(t)
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.twist.twist.linear.x = float(vx)
        m.twist.twist.linear.y = float(vy)
        self.odom.publish(m)
        self.spin(0.03)

    def scan(self, start, pose, n=450):
        m = LaserScan()
        m.header.frame_id = 'laser'
        m.header.stamp = stamp(start)
        m.angle_min = 0.0
        m.angle_increment = -2*math.pi/n
        m.angle_max = (n-1)*m.angle_increment
        m.time_increment = 0.1/n
        m.scan_time = 0.1
        m.range_min = 0.15
        m.range_max = 8.0
        values = []
        for i in range(n):
            x, y, yaw = pose(i*m.time_increment)
            c, s = math.cos(yaw), math.sin(yaw)
            sx, sy = np.array([x,y]) + np.array([[c,-s],[s,c]]) @ self.mount_xy
            a = i*m.angle_increment + yaw + self.mount_yaw
            dx, dy = math.cos(a), math.sin(a)
            best = float('inf')
            for (ax,ay),(bx,by) in self.segments:
                vx, vy = bx-ax, by-ay
                den = dx*vy - dy*vx
                if abs(den) < 1e-9:
                    continue
                distance = ((ax-sx)*vy - (ay-sy)*vx) / den
                along = ((ax-sx)*dy - (ay-sy)*dx) / den
                if distance > 0 and 0 <= along <= 1:
                    best = min(best, distance)
            values.append(float(best))
        m.ranges = values
        m.intensities = [10.0]*n
        return m

    def error(self, scan, pose):
        angles = scan.angle_min + np.arange(len(scan.ranges))*scan.angle_increment
        ranges = np.array(scan.ranges)
        good = np.isfinite(ranges)
        p = np.column_stack((ranges[good]*np.cos(angles[good]), ranges[good]*np.sin(angles[good])))
        def rotation(a):
            return np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        p = (p @ rotation(self.mount_yaw).T + self.mount_xy) @ rotation(pose[2]).T + pose[:2]
        distances = []
        for (ax,ay),(bx,by) in self.segments:
            a = np.array([ax,ay]);v=np.array([bx-ax,by-ay])
            u = np.clip(((p-a)@v)/(v@v),0,1)
            distances.append(np.linalg.norm(p-(a+u[:,None]*v),axis=1))
        return float(np.mean(np.min(distances,axis=0)))

    def receive(self, raw):
        before = len(self.output)
        self.raw.publish(raw)
        self.spin(0.15)
        assert len(self.output) == before+1, 'No deskew output, see node log'
        out = self.output[-1]
        expected = raw.header.stamp.sec + raw.header.stamp.nanosec*1e-9 + (len(raw.ranges)-1)*raw.time_increment
        actual = out.header.stamp.sec + out.header.stamp.nanosec*1e-9
        assert abs(expected-actual) < 1e-6
        assert out.time_increment == 0 and out.angle_increment > 0
        assert out.header.frame_id == raw.header.frame_id
        assert sum(np.isfinite(out.ranges)) > 350
        return out


def main():
    os.environ['ROS_DOMAIN_ID'] = '96'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    folder = Path(tempfile.mkdtemp(prefix='car_swarm_deskew_check_'))
    os.environ['ROS_LOG_DIR'] = str(folder/'ros_log')
    config = Path(get_package_share_directory('mycar_base'))/'config/deskew.yaml'
    report = {'passed':False,'folder':str(folder)}
    root = Path(__file__).resolve().parents[1]
    headers = ['sensor_msgs','std_msgs','builtin_interfaces','rosidl_runtime_cpp',
               'rosidl_runtime_c','rosidl_typesupport_interface']
    includes = ['-I'+str(Path(get_package_prefix(p))/'include'/p) for p in headers]
    binary = str(folder/'check_n10_timing')
    subprocess.run(['g++','-std=c++17','-O2',str(root/'tests/check_n10_timing.cpp'),
                    '-I'+str(root/'src/lslidar_driver/include'),*includes,'-o',binary],check=True)
    subprocess.run([binary],check=True)
    report['n10_timing_and_direction'] = True
    log = (folder/'deskew.log').open('w')
    process = subprocess.Popen(['ros2','run','mycar_base','scan_deskew','--ros-args','--params-file',str(config)],
                               stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init()
    f = Fixture()
    try:
        until = time.monotonic()+8
        while f.node.count_subscribers('/scan_raw') == 0 and time.monotonic() < until:
            f.spin(0.1)
        assert f.node.count_subscribers('/scan_raw') > 0
        f.mounts()
        f.gyros(f.t0,0)
        f.velocity(f.t0-0.01,0)
        raw = f.scan(f.t0,lambda t:(0.,0.,0.))
        out = f.receive(raw)
        report['stationary_error_m'] = f.error(out,(0.,0.,0.))
        assert report['stationary_error_m'] < 0.025

        cases = []
        for index,(vx,omega) in enumerate([(0.,1.2),(0.,-1.2),(0.18,1.2)]):
            t0=f.t0+index+1
            f.gyros(t0,omega)
            f.velocity(t0-0.02,vx)
            f.velocity(t0+0.01,-0.8)  # Future odometry must not be used for this scan.
            def pose(t):
                return (vx/omega*math.sin(omega*t),vx/omega*(1-math.cos(omega*t)),omega*t)
            raw=f.scan(t0,pose)
            out=f.receive(raw)
            end=(len(raw.ranges)-1)*raw.time_increment
            old=f.error(raw,pose(end));new=f.error(out,pose(end))
            cases.append({'vx':vx,'omega':omega,'raw_error_m':old,'deskew_error_m':new})
            assert new < 0.03 and new < old*0.30,cases[-1]
        report['rotation_translation_lever_arm_and_imu_frame']=cases

        t0=f.t0+4
        f.gyros(t0,lambda t:0.3+5*t)
        f.velocity(t0-0.01,0)
        pose=lambda t:(0.,0.,0.3*t+2.5*t*t)
        raw=f.scan(t0,pose)
        out=f.receive(raw)
        report['accelerating_rotation_error_m']=f.error(out,pose(449*raw.time_increment))
        assert report['accelerating_rotation_error_m'] < 0.03

        # No scan is released before the IMU covers its entire acquisition interval.
        t0=f.t0+5
        f.gyros(t0,0.8,offsets=(-0.05,0,0.05))
        raw=f.scan(t0,lambda t:(0.,0.,0.8*t))
        before=len(f.output)
        f.raw.publish(raw)
        f.spin(0.08)
        assert len(f.output)==before
        f.gyros(t0,0.8,offsets=(0.10,0.15))
        f.spin(0.12)
        assert len(f.output)==before+1
        report['delayed_imu_wait']=True

        f.raw.publish(raw)
        older=copy.deepcopy(raw);older.header.stamp=stamp(t0-0.5)
        f.raw.publish(older)
        invalid=copy.deepcopy(raw);invalid.header.stamp=stamp(t0+0.2);invalid.time_increment=0.0
        f.raw.publish(invalid)
        f.spin(0.1)
        assert len(f.output)==before+1
        report['duplicate_out_of_order_and_missing_timing_rejected']=True

        t0=f.t0+6
        f.gyros(t0,0,offsets=(-0.02,0.12))  # 140 ms gap exceeds the configured 120 ms.
        before=len(f.output)
        f.raw.publish(f.scan(t0,lambda t:(0.,0.,0.)))
        f.spin(0.35)
        assert len(f.output)==before
        report['imu_gap_rejected']=True

        t0=f.t0+7
        f.raw.publish(f.scan(t0,lambda t:(0.,0.,0.)))
        f.spin(0.35)
        assert len(f.output)==before
        f.gyros(f.t0+8,0)
        f.receive(f.scan(f.t0+8,lambda t:(0.,0.,0.)))
        report['imu_timeout_drops_scan_and_recovers']=True
        assert process.poll() is None
        assert f.node.count_publishers('/scan')==1
        report['passed']=True
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        log.close()
        f.node.destroy_node()
        rclpy.shutdown()
        (folder/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
