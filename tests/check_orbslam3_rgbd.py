#!/usr/bin/env python3
"""合成RGB-D离线冒烟测试：运行真实ORB库，不连接相机、串口或电机。"""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Int32
from tf2_ros import StaticTransformBroadcaster


def main():
    root = Path(__file__).resolve().parents[1]
    run = Path(tempfile.mkdtemp(prefix='orbslam3-check-'))
    print('Test artifacts:', run, flush=True)
    executable = Path(get_package_prefix('mycar_orbslam3')) / 'lib/mycar_orbslam3/rgbd_slam'
    vocabulary = root / '.orbslam3/ORB_SLAM3/Vocabulary/ORBvoc.txt'
    assert executable.is_file() and vocabulary.is_file(), 'Build ORB-SLAM3 and source install/setup.bash first'
    rclpy.init()
    node = rclpy.create_node('orbslam3_synthetic_camera')
    rgb_pub = node.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
    depth_pub = node.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
    info_pub = node.create_publisher(CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data)
    poses, states, clouds = [], [], []
    node.create_subscription(PoseStamped, '/orb_slam3/pose', poses.append, 10)
    node.create_subscription(Int32, '/orb_slam3/tracking_state', lambda m: states.append(m.data), 10)
    node.create_subscription(PointCloud2, '/orb_slam3/points', clouds.append, 10)
    broadcaster = StaticTransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id = 'base_link'
    tf.child_frame_id = 'camera_color_optical_frame'
    tf.transform.translation.x, tf.transform.translation.z = 0.10, 0.02
    # 光学x右y下z前 -> 车体x前y左z上。
    tf.transform.rotation.x, tf.transform.rotation.y = -0.5, 0.5
    tf.transform.rotation.z, tf.transform.rotation.w = -0.5, 0.5
    broadcaster.sendTransform(tf)

    width, height, fx, distance = 640, 480, 525.0, 2.0
    k = np.array([[fx, 0, 319.5], [0, fx, 239.5], [0, 0, 1]], dtype=np.float64)
    d = np.array([.02, -.01, .0002, -.0001, 0, 0, 0, 0], dtype=np.float64)
    pixels = np.stack(np.meshgrid(np.arange(width), np.arange(height)), axis=-1).astype(np.float32)
    rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), k, d, P=k).reshape(height, width, 2)
    texture = np.random.default_rng(314).integers(0, 256, (height, width, 3), dtype=np.uint8)
    texture = cv2.GaussianBlur(texture, (3, 3), .5)
    info = CameraInfo(width=width, height=height, distortion_model='rational_polynomial')
    info.k = k.ravel().tolist()
    info.d = d.tolist()
    last_stamp = None
    input_stamps = set()

    def send(x=0.0, invalid=False, old=False):
        nonlocal last_stamp
        stamp = node.get_clock().now().to_msg()
        if old and last_stamp:
            stamp = last_stamp
        else:
            last_stamp = stamp
        input_stamps.add(stamp.sec * 1000000000 + stamp.nanosec)
        image = cv2.remap(texture, rays[..., 0] + fx * x / distance, rays[..., 1],
                          cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT101)
        rgb = Image(height=height, width=width, encoding='bgr8', step=width * 3)
        rgb.header.stamp, rgb.header.frame_id = stamp, tf.child_frame_id
        rgb.data = image.tobytes()
        depth = Image(height=height, width=width, encoding='16UC1', step=width * 2)
        depth.header.stamp = stamp
        depth.header.frame_id = 'unaligned_depth_frame' if invalid else tf.child_frame_id
        depth.data = np.full((height, width), distance * 1000, dtype=np.uint16).tobytes()
        info.header = rgb.header
        info_pub.publish(info)
        depth_pub.publish(depth)
        rgb_pub.publish(rgb)

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)

    def wait_tracking(process, log, x=0.0):
        before = len(poses)
        until = time.monotonic() + 60
        while time.monotonic() < until:
            assert process.poll() is None, f'ORB exited; inspect {log}'
            send(x)
            spin(.15)
            if len(poses) >= before + 4:
                return
        raise AssertionError(f'No tracking; inspect {log}')

    def start(localization):
        log = run / ('localization.log' if localization else 'mapping.log')
        command = [str(executable), '--ros-args', '--params-file',
                   str(root / 'src/mycar_orbslam3/config/orbslam3.yaml'),
                   '-p', f'vocabulary:={vocabulary}', '-p', f'map_dir:={run}',
                   '-p', 'localization:=' + str(localization).lower()]
        handle = log.open('w')
        process = subprocess.Popen(command, cwd=run, stdout=handle, stderr=subprocess.STDOUT)
        handle.close()
        return process, log

    process = None
    try:
        process, log = start(False)
        # 错误注册坐标不应产生有效位姿。
        for _ in range(12):
            send(invalid=True)
            spin(.1)
        assert not poses, 'Unaligned depth was incorrectly accepted'
        wait_tracking(process, log)
        assert 2 in states and poses[-1].header.frame_id == 'orb_map'
        for x in np.linspace(0, .12, 25):
            send(float(x))
            spin(.15)
        wait_tracking(process, log, .12)
        moved = poses[-1].pose.position
        # 相机向光学x正方向移动12cm，对应车体y负方向；同时验证毫米->米。
        assert abs(moved.y + .12) < .04, (moved.x, moved.y, moved.z)
        assert abs(moved.x) < .04 and abs(moved.z) < .04
        assert clouds and clouds[-1].width > 100
        for pose in poses:
            p = pose.pose
            assert all(math.isfinite(v) for v in (p.position.x, p.position.y, p.position.z,
                                                 p.orientation.x, p.orientation.y,
                                                 p.orientation.z, p.orientation.w))
        # 时间戳应来自输入，不能替换为处理完成时间。
        assert all(p.header.stamp.sec * 1000000000 + p.header.stamp.nanosec in input_stamps for p in poses)
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=90) == 0, log
        process = None
        atlas = run / 'atlas.osa'
        assert atlas.is_file() and atlas.stat().st_size > 1000
        checksum = hashlib.sha256(atlas.read_bytes()).hexdigest()
        saved_position = [moved.x, moved.y, moved.z]

        poses.clear()
        states.clear()
        spin(.3)
        process, log = start(True)
        wait_tracking(process, log)
        assert 2 in states
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=90) == 0, log
        process = None
        assert hashlib.sha256(atlas.read_bytes()).hexdigest() == checksum, 'Localization overwrote the Atlas'
        result = {'hardware_used': False, 'scene': 'Synthetic textured plane at 2m, 8-parameter distortion',
                  'camera_motion_optical_x_m': .12, 'ros_base_position_m': saved_position,
                  'poses': len(poses), 'sparse_points': clouds[-1].width,
                  'atlas_roundtrip': True, 'atlas_unchanged_after_localization': True,
                  'output': str(run)}
        (run / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=90)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
