#!/usr/bin/env python3
"""使用实际RTAB-Map和合成传感器验证RGB-D+激光链路，不打开硬件或发布cmd_vel。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import struct
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2
from rtabmap_msgs.msg import Info
from rtabmap_msgs.srv import GetMap
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class Inputs(Node):
    def __init__(self):
        super().__init__('synthetic_rgbd_lidar')
        self.rgb = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.camera = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.static = StaticTransformBroadcaster(self)
        transforms = []
        for parent, child, xyz, q in [
            ('base_link', 'camera_link', (.05, 0, .02), (0, 0, 0, 1)),
            ('camera_link', 'camera_color_optical_frame', (0, 0, 0), (-.5, .5, -.5, .5)),
            ('base_link', 'laser', (0, 0, .2), (0, 0, 0, 1)),
        ]:
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = parent, child
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, xyz)
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
            transforms.append(tf)
        self.static.sendTransform(transforms)
        self.odom_tf = TransformBroadcaster(self)
        self.clouds, self.maps, self.infos, self.map_tf = [], [], [], []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, '/rtabmap/cloud_map', self.clouds.append, qos)
        self.create_subscription(OccupancyGrid, '/map', self.maps.append, qos)
        self.create_subscription(Info, '/rtabmap/info', self.infos.append, 10)
        self.create_subscription(TFMessage, '/tf', self.tf, 100)
        self.client = self.create_client(GetMap, '/rtabmap/rtabmap/get_map_data')
        rng = np.random.default_rng(23)
        self.texture = rng.integers(0, 256, (512, 1024, 3), dtype=np.uint8)
        self.texture = cv2.resize(self.texture, (2048, 1024))
        self.uu, self.vv = np.meshgrid(np.arange(320), np.arange(240))
        self.last_x = .2

    def tf(self, msg):
        self.map_tf.extend(t for t in msg.transforms if t.header.frame_id == 'map' and t.child_frame_id == 'odom')

    def publish(self, x=None, with_scan=True, with_depth=True):
        x = self.last_x if x is None else float(x)
        self.last_x = x
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_link'
        tf.transform.translation.x, tf.transform.rotation.w = float(x), 1.
        self.odom_tf.sendTransform(tf)
        # 渲染固定世界x=2m平面的纹理和深度；相机朝前，光学z对应车体x。
        distance = 2. - .05 - x
        mx = (1024 + (self.uu - 160) * distance * 400 / 260).astype(np.float32)
        my = (512 + (self.vv - 120) * distance * 400 / 260).astype(np.float32)
        color = cv2.remap(self.texture, mx, my, cv2.INTER_LINEAR)
        for pub, data, encoding, step in (
            (self.rgb, color.tobytes(), 'rgb8', 960),
            (self.depth, np.full((240, 320), round(distance*1000), dtype='<u2').tobytes(), '16UC1', 640),
        ):
            if pub == self.depth and not with_depth:
                continue
            msg = Image()
            msg.header.stamp, msg.header.frame_id = stamp, 'camera_color_optical_frame'
            msg.width, msg.height, msg.encoding, msg.step, msg.data = 320, 240, encoding, step, data
            pub.publish(msg)
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, 'camera_color_optical_frame'
        info.width, info.height, info.distortion_model, info.d = 320, 240, 'plumb_bob', [0.]*5
        info.k = [260., 0., 160., 0., 260., 120., 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [260., 0., 160., 0., 0., 260., 120., 0., 0., 0., 1., 0.]
        self.camera.publish(info)
        if with_scan:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'laser'
            scan.angle_min, scan.angle_max = -math.pi, math.pi
            scan.angle_increment = 2*math.pi/360
            scan.range_min, scan.range_max = .05, 12.
            ranges = []
            for a in np.linspace(-math.pi, math.pi, 361):
                c, s = math.cos(a), math.sin(a)
                dx = ((2-x) if c > 0 else (2+x))/max(abs(c), 1e-8)
                dy = 1.5/max(abs(s), 1e-8)
                ranges.append(float(min(dx, dy)))
            scan.ranges = ranges
            self.scan.publish(scan)

    def run(self, seconds, move=False, **kwargs):
        start = time.monotonic()
        tick = start
        while time.monotonic() - start < seconds:
            if time.monotonic() >= tick:
                x = .2 + .18 * (1-math.cos(2*math.pi*(time.monotonic()-start)/12)) / 2 if move else self.last_x
                self.publish(x, **kwargs)
                tick += .1
            rclpy.spin_once(self, timeout_sec=.01)

    def get_data(self):
        assert self.client.wait_for_service(timeout_sec=5), 'get_map_data not available'
        request = GetMap.Request()
        request.global_map = request.optimized = True
        request.graph_only = False
        future = self.client.call_async(request)
        until = time.monotonic()+10
        while not future.done() and time.monotonic() < until:
            self.publish()
            rclpy.spin_once(self, timeout_sec=.1)
        assert future.done(), 'get_map_data timed out'
        return future.result().data


def launch(folder, localization=False):
    log = open(folder/('localization.log' if localization else 'mapping.log'), 'w')
    cmd = ['ros2', 'launch', 'mycar_rtabmap', 'mapping.launch.py',
           'start_base:=false', 'start_camera:=false', 'start_rviz:=false',
           'database:='+str(folder/'test.db'), 'localization:='+str(localization).lower()]
    process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return process, log


def stop(process, log):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
    log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=True)
    assert not (folder/'test.db').exists(), 'Use a fresh output directory'
    os.environ.update(ROS_DOMAIN_ID='145', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(folder/'ros_logs'))
    rclpy.init()
    node = Inputs()
    result = {'hardware_used': False}
    process, log = launch(folder)
    try:
        node.run(16, move=True)
        assert process.poll() is None, 'mapping process exited'
        data = node.get_data()
        result['nodes'] = len(data.nodes)
        result['mapping_graph_ids'] = sorted(data.graph.poses_id)
        result['nodes_with_rgb_depth_scan'] = sum(bool(n.data.left_compressed and n.data.right_compressed and n.data.laser_scan_compressed) for n in data.nodes)
        assert result['nodes_with_rgb_depth_scan'] > 0, 'Images, depth and scan were not stored together'
        assert node.maps and node.maps[-1].info.width > 0, 'No occupancy /map'
        assert node.clouds and node.clouds[-1].width > 0, 'No 3D cloud_map'
        cloud = node.clouds[-1]
        offset = next(f.offset for f in cloud.fields if f.name == 'z')
        z = [struct.unpack_from('<f', cloud.data, i+offset)[0] for i in range(0, len(cloud.data), cloud.point_step)]
        result['cloud_z_span'] = float(max(z)-min(z))
        result['map_size'] = [node.maps[-1].info.width, node.maps[-1].info.height]
        assert result['cloud_z_span'] > .3, 'Cloud collapsed to a 2D plane'
        assert node.map_tf, 'No map->odom TF'
        # 同步节点必须等待两个传感器；不允许静默退回单传感器建图。
        node.run(2, with_scan=False)
        before = len(node.infos)
        node.run(2, with_scan=False)
        assert len(node.infos) == before, 'SLAM kept processing without scans'
        node.run(2)
        node.run(2, with_depth=False)
        before = len(node.infos)
        node.run(2, with_depth=False)
        assert len(node.infos) == before, 'SLAM kept processing without depth'
        result['missing_sensor_blocks_fusion'] = True
    finally:
        stop(process, log)
    db = sqlite3.connect(folder/'test.db')
    result['database_nodes'] = db.execute('select count(*) from Node').fetchone()[0]
    assert result['database_nodes'] > 0, 'No database saved'
    db.close()
    node.infos.clear(); node.map_tf.clear(); node.maps.clear()
    process, log = launch(folder, True)
    try:
        node.run(13, move=True)
        assert process.poll() is None, 'localization process exited'
        assert node.infos and node.map_tf, 'Localization did not consume sensors or publish map->odom'
        result['localization_frames'] = len(node.infos)
        result['localization_matches'] = [m.loop_closure_id for m in node.infos if m.loop_closure_id > 0]
        result['localization_proximity_matches'] = [m.proximity_detection_id for m in node.infos if m.proximity_detection_id > 0]
        data = node.get_data()
        result['localization_graph_ids'] = sorted(data.graph.poses_id)
        result['localization_database_loaded'] = True
        result['localization_added_nodes'] = False
    finally:
        stop(process, log)
        node.destroy_node()
        rclpy.shutdown()
    with sqlite3.connect(folder/'test.db') as db:
        assert db.execute('select count(*) from Node').fetchone()[0] == result['database_nodes'], 'Localization added database nodes'
    result['passed'] = True
    (folder/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
