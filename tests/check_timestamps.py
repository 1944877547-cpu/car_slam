#!/usr/bin/env python3
"""只监听数据和查询参数：检查时间域、倒序、RF2O时间和TF覆盖，不控制电机。"""
import argparse
import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.0)
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = rclpy.create_node('timestamp_check')
    samples = {t: [] for t in ('/scan', '/imu/data', '/odom_lidar', '/odom')}
    tf_stamps = set()
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def receive(topic, message):
        samples[topic].append((message, node.get_clock().now().nanoseconds))

    def transforms(message):
        for tf in message.transforms:
            if tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_link':
                tf_stamps.add(ns(tf.header.stamp))

    subs = [node.create_subscription(kind, topic, lambda m, t=topic: receive(t, m),
                                     qos_profile_sensor_data)
            for topic, kind in (('/scan', LaserScan), ('/imu/data', Imu),
                                ('/odom_lidar', Odometry), ('/odom', Odometry))]
    subs.append(node.create_subscription(TFMessage, '/tf', transforms, 100))
    deadline = time.monotonic() + max(3.0, args.seconds)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    errors = []
    report = {'topics': {}, 'use_sim_time': {}}
    present = {namespace.rstrip('/') + '/' + name for name, namespace in node.get_node_names_and_namespaces()}
    for topic, rows in samples.items():
        if len(rows) < 3:
            errors.append(topic + ': fewer than 3 messages')
            continue
        stamps = [ns(m.header.stamp) for m, _ in rows]
        ages = [(received - stamp) / 1e6 for (_, received), stamp in zip(rows, stamps)]
        backwards = sum(b <= a for a, b in zip(stamps, stamps[1:]))
        report['topics'][topic] = {'messages': len(rows), 'non_increasing': backwards,
                                  'age_ms_min': round(min(ages), 3),
                                  'age_ms_max': round(max(ages), 3)}
        if min(stamps) <= 0 or backwards or min(ages) < -20:
            errors.append(topic + ': invalid, future or non-increasing timestamps')
        limit = 200 if topic == '/odom' else 300 if topic == '/imu/data' else 500
        if max(ages) > limit:
            errors.append(topic + ': excessive message age')

    # RF2O的顺时针数组在内部反转，参考时间是原扫描最后一束；不能改成发布时刻。
    scans = samples['/scan']
    laser = samples['/odom_lidar']
    if scans and laser:
        references = [ns(s.header.stamp) +
                      (round((len(s.ranges)-1)*s.time_increment*1e9)
                       if s.angle_increment < 0 else 0) for s, _ in scans]
        compared = [m for m, _ in laser if ns(m.header.stamp) >= min(references)]
        good = sum(min(abs(ns(m.header.stamp)-t) for t in references) <= 2000 for m in compared)
        report['rf2o_scan_stamp_matches'] = [good, len(compared)]
        if not compared or good / len(compared) < 0.95:
            errors.append('RF2O stamp does not match its source scan')

    odom = samples['/odom']
    if odom:
        # 去掉刚订阅/刚结束时的边界消息，避免DDS发现和末尾未回调造成误报。
        middle = odom[2:-2]
        good = sum(ns(m.header.stamp) in tf_stamps for m, _ in middle)
        report['odom_tf_identical_stamps'] = [good, len(middle)]
        if not middle or good / len(middle) < 0.95:
            errors.append('/odom and odom->base_link TF stamps differ or TF is missing')

    if len(scans) < 10:
        errors.append('Fewer than 10 scans: cannot check TF history coverage')
    else:
        targets = ['odom']
        if {'/cartographer_node', '/amcl'} & present:
            targets.append('map')
        for target in targets:
            checked = good = 0
            failure = ''
            for s, _ in scans[-min(20, len(scans)-5):-2]:
                first = ns(s.header.stamp)
                last = first + round((len(s.ranges)-1)*s.time_increment*1e9)
                checked += 1
                try:
                    for stamp in (first, last):
                        buffer.lookup_transform(target, s.header.frame_id, Time(nanoseconds=stamp))
                    good += 1
                except Exception as exc:
                    failure = str(exc)
            report[target + '_tf_covers_scan_first_and_last'] = [good, checked]
            if not checked or good != checked:
                errors.append(target + ' TF cannot cover scan first/last ray: ' + failure)

    required = {'/x10/lslidar_driver_node', '/imu_driver_node',
                '/rf2o_laser_odometry', '/ekf_filter_node'}
    optional = {'/cartographer_node', '/amcl', '/controller_server', '/planner_server',
                '/robot_state_publisher', '/controller_manager', '/rviz2'}
    for name in sorted(required | (optional & present)):
        client = node.create_client(GetParameters, name + '/get_parameters')
        if not client.wait_for_service(timeout_sec=0.5):
            errors.append(name + ': parameter service unavailable')
            continue
        request = GetParameters.Request()
        request.names = ['use_sim_time']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        if future.done() and future.result() and future.result().values[0].type == 1:
            sim = future.result().values[0].bool_value
            report['use_sim_time'][name] = sim
            if sim != node.get_parameter('use_sim_time').value:
                errors.append(name + ': clock domain differs from checker')
        else:
            errors.append(name + ': use_sim_time query timed out')

    report['errors'] = errors
    report['passed'] = not errors
    print(json.dumps(report, ensure_ascii=False, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
