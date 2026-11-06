# 激光建图入口；这里启动的RGB仅用于显示，不参与Cartographer匹配或回环。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_share = get_package_share_directory('mycar_base')
    cartographer_share = get_package_share_directory('mycar_cartographer')

    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')
    start_lidar = LaunchConfiguration('start_lidar')
    baud_rate = LaunchConfiguration('baud_rate')
    max_rpm = LaunchConfiguration('max_rpm')
    imu_port = LaunchConfiguration('imu_port')
    imu_baud = LaunchConfiguration('imu_baud')
    imu_rate = LaunchConfiguration('imu_rate')
    start_rviz = LaunchConfiguration('start_rviz')
    start_camera = LaunchConfiguration('start_camera')

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'base.launch.py')),
        launch_arguments={
            'serial_port': serial_port,
            'lidar_port': lidar_port,
            'start_lidar': start_lidar,
            'baud_rate': baud_rate,
            'max_rpm': max_rpm,
            'imu_port': imu_port,
            'imu_baud': imu_baud,
            'imu_rate': imu_rate,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # /scan、EKF的/odom与原始IMU进入Cartographer，地图修正不回写EKF。
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            os.path.join(cartographer_share, 'config'),
            '-configuration_basename', 'mycar_2d.lua',
        ],
        remappings=[('scan', '/scan'), ('odom', '/odom'), ('imu', '/imu/data')],
    )

    # 将子图生成二维/map；建图与后续AMCL定位分阶段运行。
    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
    )

    # 建图时打开普通 RGB，相机单独运行，退出不会关闭建图。
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_share, 'launch', 'camera_rgb.launch.py')),
        condition=IfCondition(start_camera),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(start_rviz),
        arguments=['-d', os.path.join(cartographer_share, 'rviz', 'mycar_cartographer.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyCH341USB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyCH343USB0'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('max_rpm', default_value='120.0'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('imu_baud', default_value='115200'),
        DeclareLaunchArgument('imu_rate', default_value='20'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='Start Orbbec RGB camera'),
        base,
        cartographer,
        occupancy_grid,
        camera,
        rviz,
    ])
