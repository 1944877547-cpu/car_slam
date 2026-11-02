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
    nav_share = get_package_share_directory('mycar_navigation2')
    nav2_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')
    start_lidar = LaunchConfiguration('start_lidar')
    start_deskew = LaunchConfiguration('start_deskew')
    baud_rate = LaunchConfiguration('baud_rate')
    max_rpm = LaunchConfiguration('max_rpm')
    max_pwm = LaunchConfiguration('max_pwm')
    imu_port = LaunchConfiguration('imu_port')
    imu_baud = LaunchConfiguration('imu_baud')
    imu_rate = LaunchConfiguration('imu_rate')
    start_rviz = LaunchConfiguration('start_rviz')

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_share, 'launch', 'base.launch.py')),
        launch_arguments={
            'serial_port': serial_port,
            'lidar_port': lidar_port,
            'start_lidar': start_lidar,
            'start_deskew': start_deskew,
            'baud_rate': baud_rate,
            'max_rpm': max_rpm,
            'max_pwm': max_pwm,
            'imu_port': imu_port,
            'imu_baud': imu_baud,
            'imu_rate': imu_rate,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'slam': 'False',
            'autostart': 'true',
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(start_rviz),
        arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(nav_share, 'maps', 'map.yaml')),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(nav_share, 'param', 'nav2_params.yaml')),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyCH341USB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyCH343USB0'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_deskew', default_value='true'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('max_rpm', default_value='100.0'),
        DeclareLaunchArgument('max_pwm', default_value='3599'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('imu_baud', default_value='115200'),
        DeclareLaunchArgument('imu_rate', default_value='20'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        base,
        nav2,
        rviz,
    ])
