# 先加载RTAB-Map数据库定位，再用同一套Nav2规划控制；地图话题统一为/map。
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('mycar_rtabmap')
    defaults = {
        'database': str(Path.home() / 'car_swarm/maps/rtabmap.db'),
        'use_sim_time': 'false', 'start_base': 'true', 'start_camera': 'true', 'start_rviz': 'true',
        'serial_port': '/dev/ttyCH341USB0', 'baud_rate': '115200', 'max_rpm': '100.0',
        'imu_port': '/dev/ttyTHS1', 'imu_baud': '115200', 'imu_rate': '20',
        'lidar_port': '/dev/ttyCH343USB0', 'start_lidar': 'true',
        'camera_params': share + '/config/camera.yaml',
        'params_file': share + '/config/rtabmap.yaml',
        'nav_params': get_package_share_directory('mycar_navigation2') + '/param/nav2_params.yaml',
    }
    # 只启导航节点：地图与map->odom由RTAB-Map提供，不启动AMCL/map_server。
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share + '/launch/mapping.launch.py'),
            launch_arguments={
                **{name: LaunchConfiguration(name) for name in defaults if name != 'nav_params'},
                'localization': 'true',
            }.items(),
        # navigation_launch只带导航服务器；bringup_launch还会带AMCL，不能在这里替换使用。
        ), IncludeLaunchDescription(
            PythonLaunchDescriptionSource(get_package_share_directory('nav2_bringup') + '/launch/navigation_launch.py'),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('nav_params'),
                'autostart': 'true', 'use_composition': 'False', 'use_respawn': 'False',
            }.items(),
        )])
