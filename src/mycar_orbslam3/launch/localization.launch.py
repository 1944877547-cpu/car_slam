# 从同一相机保存的Atlas重定位；不再增量建图。
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('mycar_orbslam3') + '/launch/mapping.launch.py'),
        launch_arguments={'localization': 'true'}.items(),
    )])
