import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    node_output = LaunchConfiguration('output')
    log_level = LaunchConfiguration('log_level')
    lidar_port = LaunchConfiguration('lidar_port')
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    driver_config = os.path.join(get_package_share_directory('lslidar_driver'),'config','lslidar_n10_uart.yaml')
    rviz_config = os.path.join(get_package_share_directory('lslidar_driver'),'rviz','lslidar_x10.rviz')

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    driver_node = ""
    rviz_node = ""
    ros_version = p.communicate()[0]

    if ros_version == b'dashing\n' or ros_version == b'eloquent\n':
        driver_node = LifecycleNode(package='lslidar_driver',
                                    node_executable='lslidar_driver_node',
                                    node_name='lslidar_driver_node',
                                    node_namespace='x10', # 与对应yaml文件中命名空间一致
                                    output=node_output,
                                    parameters=[driver_config, {'serial_port': lidar_port, 'use_sim_time': sim}],
                                    arguments=['--ros-args', '--log-level', log_level],
                                    )
        rviz_node = Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            node_namespace='cx',
            arguments=['-d', rviz_config],
            output='screen')
    else:
        driver_node = LifecycleNode(package='lslidar_driver',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    namespace='x10', # 与对应yaml文件中命名空间一致
                                    parameters=[driver_config, {'serial_port': lidar_port, 'use_sim_time': sim}],
                                    output=node_output,
                                    arguments=['--ros-args', '--log-level', log_level],
                                    )
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )

    return LaunchDescription([
        DeclareLaunchArgument('output', default_value='screen'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyCH343USB0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        driver_node
    ])
