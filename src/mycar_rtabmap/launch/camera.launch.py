# RGB-D相机独立入口；安装外参camera_joint由公共底盘URDF提供。
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('mycar_rtabmap')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('camera_params', default_value=share + '/config/camera.yaml'),
        # 相机节点位于/camera命名空间；同时发布RGB、对齐深度、CameraInfo及内部TF。
        Node(
            package='orbbec_camera', executable='orbbec_camera_node',
            name='camera', namespace='camera', output='screen',
            parameters=[LaunchConfiguration('camera_params'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
