from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # SV1301S_U3：只输出普通彩色图像，MJPG 由驱动解码为 ROS 图像。
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'camera_name': 'camera',
                'enable_color': True,
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30,
                'color_format': 'MJPG',
                'enable_depth': False,
                'enable_ir': False,
                'enable_left_ir': False,
                'enable_right_ir': False,
                'enable_point_cloud': False,
                'enable_colored_point_cloud': False,
                'enable_accel': False,
                'enable_gyro': False,
                'depth_registration': False,
                'enable_frame_sync': False,
                'enable_d2c_viewer': False,
                # RViz 的 Image 显示不需要相机外参，安装位置确定后再配置 TF。
                'publish_tf': False,
            }],
        ),
    ])
