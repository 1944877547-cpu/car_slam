# 独立RGB-D算法测试，只启动相机、固定TF和视觉SLAM，不驱动底盘。
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    share = get_package_share_directory('mycar_orbslam3')
    base_share = get_package_share_directory('mycar_base')
    executable = Path(get_package_prefix('mycar_orbslam3')) / 'lib/mycar_orbslam3/rgbd_slam'
    if not executable.is_file():
        raise RuntimeError('ORB-SLAM3 is not built. Follow orb.md, then rebuild mycar_orbslam3.')
    directory = Path(LaunchConfiguration('map_dir').perform(context)).expanduser().resolve()
    name = LaunchConfiguration('atlas_name').perform(context)
    if not name or name in ('.', '..') or '/' in name or '\\' in name:
        raise RuntimeError('atlas_name must be a file name without path or .osa extension')
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    atlas = directory / (name + '.osa')
    if localization and not atlas.is_file():
        raise RuntimeError('Localization needs an existing Atlas: ' + str(atlas))
    if not localization and atlas.exists():
        raise RuntimeError('Atlas already exists. Use localization.launch.py or choose a new atlas_name: ' + str(atlas))
    directory.mkdir(parents=True, exist_ok=True)
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    camera = Node(
        package='orbbec_camera', executable='orbbec_camera_node', name='camera', namespace='camera',
        output='screen', condition=IfCondition(LaunchConfiguration('start_camera')),
        parameters=[LaunchConfiguration('camera_params'), {'use_sim_time': sim}],
    )
    # 只读取公共URDF中的固定安装外参，不启动串口、控制器或电机。
    robot = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        condition=IfCondition(LaunchConfiguration('start_robot_tf')),
        parameters=[{'use_sim_time': sim, 'robot_description': ParameterValue(
            Command(['xacro ', base_share + '/urdf/mycar.urdf.xacro']), value_type=str)}],
    )
    slam = Node(
        package='mycar_orbslam3', executable='rgbd_slam', name='orb_slam3', output='screen',
        cwd=str(directory), sigterm_timeout='120',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': sim, 'vocabulary': LaunchConfiguration('vocabulary'),
            'map_dir': str(directory), 'atlas_name': name, 'localization': localization,
            'viewer': ParameterValue(LaunchConfiguration('viewer'), value_type=bool),
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        arguments=['-d', share + '/rviz/orbslam3.rviz'], parameters=[{'use_sim_time': sim}],
    )
    return [robot, camera, slam, rviz]


def generate_launch_description():
    share = Path(get_package_share_directory('mycar_orbslam3'))
    paths = yaml.safe_load((share / 'config/dependencies.yaml').read_text())
    defaults = {
        'use_sim_time': 'false', 'start_camera': 'true', 'start_robot_tf': 'true',
        'start_rviz': 'true', 'viewer': 'false', 'localization': 'false',
        'map_dir': str(Path.home() / 'car_swarm/maps/orb_slam3'), 'atlas_name': 'atlas',
        'vocabulary': paths['vocabulary'], 'params_file': str(share / 'config/orbslam3.yaml'),
        'camera_params': str(share / 'config/camera.yaml'),
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [OpaqueFunction(function=start)])
