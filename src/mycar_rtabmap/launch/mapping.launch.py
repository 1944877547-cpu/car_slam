from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    share = get_package_share_directory('mycar_rtabmap')
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    database = Path(LaunchConfiguration('database').perform(context)).expanduser().resolve()
    if localization and not database.is_file():
        raise RuntimeError('Localization requires an existing RTAB-Map database: ' + str(database))
    database.parent.mkdir(parents=True, exist_ok=True)

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('mycar_base') + '/launch/base.launch.py'),
        condition=IfCondition(LaunchConfiguration('start_base')),
        launch_arguments={name: LaunchConfiguration(name) for name in (
            'serial_port', 'baud_rate', 'max_rpm', 'imu_port', 'imu_baud', 'imu_rate',
            'lidar_port', 'start_lidar', 'use_sim_time')}.items(),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share + '/launch/camera.launch.py'),
        condition=IfCondition(LaunchConfiguration('start_camera')),
        launch_arguments={name: LaunchConfiguration(name) for name in ('use_sim_time', 'camera_params')}.items(),
    )
    sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync', namespace='rtabmap',
        output='screen', parameters=[{
            'use_sim_time': sim, 'approx_sync': True, 'approx_sync_max_interval': 0.02,
            'topic_queue_size': 10, 'sync_queue_size': 20, 'qos': 2, 'qos_camera_info': 2,
        }],
        remappings=[('rgb/image', '/camera/color/image_raw'),
                    ('depth/image', '/camera/depth/image_raw'),
                    ('rgb/camera_info', '/camera/color/camera_info')],
    )
    slam = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace='rtabmap',
        output='screen', parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': sim, 'database_path': str(database),
            'Mem/IncrementalMemory': 'false' if localization else 'true',
            'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
        }],
        # 不同版本使用map或grid_map作为二维栅格名，均统一到Nav2的/map。
        remappings=[('scan', '/scan'), ('map', '/map'), ('grid_map', '/map'),
                    ('initialpose', '/initialpose')],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=IfCondition(LaunchConfiguration('start_rviz')), output='screen',
        arguments=['-d', share + '/rviz/rgbd.rviz'], parameters=[{'use_sim_time': sim}],
    )
    return [base, camera, sync, slam, rviz]


def generate_launch_description():
    share = get_package_share_directory('mycar_rtabmap')
    defaults = {
        'use_sim_time': 'false', 'localization': 'false',
        'database': str(Path.home() / 'car_swarm/maps/rtabmap.db'),
        'params_file': share + '/config/rtabmap.yaml',
        'camera_params': share + '/config/camera.yaml',
        'start_base': 'true', 'start_camera': 'true', 'start_rviz': 'true',
        'serial_port': '/dev/ttyCH341USB0', 'baud_rate': '115200', 'max_rpm': '100.0',
        'imu_port': '/dev/ttyTHS1', 'imu_baud': '115200', 'imu_rate': '20',
        'lidar_port': '/dev/ttyCH343USB0', 'start_lidar': 'true',
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [OpaqueFunction(function=start)])
