# 二维激光建图/定位：ICP里程计+IMU角速度经EKF融合，不使用轮式或RGB-D定位。
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
        raise RuntimeError('Localization requires an existing lidar database: ' + str(database))
    database.parent.mkdir(parents=True, exist_ok=True)

    # 底盘只负责电机、编码器关节状态和传感器；关闭原轮式+IMU EKF。
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('mycar_base') + '/launch/base.launch.py'),
        condition=IfCondition(LaunchConfiguration('start_base')),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name in (
                'serial_port', 'baud_rate', 'max_rpm', 'imu_port', 'imu_baud', 'imu_rate',
                'lidar_port', 'start_lidar', 'use_sim_time')},
            'start_ekf': 'false',
        }.items(),
    )
    # ICP只匹配/scan，不读取轮速或轮式TF作为运动初值，也不发布TF。
    icp = Node(
        package='rtabmap_odom', executable='icp_odometry', name='icp_odometry', namespace='rtabmap',
        output='screen', parameters=[share + '/config/icp_odometry.yaml', {'use_sim_time': sim}],
        remappings=[('scan', '/scan'), ('odom', '/odom_icp'), ('odom_info', '/rtabmap/icp_odom_info')],
    )
    # 只融合ICP速度与IMU角速度，独占/odom和odom->base_link。
    # WIT姿态标记为不可用，不将其欧拉角、姿态或加速度送入融合。
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='rtabmap_ekf', output='screen',
        parameters=[share + '/config/laser_imu_ekf.yaml', {'use_sim_time': sim}],
        remappings=[('odometry/filtered', '/odom')],
    )
    slam = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace='rtabmap',
        output='screen', parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': sim, 'database_path': str(database),
            'Mem/IncrementalMemory': 'false' if localization else 'true',
            'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
        }],
        # 激光栅格由RTAB-Map直接发布，map->odom也只由此节点发布。
        remappings=[('scan', '/scan'), ('odom', '/odom'), ('map', '/map'), ('grid_map', '/map'),
                    ('initialpose', '/initialpose')],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=IfCondition(LaunchConfiguration('start_rviz')), output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[{'use_sim_time': sim}],
    )
    return [base, icp, ekf, slam, rviz]


def generate_launch_description():
    share = get_package_share_directory('mycar_rtabmap')
    defaults = {
        'use_sim_time': 'false', 'localization': 'false',
        # 新方案使用独立数据库，保留原来的RGB-D地图。
        'database': str(Path.home() / 'car_swarm/maps/rtabmap_lidar.db'),
        'params_file': share + '/config/rtabmap.yaml',
        'rviz_config': share + '/rviz/navigation.rviz',
        'start_base': 'true', 'start_rviz': 'true',
        'serial_port': '/dev/ttyCH341USB0', 'baud_rate': '115200', 'max_rpm': '100.0',
        'imu_port': '/dev/ttyTHS1', 'imu_baud': '115200', 'imu_rate': '20',
        'lidar_port': '/dev/ttyCH343USB0', 'start_lidar': 'true',
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [OpaqueFunction(function=start)])
