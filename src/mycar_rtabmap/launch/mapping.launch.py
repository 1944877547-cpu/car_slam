# RTAB-Map统一建图/定位入口；此模式不启动Cartographer或AMCL。
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
    # .db保存图像、深度、激光和位姿图；定位模式必须使用已存在的数据库。
    database = Path(LaunchConfiguration('database').perform(context)).expanduser().resolve()
    if localization and not database.is_file():
        raise RuntimeError('Localization requires an existing RTAB-Map database: ' + str(database))
    database.parent.mkdir(parents=True, exist_ok=True)

    # 复用底盘与EKF；start_base=false供离线测试或已有底盘节点时使用。
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
    # 第一级近似同步：RGB、已对齐的深度、彩色内参，最大时间差20ms。
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
    # 第二级同步：RGBDImage与/scan；odom_frame_id让原生节点查询EKF历史TF。
    slam = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace='rtabmap',
        output='screen', parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': sim, 'database_path': str(database),
            # true为建图；false为数据库定位，不向地图增加新的建图节点。
            'Mem/IncrementalMemory': 'false' if localization else 'true',
            'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
        }],
        # 深度投影供检查；Nav2的/map由下面的激光栅格输出，避免投影中的地面噪声堵路。
        remappings=[('scan', '/scan'), ('map', '/rtabmap/projected_map'),
                    ('grid_map', '/rtabmap/projected_map'),
                    ('initialpose', '/initialpose')],
    )
    # 使用同一份优化位姿图和数据库扫描生成导航栅格，不增加第二套定位或TF。
    # 独立命名空间也隔离其cloud_map等输出，保留上面RTAB-Map的RGB-D三维点云。
    navigation_map = Node(
        package='rtabmap_util', executable='map_assembler', name='navigation_map',
        namespace='rtabmap/navigation_grid', output='screen',
        parameters=[share + '/config/navigation_map.yaml', {'use_sim_time': sim}],
        remappings=[('mapData', '/rtabmap/mapData'), ('map', '/map'), ('grid_map', '/map')],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=IfCondition(LaunchConfiguration('start_rviz')), output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[{'use_sim_time': sim}],
    )
    return [base, camera, sync, slam, navigation_map, rviz]


def generate_launch_description():
    share = get_package_share_directory('mycar_rtabmap')
    defaults = {
        'use_sim_time': 'false', 'localization': 'false',
        'database': str(Path.home() / 'car_swarm/maps/rtabmap.db'),
        'params_file': share + '/config/rtabmap.yaml',
        'camera_params': share + '/config/camera.yaml',
        # 建图默认看三维；导航入口会传入专用的二维配置。
        'rviz_config': share + '/rviz/rgbd.rviz',
        'start_base': 'true', 'start_camera': 'true', 'start_rviz': 'true',
        'serial_port': '/dev/ttyCH341USB0', 'baud_rate': '115200', 'max_rpm': '100.0',
        'imu_port': '/dev/ttyTHS1', 'imu_baud': '115200', 'imu_rate': '20',
        'lidar_port': '/dev/ttyCH343USB0', 'start_lidar': 'true',
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [OpaqueFunction(function=start)])
