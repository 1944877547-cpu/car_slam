-- Cartographer二维局部子图+全局位姿图，输入激光、里程计和IMU。
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- tracking原点与IMU同位置；IMU实际轴向由URDF的imu_link外参转换。
  tracking_frame = "imu_tracking",
  published_frame = "odom",
  odom_frame = "odom",
  -- 保留EKF的odom->base_link；published_frame=odom使本节点提供map->odom。
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  -- 本方案只订阅一份LaserScan，不使用RGB-D点云。
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
-- 局部SLAM范围过滤，单位m；与雷达驱动的原始有效范围是不同层。
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 在线相关性匹配提供候选初值，再由Cartographer原生Ceres扫描匹配细化。
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 运动过滤决定是否把新扫描插入子图，并非速度限幅或电机起步补偿。
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- 位姿图后端的扫描-子图匹配评分阈值，不是概率百分比。
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70

return options
