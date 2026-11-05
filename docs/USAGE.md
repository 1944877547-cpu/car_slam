# car_slam 使用与调参

ROS 2 Humble 双轮差速小车，配合 `~/STM32Cube/car_swarm` 的 STM32 + TB6612 固件。
命令沿用 `~/car_swarm`，新克隆目录不同时相应替换路径。

- `mycar_base`：RPM串口控制、真实编码器反馈、轮式里程计、IMU和EKF。
- `mycar_cartographer`：N10激光、融合里程计和IMU建图，可同时打开RGB。
- `mycar_navigation2`：AMCL地图定位和Nav2导航。
- `wit_ros2_imu`、`lslidar_driver`、`orbbec`：传感器驱动。

## 编译与环境

使用新终端，只加载系统Humble与本工作空间，不加载预装的 `wheeltec_ros2`。
PC与NX分别编译，不互相复制不同CPU架构的 `build/install/log`。

```bash
source /opt/ros/humble/setup.bash
cd ~/car_swarm
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

需要ros2_control、robot_localization、Cartographer、Nav2、传感器驱动及pyserial。
串口用户需要在 `dialout` 组，新增权限后注销并重新登录。
每个新终端加载 `/opt/ros/humble/setup.bash` 和 `~/car_swarm/install/local_setup.bash`。

## 启动

下面任选一种，不能重复打开同一串口：

```bash
# 底盘、轮式里程计、雷达、IMU、EKF。
ros2 launch mycar_base base.launch.py

# 建图，同时打开普通RGB画面。
ros2 launch mycar_cartographer mapping.launch.py

# 导航，加载已有地图。
ros2 launch mycar_navigation2 navigation.launch.py map:="$HOME/car_swarm/maps/map.yaml"
```

默认STM32 `/dev/ttyCH341USB0`、IMU `/dev/ttyTHS1`，均为115200/8N1；N10为 `/dev/ttyCH343USB0`。
可添加 `serial_port:=设备路径`、`lidar_port:=设备路径`、`imu_port:=设备路径`。
已有外部 `/scan` 时添加 `start_lidar:=false`。轮式里程计不依赖雷达，建图/导航仍需要有效扫描。

## 里程计与TF

```text
STM32真实累计编码器 → ROS关节位置 → diff_drive_controller → /odom_raw
/odom_raw + /imu/data角速度 → robot_localization EKF → /odom
/scan → Cartographer / AMCL及Nav2障碍物层
```

`open_loop: false`、`position_feedback: true`：关节状态来自真实计数，不能用目标RPM代替。
累计计数换算轮角度；计数差和STM32采样时间差换算实测轮速。
EKF融合轮式前向速度、侧向零速度约束、转向角速度和IMU转向角速度，不重复融合轮式位置。
IMU姿态尚未核实，当前EKF不融合姿态或加速度，不做加速度双积分。

- `odom → base_link`：仅由EKF发布。
- `map → odom`：建图时Cartographer、导航时AMCL发布。
- 车体到雷达、IMU：URDF固定安装关系。

固件必须配套更新。启动时未收到真实编码器反馈，底盘拒绝激活；反馈中断500ms时停车并报错。
反馈重复包不能保活；STM32采样时间重启或异常跳变时停止，需要重启底盘入口重新建立基线。
不会回退到速度指令估算位置。

## 参数

- 轮半径 **0.0325m**、轮距 **0.125m**：`controllers.yaml` 与 `mycar.urdf.xacro` 保持一致。
- 每圈四倍频计数 **1468.2**：Xacro的 `encoder_cpr` 与STM32的 `ENC_CPR` 保持一致。
- 左编码器A/B：PB8/PB9；右编码器A/B：PB6/PB7。手转前进时两轮累计计数都应增加。
- STM32轮速PI保持 `KP=7`、`KI=18`，控制周期10ms，PWM上限1200、周期3599。实车整定，不添加固定起步补偿。
- 导航上限0.25m/s、1.20rad/s，加速度0.50m/s²、2.00rad/s²。达到单轮100RPM上限时，两轮目标同比缩放以保持曲率，弯道实际车体速度可能低于指令上限。
- IMU安装x右、y后、z下；`imu_joint` RPY为 `3.1415926536 0 -1.5707963268`。角速度经TF转换到车体坐标后融合。
- IMU与车体基准同高，位置在 `imu_tracking_joint`；雷达位置在 `laser_joint`，当前相对车体高0.20m。
- IMU默认请求20Hz，以 `ros2 topic hz /imu/data` 实测为准。
- 轮式协方差在 `controllers.yaml`、IMU协方差在驱动中，按实测噪声调整。

## STM32串口协议

下发保持8字节，有符号16位大端数，单位 **0.1 RPM**，最大±100RPM，每20ms发送：

```text
AA 55 左高 左低 右高 右低 0D 0A
AA 55 01 2C FE D4 0D 0A    左+30.0 RPM，右-30.0 RPM
AA 55 00 00 00 00 0D 0A    停车
```

反馈为17字节，每20ms主动发送，无运动指令时也回传：

```text
AA 66 T0 T1 T2 T3 L0 L1 L2 L3 R0 R1 R2 R3 SUM 0D 0A
```

T为无符号32位STM32毫秒计数，L/R为有符号32位累计编码器计数，均为大端。
SUM为前14字节相加的低8位。累计计数和毫秒差分支持32位回绕。
反馈不是目标RPM回显，ROS不接受旧8字节ACK作为轮反馈，必须两端同时更新。
STM32保留零速和500ms命令超时停车；每圈计数设为0时禁止驱动。

## 检查与控制

```bash
ros2 control list_controllers
ros2 control list_hardware_components
ros2 topic echo /joint_states --once
ros2 topic echo /odom_raw --once
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once
ros2 topic hz /odom_raw
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.3 -r cmd_vel:=/cmd_vel
```

轮式里程计会响应手推车体；车轮架空旋转也会累计里程，编码器无法独自分辨打滑。
ROS的 `/cmd_vel` 超时约0.5秒会清零指令，STM32另外独立检查串口命令超时。

## 保存地图与RGB

保持建图运行，另开已加载环境的终端：

```bash
mkdir -p ~/car_swarm/maps
ros2 run nav2_map_server map_saver_cli -t /map -f "$HOME/car_swarm/maps/map" --ros-args -p save_map_timeout:=10.0
```

Orbbec只显示普通RGB，不参与里程计或SLAM；配置640×480、30FPS、MJPG，关闭深度、红外和点云。

```bash
# 不接相机时。
ros2 launch mycar_cartographer mapping.launch.py start_camera:=false
# 单独查看RGB。
ros2 launch mycar_cartographer camera_rgb.launch.py
# 新设备USB权限配置，执行后重新插拔相机。
sudo bash ~/car_swarm/src/orbbec/orbbec_camera/scripts/install_udev_rules.sh
```

## 离线模拟与时间检查

```bash
# 隔离ROS域与虚拟串口，不访问真实电机。
python3 tests/check_wheel_odometry.py
# 底盘、建图或导航运行后，只监听时间戳与TF。
python3 tests/check_timestamps.py --seconds 10
```

实车 `use_sim_time:=false`。轮式里程计按ROS控制周期打时间戳；轮速计算采用反馈内STM32采样时间差。
IMU使用本组有效陀螺仪帧的NX接收时刻。EKF预测到当前ROS时间，`/odom`与TF同一时间戳，不人为添加时间偏移。
STM32毫秒值不能直接当作ROS绝对时间；串口与控制周期仍有小量延迟，这不是硬件同步。
N10仍约10Hz，整圈结束时间戳，`time_increment` 与 `scan_time` 为0，没有扫描运动补偿。
时间检查只能检查扫描参考时刻的TF覆盖，不能证明每束激光采样已同步。AMCL的 `map → odom` 含前置有效期，不能要求所有时间戳数值相同。


## RTAB-Map：激光＋IMU方案

当前RTAB-Map入口只使用N10激光与维特IMU，不启动相机、不融合轮式里程计。
`/scan → ICP → /odom_icp`与IMU角速度经EKF生成`/odom`，再交给RTAB-Map和Nav2。
建图与导航默认显示二维栅格，速度上限0.25m/s，局部及全局膨胀半径0.20m。
原RGB-D数据库保留，新方案请使用`rtabmap_lidar.db`重新建图。

```bash
ros2 launch mycar_rtabmap mapping.launch.py database:="$HOME/car_swarm/maps/rtabmap_lidar.db"
```

建图保存退出后启动导航：

```bash
ros2 launch mycar_rtabmap navigation.launch.py database:="$HOME/car_swarm/maps/rtabmap_lidar.db"
```

先用2D Pose Estimate设置当前位姿，再用Nav2 Goal。完整步骤和参数见[RTAB_MAP.md](../RTAB_MAP.md)。
编码器仍用于电机转速闭环，原Cartographer入口继续保留轮式＋IMU EKF。
