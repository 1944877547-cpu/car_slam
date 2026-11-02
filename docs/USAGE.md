# car_slam 使用与调参

下文沿用已有工作空间目录 `~/car_swarm`。新克隆目录为 `~/car_slam` 时，请相应替换命令中的工作空间路径。

ROS 2 Humble 双轮差速小车上位机，配合 `~/STM32Cube/car_swarm` 的 STM32 + TB6612 固件。

- `mycar_base`：差速控制、HEX PWM 串口、雷达、激光里程计、IMU 和 EKF。
- `rf2o_laser_odometry`：RF2O 扫描匹配，输入 `/scan`，输出 `/odom_lidar`，保留上游源码和许可证。
- `mycar_cartographer`：N10 雷达 + IMU 建图。
- `mycar_navigation2`：Nav2 导航。
- `wit_ros2_imu`：从 `imu_test` 当前使用的维特驱动复制而来。

**不需要轮编码器。平移来自激光扫描匹配，转向角速度融合激光与 IMU，EKF 输出 `/odom`。** 运动指令只负责控制电机，`/odom_raw` 保留为指令估算对照，不再进入 EKF。激光速度与位置来自同一次匹配，EKF 只融合速度，避免重复使用同一信息。

IMU 不做加速度双积分位置；当前模块姿态尚未验证，EKF 不融合它的姿态。`/imu/data` 姿态协方差首项保持 `-1`，`/imu/angle` 仍提供原始角度供核对。

## 编译

在新终端执行，避免混入 `my_roscar` 等同名包的旧环境：

```bash
source /opt/ros/humble/setup.bash
cd ~/car_swarm
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

依赖 ROS 2 Humble、ros2_control、robot_localization、Cartographer、Nav2 和 Python 的 pyserial。新设备需要先安装相应依赖。

## 新设备依赖

PC 与 NX 分别在本机编译，不复制另一种 CPU 架构的 `build/install/log`。RF2O 源码已包含在工程中。新增依赖为 Eigen3 和 ROS 的 Eigen CMake 模块：

```bash
sudo apt install -y libeigen3-dev ros-humble-eigen3-cmake-module
```

其他依赖包含 ros2_control、robot_localization、Cartographer、Nav2、雷达、相机驱动及 pyserial。当前工作空间已具备依赖时，直接执行上面的编译命令。
串口用户需要在 `dialout` 组；新增组权限后需注销并重新登录。

## 启动

每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/car_swarm/install/local_setup.bash
```

请使用新终端。当前只加载系统 ROS Humble 和本工程，不加载预装的 `wheeltec_ros2`。
直接加载环境会保留其他已配置的 Python 和动态库路径，无需单独的启动脚本。
`ros2 pkg prefix nav2_map_server` 应输出 `/opt/ros/humble`。

当前恢复最初的 1/1 指令上限：直行 0.20 m/s、转向 1.20 rad/s；导航线加速度 0.50 m/s²、角加速度 2.00 rad/s²。实际速度需要实车标定。

本机 NX 的默认端口：STM32 为 `/dev/ttyCH341USB0`，IMU 为 `/dev/ttyTHS1`，两者均为 115200、8N1；N10 雷达为 `/dev/ttyCH343USB0`。建图和导航可用 `lidar_port:=设备路径` 单独覆盖雷达端口。

下面三种启动方式选择一种，均包含底盘、雷达、RF2O、IMU 和 EKF，勿重复启动或重复打开串口：

```bash
# 底盘与传感器；需要雷达数据才能得到平移里程计。
ros2 launch mycar_base base.launch.py

# 建图；串口参数与上面相同。
ros2 launch mycar_cartographer mapping.launch.py

# 导航；使用已保存的地图。
ros2 launch mycar_navigation2 navigation.launch.py map:="$HOME/car_swarm/maps/map.yaml"
```

建图后保存地图：

```bash
mkdir -p ~/car_swarm/maps
ros2 run nav2_map_server map_saver_cli -t /map -f "$HOME/car_swarm/maps/map" --ros-args -p save_map_timeout:=10.0
```

## 检查和控制

```bash
ros2 topic echo /imu/data --once
ros2 topic echo /odom_lidar --once
ros2 topic echo /odom --once
ros2 control list_controllers

# 低速前进；Ctrl+C停止发送后，控制器约0.5秒后清零指令。
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

`/scan_raw → scan_deskew → /scan → RF2O → /odom_lidar` 提供激光位姿和速度；EKF 融合激光的平面速度、角速度与 IMU 的 Z 轴角速度，输出 `/odom`。`/odom_raw` 仅为指令估算对照，不是当前导航里程计的输入。只有 EKF 发布 `odom → base_link`，建图或 AMCL 发布 `map → odom`。

雷达驱动集中在底盘入口启动。外部驱动提供 `/scan_raw` 时添加 `start_lidar:=false`；外部节点已提供校正后的 `/scan` 时，同时添加 `start_lidar:=false start_deskew:=false`。初次启动允许等待扫描和 `base_link → laser` 静态 TF 最多 30 秒；正常运行后连续 1 秒没有有效激光里程计，RF2O 退出并关闭当前底盘启动组。没有命令估算回退。扫描匹配在走廊、空旷或大量动态障碍物场景中可能退化，需要实车确认。

## 首次需要调整

- 轮半径初值 **0.0325 m**、轮距初值 **0.125 m**：同时修改 `src/mycar_base/config/controllers.yaml` 和 `src/mycar_base/urdf/mycar.urdf.xacro`。
- `max_rpm` 默认 **100**，表示满 PWM 时轮速的初始标定值，需实测校准；启动时可加 `max_rpm:=实际值`。左右差异可在 Xacro 中调整 `left_pwm_scale`、`right_pwm_scale`。
- IMU 当前安装为 x 向车体右侧、y 向车尾、z 向下，Xacro 的 `imu_joint` 已设置 `rpy="3.1415926536 0 -1.5707963268"`。从上方看车体逆时针转动时，原始 IMU 的 z 角速度为负，经 TF 转换后的 `/odom` z 角速度为正。与车体同向的 `imu_tracking` 与传感器原点重合，供 Cartographer 使用；安装位置需在 `imu_tracking_joint` 的 `xyz` 中按实车测量。
- IMU 默认请求 20 Hz，与当前实测一致；用 `ros2 topic hz /imu/data` 验证实际频率。当前设备在请求 100 Hz 后仍输出 20 Hz，且未回复寄存器查询；需要进一步核对 NX TX 到 IMU RX 的接线以及模块型号和配置协议，再提高 `imu_rate`。波特率为 115200。
- 激光安装位置、朝向在 Xacro 的 `laser_joint` 中测量设置；当前保留 `xyz="0 0 0.2"`，表示相对 `base_link` 高 20cm。
- `src/mycar_base/config/laser_odom.yaml` 中的 `linear_variance`、`angular_variance` 是方差初值，按实际匹配噪声调整。
- 激光里程计不能代替电机转速闭环；PWM 与实际车速的换算仍需标定。

## STM32 通信

发送和回包都是8字节二进制，左右PWM为有符号16位大端数，范围 **−3599～+3599**：

```text
AA 55 左高 左低 右高 右低 0D 0A
AA 55 03 E8 FC 18 0D 0A    左+1000，右-1000
AA 55 00 00 00 00 0D 0A    停车
```

上位机以50 Hz发送。STM32超过500 ms未收到有效包会停车，回包只确认PWM指令，不是实测轮速。左右顺序与当前STM32固件一致，方向修正在STM32的 `L_REV`、`R_REV` 中完成。

## 建图时显示 RGB 相机

Orbbec 驱动从本机 `~/orbbec_ws/src/OrbbecSDK_ROS2-main` 引入到 `src/orbbec`，
保留驱动 1.5.15 和 SDK 1.10.35 的源码、许可证及 ARM64/x64 库，需在目标机器编译。
当前相机识别为 `SV1301S_U3`，默认彩色流为 **640×480、30 FPS、MJPG**。

首次接入后编译：

```bash
source /opt/ros/humble/setup.bash
cd ~/car_swarm
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --executor sequential --packages-select orbbec_camera_msgs orbbec_camera mycar_cartographer --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
ros2 launch mycar_cartographer mapping.launch.py
```

建图启动后，RViz 自动增加 `RGB` 图像窗口，订阅 `/camera/color/image_raw`。
相机只用于观看彩色画面；深度、红外、点云和相机内置 IMU 均关闭，不参与建图算法。
关闭建图启动终端会同时关闭相机。需要临时停用相机时：

```bash
ros2 launch mycar_cartographer mapping.launch.py start_camera:=false
```

单独检查相机（先关闭正在使用相机的建图程序）：

```bash
ros2 launch mycar_cartographer camera_rgb.launch.py
```

另一个已加载工作空间的终端可运行 `ros2 topic hz /camera/color/image_raw` 检查帧率。
当前 NX 已有 Orbbec USB 规则，无需修改 GPIO 权限；其他机器若提示 USB 权限不足，执行一次：

```bash
sudo bash ~/car_swarm/src/orbbec/orbbec_camera/scripts/install_udev_rules.sh
```

执行后重新插拔相机。RGB 图像显示无需相机外参；以后将深度或视觉加入算法时，再按实际安装位置配置相机 TF。

## PC 离线验证激光里程计

先完成编译、加载系统 ROS 与 `install/local_setup.bash`，再执行：

```bash
python3 tests/check_laser_odometry.py
```

测试使用独立 ROS 域和合成扫描，不启动电机或真实串口，覆盖平移、转动安装偏移、TF 延迟、错误输入和断流退出。

## N10 时间修正与二维去畸变

默认启动流程为：N10 `/scan_raw` → `scan_deskew` → `/scan`。
RF2O、AMCL、Cartographer 和代价地图共用校正后的 `/scan`；原始点云仅作诊断。

- `/scan_raw` 按采集顺序排列，N10 的 `angle_increment` 为负，`time_increment` 为正。
  时间戳对应首束角度，扫描周期来自相邻整圈边界；初始不完整圈不发布。
- N10 当前协议没有硬件时钟，主机接收时刻和 4.5kHz 采样率用于估计时间。
  `lslidar_n10_uart.yaml` 的 `timestamp_offset` 单位为秒，需实测传输延迟后再校准，默认0。
- 陀螺仪通过 URDF 安装姿态变换到车体坐标，插值积分补偿每束点的转向，包含雷达安装偏移。
  平移使用扫描开始之前的新鲜 `/odom` 速度做恒速预测；无可用速度时只做转动补偿。
- `/scan` 已统一到最后一束的参考时刻，因此其 `time_increment=0` 是正确的，防止下游再次去畸变。
  `scan_time` 仍保留真实扫描周期，角度重新按逆时针排列。
- 参数位于 `src/mycar_base/config/deskew.yaml`。IMU需覆盖整圈；缺数据/缺TF超时则丢弃该帧，
  不发布未校正的扫描。持续缺扫描仍由既有 RF2O 超时机制退出底盘。

正常建图、导航命令不变。如果外部节点已经发布校正后的 `/scan`，启动时同时添加
`start_lidar:=false start_deskew:=false`；如果外部只提供 `/scan_raw`，仅关闭 `start_lidar`。

离线检查（不启动电机和真实串口）：

```bash
source /opt/ros/humble/setup.bash
source ~/car_swarm/install/local_setup.bash
cd ~/car_swarm
python3 tests/check_scan_deskew.py
```

该测试验证采集方向、时间边界、20Hz倒置IMU、顺逆转、加速转向、安装偏移、历史速度选择及缺帧恢复。
二维补偿依赖安装外参和时间估计；快速颠簸、陀螺仪零偏、USB缓存延迟仍需实车采集标定。
