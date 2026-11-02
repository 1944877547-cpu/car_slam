# car_slam 使用与调参

下文沿用已有工作空间目录 `~/car_swarm`。新克隆目录为 `~/car_slam` 时，请相应替换命令中的工作空间路径。

ROS 2 Humble 双轮差速小车上位机，配合 `~/STM32Cube/car_swarm` 的 STM32 + TB6612 固件。

- `mycar_base`：差速控制、HEX RPM 串口、雷达、激光里程计、IMU 和 EKF。
- `rf2o_laser_odometry`：RF2O 扫描匹配，输入 `/scan`，输出 `/odom_lidar`，保留上游源码和许可证。
- `mycar_cartographer`：N10 雷达 + IMU 建图。
- `mycar_navigation2`：Nav2 导航。
- `wit_ros2_imu`：从 `imu_test` 当前使用的维特驱动复制而来。

**ROS定位不使用轮编码器：平移来自激光扫描匹配，转向角速度融合激光与 IMU，EKF 输出 `/odom`。** 编码器仅用于STM32轮速PI闭环，`/odom_raw` 保留为指令估算对照，不进入 EKF。激光速度与位置来自同一次匹配，EKF 只融合速度，避免重复使用同一信息。

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

当前导航指令上限：直行 0.20 m/s、转向 1.20 rad/s；导航线加速度 0.50 m/s²、角加速度 2.00 rad/s²。实际速度需要实车标定。

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

`/scan → RF2O → /odom_lidar` 提供激光位姿和速度；EKF 融合激光的平面速度、角速度与 IMU 的 Z 轴角速度，输出 `/odom`。`/odom_raw` 仅为指令估算对照，不是当前导航里程计的输入。只有 EKF 发布 `odom → base_link`，建图或 AMCL 发布 `map → odom`。

雷达驱动集中在底盘入口启动。外部驱动已经提供 `/scan` 时添加 `start_lidar:=false`。初次启动允许等待扫描和 `base_link → laser` 静态 TF 最多 30 秒；正常运行后连续 1 秒没有有效激光里程计，RF2O 退出并关闭当前底盘启动组。没有命令估算回退。扫描匹配在走廊、空旷或大量动态障碍物场景中可能退化，需要实车确认。

## 首次需要调整

- 轮半径初值 **0.0325 m**、轮距初值 **0.125 m**：同时修改 `src/mycar_base/config/controllers.yaml` 和 `src/mycar_base/urdf/mycar.urdf.xacro`。
- `max_rpm` 默认 **100**，是左右轮目标RPM上限；不再在ROS中把RPM映射成PWM。编码器每圈计数、方向和PI参数在STM32内标定。上位机不接收实测轮速或编码器计数。
- IMU 当前安装为 x 向车体右侧、y 向车尾、z 向下，Xacro 的 `imu_joint` 已设置 `rpy="3.1415926536 0 -1.5707963268"`。从上方看车体逆时针转动时，原始 IMU 的 z 角速度为负，经 TF 转换后的 `/odom` z 角速度为正。与车体同向的 `imu_tracking` 与传感器原点重合，供 Cartographer 使用；安装位置需在 `imu_tracking_joint` 的 `xyz` 中按实车测量。
- IMU 默认请求 20 Hz，与当前实测一致；用 `ros2 topic hz /imu/data` 验证实际频率。当前设备在请求 100 Hz 后仍输出 20 Hz，且未回复寄存器查询；需要进一步核对 NX TX 到 IMU RX 的接线以及模块型号和配置协议，再提高 `imu_rate`。波特率为 115200。
- 激光安装位置、朝向在 Xacro 的 `laser_joint` 中测量设置；当前保留 `xyz="0 0 0.2"`，表示相对 `base_link` 高 20cm。
- `src/mycar_base/config/laser_odom.yaml` 中的 `linear_variance`、`angular_variance` 是方差初值，按实际匹配噪声调整。
- STM32用编码器做轮速闭环；定位仍来自RF2O和IMU，`/odom_raw`保持指令估算对照，不能用于融合。

## STM32 通信

发送和ACK都是8字节HEX，左右字段为有符号16位大端数，**单位0.1 RPM**，有效目标范围为−100～+100 RPM：

```text
AA 55 左高 左低 右高 右低 0D 0A
AA 55 01 2C FE D4 0D 0A    左+30.0 RPM，右-30.0 RPM
AA 55 00 00 00 00 0D 0A    停车
```

上位机以50 Hz下发目标RPM。STM32用PB6/PB7（左A/B）、PB8/PB9（右A/B）中断计数，TIM2以100 Hz做双轮PI控制，TIM3仍在PA6/PA7输出PWM，周期3599、输出上限1200。0和500ms通信超时停车。ACK仅回显接受的目标RPM，不是实测速度；与旧PWM字段不兼容，必须配套更新两端。`ENC_CPR`默认0，接线并实测车轮每圈四倍频计数后填写，未标定时电机保持关闭。引脚信号按3.3V TTL处理，具体供电以编码器规格为准。

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

## N10 扫描输出

N10 的组帧、角度转换和时间处理与 `ros_car_diff` 原驱动一致，整圈点云转换为 `/scan`。
角度按 `-π` 到 `π` 排列，`angle_increment` 为正；不再使用新增的 N10 角度格子重建或按4500点/秒回推逐点时间。
扫描时间戳使用原驱动估计的整圈末端时间，`time_increment` 和 `scan_time` 均为0，表示未提供逐点时间间隔和整圈周期。
N10 仍在接收线程发布完整一圈，避免共享缓冲被下一圈覆盖；不做运动补偿。

```bash
timeout 10s ros2 topic hz /scan
ros2 topic echo /scan --once --field scan_time
ros2 topic echo /scan --once --field time_increment
```

实际扫描频率仍约10Hz，以 `ros2 topic hz /scan` 为准，不能由值为0的 `scan_time` 计算。
`N10Plus_hz` 仅用于N10Plus配置，不能改变N10的真实转速。

## 时间戳检查

实车运行统一使用系统时钟（`use_sim_time:=false`），雷达、IMU和算法都在NX上打ROS时间戳。
`/scan` 沿用 `ros_car_diff` 的整圈末端时间，不宣称精确的首束采样时间；逐点时间未提供，无法按消息字段恢复首末束采样时刻。
RF2O保留输入扫描的时间戳，不会用计算完成时刻覆盖它。
IMU使用有效帧的接收时刻，`/imu/data`取本组陀螺仪帧的接收时间，缺少本组加速度/陀螺仪帧时不发布旧测量。
一次读到多组时解析所有数据、发布最新完整组，保留未完成帧供下次接收继续解析；不按固定字节数清空串口，不回推字节传输耗时。
EKF保留0.5秒历史处理迟到的激光结果，再预测到当前时刻，`/odom`和`odom → base_link`使用相同时间戳。
AMCL的`map → odom`含`transform_tolerance`的前置有效期，静态TF时间为0也正常，不能要求所有话题时间戳数值相同。

启动底盘、建图或导航后，另开终端检查（只监听，不发送运动指令）：

```bash
python3 tests/check_timestamps.py --seconds 10
```

输出`passed: true`表示本次采样的时钟设置、时间顺序、RF2O扫描时间对应关系、里程计TF和扫描参考时间的TF覆盖检查通过。
当前N10未提供逐点时间，该检查不能验证整圈内每一束的TF覆盖。
雷达和此IMU协议均未提供可用的硬件采样时间，这项检查不能证明物理采样已硬同步；内部滤波、USB缓冲等残余延迟需在NX实测。
