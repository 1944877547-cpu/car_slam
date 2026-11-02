# car_slam

基于 **ROS 2 Humble** 的双轮差速小车建图与导航工程，适用于 Ubuntu 22.04、PC 和 Jetson Orin NX。当前分支：`cartographer`。

- **建图与导航**：Cartographer 二维建图，AMCL 定位与 Nav2 自主导航。
- **无编码器里程计**：N10 激光扫描经 IMU 去畸变，RF2O 与 IMU 通过 EKF 融合。
- **底盘控制**：STM32 + TB6612，串口 HEX 数据包下发左右轮 PWM。
- **相机显示**：建图时可查看 Orbbec RGB，图像不参与建图。

```text
N10 → /scan_raw → IMU 去畸变 → /scan → RF2O ─┐
维特 IMU ─────────────────────────────────┤→ EKF → /odom
/scan + /odom → Cartographer 建图 / AMCL 定位 → Nav2
```

## 编译

```bash
git clone -b cartographer https://github.com/1944877547-cpu/car_slam.git
cd car_slam
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

现有工作空间 `/home/yiheng/car_swarm`、`/home/wheeltec/car_swarm` 可继续使用，无需移动目录。PC 与 NX 分别在本机编译。

## 使用

加载工作空间后，以下两种方式选择一种：

```bash
# 建图，默认包含 RGB 预览
ros2 launch mycar_cartographer mapping.launch.py
```

```bash
# 加载已保存地图导航
ros2 launch mycar_navigation2 navigation.launch.py map:="$(pwd)/maps/map.yaml"
```

默认接口：底盘 `/dev/ttyCH341USB0`、雷达 `/dev/ttyCH343USB0`、IMU `/dev/ttyTHS1`，可通过启动参数修改。

详细说明：[NX 全流程指令](NX_COMMANDS.md) · [配置与调参](docs/USAGE.md) · [RF2O 来源](src/rf2o_laser_odometry/UPSTREAM.md)。

扫描时间和去畸变已通过离线模拟及伪串口测试；外参、传感器延迟和 PWM 与车速的对应关系仍需实车标定。
