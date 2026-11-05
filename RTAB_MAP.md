# RTAB-Map 三维建图与定位

适用于NX的 `~/car_swarm`。融合激光与RGB-D，复用编码器＋IMU里程计。
与Cartographer分别运行，切换前先退出当前建图/导航。原方案见 [cartographer.md](cartographer.md)。
三维地图保留RGB-D＋激光融合；Nav2的二维栅格按激光生成，共用RTAB-Map优化后的位姿图。

## 1. 安装、编译和加载环境

首次安装依赖：

```bash
sudo apt update
sudo apt install -y ros-humble-rtabmap-ros
```

源码更新后，退出运行中的程序再编译：

```bash
cd ~/car_swarm
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --executor sequential --cmake-args -DBUILD_TESTING=OFF
```

每个新终端加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/car_swarm/install/local_setup.bash
```

## 2. 建图与键盘控制

终端A：

```bash
ros2 launch mycar_rtabmap mapping.launch.py database:="$HOME/car_swarm/maps/rtabmap.db"
```

终端B加载环境后运行，英文输入法，`i`前进、`,`后退、`j/l`转向、`k`停车：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.3 -r cmd_vel:=/cmd_vel
```

RViz：`/rtabmap/cloud_map`为三维地图，`/map`为激光导航栅格，深度投影在`/rtabmap/projected_map`。停车后退出遥控，再在终端A按 `Ctrl+C`，等待 `.db`保存完成。
重定位需要保留数据库；新地图使用新文件名，已有数据库默认继续建图。

## 3. 重定位与导航（二选一）

仅重定位：

```bash
ros2 launch mycar_rtabmap mapping.launch.py localization:=true database:="$HOME/car_swarm/maps/rtabmap.db"
```

重定位＋Nav2：

```bash
ros2 launch mycar_rtabmap navigation.launch.py database:="$HOME/car_swarm/maps/rtabmap.db"
```

先用 `2D Pose Estimate`设置当前所在位置及朝向，确认激光与地图墙面对齐后再用 `Nav2 Goal`。
导航默认打开二维俯视图，显示地图、机器人、激光及规划路径；建图入口继续显示三维点云。
保留图像回环，并启用RTAB-Map原生全局激光地图邻近匹配。只有TF存在不代表已重定位成功。
每次只启动一套完整入口；已有底盘节点时加`start_base:=false`，避免重复占用串口。

## 4. 参数与检查

默认串口：底盘 `/dev/ttyCH341USB0`，雷达 `/dev/ttyCH343USB0`，IMU `/dev/ttyTHS1`；可用 `serial_port`、`lidar_port`、`imu_port`启动参数覆盖。

| 参数 | 位置 |
| --- | --- |
| 处理上限1Hz、配准和地图 | `src/mycar_rtabmap/config/rtabmap.yaml` |
| 激光导航栅格、旧栅格缓存重建 | `src/mycar_rtabmap/config/navigation_map.yaml` |
| 彩深对齐、采集规格 | `src/mycar_rtabmap/config/camera.yaml` |
| 相机安装外参，按实车测量值填写 | URDF的 `camera_joint` |
| 导航上限0.25m/s，局部/全局膨胀半径0.20m | `src/mycar_navigation2/param/nav2_params.yaml` |

检查命令逐条运行，频率检查用 `Ctrl+C`结束：

```bash
ros2 param get /rtabmap/rtabmap Rtabmap/DetectionRate
ros2 topic hz /rtabmap/info
ros2 topic hz /camera/depth/image_raw
ros2 run tf2_ros tf2_echo map base_link
```

1Hz为处理上限，不保证每秒保存1个关键帧。观察启动日志中的处理耗时、同步丢帧/等待告警和TF等待；每组处理预算约1秒，实际效果需在NX验证。
相机请求彩色640×480、深度640×400、30FPS；实际支持规格以设备为准。提高处理频率不能替代轮径、轮距和传感器外参标定。

参考：[教程的RTABMap章节](https://gitee.com/gwmunan/ros2/wikis/pages?sort_id=11078218&doc_id=4855084)。教程使用RealSense双目，本工程保留Orbbec RGB-D和轮式＋IMU EKF。
