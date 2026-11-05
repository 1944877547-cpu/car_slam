# Cartographer 二维建图与导航

以下命令在NX的 `~/car_swarm` 执行。先退出其他建图/导航程序；RTAB-Map流程见 [RTAB_MAP.md](RTAB_MAP.md)。

## 1. 编译和加载环境

源码更新后，先退出运行中的程序再编译：

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

默认串口：底盘 `/dev/ttyCH341USB0`，雷达 `/dev/ttyCH343USB0`，IMU `/dev/ttyTHS1`。
设备名变化时使用 `serial_port:=路径 lidar_port:=路径 imu_port:=路径`覆盖。

## 2. 建图与键盘控制

终端A启动建图、底盘、传感器和RViz；相机仅显示RGB画面：

```bash
ros2 launch mycar_cartographer mapping.launch.py
```

不接相机时添加 `start_camera:=false`。
终端B加载环境后运行，英文输入法，`i`前进、`,`后退、`j/l`转向、`k`停车：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.3 -r cmd_vel:=/cmd_vel
```

## 3. 保存地图

先按 `k`停车，保持建图程序运行。在另一个已加载环境的终端执行：

```bash
mkdir -p ~/car_swarm/maps
ros2 run nav2_map_server map_saver_cli -t /map -f "$HOME/car_swarm/maps/map" --ros-args -p save_map_timeout:=10.0
```

看到 `Map saved successfully`后，再退出遥控与建图。输出为 `map.pgm`和 `map.yaml`。

## 4. AMCL定位与Nav2导航

```bash
ros2 launch mycar_navigation2 navigation.launch.py map:="$HOME/car_swarm/maps/map.yaml"
```

RViz先用 `2D Pose Estimate`设置初始位置，确认激光与地图重合后，再用 `Nav2 Goal`。

## 5. 检查与参数

```bash
ros2 control list_controllers
ros2 control list_hardware_components
ros2 topic echo /odom --once
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
```

轮半径/轮距在 `src/mycar_base/config/controllers.yaml`，传感器安装位置在URDF，建图参数在 `src/mycar_cartographer/config/mycar_2d.lua`，导航参数在 `src/mycar_navigation2/param/nav2_params.yaml`。
