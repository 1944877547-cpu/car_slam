# RTAB-Map 建图与导航

N10 雷达＋维特 IMU，激光 ICP＋IMU 角速度 EKF 提供里程计。当前入口不启动相机。
**建图和导航统一使用 `~/car_swarm/maps/rtabmap_lidar.db`。正常退出建图即保存，之后每次导航都读取这个数据库。**

## 1. 编译

首次安装依赖：

```bash
sudo apt install -y ros-humble-rtabmap-ros ros-humble-robot-localization
```

退出旧的底盘、建图和导航程序，再执行：

```bash
cd ~/car_swarm
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --executor sequential --packages-select mycar_base mycar_navigation2 mycar_rtabmap --cmake-args -DBUILD_TESTING=OFF
```

**每个新终端**先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/car_swarm/install/local_setup.bash
```

## 2. 重新建图

先关闭旧建图、导航。下面将已有数据库改名备份，让本次生成一张新地图；若要继续扩建旧地图，跳过 `if` 这一段。

```bash
mkdir -p ~/car_swarm/maps
if [ -f "$HOME/car_swarm/maps/rtabmap_lidar.db" ]; then
  mv "$HOME/car_swarm/maps/rtabmap_lidar.db" "$HOME/car_swarm/maps/rtabmap_lidar_$(date +%Y%m%d_%H%M%S_%N).db"
fi
ros2 launch mycar_rtabmap mapping.launch.py database:="$HOME/car_swarm/maps/rtabmap_lidar.db"
```

新终端加载环境后遥控：`i` 前进、`,` 后退、`j/l` 转向、`k` 停车。

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.3 -r cmd_vel:=/cmd_vel
```

**保存：**先按 `k` 停车并退出遥控；在建图终端按 `Ctrl+C`，等待进程正常退出、数据库写入完成，再查看更新时间：

```bash
ls -lh --time-style=long-iso ~/car_swarm/maps/rtabmap_lidar.db
```

## 3. 使用刚保存的地图导航

每次启动都执行同一条命令，不需要复制地图或改 YAML 路径：

```bash
ros2 launch mycar_rtabmap navigation.launch.py database:="$HOME/car_swarm/maps/rtabmap_lidar.db"
```

启动日志应包含 `RTAB-Map localization (saved map): /home/wheeltec/car_swarm/maps/rtabmap_lidar.db`（PC 上用户名不同）。导航以定位模式读取该数据库，RTAB-Map 发布 `/map` 给 Nav2；不读取旧的 `map.yaml`。数据库缺失或为空时拒绝启动。

在 RViz 用 **2D Pose Estimate** 指定当前位置与朝向，激光与地图对齐后，用 **Nav2 Goal** 下发目标。每次只运行一套完整入口。

## 4. 常用参数

- 直行上限 **0.25 m/s**，转向上限 **1.60 rad/s**，角加速度上限 **2.00 rad/s²**；实际速度由规划器选择。
- Nav2：`src/mycar_navigation2/param/nav2_params.yaml`；底盘限速：`src/mycar_base/config/controllers.yaml`。
- 传感器位置、朝向：`src/mycar_base/urdf/mycar.urdf.xacro`。
- 默认串口：底盘 `/dev/ttyCH341USB0`，雷达 `/dev/ttyCH343USB0`，IMU `/dev/ttyTHS1`。
- TF：RTAB-Map 发布 `map → odom`，EKF 发布 `odom → base_link`，固定外参来自 URDF。
