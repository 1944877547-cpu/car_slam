# RTAB-Map：雷达＋IMU建图与导航

适用于NX的`~/car_swarm`，使用N10激光与维特IMU。当前入口不启动相机、不融合轮式里程计。
流程：`/scan → ICP → /odom_icp`，再与`/imu/data`角速度经EKF生成`/odom`，供RTAB-Map和Nav2使用。
编码器继续用于STM32电机转速闭环及关节显示。原Cartographer入口保留原来的轮式＋IMU方案。

## 1. 编译与环境

首次安装依赖：

```bash
sudo apt install -y ros-humble-rtabmap-ros ros-humble-robot-localization
```

源码更新后退出旧建图、导航和底盘程序，再编译：

```bash
cd ~/car_swarm
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --executor sequential --packages-select mycar_base mycar_rtabmap --cmake-args -DBUILD_TESTING=OFF
```

每个新终端加载：

```bash
source /opt/ros/humble/setup.bash
source ~/car_swarm/install/local_setup.bash
```

## 2. 先建立二维激光地图

新方案使用独立数据库`rtabmap_lidar.db`，原RGB-D数据库`rtabmap.db`保留；请用新方案重新建图。

```bash
ros2 launch mycar_rtabmap mapping.launch.py database:="$HOME/car_swarm/maps/rtabmap_lidar.db"
```

新终端加载环境后遥控，`i`前进、`,`后退、`j/l`转向、`k`停车：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.3 -r cmd_vel:=/cmd_vel
```

RViz显示二维地图、激光和机器人。停车并退出遥控，再在建图终端按`Ctrl+C`，等待数据库保存完成。

## 3. 定位与导航

先关闭建图，再使用保存的新数据库：

```bash
ros2 launch mycar_rtabmap navigation.launch.py database:="$HOME/car_swarm/maps/rtabmap_lidar.db"
```

用`2D Pose Estimate`给出当前的位置和朝向，确认激光与墙面对齐后，再用`Nav2 Goal`下发目标。
当前使用ICP几何匹配和激光邻近回环，不再使用图像回环或深度三维建图。
每次只启动一套完整入口，避免重复占用串口。若单独启动底盘，必须加`start_ekf:=false`，再给RTAB-Map加`start_base:=false`。

## 4. 参数与TF

默认串口：底盘`/dev/ttyCH341USB0`，雷达`/dev/ttyCH343USB0`，IMU`/dev/ttyTHS1`。

| 内容 | 文件 |
| --- | --- |
| 激光ICP，无轮式运动初值 | `src/mycar_rtabmap/config/icp_odometry.yaml` |
| 激光速度＋IMU角速度融合 | `src/mycar_rtabmap/config/laser_imu_ekf.yaml` |
| ICP回环、1Hz建图、二维激光栅格 | `src/mycar_rtabmap/config/rtabmap.yaml` |
| 0.25m/s前进上限、0.20m局部/全局膨胀半径 | `src/mycar_navigation2/param/nav2_params.yaml` |
| 传感器安装位置与朝向 | `src/mycar_base/urdf/mycar.urdf.xacro` |

`odom → base_link`只由`rtabmap_ekf`发布，`map → odom`只由RTAB-Map发布，传感器固定TF来自URDF。
WIT姿态当前标记为不可用，只融合角速度Z；不积分加速度推算平移，不改写雷达或IMU时间戳。
二维激光不能恢复深度相机的完整三维结构；长直走廊或几何特征不足时，激光里程计仍可能退化。

参考：[教程RTABMap章节](https://gitee.com/gwmunan/ros2/wikis/pages?sort_id=11078218&doc_id=4855084)、[RTAB-Map激光示例](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/turtlebot3/turtlebot3_scan.launch.py)。
教程中的双目前端改为原生激光ICP，IMU通过`robot_localization`的EKF融合角速度。
