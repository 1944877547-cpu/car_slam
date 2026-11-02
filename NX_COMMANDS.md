# car_slam：NX 全流程指令

适用于 **Orin NX + Ubuntu 22.04 + ROS 2 Humble**。仓库为 `1944877547-cpu/car_slam`，当前代码位于 `cartographer` 分支。
命令沿用 NX 已有目录 `~/car_swarm`；仓库改名无需移动本地目录。更新源码后先执行第 2 步编译。
以下命令均在 NX 执行；启动 RViz、键盘控制时，使用 NX 桌面内的终端。

## 1. 上电与设备检查

接好 STM32、N10 雷达、维特 IMU 和 Orbbec 相机，电机使用独立电源并与 STM32 共地。

| 设备 | 当前接口 | 参数 |
| --- | --- | --- |
| STM32 / TB6612 底盘 | `/dev/ttyCH341USB0` | 115200，8N1 |
| N10 雷达 | `/dev/ttyCH343USB0` | 230400 |
| 维特 IMU | `/dev/ttyTHS1` | 115200，约 20 Hz |
| Orbbec RGB | USB，驱动自动查找 | 640×480，配置 30 FPS |

```bash
ls -l /dev/ttyCH* /dev/ttyTHS* /dev/video*
lsusb
id -nG
```

当前 NX 已配置串口和相机权限。设备节点缺失时检查 USB、接线和驱动；出现 `Permission denied` 时再检查用户组与 USB 规则。

## 2. 编译（修改代码后执行）

首次使用或更新源码后重新编译，完成后日常使用可从第 3 步开始。先关闭运行中的建图、底盘或导航程序，再编译。

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
cd ~/car_swarm
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --executor sequential --cmake-args -DBUILD_TESTING=OFF
```

## 3. 每个新终端加载环境

```bash
source /opt/ros/humble/setup.bash
source ~/car_swarm/install/local_setup.bash
```

请使用新终端，直接加载系统 ROS Humble 和本工程。NX 的 `.bashrc` 已停止加载预装的 `wheeltec_ros2`；
无需单独的启动脚本，也无需清空其他已配置的 Python 和动态库路径。
可用 `ros2 pkg prefix nav2_map_server` 检查，当前应输出 `/opt/ros/humble`。

当前导航指令上限：线速度 **0.20 m/s（20 cm/s）**，转向 **1.20 rad/s（约 69°/s）**。导航线加速度为 0.50 m/s²，角加速度为 2.00 rad/s²；实际速度仍需按实车标定。

串口直接传实际PWM，默认范围为 **−1200～+1200**。STM32新固件只限幅，不增加起步补偿、二次缩放或额外斜坡；两端需配套更新。`max_rpm=100`仍是PWM=1200时轮速的待标定初值。

膨胀半径等避障参数以 `src/mycar_navigation2/param/nav2_params.yaml` 的当前设置为准。

## 4. 终端 A：建图并打开 RGB

```bash
ros2 launch mycar_cartographer mapping.launch.py
```

保持终端 A 运行。它会启动底盘、雷达、RF2O 激光里程计、IMU、EKF、Cartographer、RGB 和 RViz。
RViz 的 `Fixed Frame` 使用 `map`，`RGB` 面板显示普通彩色画面；相机只用于观看。

没有接相机时，改用：

```bash
ros2 launch mycar_cartographer mapping.launch.py start_camera:=false
```

串口变化时，可在启动命令后添加 `serial_port:=设备路径`、`lidar_port:=设备路径`、`imu_port:=设备路径`。

## 5. 终端 B：键盘控制小车建图

先执行第 3 步加载环境，再运行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.3 -r cmd_vel:=/cmd_vel
```

点击终端 B，使其获得键盘焦点，切换英文输入法：

| 按键 | 动作 |
| --- | --- |
| `i` | 前进 |
| `,` | 后退 |
| `j` / `l` | 原地左转 / 右转 |
| `k` | 停车 |
| `q` / `z` | 同时增加 / 降低线速度和转速 |
| `Ctrl+C` | 停车并退出遥控 |

此版本按键触发发送，持续运动需按住方向键；控制器约 0.5 秒未收到新指令会停车。缓慢移动，覆盖房间并回到经过的区域。

## 6. 终端 C：检查数据

先执行第 3 步。以下命令逐条运行，`topic hz` 检查完成后按 `Ctrl+C` 退出。

```bash
ros2 control list_controllers
ros2 topic info /cmd_vel
ros2 topic echo /odom_lidar --once
ros2 topic echo /odom --once
ros2 topic hz /odom_lidar
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /camera/color/image_raw
```

预期：两个控制器均为 `active`，`/cmd_vel` 有订阅者；雷达及 `/odom_lidar` 约 10 Hz，IMU 约 20 Hz，RGB 通常接近 30 Hz。
键盘无反应时，先检查终端 A 是否仍在运行，以及 `/cmd_vel` 是否有订阅者。

## 7. 保存地图

先在终端 B 按 `k` 停车，保持终端 A 的建图程序运行。在加载环境的终端 C 确认地图有发布者：

```bash
ros2 topic info /map
```

应看到 `Publisher count` 至少为 1。若提示 `Unknown topic` 或发布者为 0，先检查终端 A 的建图程序。
随后保存；等待时间设为 10 秒，只有保存成功才列出文件：

```bash
mkdir -p ~/car_swarm/maps
ros2 run nav2_map_server map_saver_cli -t /map -f "$HOME/car_swarm/maps/map" --ros-args -p save_map_timeout:=10.0 &&
ls -l ~/car_swarm/maps/map.*
```

以 `Map saved successfully` 为成功标志，并检查文件更新时间。保存失败时，已有文件可能是上次留下的旧地图。
确认成功后，再按 `Ctrl+C` 关闭终端 B 的遥控和终端 A 的建图。再次使用相同文件名保存会覆盖原地图。

## 8. 使用保存的地图导航

在 NX 桌面终端加载第 3 步的环境，然后运行：

```bash
ros2 launch mycar_navigation2 navigation.launch.py map:="$HOME/car_swarm/maps/map.yaml"
```

导航会启动所需的底盘、雷达、RF2O、IMU、EKF 和 RViz。建图与导航应分别运行。

1. 在 RViz 点击 `2D Pose Estimate`，在地图上拖出小车当前的位置和朝向。
2. 确认激光点与地图墙体大致重合。
3. 点击 `Nav2 Goal`，拖出目标位置和朝向，先测试附近的目标点。
4. 导航结束前，通过 Nav2 面板取消当前目标，再在启动终端按 `Ctrl+C` 退出。

## 9. 单独检查设备（可选）

先关闭建图或导航，避免重复打开设备。加载环境后，下面命令任选一个：

```bash
ros2 launch mycar_base base.launch.py
```

```bash
ros2 launch mycar_cartographer camera_rgb.launch.py
```

单独启动相机只发布 `/camera/color/image_raw` 等话题；需要显示画面时使用第 4 步的建图入口。

当前底盘没有轮编码器，平移里程计来自 `/scan → RF2O → /odom_lidar`；EKF 融合激光平面速度、角速度与 IMU 角速度后输出 `/odom`。`/odom_raw` 只保留为指令估算对照，不进入 EKF。轮径、轮距、PWM 与实际车速的对应关系仍需实车标定。

底盘入口已包含雷达驱动。若雷达驱动已单独运行，可加 `start_lidar:=false` 复用其 `/scan`。首次启动等待扫描/TF 最多 30 秒，之后连续 1 秒无有效激光里程计将关闭当前底盘启动组；不会回退到指令估算。

## N10 扫描检查

雷达直接发布 `/scan`，保留原始采集时序，频率维持约10Hz。

```bash
timeout 10s ros2 topic hz /scan
ros2 topic echo /scan --once --field scan_time
```

`scan_time` 通常约0.1秒。IMU继续用于EKF融合，不再经过独立的扫描运动补偿节点。
外部雷达驱动已提供 `/scan` 时，启动参数添加 `start_lidar:=false`。
