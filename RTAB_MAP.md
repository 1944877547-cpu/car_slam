# RTAB-Map：激光＋RGB-D 三维建图与定位

适用：Orin NX、ROS 2 Humble、N10 雷达、SV1301S_U3 深度相机及现有编码器＋IMU底盘。
以下命令在 NX 的 `~/car_swarm` 工程中执行，启动 RViz 和键盘控制请使用 NX 桌面终端。

轮式＋IMU EKF提供连续里程计；RTAB-Map融合RGB-D和激光，生成三维地图，通过图像回环及几何匹配修正全局位置。
地图保留高度信息，小车位姿采用平面运动约束。与原Cartographer/AMCL方案分别运行，切换前先退出当前方案。

## 1. 安装与编译（首次或更新代码后）

先将包含 `src/mycar_rtabmap` 的源码同步到 NX，并退出运行中的底盘、建图和导航程序。

```bash
sudo apt update
sudo apt install -y ros-humble-rtabmap-ros

cd ~/car_swarm
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --executor sequential --cmake-args -DBUILD_TESTING=OFF
```

## 2. 每个新终端加载环境

```bash
source /opt/ros/humble/setup.bash
source ~/car_swarm/install/local_setup.bash
```

默认设备：底盘 `/dev/ttyCH341USB0`、雷达 `/dev/ttyCH343USB0`、IMU `/dev/ttyTHS1`。
设备名变化时，在完整建图或导航启动命令后添加 `serial_port:=路径 lidar_port:=路径 imu_port:=路径`。

## 3. 首次检查相机

```bash
ros2 launch mycar_rtabmap camera.launch.py
```

另开终端、加载第2步环境，再逐条执行；频率检查按 `Ctrl+C` 结束。

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/image_raw --once --field header
```

当前请求彩色640×480、深度640×400、30FPS，深度由SDK对齐至彩色图像。应确认有效深度和内参正常。
若提示采集规格不支持，先退出相机，再查看设备支持列表，并修改 `config/camera.yaml`：

```bash
ros2 run orbbec_camera list_camera_profile
```

检查完成后，先 `Ctrl+C` 退出单独的相机节点。

## 4. 启动三维建图

终端A加载环境后执行：

```bash
ros2 launch mycar_rtabmap mapping.launch.py database:="$HOME/car_swarm/maps/rtabmap.db"
```

该入口统一启动底盘、IMU、雷达、EKF、RGB-D相机、RTAB-Map和RViz。

| RViz内容 | 话题 |
| --- | --- |
| 三维地图 | `/rtabmap/cloud_map` |
| 二维地图投影 | `/map` |
| 彩色画面 | `/camera/color/image_raw` |
| 激光扫描 | `/scan` |

终端B加载环境后执行键盘控制：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.3 -r cmd_vel:=/cmd_vel
```

英文输入法下，`i`前进、`,`后退、`j/l`左转/右转、`k`停车。终端保持键盘焦点，持续运动需按住按键。
缓慢移动，覆盖环境并回到经过的区域，让相机看到纹理清楚的场景。

相机输入配置为30FPS，激光约10Hz，RTAB-Map处理频率为1Hz；是否保存新关键帧还取决于运动和匹配结果。

## 5. 保存地图

先在终端B按 `k` 停车并退出遥控，再在终端A按 `Ctrl+C` 正常退出建图，等待数据库保存完成。

```bash
ls -lh ~/car_swarm/maps/rtabmap.db
```

`.db`保存关键帧图像、深度、激光及位姿图，重定位需要保留这个文件。
默认使用已有数据库继续建图；建立另一张地图时，将 `database` 改成新的文件名，例如 `room2.db`。

可选：在建图仍运行时导出二维地图，另开已加载环境的终端执行：

```bash
ros2 run nav2_map_server map_saver_cli -t /map -f "$HOME/car_swarm/maps/rtabmap_2d" --ros-args -p save_map_timeout:=10.0
```

生成的PGM/YAML用于二维地图查看或其他定位方案，不能替代三维数据库。

## 6. 加载地图定位

先退出建图，加载环境后执行：

```bash
ros2 launch mycar_rtabmap mapping.launch.py localization:=true database:="$HOME/car_swarm/maps/rtabmap.db"
```

该模式加载已有数据库进行重定位，不增加建图节点。让镜头朝向建图时见过的场景，缓慢移动观察激光与地图是否重合。
必要时在RViz用 `2D Pose Estimate` 给出初始位置。

## 7. 加载地图并导航

先退出上一套建图或定位程序，再执行：

```bash
ros2 launch mycar_rtabmap navigation.launch.py database:="$HOME/car_swarm/maps/rtabmap.db"
```

确认重定位正确后，在RViz使用 `Nav2 Goal` 指定目标。
RTAB-Map提供地图和 `map → odom`，EKF提供 `odom → base_link`；此入口使用现有Nav2控制参数及激光避障配置。

## 8. 数据检查与参数位置

```bash
ros2 topic echo /rtabmap/info --once
ros2 topic info /map
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo map odom
```

`loop_closure_id`非0表示接受了全局回环，`proximity_detection_id`非0表示近邻匹配；回环不要求每帧发生。

| 配置 | 文件（相对工程根目录） |
| --- | --- |
| 相机采集规格、彩深对齐 | `src/mycar_rtabmap/config/camera.yaml` |
| 融合、回环和地图参数 | `src/mycar_rtabmap/config/rtabmap.yaml` |
| 相机安装位置 | `src/mycar_base/urdf/mycar.urdf.xacro` 的 `camera_joint` |
| 导航速度、车体轮廓及避障参数 | `src/mycar_navigation2/param/nav2_params.yaml` |

当前相机外参：`xyz="0.05 0 0.02"`、`rpy="0 0 0"`，表示前5cm、居中、上2cm，按镜头水平朝前配置。
实际镜头有俯仰或侧移时修改该关节；修改后重新编译。

PC已通过编译、合成数据建图、近邻重定位和Nav2启动检查；NX上的实际深度质量、彩深对齐和全局图像回环效果仍需实测。
