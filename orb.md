# ORB-SLAM3 RGB-D 测试

独立测试 Orbbec SV1301S_U3 的视觉建图、轨迹、回环和重定位。算法来自 [官方 ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)，ROS接口参考 [zang09/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2)。当前先用 RGB-D；维特IMU尚未完成相机外参与时间标定，不接入视觉惯性模式。

## 首次准备（PC、NX分别执行）

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-yaml python3-opencv libopencv-dev libeigen3-dev \
  libboost-serialization-dev libssl-dev libglew-dev libgl1-mesa-dev libglu1-mesa-dev \
  libjpeg-dev libpng-dev libx11-dev ros-humble-cv-bridge ros-humble-message-filters

cd ~/car_swarm
source /opt/ros/humble/setup.bash
bash src/mycar_orbslam3/scripts/build_orb_slam3.sh 2
colcon build --packages-select mycar_orbslam3 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

依赖保存在 `.orbslam3/`。脚本固定上游版本并在本机编译，不需要修改 `.bashrc`；NX内存紧张时将末尾 `2` 改成 `1`。相机、公共底盘包需已按原工程编译完成。

## 建图测试

先退出其他正在占用相机的进程，再执行：

```bash
source /opt/ros/humble/setup.bash
source ~/car_swarm/install/setup.bash
ros2 launch mycar_orbslam3 mapping.launch.py
```

默认只启动相机、固定外参、ORB-SLAM3和RViz，不启动电机、串口或Nav2。相机已由其他入口启动时加 `start_camera:=false`；公共底盘TF已启动时加 `start_robot_tf:=false`。若想查看官方全局稀疏地图窗口，加 `viewer:=true`，需要可用的OpenGL桌面。

缓慢移动相机/车体，先看见轨迹与黄色稀疏跟踪点，再回到拍摄过的位置测试回环。使用 `Ctrl+C` 正常退出，等待地图保存完成；默认文件位于 `~/car_swarm/maps/orb_slam3/atlas.osa`，关键帧轨迹为同目录 `keyframes.txt`。

已有同名Atlas时，建图入口拒绝覆盖。新建一张地图可加 `atlas_name:=room2`。

## 保存地图后重定位

```bash
ros2 launch mycar_orbslam3 localization.launch.py
```

使用原相机、分辨率与安装位置，在已建图区域中缓慢移动，等待恢复跟踪。测试其他地图时加 `atlas_name:=room2`；正常定位退出不覆盖原Atlas。

## 查看输入与结果

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /orb_slam3/tracking_state
ros2 topic echo /orb_slam3/pose --once
```

- `tracking_state=2` 表示跟踪成功；初始化或丢失时不发布假位姿。
- `/orb_slam3/pose`：车体在测试地图中的位姿；`/orb_slam3/path`：轨迹显示。
- `/orb_slam3/points`：当前跟踪的稀疏三维特征点，**不是完整稠密地图，也不是可直接导航的 `/map`**。
- TF为独立的 `orb_map → orb_base_link`；安装外参读取公共URDF及相机SDK的 `base_link → 彩色光学坐标系`。不写入现有 `map → odom → base_link` 链路。

配置位于 `src/mycar_orbslam3/config/`。图像内参从 `/camera/color/camera_info` 自动读取；深度必须已对齐到彩色图且分辨率、光学frame一致。Orbbec的8参数畸变用于同时校正RGB和深度，再把无畸变图送入ORB，不截断畸变参数。16UC1当前按毫米转换为米，32FC1按米读取。RGB-D配对容差20ms，不改写传感器时间戳。`input_camera.yaml` 保存原始标定，`active_camera.yaml` 保存校正后的实际配置。

`keyframes.txt` 是官方TUM格式的相机轨迹，坐标基准与ROS输出的车体轨迹不同。

这套入口用于视觉算法测试。原来的Cartographer/RTAB-Map导航入口继续按各自文档使用。

## 不接硬件的离线检查

完成编译并加载环境后，在工作空间运行：

```bash
cd ~/car_swarm
ROS_DOMAIN_ID=92 python3 tests/check_orbslam3_rgbd.py
```

使用合成RGB-D图像调用真实ORB核心，检查8参数畸变、深度单位、车体坐标方向、原始时间戳和Atlas保存/重定位；不启动相机或电机。这是接口冒烟测试，不代表实车精度或回环性能已经验证。
