# car_slam

当前分支：**RTAB-MAP**，使用N10激光雷达＋维特IMU进行二维建图、几何回环与重定位。Cartographer方案位于 **cartographer** 分支。

基于 ROS 2 Humble 的双轮差速小车建图与导航项目，包含底盘控制、传感器驱动、里程计、SLAM 和自主导航等模块。

仓库主要内容：

- **底盘控制**：STM32 与 TB6612 双电机控制的串口通信接口。
- **传感器驱动**：N10 激光雷达、维特 IMU 和 Orbbec 相机驱动。
- **里程计处理**：RTAB-Map使用激光ICP里程计＋IMU角速度EKF，不融合轮式里程计；原Cartographer入口保留轮式＋IMU方案。
- **建图与导航**：Cartographer或RTAB-Map二维建图，配合Nav2自主导航。当前RTAB-Map入口不启动相机。
- **配套文件**：机器人模型、RViz 配置、地图、测试程序和使用文档。

另提供 **ORB-SLAM3 RGB-D 独立测试**，用于Orbbec相机视觉建图、轨迹和重定位。

使用说明：[RTAB_MAP.md](RTAB_MAP.md) · [cartographer.md](cartographer.md) · [orb.md](orb.md)。
