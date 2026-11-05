# car_slam

当前分支：**RTAB-MAP**，提供激光＋RGB-D三维建图、图像回环与重定位。原激光方案位于 **cartographer** 分支。

基于 ROS 2 Humble 的双轮差速小车建图与导航项目，包含底盘控制、传感器驱动、里程计、SLAM 和自主导航等模块。

仓库主要内容：

- **底盘控制**：STM32 与 TB6612 双电机控制的串口通信接口。
- **传感器驱动**：N10 激光雷达、维特 IMU 和 Orbbec 相机驱动。
- **里程计处理**：真实轮式编码器里程计，以及基于 EKF 的轮式里程计与 IMU 融合。
- **建图与导航**：Cartographer 二维建图、AMCL 定位，以及 RTAB-Map 激光＋RGB-D 三维建图、图像回环与重定位，配合 Nav2 自主导航。
- **配套文件**：机器人模型、RViz 配置、地图、测试程序和使用文档。

使用说明：[RTAB_MAP.md](RTAB_MAP.md) · [cartographer.md](cartographer.md)。
