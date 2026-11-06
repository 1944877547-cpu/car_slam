# 上游与适配

- 算法：[UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)，固定提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`，保留官方 GPL 许可。
- ROS 2 RGB-D 接口参考：[zang09/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2)，参考提交 `00c54335ccc010d74c1e24e336aa817604124947`；上游 package.xml 声明 MIT。本项目重新实现了相机参数、数据检查和ROS输出适配。
- 可视化依赖：[stevenlovegrove/Pangolin](https://github.com/stevenlovegrove/Pangolin)，v0.6，提交 `dd801d244db3a8e27b7fe8020cd751404aa818fd`。

算法源码和词典由构建脚本下载到工作空间 `.orbslam3/`，不提交PC二进制。构建兼容修改包括 C++14、去掉 `-march=native`、显式先构建g2o和不编译示例程序。

纯定位模式修复了上游 [issue #515](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/515) 对应的问题：本适配器显式设置 `System.LocalizeOnly=1` 时，选择Atlas中关键帧最多的有效已有地图；首帧进入重定位，RECENTLY_LOST状态也允许重定位。建图入口仍使用原始初始化路径，不修改ORB特征、匹配、PnP、局部优化或回环算法。补丁由 `scripts/prepare_core.py` 按固定版本应用。

本项目适配：CameraInfo真实内参、对齐深度单位、SensorDataQoS、RGB-D配对、原始采集时间戳、公共URDF与SDK提供的安装外参、独立测试TF、轨迹及稀疏跟踪点发布、Atlas保存与加载。稀疏跟踪点不是稠密点云或Nav2栅格地图。
