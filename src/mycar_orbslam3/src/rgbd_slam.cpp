// SPDX-License-Identifier: GPL-3.0-or-later
// 参考zang09/ORB_SLAM3_ROS2的RGB-D调用方式，适配Humble、Orbbec和本项目TF。
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "System.h"

class RgbdSlam : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using Info = sensor_msgs::msg::CameraInfo;
  using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;

public:
  RgbdSlam() : Node("orb_slam3")
  {
    vocabulary_ = declare_parameter<std::string>("vocabulary", "");
    directory_ = declare_parameter<std::string>("map_dir", "");
    atlas_ = declare_parameter<std::string>("atlas_name", "atlas");
    base_ = declare_parameter<std::string>("base_frame", "base_link");
    world_ = declare_parameter<std::string>("map_frame", "orb_map");
    child_ = declare_parameter<std::string>("output_frame", "orb_base_link");
    localization_ = declare_parameter("localization", false);
    viewer_ = declare_parameter("viewer", false);
    publish_tf_ = declare_parameter("publish_tf", true);
    scale_ = declare_parameter("depth_scale", 0.001);
    tolerance_ = declare_parameter("sync_tolerance", 0.02);
    fps_ = declare_parameter("fps", 30.0);
    features_ = declare_parameter("features", 1200);
    baseline_ = declare_parameter("virtual_baseline", 0.05);
    close_depth_ = declare_parameter("close_depth", 4.0);
    vis_rate_ = declare_parameter("visualization_rate", 5.0);
    path_size_ = declare_parameter("max_path_size", 3000);
    if (!std::filesystem::is_regular_file(vocabulary_) || directory_.empty() ||
      atlas_.empty() || atlas_.find_first_of("/\\") != std::string::npos ||
      atlas_ == "." || atlas_ == ".." || scale_ <= 0 || tolerance_ <= 0 ||
      fps_ <= 0 || features_ < 100 || baseline_ <= 0 || close_depth_ <= 0 ||
      vis_rate_ <= 0 || path_size_ < 1 || world_ == child_ || child_ == base_)
      throw std::runtime_error("Invalid ORB parameters or vocabulary; see orb.md");
    std::filesystem::create_directories(directory_);
    // 官方Atlas接口使用相对文件名；由launch将进程工作目录设为map_dir。
    if (std::filesystem::canonical(directory_) != std::filesystem::current_path())
      throw std::runtime_error("Run the mapping/localization launch so cwd equals map_dir");
    if (localization_ && !std::filesystem::is_regular_file(atlas_ + ".osa"))
      throw std::runtime_error("Localization requires an existing Atlas: " + atlas_ + ".osa");

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/orb_slam3/path", rclcpp::QoS(1).transient_local());
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/points", 1);
    state_pub_ = create_publisher<std_msgs::msg::Int32>("/orb_slam3/tracking_state", rclcpp::QoS(1).transient_local());
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, false);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto qos = rclcpp::SensorDataQoS().keep_last(3).get_rmw_qos_profile();
    rgb_.subscribe(this, declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw"), qos);
    depth_.subscribe(this, declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw"), qos);
    sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(Policy(5), rgb_, depth_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(tolerance_));
    sync_->registerCallback(&RgbdSlam::frame, this);
    info_sub_ = create_subscription<Info>(
      declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info"),
      rclcpp::SensorDataQoS(), [this](Info::ConstSharedPtr msg) { info_ = msg; });
    RCLCPP_INFO(get_logger(), "Waiting for CameraInfo, aligned RGB-D and camera extrinsic TF");
  }

  ~RgbdSlam() override
  {
    if (!slam_) return;
    slam_->Shutdown();                  // 调用官方关闭和Atlas保存流程，不要直接kill -9。
    if (tracked_ && !localization_) slam_->SaveKeyFrameTrajectoryTUM("keyframes.txt");
  }

private:
  void initialize(const Info & info, const std::string & frame)
  {
    const auto tf = buffer_->lookupTransform(base_, frame, tf2::TimePointZero);
    const auto & q = tf.transform.rotation;
    const auto & t = tf.transform.translation;
    base_camera_ = Eigen::Isometry3f::Identity();
    base_camera_.linear() = Eigen::Quaternionf(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    base_camera_.translation() = Eigen::Vector3f(t.x, t.y, t.z);
    // 固定地图坐标取首次车体坐标；包含光学轴转换和真实安装偏移。
    // 当前位姿：T_map_base = T_base_camera * inverse(T_camera_world) * inverse(T_base_camera)。
    const cv::Mat intrinsic(3, 3, CV_64F, const_cast<double *>(info.k.data()));
    // Orbbec输出8参数rational_polynomial，而ORB的PinHole只接收5参数。
    // 用完整原始标定同时校正彩色和深度，输出相同K的无畸变RGB-D，不能截断D。
    cv::initUndistortRectifyMap(intrinsic, cv::Mat(info.d), cv::Mat(), intrinsic,
      cv::Size(info.width, info.height), CV_32FC1, map_x_, map_y_);
    // OpenCV读取ORB的点号键名，但其YAML写入器拒绝点号；按原版格式写出。
    std::ofstream f("active_camera.yaml");
    if (!f) throw std::runtime_error("Cannot write active_camera.yaml");
    f << "%YAML:1.0\n" << std::setprecision(17) << std::showpoint;
    const auto put = [&f](const char * name, auto value) { f << name << ": " << value << "\n"; };
    f << "Camera.type: \"PinHole\"\n";
    put("Camera.fx", info.k[0]); put("Camera.fy", info.k[4]);
    put("Camera.cx", info.k[2]); put("Camera.cy", info.k[5]);
    for (const auto * name : {"Camera.k1", "Camera.k2", "Camera.p1", "Camera.p2", "Camera.k3"}) put(name, 0.0);
    put("Camera.width", int(info.width)); put("Camera.height", int(info.height));
    put("Camera.fps", fps_); put("Camera.RGB", 0);
    put("Camera.bf", baseline_ * info.k[0]); put("ThDepth", close_depth_ / baseline_);
    put("DepthMapFactor", 1.0);          // 已将深度转换为米，库内不再次缩放。
    put("ORBextractor.nFeatures", features_); put("ORBextractor.scaleFactor", 1.2);
    put("ORBextractor.nLevels", 8); put("ORBextractor.iniThFAST", 20); put("ORBextractor.minThFAST", 7);
    put("Viewer.KeyFrameSize", 0.05); put("Viewer.KeyFrameLineWidth", 1.0);
    put("Viewer.GraphLineWidth", 0.9); put("Viewer.PointSize", 2.0);
    put("Viewer.CameraSize", 0.08); put("Viewer.CameraLineWidth", 3.0);
    put("Viewer.ViewpointX", 0.0); put("Viewer.ViewpointY", -0.7);
    put("Viewer.ViewpointZ", -1.8); put("Viewer.ViewpointF", 500.0);
    f << (localization_ ? "System.LoadAtlasFromFile: " : "System.SaveAtlasToFile: ")
      << std::quoted(atlas_) << "\n";
    put("System.LocalizeOnly", localization_ ? 1 : 0);
    f.close();
    cv::FileStorage input("input_camera.yaml", cv::FileStorage::WRITE);
    input << "K" << intrinsic << "D" << cv::Mat(info.d);
    input << "distortion_model" << info.distortion_model;
    input.release();
    slam_ = std::make_unique<ORB_SLAM3::System>(vocabulary_, "active_camera.yaml", ORB_SLAM3::System::RGBD, viewer_);
    if (localization_) slam_->ActivateLocalizationMode();
    calibration_ = info;
    RCLCPP_INFO(get_logger(), "RGB-D ready: %ux%u; intrinsics from CameraInfo; TF %s <- %s",
      info.width, info.height, base_.c_str(), frame.c_str());
  }

  void frame(const Image::ConstSharedPtr & rgb, const Image::ConstSharedPtr & depth)
  {
    if (!info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for color CameraInfo");
      return;
    }
    const auto stamp = rclcpp::Time(rgb->header.stamp);
    const double time = stamp.seconds();
    if ((have_stamp_ && time <= last_stamp_) ||
      std::abs((stamp - rclcpp::Time(depth->header.stamp)).seconds()) > tolerance_) return;
    const auto & k = info_->k;
    const bool valid_d = info_->d.empty() || info_->d.size() == 4 ||
      info_->d.size() == 5 || info_->d.size() == 8;
    if (rgb->width != depth->width || rgb->height != depth->height ||
      rgb->width != info_->width || rgb->height != info_->height ||
      rgb->header.frame_id.empty() || rgb->header.frame_id != depth->header.frame_id ||
      rgb->header.frame_id != info_->header.frame_id || k[0] <= 0 || k[4] <= 0 ||
      (!info_->distortion_model.empty() && info_->distortion_model != "plumb_bob" &&
      info_->distortion_model != "rational_polynomial") || !valid_d ||
      !std::all_of(k.begin(), k.end(), [](double v) { return std::isfinite(v); }) ||
      !std::all_of(info_->d.begin(), info_->d.end(), [](double v) { return std::isfinite(v); }))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "RGB-D calibration/alignment mismatch: require aligned depth, equal size and color optical frame");
      return;
    }
    if (slam_ && (info_->k != calibration_.k || info_->d != calibration_.d ||
      info_->width != calibration_.width || info_->height != calibration_.height ||
      info_->header.frame_id != calibration_.header.frame_id))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Camera calibration changed; restart this test");
      return;
    }
    try {
      const auto color = cv_bridge::toCvShare(rgb, sensor_msgs::image_encodings::BGR8);
      const auto raw = cv_bridge::toCvShare(depth);
      cv::Mat meters;
      if (raw->image.type() == CV_16UC1) raw->image.convertTo(meters, CV_32FC1, scale_);
      else if (raw->image.type() == CV_32FC1) meters = raw->image.clone();
      else throw std::runtime_error("Depth encoding must be 16UC1 or 32FC1");
      for (int row = 0; row < meters.rows; ++row) {
        auto * p = meters.ptr<float>(row);
        for (int col = 0; col < meters.cols; ++col)
          if (!std::isfinite(p[col]) || p[col] < 0) p[col] = 0;
      }
      if (!slam_) initialize(*info_, rgb->header.frame_id);
      cv::Mat rectified_rgb, rectified_depth;
      cv::remap(color->image, rectified_rgb, map_x_, map_y_, cv::INTER_LINEAR);
      // 深度使用最近邻，防止在物体边缘混合出不存在的深度。
      cv::remap(meters, rectified_depth, map_x_, map_y_, cv::INTER_NEAREST);
      last_stamp_ = time; have_stamp_ = true;
      const Sophus::SE3f camera_world = slam_->TrackRGBD(rectified_rgb, rectified_depth, time);
      std_msgs::msg::Int32 state; state.data = slam_->GetTrackingState(); state_pub_->publish(state);
      // 初始化/丢失期间不发布假位姿，也不修改传感器采集时间戳。
      if (state.data != 2) return;
      tracked_ = true;
      const Eigen::Isometry3f world_camera(camera_world.inverse().matrix());
      const Eigen::Isometry3f pose = base_camera_ * world_camera * base_camera_.inverse();
      if (!pose.matrix().allFinite()) return;
      publish(pose, rgb->header.stamp, time);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "RGB-D frame skipped: %s", e.what());
    }
  }

  void publish(const Eigen::Isometry3f & pose, const builtin_interfaces::msg::Time & stamp, double time)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = world_; msg.header.stamp = stamp;
    const auto t = pose.translation(); const Eigen::Quaternionf q(pose.rotation());
    msg.pose.position.x = t.x(); msg.pose.position.y = t.y(); msg.pose.position.z = t.z();
    msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
    pose_pub_->publish(msg);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = msg.header; tf.child_frame_id = child_;
      tf.transform.translation.x = t.x(); tf.transform.translation.y = t.y(); tf.transform.translation.z = t.z();
      tf.transform.rotation = msg.pose.orientation; broadcaster_->sendTransform(tf);
    }
    if (time - last_vis_ < 1.0 / vis_rate_) return;
    last_vis_ = time; path_.header = msg.header; path_.poses.push_back(msg);
    if (path_.poses.size() > size_t(path_size_)) path_.poses.erase(path_.poses.begin());
    path_pub_->publish(path_);
    // 这里只发布当前跟踪的稀疏地图点；全局地图由官方Atlas管理，不拼接假稠密地图。
    std::vector<Eigen::Vector3f> points;
    for (auto * point : slam_->GetTrackedMapPoints()) {
      if (!point || point->isBad()) continue;
      const Eigen::Vector3f p = base_camera_ * point->GetWorldPos();
      if (p.allFinite()) points.push_back(p);
    }
    sensor_msgs::msg::PointCloud2 cloud; cloud.header = msg.header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz"); modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto & p : points) { *x = p.x(); *y = p.y(); *z = p.z(); ++x; ++y; ++z; }
    cloud.is_dense = true; cloud_pub_->publish(cloud);
  }

  std::string vocabulary_, directory_, atlas_, base_, world_, child_;
  bool localization_, viewer_, publish_tf_, tracked_{false}, have_stamp_{false};
  double scale_, tolerance_, fps_, baseline_, close_depth_, vis_rate_, last_stamp_{0}, last_vis_{0};
  int features_, path_size_;
  Info::ConstSharedPtr info_;
  Info calibration_;
  Eigen::Isometry3f base_camera_;
  cv::Mat map_x_, map_y_;
  nav_msgs::msg::Path path_;
  message_filters::Subscriber<Image> rgb_, depth_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Subscription<Info>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<ORB_SLAM3::System> slam_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<RgbdSlam>()); }
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("orb_slam3"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
