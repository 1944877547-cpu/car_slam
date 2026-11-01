/**
 * RF2O: Planar Odometry from a Radial Laser Scanner, ICRA 2016.
 * Original authors: Mariano Jaimez, Javier G. Monroy, MAPIR group.
 * Source: https://github.com/MAPIRlab/rf2o_laser_odometry (ros2 branch).
 * car_swarm ROS adapter changes are described in UPSTREAM.md. GPL-3.0.
 */
#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"
#include <cmath>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rf2o {
CLaserOdometry2DNode::CLaserOdometry2DNode() : Node("rf2o_laser_odometry")
{
  const auto input = declare_parameter<std::string>("laser_scan_topic", "/scan");
  const auto output = declare_parameter<std::string>("odom_topic", "/odom_lidar");
  base_frame_ = declare_parameter<std::string>("base_frame_id", "base_link");
  odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
  publish_tf_ = declare_parameter<bool>("publish_tf", false);
  frequency_ = declare_parameter<double>("freq", 20.0);
  linear_variance_ = declare_parameter<double>("linear_variance", 0.0025);
  angular_variance_ = declare_parameter<double>("angular_variance", 0.01);
  scan_timeout_ = declare_parameter<double>("scan_timeout", 1.0);
  startup_timeout_ = declare_parameter<double>("startup_timeout", 30.0);
  for (double v : {frequency_, linear_variance_, angular_variance_, scan_timeout_, startup_timeout_}) {
    if (!std::isfinite(v) || v <= 0.0) throw std::invalid_argument("RF2O parameters must be positive");
  }
  initial_pose_.orientation.w = 1.0;
  const auto initial = declare_parameter<std::string>("init_pose_from_topic", "");
  if (!initial.empty()) {
    pose_ready_ = false;
    initial_sub_ = create_subscription<nav_msgs::msg::Odometry>(initial, 1,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        if (!pose_ready_) {
          const auto& q = msg->pose.pose.orientation;
          const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
          if (std::isfinite(norm) && std::abs(norm - 1.0) < 0.01 &&
              std::isfinite(msg->pose.pose.position.x) && std::isfinite(msg->pose.pose.position.y)) {
            initial_pose_ = msg->pose.pose;
            pose_ready_ = true;
          }
        }
      });
  }
  buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  if (publish_tf_) broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  publisher_ = create_publisher<nav_msgs::msg::Odometry>(output, 10);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(input, rclcpp::SensorDataQoS(),
    std::bind(&CLaserOdometry2DNode::scan, this, std::placeholders::_1));
  last_good_ = std::chrono::steady_clock::now();
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / frequency_)), std::bind(&CLaserOdometry2DNode::process, this));
  RCLCPP_INFO(get_logger(), "Laser odometry: %s -> %s, publish_tf=%s",
              input.c_str(), output.c_str(), publish_tf_ ? "true" : "false");
}

void CLaserOdometry2DNode::scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  // 空帧、无效时间、无效量程不能进入匹配，避免NaN和错误速度。
  const auto& m = *message;
  if (!pose_ready_ || m.header.frame_id.empty() || m.ranges.size() < 64 ||
      rclcpp::Time(m.header.stamp).nanoseconds() <= 0 ||
      !std::isfinite(m.angle_min) || !std::isfinite(m.angle_increment) || m.angle_increment <= 0 ||
      !std::isfinite(m.range_min) || !std::isfinite(m.range_max) ||
      m.range_min < 0 || m.range_max <= m.range_min) return;
  if (!scan_.ranges.empty() && rclcpp::Time(m.header.stamp) <= rclcpp::Time(scan_.header.stamp)) return;
  auto clean = m;
  size_t valid = 0;
  for (auto& value : clean.ranges) {
    if (std::isfinite(value) && value >= m.range_min && value <= m.range_max && value > 0) valid++;
    else value = 0.0f;
  }
  if (valid < 30) return;
  // 用采样点和角度步长确定实际视场，兼容N10的angle_max端点写法。
  clean.angle_max = clean.angle_min + (clean.ranges.size() - 1) * clean.angle_increment;
  if (!std::isfinite(clean.angle_max) || clean.angle_max - clean.angle_min > 6.284) return;
  scan_ = std::move(clean);
  ready_ = true;
}

bool CLaserOdometry2DNode::laserTransform()
{
  try {
    const auto tf = buffer_->lookupTransform(base_frame_, scan_.header.frame_id, tf2::TimePointZero);
    const auto& q = tf.transform.rotation;
    Pose3d pose = Pose3d::Identity();
    pose.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    pose.translation() << tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z;
    // RF2O内部角度以视场中心为0，补偿实际LaserScan的角度中心。
    Pose3d center = Pose3d::Identity();
    center.linear() = matrixYaw(double((scan_.angle_min + scan_.angle_max) * 0.5));
    core_.setLaserPose(pose * center);
    return true;
  } catch (const tf2::TransformException& error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for laser TF: %s", error.what());
    return false;
  }
}

void CLaserOdometry2DNode::seed(const Pose3d& pose)
{
  geometry_msgs::msg::Pose p;
  p.position.x = pose.translation().x();
  p.position.y = pose.translation().y();
  const Eigen::Quaterniond q(pose.rotation());
  p.orientation.w = q.w(); p.orientation.x = q.x();
  p.orientation.y = q.y(); p.orientation.z = q.z();
  core_.init(scan_, p);
  // 首帧用于建立参考，不把第一段运动丢掉。
  core_.range_wf = Eigen::Map<const Eigen::MatrixXf>(scan_.ranges.data(), scan_.ranges.size(), 1);
  core_.createImagePyramid();
  width_ = scan_.ranges.size();
  scan_frame_ = scan_.header.frame_id;
  angle_min_ = scan_.angle_min;
  angle_increment_ = scan_.angle_increment;
}

void CLaserOdometry2DNode::process()
{
  const auto now = std::chrono::steady_clock::now();
  const double silence = std::chrono::duration<double>(now - last_good_).count();
  if (silence > (core_.is_initialized() ? scan_timeout_ : startup_timeout_)) {
    RCLCPP_ERROR(get_logger(), "No valid laser odometry for %.1f s; stopping instead of using command estimates", silence);
    failed_ = true;
    rclcpp::shutdown();
    return;
  }
  if (!ready_) return;
  ready_ = false;
  if (!core_.is_initialized()) {
    if (!laserTransform()) return;
    Pose3d pose = Pose3d::Identity();
    const auto& q = initial_pose_.orientation;
    pose.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    pose.translation().x() = initial_pose_.position.x;
    pose.translation().y() = initial_pose_.position.y;
    seed(pose);
    last_good_ = now;
    return;
  }
  const Pose3d old_pose = core_.robot_pose_;
  if (width_ != scan_.ranges.size() || scan_frame_ != scan_.header.frame_id ||
      angle_min_ != scan_.angle_min || angle_increment_ != scan_.angle_increment) {
    if (laserTransform()) seed(old_pose);
    return;
  }
  const double dt = (rclcpp::Time(scan_.header.stamp) - core_.last_odom_time).seconds();
  if (dt <= 0.0001 || dt > scan_timeout_) {
    seed(old_pose);
    return;
  }
  core_.current_scan_time = scan_.header.stamp;
  if (!core_.odometryCalculation(scan_) || !core_.robot_pose_.matrix().allFinite()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Laser match failed; skipping this measurement");
    seed(old_pose);
    return;
  }
  // 速度属于base_link；使用车体位姿增量，补偿雷达前后/左右安装偏移。
  const Pose3d delta = old_pose.inverse() * core_.robot_pose_;
  const Eigen::Vector3d velocity(delta.translation().x()/dt, delta.translation().y()/dt,
                                getYaw(delta.rotation())/dt);
  if (!velocity.allFinite() || velocity.head<2>().norm() > 2.0 || std::abs(velocity.z()) > 6.0) {
    seed(old_pose);
    return;
  }
  publish(velocity);
  last_good_ = now;
}

void CLaserOdometry2DNode::publish(const Eigen::Vector3d& velocity)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = core_.last_odom_time;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = core_.robot_pose_.translation().x();
  odom.pose.pose.position.y = core_.robot_pose_.translation().y();
  tf2::Quaternion q;
  q.setRPY(0, 0, getYaw(core_.robot_pose_.rotation()));
  odom.pose.pose.orientation = tf2::toMsg(q);
  odom.twist.twist.linear.x = velocity.x();
  odom.twist.twist.linear.y = velocity.y();
  odom.twist.twist.angular.z = velocity.z();
  // 这些是可调初值，不把未知精度写成0；EKF只取速度，不重复融合位置。
  for (size_t axis = 0; axis < 6; axis++) {
    odom.pose.covariance[axis*7] = axis < 2 ? 0.05 : axis == 5 ? 0.1 : 1e6;
    odom.twist.covariance[axis*7] = axis < 2 ? linear_variance_ : axis == 5 ? angular_variance_ : 1e6;
  }
  publisher_->publish(odom);
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.rotation = odom.pose.pose.orientation;
    broadcaster_->sendTransform(tf);
  }
}
}  // namespace rf2o

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rf2o::CLaserOdometry2DNode>();
  rclcpp::spin(node);
  const bool failed = node->failed();
  rclcpp::shutdown();
  return failed ? 1 : 0;
}
