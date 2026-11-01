// RF2O ROS 2 adapter. Original algorithm: MAPIRlab, GPL-3.0; see LICENSE/UPSTREAM.md.
#ifndef RF2O_LASER_ODOMETRY_NODE_HPP
#define RF2O_LASER_ODOMETRY_NODE_HPP
#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"
#include <chrono>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace rf2o {
class CLaserOdometry2DNode : public rclcpp::Node
{
public:
  CLaserOdometry2DNode();
  bool failed() const { return failed_; }
private:
  void scan(const sensor_msgs::msg::LaserScan::SharedPtr message);
  void process();
  bool laserTransform();
  void seed(const Pose3d& pose);
  void publish(const Eigen::Vector3d& velocity);
  CLaserOdometry2D core_;
  sensor_msgs::msg::LaserScan scan_;
  geometry_msgs::msg::Pose initial_pose_;
  std::string base_frame_, odom_frame_, scan_frame_;
  bool ready_{false}, pose_ready_{true}, publish_tf_, failed_{false};
  double frequency_, linear_variance_, angular_variance_, scan_timeout_, startup_timeout_;
  size_t width_{0};
  float angle_min_{0}, angle_increment_{0};
  std::chrono::steady_clock::time_point last_good_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr initial_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace rf2o
#endif
