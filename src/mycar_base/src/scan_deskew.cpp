// 二维扫描去畸变：陀螺仪插值积分 + 历史平面速度预测。
// /scan_raw保留采集时序；/scan中的点统一到最后一束的时刻。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ScanDeskew : public rclcpp::Node {
    using Scan = sensor_msgs::msg::LaserScan;
    struct Gyro { int64_t t; double w; };
    struct Velocity { int64_t t; double x, y; };
    struct Pending { Scan::ConstSharedPtr scan; std::chrono::steady_clock::time_point arrival; };
    std::deque<Gyro> gyro_;
    std::deque<Velocity> velocity_;
    std::deque<Pending> pending_;
    std::string base_;
    double wait_, gap_, max_age_, max_speed_;
    bool translation_;
    int64_t last_scan_ = 0;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    rclcpp::Publisher<Scan>::SharedPtr pub_;
    rclcpp::Subscription<Scan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool transform(const std::string& from, tf2::Transform& tf) {
        try {
            const auto value = buffer_.lookupTransform(base_, from, tf2::TimePointZero).transform;
            const auto& q = value.rotation;
            const auto& p = value.translation;
            const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
            if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.01 ||
                !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
            tf2::fromMsg(value, tf);
            return true;
        } catch (const tf2::TransformException&) { return false; }
    }

    void imu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        const int64_t t = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (t <= 0 || (!gyro_.empty() && t <= gyro_.back().t)) return;
        if (msg->angular_velocity_covariance[0] < 0) return;
        const auto& w = msg->angular_velocity;
        if (!std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z)) return;
        tf2::Transform tf;
        if (!transform(msg->header.frame_id, tf)) return;
        const double z = (tf.getBasis() * tf2::Vector3(w.x, w.y, w.z)).z();
        if (!std::isfinite(z) || std::abs(z) > 10.0) return;
        gyro_.push_back({t, z});
        while (gyro_.size() > 2 && gyro_.back().t - gyro_.front().t > 2000000000LL) gyro_.pop_front();
    }

    void odom(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const int64_t t = rclcpp::Time(msg->header.stamp).nanoseconds();
        const auto& v = msg->twist.twist.linear;
        if (t <= 0 || msg->child_frame_id != base_ || !std::isfinite(v.x) || !std::isfinite(v.y) ||
            std::hypot(v.x, v.y) > max_speed_ || (!velocity_.empty() && t <= velocity_.back().t)) return;
        velocity_.push_back({t, v.x, v.y});
        while (velocity_.size() > 2 && velocity_.back().t - velocity_.front().t > 2000000000LL) velocity_.pop_front();
    }

    void scan(Scan::ConstSharedPtr msg) {
        const int64_t t = rclcpp::Time(msg->header.stamp).nanoseconds();
        const double duration = (msg->ranges.size() - std::min<size_t>(1, msg->ranges.size())) *
                                double(msg->time_increment);
        if (t <= last_scan_ || t <= 0 || msg->ranges.size() < 64 || msg->header.frame_id.empty() ||
            !std::isfinite(msg->angle_min) || !std::isfinite(msg->angle_increment) ||
            msg->angle_increment == 0 || !std::isfinite(msg->scan_time) || msg->scan_time <= 0 ||
            !std::isfinite(duration) || duration <= 0 || duration > 0.30 ||
            !std::isfinite(msg->range_min) || !std::isfinite(msg->range_max) ||
            msg->range_min < 0 || msg->range_max <= msg->range_min) return;
        if (std::abs(msg->angle_increment) * msg->ranges.size() > 6.284) return;
        last_scan_ = t;
        if (pending_.size() >= 3) {
            pending_.pop_front();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Deskew queue full; dropping oldest scan");
        }
        pending_.push_back({msg, std::chrono::steady_clock::now()});
    }

    void process() {
        while (!pending_.empty()) {
            const auto& input = *pending_.front().scan;
            const int64_t start = rclcpp::Time(input.header.stamp).nanoseconds();
            const int64_t end = start + std::llround((input.ranges.size() - 1) * double(input.time_increment) * 1e9);
            tf2::Transform laser;
            if (gyro_.size() < 2 || gyro_.front().t > start || gyro_.back().t < end ||
                !transform(input.header.frame_id, laser)) {
                const double waited = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - pending_.front().arrival).count();
                if (waited < wait_) return;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "Deskew dropped scan: waiting for complete IMU history or static TF");
                pending_.pop_front();
                continue;
            }
            bool complete = true;
            for (size_t j = 1; j < gyro_.size(); j++) {
                if (gyro_[j].t >= start && gyro_[j-1].t <= end &&
                    (gyro_[j].t - gyro_[j-1].t) * 1e-9 > gap_) complete = false;
            }
            if (!complete) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Deskew dropped scan: IMU history has a gap");
                pending_.pop_front();
                continue;
            }
            double vx = 0, vy = 0;
            if (translation_) {
                // 只取扫描开始前的速度，不用本帧RF2O/EKF结果反过来补偿本帧。
                auto it = std::find_if(velocity_.rbegin(), velocity_.rend(),
                                      [start](const Velocity& v) { return v.t <= start; });
                if (it != velocity_.rend() && (start - it->t) * 1e-9 <= max_age_) {
                    vx = it->x; vy = it->y;
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                        "No recent historical velocity; applying IMU rotation compensation only");
                }
            }
            publish(input, laser, start, end, vx, vy);
            pending_.pop_front();
        }
    }

    void publish(const Scan& input, const tf2::Transform& laser, int64_t start, int64_t end,
                 double vx, double vy) {
        const size_t n = input.ranges.size();
        std::vector<double> yaw(n, 0), x(n, 0), y(n, 0), w(n);
        size_t j = 0;
        for (size_t i = 0; i < n; i++) {
            const int64_t t = start + std::llround(i * double(input.time_increment) * 1e9);
            while (j + 1 < gyro_.size() && gyro_[j+1].t < t) j++;
            const auto& a = gyro_[j]; const auto& b = gyro_[j+1];
            const double u = double(t - a.t) / double(b.t - a.t);
            w[i] = a.w + u * (b.w - a.w);
            if (i == 0) continue;
            const double dt = input.time_increment;
            yaw[i] = yaw[i-1] + 0.5 * (w[i-1] + w[i]) * dt;
            const double middle = 0.5 * (yaw[i-1] + yaw[i]);
            x[i] = x[i-1] + (std::cos(middle)*vx - std::sin(middle)*vy) * dt;
            y[i] = y[i-1] + (std::sin(middle)*vx + std::cos(middle)*vy) * dt;
        }
        Scan output = input;
        output.header.stamp = rclcpp::Time(end);
        output.angle_min = -std::acos(-1.0);
        output.angle_increment = 2.0 * std::acos(-1.0) / n;
        output.angle_max = output.angle_min + (n - 1) * output.angle_increment;
        // 已统一到一个时刻，设0避免Cartographer等下游重复运动补偿。
        output.time_increment = 0.0;
        output.ranges.assign(n, std::numeric_limits<float>::infinity());
        output.intensities.assign(n, 0.0f);
        const auto inverse = laser.inverse();
        const double c = std::cos(yaw.back()), s = std::sin(yaw.back());
        for (size_t i = 0; i < n; i++) {
            const double r = input.ranges[i];
            if (!std::isfinite(r) || r < input.range_min || r > input.range_max) continue;
            const double a = input.angle_min + i * double(input.angle_increment);
            const auto p = laser * tf2::Vector3(r*std::cos(a), r*std::sin(a), 0);
            const double px = std::cos(yaw[i])*p.x() - std::sin(yaw[i])*p.y() + x[i] - x.back();
            const double py = std::sin(yaw[i])*p.x() + std::cos(yaw[i])*p.y() + y[i] - y.back();
            const auto q = inverse * tf2::Vector3(c*px + s*py, -s*px + c*py, p.z());
            const double distance = std::hypot(q.x(), q.y());
            if (distance < input.range_min || distance > input.range_max) continue;
            const double angle = std::atan2(q.y(), q.x());
            auto bin = static_cast<long>(std::lround((angle - output.angle_min) / output.angle_increment));
            bin = (bin % long(n) + long(n)) % long(n);
            if (distance < output.ranges[bin]) {
                output.ranges[bin] = distance;
                if (input.intensities.size() == n) output.intensities[bin] = input.intensities[i];
            }
        }
        pub_->publish(output);
    }

public:
    ScanDeskew() : Node("scan_deskew"), buffer_(get_clock()), listener_(buffer_) {
        base_ = declare_parameter<std::string>("base_frame", "base_link");
        wait_ = declare_parameter<double>("wait_timeout", 0.25);
        gap_ = declare_parameter<double>("imu_max_gap", 0.12);
        max_age_ = declare_parameter<double>("velocity_max_age", 0.30);
        max_speed_ = declare_parameter<double>("max_linear_speed", 1.0);
        translation_ = declare_parameter<bool>("use_translation", true);
        for (double v : {wait_, gap_, max_age_, max_speed_})
            if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Deskew limits must be positive");
        const auto in = declare_parameter<std::string>("input_topic", "/scan_raw");
        const auto out = declare_parameter<std::string>("output_topic", "/scan");
        if (in == out) throw std::invalid_argument("Raw and deskewed scan topics must differ");
        pub_ = create_publisher<Scan>(out, rclcpp::QoS(10));
        scan_sub_ = create_subscription<Scan>(in, rclcpp::SensorDataQoS(),
            std::bind(&ScanDeskew::scan, this, std::placeholders::_1));
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            declare_parameter<std::string>("imu_topic", "/imu/data"), rclcpp::SensorDataQoS(),
            std::bind(&ScanDeskew::imu, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            declare_parameter<std::string>("odom_topic", "/odom"), rclcpp::QoS(100),
            std::bind(&ScanDeskew::odom, this, std::placeholders::_1));
        timer_ = create_wall_timer(std::chrono::milliseconds(5), std::bind(&ScanDeskew::process, this));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanDeskew>());
    rclcpp::shutdown();
    return 0;
}
