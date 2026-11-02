#pragma once
// N10：协议角度随采集顺序递增，ROS扫描方向为顺时针。
// 用相邻过零时刻计算圈周期，原始扫描保留采集顺序和逐点时间。
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <cstdint>
#include <string>
#include <utility>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace lslidar_driver {
class N10Scan {
    struct Ray { int angle; float range, intensity; };
    std::vector<Ray> rays_;
    int previous_ = -1;
    double start_ = 0.0, period_ = 0.1, last_time_ = 0.0;
    size_t bins_;
    std::string frame_;
    float min_, max_;
    bool collecting_ = false;
public:
    N10Scan(size_t bins, std::string frame, float min, float max)
        : bins_(bins), frame_(std::move(frame)), min_(min), max_(max) {}

    bool add(int angle, float range, float intensity, double time,
             sensor_msgs::msg::LaserScan& scan) {
        if (angle < 0 || angle >= 36000 || !std::isfinite(time)) return false;
        // 系统时间跳变后重新等待完整一圈，避免错误的扫描周期。
        if (last_time_ > 0.0 && (time < last_time_ - 0.02 || time > last_time_ + 1.0)) {
            collecting_ = false; previous_ = -1; rays_.clear();
        }
        last_time_ = time;
        bool ready = false;
        if (previous_ >= 0 && previous_ - angle > 18000) {
            const double boundary = time - angle * period_ / 36000.0;
            const double duration = boundary - start_;
            if (collecting_ && duration >= 0.04 && duration <= 0.30) {
                period_ = duration;
                scan = sensor_msgs::msg::LaserScan();
                scan.header.frame_id = frame_;
                const auto ns = static_cast<int64_t>(std::llround(start_ * 1e9));
                scan.header.stamp.sec = ns / 1000000000LL;
                scan.header.stamp.nanosec = ns % 1000000000LL;
                scan.angle_min = 0.0;
                scan.angle_increment = -2.0 * std::acos(-1.0) / bins_;
                scan.angle_max = (bins_ - 1) * scan.angle_increment;
                scan.scan_time = duration;
                scan.time_increment = duration / bins_;
                scan.range_min = min_; scan.range_max = max_;
                scan.ranges.assign(bins_, std::numeric_limits<float>::infinity());
                scan.intensities.assign(bins_, 0.0f);
                size_t valid = 0;
                for (const auto& ray : rays_) {
                    if (!std::isfinite(ray.range) || ray.range < min_ || ray.range > max_) continue;
                    const size_t bin = std::min(bins_ - 1,
                        static_cast<size_t>(std::lround(ray.angle * double(bins_) / 36000.0)));
                    if (ray.range < scan.ranges[bin]) {
                        if (!std::isfinite(scan.ranges[bin])) valid++;
                        scan.ranges[bin] = ray.range;
                        scan.intensities[bin] = ray.intensity;
                    }
                }
                ready = valid >= 50;
            }
            start_ = boundary;
            collecting_ = true;
            rays_.clear();
        } else if (previous_ >= 0 && angle < previous_) {
            // 重复/乱序数据不拼成一帧。
            collecting_ = false; rays_.clear();
        }
        previous_ = angle;
        if (collecting_ && (time - start_ > 0.5 || rays_.size() >= 4500)) {
            collecting_ = false; rays_.clear();  // 停转/异常角度不能无限积攒点。
        }
        if (collecting_) rays_.push_back({angle, range, intensity});
        return ready;
    }
};
}
