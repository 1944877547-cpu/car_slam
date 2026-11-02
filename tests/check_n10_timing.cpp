// 离线验证N10整圈边界、采集方向、逐点时间和异常帧恢复。
#include <cassert>
#include <cmath>
#include <iostream>
#include "lslidar_driver/n10_scan.hpp"

int main() {
    lslidar_driver::N10Scan builder(450, "laser", 0.15f, 8.0f);
    sensor_msgs::msg::LaserScan scan;
    int frames = 0;
    for (int i = 0; i <= 1350; i++) {
        const int angle = i * 80 % 36000;
        const float range = 3.0f + std::cos(angle * std::acos(-1.0) / 18000.0);
        if (!builder.add(angle, range, float(i % 256), 1000.0 + i / 4500.0, scan)) continue;
        frames++;
        const double stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9;
        assert(std::abs(stamp - (1000.0 + frames * 0.1)) < 1e-6);
        assert(std::abs(scan.scan_time - 0.1) < 1e-6);
        assert(std::abs(scan.time_increment - 0.1 / 450) < 1e-8);
        assert(scan.angle_increment < 0 && scan.time_increment > 0);
        assert(scan.ranges.size() == 450);
        assert(std::abs(scan.angle_max - 449 * scan.angle_increment) < 1e-6);
        for (size_t j = 0; j < scan.ranges.size(); j++) {
            assert(std::isfinite(scan.ranges[j]));
            assert(std::abs(scan.ranges[j] - (3.0 + std::cos(j * 2*std::acos(-1.0)/450))) < 1e-5);
        }
    }
    assert(frames == 2);  // 初始半圈/不完整圈不发布。
    for (int i = 1351; i <= 1800; i++) {
        assert(!builder.add(i*80%36000, INFINITY, 0, 1000.0+i/4500.0, scan));
    }
    bool recovered = false;
    for (int i = 1801; i <= 2250; i++) {
        recovered = builder.add(i*80%36000, 2, 0, 1000.0+i/4500.0, scan) || recovered;
    }
    assert(recovered);
    assert(!builder.add(-1, 2, 0, 1001, scan));
    assert(!builder.add(40000, 2, 0, 1001, scan));
    std::cout << "N10 timing, clockwise acquisition, invalid-frame rejection and recovery passed.\n";
}
