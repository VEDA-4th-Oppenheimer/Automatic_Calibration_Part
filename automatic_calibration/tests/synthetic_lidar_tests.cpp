#include "auto_calib/synthetic_lidar.hpp"
#include <cmath>
#include <iostream>
#include <opencv2/core.hpp>
#include <stdexcept>
void near(double a, double b, double e, const char *n) {
  if (std::abs(a - b) > e)
    throw std::runtime_error(n);
}
int main() {
  try {
    cv::Mat d(3, 3, CV_16UC1, cv::Scalar(65535));
    d.at<std::uint16_t>(1, 1) = 1024;
    auto_calib::CameraModel c;
    c.k << 2, 0, 1, 0, 2, 1, 0, 0, 1;
    auto p = auto_calib::projectDepth(d, c);
    if (p.size() != 1)
      throw std::runtime_error("depth count");
    near(p[0].x(), 0, 1e-12, "x");
    near(p[0].y(), 0, 1e-12, "y");
    near(p[0].z(), 2, 1e-12, "depth scale");
    auto tf = auto_calib::makeTransform({.1, -.2, .3}, {.05, -.04, .03});
    Eigen::Vector3d q{1, 2, 3};
    near((tf.cameraToLidar(tf.lidarToCamera(q)) - q).norm(), 0, 1e-12,
         "transform roundtrip");
    auto_calib::ScanConfig s;
    s.rows = 3;
    s.columns = 3;
    s.pan_min = s.tilt_min = -.1;
    s.pan_max = s.tilt_max = .1;
    s.dropout = 0;
    s.noise_stddev = 0;
    std::vector<Eigen::Vector3d> source{{0, 0, 3}, {0, 0, 2}};
    auto scan = auto_calib::generateScan(source, {}, s);
    if (scan.points.size() != 9 || scan.valid_count != 1 ||
        !scan.points[4].valid())
      throw std::runtime_error("topology");
    near(scan.points[4].range, 2, 1e-6, "nearest return");
    std::cout << "All synthetic LiDAR tests passed.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Test failure: " << e.what() << '\n';
    return 1;
  }
}
