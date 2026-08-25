#pragma once

// Perspective remapping engine (remediation Task 2.2).
//
// Pre-caches the per-pixel normalized camera rays of the downscaled
// perspective view and synthesizes, for any candidate rotation
// R_camera_lidar, the virtual LiDAR edge map through cv::remap:
//   p_lidar = R^T d_cam(u,v)
//   pan = atan2(X, Z);  tilt = -asin(Y / |p|)
//   u_map = (pan - pan_min)/(pan_max - pan_min) * (columns - 1)
//   v_map = (tilt_max - tilt)/(tilt_max - tilt_min) * (rows - 1)

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace auto_calib {

class PerspectiveRemapper {
public:
  PerspectiveRemapper() = default;
  PerspectiveRemapper(const Eigen::Matrix3d &k, cv::Size size,
                      double pan_min_rad, double pan_max_rad,
                      double tilt_min_rad, double tilt_max_rad, int pano_rows,
                      int pano_columns);

  // Synthesizes the virtual LiDAR edge view for R_camera_lidar. Pixels whose
  // ray leaves the panorama bounds are set to zero.
  void remap(const cv::Mat &pano_edge, const Eigen::Matrix3d &r_camera_lidar,
             cv::Mat &dst) const;

  cv::Size size() const { return size_; }

private:
  cv::Size size_;
  int pano_rows_ = 0, pano_columns_ = 0;
  double pan_min_rad_ = 0, pan_span_rad_ = 1.0;
  double tilt_min_rad_ = 0, tilt_max_rad_ = 0, tilt_span_rad_ = 1.0;
  cv::Mat ray_x_, ray_y_, ray_z_; // CV_32F per-pixel unit rays (camera frame)
};

} // namespace auto_calib
