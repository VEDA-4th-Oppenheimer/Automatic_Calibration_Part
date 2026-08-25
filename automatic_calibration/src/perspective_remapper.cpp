#include "auto_calib/perspective_remapper.hpp"

#include <Eigen/LU>
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace auto_calib {

PerspectiveRemapper::PerspectiveRemapper(const Eigen::Matrix3d &k,
                                         cv::Size size, double pan_min_rad,
                                         double pan_max_rad,
                                         double tilt_min_rad,
                                         double tilt_max_rad, int pano_rows,
                                         int pano_columns)
    : size_(size), pano_rows_(pano_rows), pano_columns_(pano_columns),
      pan_min_rad_(pan_min_rad), pan_span_rad_(pan_max_rad - pan_min_rad),
      tilt_min_rad_(tilt_min_rad), tilt_max_rad_(tilt_max_rad),
      tilt_span_rad_(tilt_max_rad - tilt_min_rad) {
  ray_x_ = cv::Mat(size, CV_32F);
  ray_y_ = cv::Mat(size, CV_32F);
  ray_z_ = cv::Mat(size, CV_32F);
  const Eigen::Matrix3d k_inv = k.inverse();
  for (int v = 0; v < size.height; ++v)
    for (int u = 0; u < size.width; ++u) {
      const Eigen::Vector3d d = k_inv * Eigen::Vector3d(u, v, 1.0);
      const double norm = d.norm();
      ray_x_.at<float>(v, u) = static_cast<float>(d.x() / norm);
      ray_y_.at<float>(v, u) = static_cast<float>(d.y() / norm);
      ray_z_.at<float>(v, u) = static_cast<float>(d.z() / norm);
    }
}

void PerspectiveRemapper::remap(const cv::Mat &pano_edge,
                                const Eigen::Matrix3d &r_camera_lidar,
                                cv::Mat &dst) const {
  const Eigen::Matrix3d r_t = r_camera_lidar.transpose();
  const double two_pi = 2.0 * std::acos(-1.0);
  cv::Mat map_x(size_, CV_32F), map_y(size_, CV_32F);
  for (int v = 0; v < size_.height; ++v)
    for (int u = 0; u < size_.width; ++u) {
      const Eigen::Vector3d d(
          ray_x_.at<float>(v, u), ray_y_.at<float>(v, u),
          ray_z_.at<float>(v, u));
      const Eigen::Vector3d p = r_t * d;
      double pan = std::atan2(p.x(), p.z());
      // Wrap into [pan_min, pan_min + 2pi) then reject outside coverage.
      while (pan < pan_min_rad_)
        pan += two_pi;
      while (pan >= pan_min_rad_ + two_pi)
        pan -= two_pi;
      const double norm = p.norm();
      const double tilt =
          -std::asin(std::clamp(p.y() / norm, -1.0, 1.0));
      const double mu = pan_span_rad_ > 1e-9
                            ? (pan - pan_min_rad_) / pan_span_rad_ *
                                  (pano_columns_ - 1)
                            : -1.0;
      const double mv = tilt_span_rad_ > 1e-9
                            ? (tilt_max_rad_ - tilt) / tilt_span_rad_ *
                                  (pano_rows_ - 1)
                            : -1.0;
      if (mu < 0.0f || mu > pano_columns_ - 1 || mv < 0.0f ||
          mv > pano_rows_ - 1) {
        map_x.at<float>(v, u) = -1.0f;
        map_y.at<float>(v, u) = -1.0f;
      } else {
        map_x.at<float>(v, u) = static_cast<float>(mu);
        map_y.at<float>(v, u) = static_cast<float>(mv);
      }
    }
  cv::remap(pano_edge, dst, map_x, map_y, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0));
}

} // namespace auto_calib
