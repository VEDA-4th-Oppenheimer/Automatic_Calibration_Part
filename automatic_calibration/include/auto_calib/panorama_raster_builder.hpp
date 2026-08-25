#pragma once

// Seam-safe masked panorama raster generator (remediation Task 2.1).
//
// Builds the native organized range/valid rasters and multi-channel
// structural edges. Fake edges at invalid-cell boundaries are suppressed by
// masked finite differences; the 0/360 seam stays continuous through
// circular column indexing (BORDER_WRAP-equivalent).

#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace auto_calib {

struct PanoramaRaster {
  int rows = 0, columns = 0;
  double pan_min_rad = 0, pan_max_rad = 0;
  double tilt_min_rad = 0, tilt_max_rad = 0;
  cv::Mat range_m;            // CV_32F, 0 where invalid
  cv::Mat valid;              // CV_8U 255/0
  cv::Mat range_edge;         // CV_8U curvature-gated depth steps
  cv::Mat normal_edge;        // CV_8U surface-normal creases
  cv::Mat plane_intersection; // CV_8U planar patch boundaries
  double coverage = 0.0;
};

// Throws std::runtime_error on contract violations (shape, duplicates,
// missing cells). Requires the PAN_TILT JSON contract (frame.name=lidar_scan,
// right-handed +x right +y down +z forward).
PanoramaRaster buildPanoramaRaster(const nlohmann::json &root);

// Combined structural edge channel used for perspective matching.
cv::Mat combinedPanoramaEdge(const PanoramaRaster &raster);

} // namespace auto_calib
