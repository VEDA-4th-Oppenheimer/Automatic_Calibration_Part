#include "auto_calib/panorama_orientation_analyzer.hpp"
#include <cassert>
#include <fstream>
#include <iostream>

int main() {
  assert(auto_calib::circularDistanceDeg(359.0, 0.0) == 1.0);
  const char *name = "/tmp/t2_panorama_test.json";
  std::ofstream f(name); f << R"({"scan":{"rows":2,"columns":4,"sample_count":8},"measurements":[
    {"row":0,"column":0,"valid":true,"distance_m":1},{"row":0,"column":1,"valid":true,"distance_m":1},{"row":0,"column":2,"valid":true,"distance_m":4},{"row":0,"column":3,"valid":true,"distance_m":1},
    {"row":1,"column":0,"valid":true,"distance_m":1},{"row":1,"column":1,"valid":true,"distance_m":1},{"row":1,"column":2,"valid":true,"distance_m":4},{"row":1,"column":3,"valid":true,"distance_m":1}]})"; f.close();
  auto r = auto_calib::analyzePanorama(name); assert(r.status == "PROPOSALS_READY"); assert(r.rows == 2 && r.columns == 4); assert(cv::countNonZero(r.valid) == 8);
  std::ofstream bad(name); bad << R"({"scan":{"rows":1,"columns":2,"sample_count":2},"measurements":[{"row":0,"column":0,"valid":true,"distance_m":1},{"row":0,"column":0,"valid":true,"distance_m":1}]})"; bad.close();
  r = auto_calib::analyzePanorama(name); assert(r.fallback_required); assert(r.status == "INVALID_INPUT"); std::cout << "panorama analyzer tests passed\n";
}
