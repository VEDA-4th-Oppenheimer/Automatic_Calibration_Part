#include "auto_calib/synthetic_lidar.hpp"
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
namespace {
using Args = std::unordered_map<std::string, std::string>;
constexpr double pi = 3.14159265358979323846;
double rad(double d) { return d * pi / 180; }
auto args(int argc, char **argv) {
  std::unordered_map<std::string, std::string> out;
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    if (k == "--help") {
      out[k] = "";
      continue;
    }
    if (k.rfind("--", 0) != 0 || i + 1 >= argc)
      throw std::invalid_argument("Use --key value arguments");
    out[k] = argv[++i];
  }
  return out;
}
double dval(const Args &a, const std::string &k, double d) {
  auto i = a.find(k);
  return i == a.end() ? d : std::stod(i->second);
}
std::uint32_t uval(const Args &a, const std::string &k, std::uint32_t d) {
  auto i = a.find(k);
  return i == a.end() ? d : std::stoul(i->second);
}
void usage() {
  std::cout
      << "generate_synthetic_scan --dataset-root PATH --output PATH "
         "[--frame-id ID] [--columns N --rows N --pixel-stride N] "
         "[--pan-min-deg D --pan-max-deg D --tilt-min-deg D --tilt-max-deg D] "
         "[--tx-m M --ty-m M --tz-m M --roll-deg D --pitch-deg D --yaw-deg D] "
         "[--noise-stddev-m M --dropout P --seed N]\n";
}
} // namespace
int main(int argc, char **argv) {
  try {
    auto a = args(argc, argv);
    if (a.count("--help")) {
      usage();
      return 0;
    }
    if (!a.count("--dataset-root") || !a.count("--output")) {
      usage();
      return 2;
    }
    std::optional<std::string> id;
    if (a.count("--frame-id"))
      id = a.at("--frame-id");
    auto_calib::ScanConfig c;
    c.columns = uval(a, "--columns", 321);
    c.rows = uval(a, "--rows", 121);
    c.pixel_stride = uval(a, "--pixel-stride", 2);
    c.pan_min = rad(dval(a, "--pan-min-deg", -40));
    c.pan_max = rad(dval(a, "--pan-max-deg", 40));
    c.tilt_min = rad(dval(a, "--tilt-min-deg", -25));
    c.tilt_max = rad(dval(a, "--tilt-max-deg", 25));
    c.noise_stddev = dval(a, "--noise-stddev-m", 0.005);
    c.dropout = dval(a, "--dropout", 0.01);
    c.seed = uval(a, "--seed", 7);
    Eigen::Vector3d t{dval(a, "--tx-m", 0.15), dval(a, "--ty-m", -0.02),
                      dval(a, "--tz-m", 0.08)},
        rpy{rad(dval(a, "--roll-deg", 2)), rad(dval(a, "--pitch-deg", -4)),
            rad(dval(a, "--yaw-deg", 6))};
    auto tf = auto_calib::makeTransform(t, rpy);
    auto f = auto_calib::loadStanfordFrame(a.at("--dataset-root"), id);
    auto points = auto_calib::projectDepth(f.depth, f.camera, c.pixel_stride);
    auto scan = auto_calib::generateScan(points, tf, c);
    auto_calib::writePackage(a.at("--output"), f, scan, tf);
    std::cout << "frame_id: " << f.id
              << "\nsource_points: " << scan.source_count
              << "\nvalid_scan_points: " << scan.valid_count << " / "
              << scan.points.size() << "\noutput: " << a.at("--output") << '\n';
    return scan.valid_count ? 0 : 3;
  } catch (const std::exception &e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
}
