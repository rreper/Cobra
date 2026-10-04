#include <pntos/cobra/utils/geoid.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>

namespace pntos::cobra::nav {

std::optional<Geoid> Geoid::load(const std::string& path, std::string* error) {
  auto fail = [&](const std::string& why) {
    if (error) *error = path + ": " + why;
    return std::optional<Geoid>{};
  };
  std::ifstream in(path, std::ios::binary);
  if (!in) return fail("cannot open");
  char magic[8];
  in.read(magic, 8);
  if (!in || std::memcmp(magic, "EGM96I16", 8) != 0) return fail("not an EGM96I16 grid");
  Geoid g;
  std::int32_t rows = 0, cols = 0;
  in.read(reinterpret_cast<char*>(&rows), 4);
  in.read(reinterpret_cast<char*>(&cols), 4);
  in.read(reinterpret_cast<char*>(&g.lat_top_), 8);
  in.read(reinterpret_cast<char*>(&g.lon_left_), 8);
  in.read(reinterpret_cast<char*>(&g.dlat_), 8);
  in.read(reinterpret_cast<char*>(&g.dlon_), 8);
  if (!in || rows < 2 || cols < 2 || rows > 100000 || cols > 100000 || g.dlat_ <= 0 || g.dlon_ <= 0) return fail("bad header");
  g.rows_ = rows;
  g.cols_ = cols;
  g.cm_.resize(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols));
  in.read(reinterpret_cast<char*>(g.cm_.data()), static_cast<std::streamsize>(g.cm_.size() * 2));
  if (!in) return fail("truncated");
  return g;
}

double Geoid::undulation(double lat_rad, double lon_rad) const {
  const double lat = lat_rad * 180.0 / M_PI;
  double lon = lon_rad * 180.0 / M_PI;
  lon = std::fmod(lon - lon_left_, 360.0);
  if (lon < 0) lon += 360.0;
  double r = (lat_top_ - lat) / dlat_;  // row 0 = northern edge
  double c = lon / dlon_;
  r = std::clamp(r, 0.0, static_cast<double>(rows_ - 1));
  const int r0 = std::min(static_cast<int>(std::floor(r)), rows_ - 2);
  const int c0 = static_cast<int>(std::floor(c)) % cols_;
  const int c1 = (c0 + 1) % cols_;
  const double fr = r - r0, fc = c - std::floor(c);
  auto at = [&](int rr, int cc) { return cm_[static_cast<std::size_t>(rr) * static_cast<std::size_t>(cols_) + static_cast<std::size_t>(cc)] * 0.01; };
  const double top = at(r0, c0) * (1 - fc) + at(r0, c1) * fc;
  const double bot = at(r0 + 1, c0) * (1 - fc) + at(r0 + 1, c1) * fc;
  return top * (1 - fr) + bot * fr;
}

std::shared_ptr<const Geoid> default_geoid() {
  static std::once_flag once;
  static std::shared_ptr<const Geoid> cached;
  std::call_once(once, [] {
    std::vector<std::string> candidates;
    if (const char* env = std::getenv("PNTOS_GEOID_FILE")) candidates.emplace_back(env);
    candidates.emplace_back("data/egm96_15min.bin");
    for (const auto& p : candidates)
      if (auto g = Geoid::load(p)) {
        cached = std::make_shared<const Geoid>(std::move(*g));
        return;
      }
  });
  return cached;
}

}  // namespace pntos::cobra::nav
