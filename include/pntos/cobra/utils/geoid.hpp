// EGM96 geoid undulation from the compact 15-minute grid in data/egm96_15min.bin (tools/make_geoid.py
// converts NGA's WW15MGH.GRD). N = HAE - MSL, bilinearly interpolated; accuracy of the 15' grid is a
// few decimetres, well below the altitude measurements it corrects.
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace pntos::cobra::nav {

class Geoid {
 public:
  /// Loads the binary grid; nullopt (with the reason in `error` if given) when the file is missing or malformed.
  static std::optional<Geoid> load(const std::string& path, std::string* error = nullptr);
  /// Undulation N in metres at geodetic latitude/longitude in radians (longitude of any sign).
  double undulation(double lat_rad, double lon_rad) const;
  int rows() const { return rows_; }
  int cols() const { return cols_; }

 private:
  int rows_ = 0, cols_ = 0;
  double lat_top_ = 90, lon_left_ = 0, dlat_ = 0.25, dlon_ = 0.25;
  std::vector<std::int16_t> cm_;
};

/// The grid named by the environment variable PNTOS_GEOID_FILE (or data/egm96_15min.bin next to the
/// working directory), loaded once; nullptr if neither exists.
std::shared_ptr<const Geoid> default_geoid();

}  // namespace pntos::cobra::nav
