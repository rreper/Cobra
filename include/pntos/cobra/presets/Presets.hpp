// Named sensor presets: IMU error models and GNSS receiver profiles selectable by name from a config
// file (`"imu_model": {"preset": "stim300"}`). The VN-100 entry is the model Cobra tuned on the example
// dataset; the others are converted from manufacturer datasheets with imu_from_datasheet() and are
// starting points to be validated on data (roadmap Phase 2), not tuned values.
#pragma once

#include <pntos/cobra/config/configs.hpp>

#include <optional>
#include <string>
#include <vector>

namespace pntos::cobra::presets {

/// Datasheet figures in the units manufacturers publish them in.
struct ImuDatasheet {
  double gyro_arw_deg_sqrt_h = 0;          ///< angular random walk, deg/sqrt(h)
  double gyro_bias_instability_deg_h = 0;  ///< in-run bias stability, deg/h
  double gyro_turn_on_bias_deg_h = 0;      ///< turn-on bias repeatability, deg/h (initial sigma)
  double accel_vrw_mps_sqrt_h = 0;         ///< velocity random walk, m/s/sqrt(h)
  double accel_bias_instability_mg = 0;    ///< in-run bias stability, mg
  double accel_turn_on_bias_mg = 0;        ///< turn-on bias repeatability, mg (initial sigma)
  double bias_tau_s = 500.0;               ///< FOGM time constant used for both sensors
};

/// Converts datasheet figures to Cobra's ImuConfig units (m/s², rad/s, m/s^1.5, rad/sqrt(s)).
ImuConfig imu_from_datasheet(const ImuDatasheet& d, const std::string& group);

struct ImuPreset {
  std::string name;
  std::string description;
  std::string source;  ///< "cobra-tuned" or the datasheet the figures came from
  ImuConfig config;
};

const std::vector<ImuPreset>& imu_presets();
std::vector<std::string> imu_preset_names();
/// The preset's ImuConfig with `group` set; nullopt for an unknown name.
std::optional<ImuConfig> imu_preset(const std::string& name, const std::string& group);
/// Name of the preset whose six model fields (bias sigma/tau, random walk for both sensors) equal `c`'s;
/// the initial sigmas are not compared. Empty if none matches.
std::string matching_imu_preset(const ImuConfig& c);
/// The preset to use for the other Pinson-Q mode ("vn100" <-> "vn100_corrected"); `name` itself otherwise.
std::string imu_preset_for_mode(const std::string& name, bool legacy_q_rotation);

/// What a receiver emits and how Cobra's apps treat it.
struct GnssPreset {
  std::string name;
  std::string description;
  bool position = true;   ///< MeasurementPosition stream
  bool velocity = false;  ///< MeasurementVelocity stream
  bool pva = false;       ///< MeasurementPositionVelocityAttitude stream
  double time_bias_sec = 0;  ///< latency subtracted by a time-bias preprocessor
  std::array<double, 3> position_fogm_sigma{1.5, 1.5, 2.0};  ///< NED position sensor-error FOGM sigma, m
  std::array<double, 3> position_fogm_tau{300.0, 300.0, 200.0};  ///< s
};

const std::vector<GnssPreset>& gnss_presets();
std::vector<std::string> gnss_preset_names();
std::optional<GnssPreset> gnss_preset(const std::string& name);

}  // namespace pntos::cobra::presets
