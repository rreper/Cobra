#include <pntos/cobra/presets/Presets.hpp>

#include <cmath>

namespace pntos::cobra::presets {

namespace {
constexpr double kDeg = M_PI / 180.0;
constexpr double kG = 9.80665;

ImuConfig vn100_cobra() {  // the values every Cobra app ships with (tuned on the example dataset)
  ImuConfig m;
  m.accel_bias_sigma = {2.4e-3, 2.4e-3, 2.4e-3};
  m.accel_bias_tau = {300.0, 300.0, 300.0};
  m.accel_random_walk_sigma = {3.887e-6, 3.887e-6, 3.887e-6};
  m.gyro_bias_sigma = {2e-4, 2e-4, 2e-4};
  m.gyro_bias_tau = {500.0, 500.0, 500.0};
  m.gyro_random_walk_sigma = {9.9e-4, 9.9e-4, 6.7e-5};
  m.accel_bias_initial_sigma = {0.072, 0.072, 0.072};
  m.gyro_bias_initial_sigma = {0.003, 0.003, 0.003};
  return m;
}

std::vector<ImuPreset> make_presets() {
  std::vector<ImuPreset> v;
  v.push_back({"vn100", "VectorNav VN-100 as tuned by Cobra for the example dataset; pairs with the legacy (Python) Pinson-Q rotation",
               "cobra-tuned", vn100_cobra()});
  {
    // Same model retuned for the corrected Pinson-Q rotation: the yaw gyro random walk is raised from
    // 6.7e-5 to 6.0e-4 rad/sqrt(s) so that 68 % of the yaw errors fall inside one sigma on the example
    // dataset (sweep in docs/DESIGN.md section 9.5). Pairs with legacy_q_rotation = false.
    ImuConfig c = vn100_cobra();
    c.gyro_random_walk_sigma[2] = 6.0e-4;
    v.push_back({"vn100_corrected", "VectorNav VN-100, Cobra tuning with the yaw gyro random walk retuned for the corrected Pinson-Q rotation",
                 "cobra-tuned, retuned 2026-10-04", c});
  }
  {
    // Measured on the recorded dataset (tools/imu_allan.py, 2026-10-06): Allan deviation over the 128 s static
    // start gives a gyro white noise of 0.5-1.2e-4 rad/s/sqrt(Hz) and an accelerometer white noise of 2-2.7e-3
    // m/s^2/sqrt(Hz) (engine vibration included); the Allan floors at 20-30 s put the bias instabilities near
    // 1-2e-5 rad/s and 3-6e-4 m/s^2, and the bias wander between the thirteen stops of the drive stays below
    // 1.5e-4 rad/s. The Cobra tuning above is about ten times this on the gyro white noise and four times on
    // the bias sigmas; it absorbs installation effects the measurement does not show. Initial sigmas unchanged.
    ImuConfig c = vn100_cobra();
    c.accel_random_walk_sigma = {2.4e-3, 2.4e-3, 2.4e-3};
    c.gyro_random_walk_sigma = {1.0e-4, 1.0e-4, 5.0e-5};
    c.accel_bias_sigma = {6.0e-4, 6.0e-4, 6.0e-4};
    c.gyro_bias_sigma = {5.0e-5, 5.0e-5, 5.0e-5};
    v.push_back({"vn100_measured", "VectorNav VN-100 as measured on the example dataset (Allan deviation of the static start, bias wander across stops)",
                 "tools/imu_allan.py on cobra_gps_ins_example_data.log, 2026-10-06", c});
  }
  // Datasheet-derived starting points. Figures are the manufacturers' typical values.
  v.push_back({"vn100_datasheet", "VectorNav VN-100 from the datasheet (industrial MEMS)", "VN-100 datasheet rev. 2.x",
               imu_from_datasheet({0.21, 10.0, 0.5 * 3600 * 0 + 36.0, 0.14 * kG * 1e-3 * 60, 0.04, 2.0, 500.0}, "")});
  v.push_back({"stim300", "Safran Sensing STIM300 (tactical MEMS)", "STIM300 datasheet",
               imu_from_datasheet({0.15, 0.5, 4.0, 0.07, 0.05, 0.75, 500.0}, "")});
  v.push_back({"adis16488", "Analog Devices ADIS16488A (industrial MEMS)", "ADIS16488A datasheet",
               imu_from_datasheet({0.3, 6.25, 72.0, 0.029, 0.1, 16.0, 500.0}, "")});
  v.push_back({"hg1700", "Honeywell HG1700 AG58 (tactical RLG)", "HG1700 datasheet",
               imu_from_datasheet({0.125, 1.0, 1.0, 0.198, 1.0, 1.0, 1000.0}, "")});
  v.push_back({"hg4930", "Honeywell HG4930 (tactical MEMS)", "HG4930 datasheet",
               imu_from_datasheet({0.04, 0.25, 7.0, 0.03, 0.025, 0.5, 1000.0}, "")});
  v.push_back({"consumer_mems", "Consumer-grade MEMS (ICM-20948 / BMI088 class)", "ICM-20948 and BMI088 datasheets",
               imu_from_datasheet({0.9, 50.0, 1800.0, 0.23 * kG * 1e-3 * 60, 1.0, 40.0, 300.0}, "")});
  return v;
}

std::vector<GnssPreset> make_gnss() {
  std::vector<GnssPreset> v;
  v.push_back({"ublox_f9", "u-blox ZED-F9 series: position, velocity and PVA streams; 150 ms latency as in the Cobra apps",
               true, true, true, 0.15});
  v.push_back({"ublox_m8", "u-blox NEO/ZED-M8 series: position and velocity streams", true, true, false, 0.10});
  v.push_back({"novatel_oem7", "NovAtel OEM7: PVA stream (BESTPOS/BESTVEL fused upstream)", true, true, true, 0.05});
  v.push_back({"septentrio_mosaic", "Septentrio mosaic-X5: position and velocity streams", true, true, false, 0.05});
  v.push_back({"generic_position", "Any receiver providing geodetic position only", true, false, false, 0.0});
  return v;
}
}  // namespace

ImuConfig imu_from_datasheet(const ImuDatasheet& d, const std::string& group) {
  ImuConfig m;
  m.group_ = group;
  const double arw = d.gyro_arw_deg_sqrt_h * kDeg / 60.0;             // deg/sqrt(h) -> rad/sqrt(s)
  const double gbias = d.gyro_bias_instability_deg_h * kDeg / 3600.0;  // deg/h -> rad/s
  const double gturn = d.gyro_turn_on_bias_deg_h * kDeg / 3600.0;
  const double vrw = d.accel_vrw_mps_sqrt_h / 60.0;                    // m/s/sqrt(h) -> m/s/sqrt(s)
  const double abias = d.accel_bias_instability_mg * kG * 1e-3;        // mg -> m/s^2
  const double aturn = d.accel_turn_on_bias_mg * kG * 1e-3;
  for (int i = 0; i < 3; ++i) {
    m.gyro_random_walk_sigma[static_cast<std::size_t>(i)] = arw;
    m.gyro_bias_sigma[static_cast<std::size_t>(i)] = gbias;
    m.gyro_bias_tau[static_cast<std::size_t>(i)] = d.bias_tau_s;
    m.gyro_bias_initial_sigma[static_cast<std::size_t>(i)] = std::max(gturn, 3 * gbias);
    m.accel_random_walk_sigma[static_cast<std::size_t>(i)] = vrw;
    m.accel_bias_sigma[static_cast<std::size_t>(i)] = abias;
    m.accel_bias_tau[static_cast<std::size_t>(i)] = d.bias_tau_s;
    m.accel_bias_initial_sigma[static_cast<std::size_t>(i)] = std::max(aturn, 3 * abias);
  }
  return m;
}

const std::vector<ImuPreset>& imu_presets() {
  static const std::vector<ImuPreset> presets = make_presets();
  return presets;
}

std::vector<std::string> imu_preset_names() {
  std::vector<std::string> out;
  for (const auto& p : imu_presets()) out.push_back(p.name);
  return out;
}

std::optional<ImuConfig> imu_preset(const std::string& name, const std::string& group) {
  for (const auto& p : imu_presets())
    if (p.name == name) {
      ImuConfig c = p.config;
      c.group_ = group;
      return c;
    }
  return std::nullopt;
}

std::string matching_imu_preset(const ImuConfig& c) {
  for (const auto& p : imu_presets()) {
    const ImuConfig& q = p.config;
    if (q.accel_bias_sigma == c.accel_bias_sigma && q.accel_bias_tau == c.accel_bias_tau &&
        q.accel_random_walk_sigma == c.accel_random_walk_sigma && q.gyro_bias_sigma == c.gyro_bias_sigma &&
        q.gyro_bias_tau == c.gyro_bias_tau && q.gyro_random_walk_sigma == c.gyro_random_walk_sigma)
      return p.name;
  }
  return "";
}

std::string imu_preset_for_mode(const std::string& name, bool legacy_q_rotation) {
  if (name == "vn100" || name == "vn100_corrected") return legacy_q_rotation ? "vn100" : "vn100_corrected";
  return name;
}

const std::vector<GnssPreset>& gnss_presets() {
  static const std::vector<GnssPreset> presets = make_gnss();
  return presets;
}

std::vector<std::string> gnss_preset_names() {
  std::vector<std::string> out;
  for (const auto& p : gnss_presets()) out.push_back(p.name);
  return out;
}

std::optional<GnssPreset> gnss_preset(const std::string& name) {
  for (const auto& p : gnss_presets())
    if (p.name == name) return p;
  return std::nullopt;
}

}  // namespace pntos::cobra::presets
