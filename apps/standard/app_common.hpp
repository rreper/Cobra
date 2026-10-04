// Shared configuration of the standard apps (the Python apps each repeat this block verbatim).
#pragma once

#include <pntos/cobra/EkfFusionStrategyPlugin.hpp>
#include <pntos/cobra/StandardLoggingPlugin.hpp>
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/controller/StandardControllerPlugin.hpp>
#include <pntos/cobra/fusion/StandardFusionPlugin.hpp>
#include <pntos/cobra/inertial/StandardInertialPlugin.hpp>
#include <pntos/cobra/initialization/InitializationPlugins.hpp>
#include <pntos/cobra/orchestration/StandardOrchestrationPlugin.hpp>
#include <pntos/cobra/preprocessing/StandardPreprocessorPlugin.hpp>
#include <pntos/cobra/state_modeling/StandardStateModelingPlugin.hpp>
#include <pntos/cobra/transport/LcmLogTransportPlugin.hpp>
#include <pntos/cobra/diagnostics/DiagnosticLogPlugin.hpp>
#include <pntos/cobra/extras/AdvancedPreprocessorPlugin.hpp>
#include <pntos/cobra/app/AppBuilder.hpp>
#include <pntos/cobra/presets/Presets.hpp>

#include <cstdlib>
#include <iostream>

namespace pntos::apps {

using namespace pntos::cobra;
namespace api = pntos::api;

inline const std::string kImuChannel = "/sensor/vn-100/imu";
inline const std::string kPosChannel = "/sensor/ublox-ZED-F9T/position";
inline const std::string kVelChannel = "/sensor/ublox-ZED-F9T/velocity";
inline const std::string kPvaChannel = "/sensor/ublox-ZED-F9T/pva";
inline const std::string kBestSol = "/solution/pntos/pva";
inline const std::string kImuSol = "/solution/pntos-imu/pva";

inline std::string default_input_log() {
  const char* home = std::getenv("HOME");
  return std::string(home ? home : "") +
         "/orin/work/pntos/Cobra/.venv/lib/python3.12/site-packages/pntos_python_datasets_lcm/cobra_gps_ins_example_data.log";
}

inline Mat3 C_imu_to_platform() {
  return {{{0.99802515, 0.01772605, 0.06026269}, {-0.01742059, 0.99983262, -0.00559042}, {-0.0603517, 0.00452957, 0.9981669}}};
}

/// The Cobra-tuned VN-100 model (preset "vn100", the Python apps' values) for the legacy Pinson-Q
/// rotation, or its retuned variant (preset "vn100_corrected", yaw gyro random walk 6e-4) for the
/// corrected rotation. See presets/Presets.hpp and DESIGN.md section 9.5.
inline ImuConfig imu_model(bool legacy_q = false) {
  return *presets::imu_preset(legacy_q ? "vn100" : "vn100_corrected", "config/inertial_state");
}

/// 3-state NED position-error FOGM block ("pos_sensor_error") as configured by pos_ins.
inline std::shared_ptr<FogmStateBlockConfig> pos_fogm_block(api::Matrix cov = api::Matrix::Identity(3, 3) * 9.0,
                                                           std::vector<double> sigma = {1.5, 1.5, 2.0}) {
  auto f = std::make_shared<FogmStateBlockConfig>();
  f->group_ = "config/pos_fogm_block";
  f->label = "pos_sensor_error";
  f->estimate_with_covariance =
      api::EstimateWithCovariance{api::EstimateWithCovarianceType::EWC_GENERIC, api::Vector::Zero(3), std::move(cov)};
  f->fogm_model.group_ = "config/pos_sensor_error";
  f->fogm_model.sigma = std::move(sigma);
  f->fogm_model.tau = {300.0, 300.0, 200.0};
  return f;
}

/// Everything pos_ins configures except the measurement processors / extra blocks / VSBs /
/// transport channels, which each app sets on the returned objects.
struct BaseConfig {
  std::shared_ptr<LcmLogTransportConfig> transport = std::make_shared<LcmLogTransportConfig>();
  std::shared_ptr<StandardOrchestrationConfig> orch = std::make_shared<StandardOrchestrationConfig>();
  std::shared_ptr<ImuRotatorConfig> imu_rotator = std::make_shared<ImuRotatorConfig>();
  std::shared_ptr<TimeAdjusterConfig> time_adjuster = std::make_shared<TimeAdjusterConfig>();
  std::shared_ptr<TimeBiasConfig> time_bias = std::make_shared<TimeBiasConfig>();
  std::shared_ptr<ManualHeadingAlignmentConfig> alignment = std::make_shared<ManualHeadingAlignmentConfig>();
  std::shared_ptr<FusionEngineConfig> fusion = std::make_shared<FusionEngineConfig>();

  std::vector<std::shared_ptr<const cobra::BaseConfig>> all() const {
    return {transport, std::make_shared<ControllerConfig>(), fusion, orch};
  }
};

inline BaseConfig base_config(const std::string& input_log, const std::string& output_log, bool legacy_q = false) {
  // `legacy_q` selects both the Pinson-Q rotation and the matching VN-100 tuning.
  BaseConfig b;
  b.transport->input_file = input_log;
  b.transport->output_file = output_log;
  b.transport->channels_to_process = std::vector<std::string>{kImuChannel, kPosChannel};

  auto& o = *b.orch;
  o.best_sol_channel = kBestSol;
  o.imu_sol_channel = kImuSol;
  o.alignment_channels = {kPosChannel, kImuChannel};
  o.pinson_sb_config.group_ = "config/pinson_block";
  o.pinson_sb_config.label = "pinson15";
  o.pinson_sb_config.imu_model = imu_model(legacy_q);
  o.pinson_sb_config.legacy_q_rotation = legacy_q;
  o.additional_sb_configs = std::vector<std::shared_ptr<const StateBlockConfig>>{pos_fogm_block()};
  o.inertial_config.group_ = "config/inertial";
  o.inertial_config.expected_dt = 0.01;
  o.inertial_config.channels = {kImuChannel};
  o.inertial_config.C_imu_to_platform = C_imu_to_platform();
  o.inertial_config.inertial_buffer_length = 10.0;

  b.alignment->group_ = "config/default/alignment";
  b.alignment->static_time = 10.0;
  b.alignment->imu_model = imu_model(legacy_q);
  b.alignment->heading = 0.06895795874629593;
  b.alignment->heading_sigma = 0.02236067977;
  o.alignment_config = b.alignment;

  b.imu_rotator->group_ = "config/imu_rotator";
  b.imu_rotator->channels = std::vector<std::string>{kImuChannel};
  b.imu_rotator->C_imu_to_platform = C_imu_to_platform();
  b.time_adjuster->group_ = "config/time_adjuster";
  b.time_adjuster->channels = std::vector<std::string>{kImuChannel};
  b.time_adjuster->expected_dt_nsec = 10'000'000;
  b.time_bias->group_ = "config/time_bias";
  b.time_bias->channels = std::vector<std::string>{kPosChannel};
  b.time_bias->time_bias = 150'000'000;
  o.preprocessor_configs =
      std::vector<std::shared_ptr<const PreprocessorConfig>>{b.imu_rotator, b.time_adjuster, b.time_bias};
  return b;
}

/// Command line: `[output.log] [input.log] [--no-joseph] [--legacy-q|--corrected-q] [--dump-config f]
/// [--dump-registry f] [--quiet]`. The defaults are the Cobra example log, `pntos_output.log`, Joseph
/// form and the corrected Pinson-Q rotation (`--legacy-q` reproduces the Python results exactly).
struct Args {
  std::string output_log = "pntos_output.log";
  std::string input_log = default_input_log();
  bool joseph = true;
  bool legacy_q = false;  ///< default: corrected Pinson-Q rotation with the retuned VN-100 model; --legacy-q reproduces Python
  app::RunOptions options;
};
inline Args parse_args(int argc, char** argv) {
  Args a;
  a.options = app::parse_run_options(argc, argv, 1);
  if (a.options.output_log) a.output_log = *a.options.output_log;
  if (a.options.input_log) a.input_log = *a.options.input_log;
  if (a.options.joseph_form) a.joseph = *a.options.joseph_form;
  if (a.options.legacy_q_rotation) a.legacy_q = *a.options.legacy_q_rotation;
  return a;
}

/// `output.log` -> `output.hdf5` (the diagnostic log written next to the LCM output log).
inline std::string hdf5_path_for(const std::string& output_log) {
  const auto dot = output_log.rfind('.');
  const auto slash = output_log.rfind('/');
  const bool has_ext = dot != std::string::npos && (slash == std::string::npos || dot > slash);
  return (has_ext ? output_log.substr(0, dot) : output_log) + ".hdf5";
}

/// Runs the plugin set `spec` describes (default: the standard set) around `configs` through
/// app::run_app, which is also what `cobra_run config.json` does. Returns the exit code.
inline int run_standard_app(const char* name, const Args& args,
                            const std::vector<std::shared_ptr<const cobra::BaseConfig>>& configs,
                            std::optional<AppSpec> spec = std::nullopt) {
  AppConfig config;
  config.app = spec.value_or(app::standard_app_spec(name));
  config.app.name = name;
  config.app.joseph_form = args.joseph;
  config.app.legacy_q_rotation = args.legacy_q;
  config.configs = configs;
  try {
    return app::run_app(config, args.options);
  } catch (const std::exception& e) {
    std::cerr << name << ": " << e.what() << "\n";
    return 2;
  }
}

}  // namespace pntos::apps
