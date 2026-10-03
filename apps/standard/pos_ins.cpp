// Port of pntos.apps.standard.pos_ins: GPS position + IMU, manual-heading static alignment,
// Pinson15 + 3-state NED position-error FOGM, replayed from the example LCM log.
//
//   pos_ins [output.log] [input.log]
//
// The input defaults to the Cobra example dataset inside the Python venv (same file the Python app
// uses); the output defaults to pntos_output.log in the working directory.
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

#include <cstdlib>
#include <iostream>

using namespace pntos;
using namespace pntos::cobra;

namespace {
std::string default_input_log() {
  const char* home = std::getenv("HOME");
  return std::string(home ? home : "") +
         "/orin/work/pntos/Cobra/.venv/lib/python3.12/site-packages/pntos_python_datasets_lcm/cobra_gps_ins_example_data.log";
}
}  // namespace

int main(int argc, char** argv) {
  const std::string output_log = argc > 1 ? argv[1] : "pntos_output.log";
  const std::string input_log = argc > 2 ? argv[2] : default_input_log();
  const std::string imu_channel = "/sensor/vn-100/imu";
  const std::string pos_channel = "/sensor/ublox-ZED-F9T/position";

  // ---- Config (identical values to the Python app)
  const Mat3 C_imu_to_platform{{{0.99802515, 0.01772605, 0.06026269},
                                {-0.01742059, 0.99983262, -0.00559042},
                                {-0.0603517, 0.00452957, 0.9981669}}};
  ImuConfig imu_model;
  imu_model.group_ = "config/inertial_state";
  imu_model.accel_bias_sigma = {2.4e-3, 2.4e-3, 2.4e-3};
  imu_model.accel_bias_tau = {300.0, 300.0, 300.0};
  imu_model.accel_random_walk_sigma = {3.887e-6, 3.887e-6, 3.887e-6};
  imu_model.gyro_bias_sigma = {2e-4, 2e-4, 2e-4};
  imu_model.gyro_bias_tau = {500.0, 500.0, 500.0};
  imu_model.gyro_random_walk_sigma = {9.9e-4, 9.9e-4, 6.7e-5};
  imu_model.accel_bias_initial_sigma = {0.072, 0.072, 0.072};
  imu_model.gyro_bias_initial_sigma = {0.003, 0.003, 0.003};

  auto transport_cfg = std::make_shared<LcmLogTransportConfig>();
  transport_cfg->input_file = input_log;
  transport_cfg->output_file = output_log;
  transport_cfg->channels_to_process = std::vector<std::string>{imu_channel, pos_channel};

  auto orch = std::make_shared<StandardOrchestrationConfig>();
  orch->best_sol_channel = "/solution/pntos/pva";
  orch->imu_sol_channel = "/solution/pntos-imu/pva";
  orch->alignment_channels = {pos_channel, imu_channel};
  orch->pinson_sb_config.group_ = "config/pinson_block";
  orch->pinson_sb_config.label = "pinson15";
  orch->pinson_sb_config.imu_model = imu_model;

  auto fogm = std::make_shared<FogmStateBlockConfig>();
  fogm->group_ = "config/pos_fogm_block";
  fogm->label = "pos_sensor_error";
  fogm->estimate_with_covariance = api::EstimateWithCovariance{api::EstimateWithCovarianceType::EWC_GENERIC,
                                                               api::Vector::Zero(3), api::Matrix::Identity(3, 3) * 9.0};
  fogm->fogm_model.group_ = "config/pos_sensor_error";
  fogm->fogm_model.sigma = {1.5, 1.5, 2.0};
  fogm->fogm_model.tau = {300.0, 300.0, 200.0};
  orch->additional_sb_configs = std::vector<std::shared_ptr<const StateBlockConfig>>{fogm};

  auto mp = std::make_shared<LeverArmMPConfig>(mp::PinsonWithNedFogmPositionMPConfig());
  mp->group_ = "config/pos_measurement_processor";
  mp->label = "pos";
  mp->channel = pos_channel;
  mp->state_block_labels = {"pinson15", "pos_sensor_error"};
  mp->lever_arm = {-0.50, 0.38, -0.05};
  orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{mp};

  orch->inertial_config.group_ = "config/inertial";
  orch->inertial_config.expected_dt = 0.01;
  orch->inertial_config.channels = {imu_channel};
  orch->inertial_config.C_imu_to_platform = C_imu_to_platform;
  orch->inertial_config.inertial_buffer_length = 10.0;

  auto align = std::make_shared<ManualHeadingAlignmentConfig>();
  align->group_ = "config/default/alignment";
  align->static_time = 10.0;
  align->imu_model = imu_model;
  align->heading = 0.06895795874629593;
  align->heading_sigma = 0.02236067977;
  orch->alignment_config = align;

  auto rot = std::make_shared<ImuRotatorConfig>();
  rot->group_ = "config/imu_rotator";
  rot->channels = std::vector<std::string>{imu_channel};
  rot->C_imu_to_platform = C_imu_to_platform;
  auto ta = std::make_shared<TimeAdjusterConfig>();
  ta->group_ = "config/time_adjuster";
  ta->channels = std::vector<std::string>{imu_channel};
  ta->expected_dt_nsec = 10'000'000;
  auto tb = std::make_shared<TimeBiasConfig>();
  tb->group_ = "config/time_bias";
  tb->channels = std::vector<std::string>{pos_channel};
  tb->time_bias = 150'000'000;
  orch->preprocessor_configs = std::vector<std::shared_ptr<const PreprocessorConfig>>{rot, ta, tb};

  std::vector<std::shared_ptr<const BaseConfig>> configs{transport_cfg, std::make_shared<ControllerConfig>(),
                                                         std::make_shared<FusionEngineConfig>(), orch};
  // ---- End config

  auto transport = std::make_shared<LcmLogTransportPlugin>("Cobra LCM Log Transport Plugin");
  transport->set_progress_callback([](std::uint64_t done, std::uint64_t total) {
    std::cerr << "\r[pos_ins] " << (100 * done / std::max<std::uint64_t>(total, 1)) << "%" << std::flush;
  });
  api::PluginList plugins{
      transport,
      std::make_shared<EkfFusionStrategyPlugin>("Cobra EKF Fusion Strategy Plugin"),
      std::make_shared<StandardFusionPlugin>("Cobra Standard Fusion Plugin"),
      std::make_shared<StandardStateModelingPlugin>("Cobra Standard State Modeling Plugin"),
      std::make_shared<StandardInertialPlugin>("Cobra Standard Inertial Plugin"),
      std::make_shared<ManualHeadingAlignInitializationPlugin>("Cobra Manual Heading Static Align Initialization Plugin"),
      std::make_shared<StandardLoggingPlugin>("Cobra Standard Logging Plugin", true, api::LoggingLevel::INFO),
      std::make_shared<StandardRegistryPlugin>("Cobra Standard Registry Plugin", configs),
      std::make_shared<StandardPreprocessorPlugin>("Cobra Standard Preprocessor Plugin"),
      std::make_shared<StandardOrchestrationPlugin>("Cobra Standard Orchestration Plugin"),
  };

  StandardControllerPlugin::install_sigint_handler();
  StandardControllerPlugin controller("Cobra Standard Controller Plugin");
  controller.init_plugin(std::nullopt, nullptr);
  controller.take_control(plugins);
  std::cerr << "\n";
  return controller.exit_code() == ExitCode::SUCCESS ? 0 : 1;
}
