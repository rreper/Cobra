// Shared configuration of the tutorial apps (pntos.apps.tutorial.pos_ins / pos_vel_ins): a manual
// initial solution instead of an alignment, flat config groups instead of the nested
// StandardOrchestrationConfig, the tutorial state model and orchestration, and the UI log plotting
// plugin at shutdown.
#pragma once

#include "../standard/app_common.hpp"

#include <pntos/cobra/tutorial/TutorialPlugins.hpp>

namespace pntos::apps {

inline std::vector<std::shared_ptr<const cobra::BaseConfig>> tutorial_configs(const Args& args, bool with_velocity) {
  auto transport = std::make_shared<LcmLogTransportConfig>();
  transport->input_file = args.input_log;
  transport->output_file = args.output_log;  // no channel filter: the tutorial transport replays everything

  auto imu = std::make_shared<ImuConfig>(imu_model(args.legacy_q));
  imu->accel_bias_initial_sigma = {0, 0, 0};  // the tutorial ImuConfig has no initial-sigma fields
  imu->gyro_bias_initial_sigma = {0, 0, 0};

  auto align = std::make_shared<ManualAlignmentConfig>();
  align->group_ = "config/default/alignment";
  align->initial_pos_var = {0.1, 0.1, 0.1};
  align->initial_vel_var = {1e-3, 1e-3, 1e-3};
  align->initial_tilt_var = {5e-4, 5e-4, 5e-4};
  align->initial_accel_bias_var = {5.2e-3, 5.2e-3, 5.2e-3};
  align->initial_gyro_bias_var = {9e-6, 9e-6, 9e-6};
  align->initial_accel_bias = {-0.0023383, 0.00085563, -0.05412892};
  align->initial_accel_scale_factor = {0.0, 0.0, 0.0};
  align->initial_accel_scale_factor_var = {0.0, 0.0, 0.0};
  align->initial_gyro_bias = {-0.00160958, -0.00204483, -0.00267885};
  align->initial_gyro_scale_factor = {0.0, 0.0, 0.0};
  align->initial_gyro_scale_factor_var = {0.0, 0.0, 0.0};
  align->initial_pos = {0.6938996038254822, -1.4679920679462133, 225.493};
  align->initial_rpy = {-0.014713125594312194, -0.040718531449027706, 0.06895795874629593};
  align->initial_time = 1747680879.539799718;
  align->initial_vel = {0.0, 0.0, 0.0};

  auto mounting = std::make_shared<MountingConfig>();
  mounting->group_ = "config/gp3d_state_modeling";
  mounting->lever_arm = {-0.50, 0.38, -0.05};
  mounting->orientation = {1.0, 0.0, 0.0, 0.0};

  auto inertial = std::make_shared<InertialConfig>();
  inertial->group_ = "config/inertial";
  inertial->expected_dt = 0.01;
  inertial->channels = {kImuChannel};
  inertial->C_imu_to_platform = C_imu_to_platform();
  inertial->inertial_buffer_length = 10.0;

  auto fogm = std::make_shared<FogmConfig>();
  fogm->group_ = "config/pos_sensor_error";
  fogm->sigma = {1.5, 1.5, 2.0};
  fogm->tau = {300.0, 300.0, 200.0};

  auto orch = std::make_shared<TutorialOrchestrationConfig>();
  orch->position_channel = kPosChannel;
  if (with_velocity) orch->velocity_channel = kVelChannel;

  auto time_adjuster = std::make_shared<TimeAdjusterConfig>();
  time_adjuster->group_ = "config/time_adjuster";
  time_adjuster->channels = std::vector<std::string>{kImuChannel};
  time_adjuster->expected_dt_nsec = 10'000'000;

  auto ui = std::make_shared<UiLogPlottingConfig>();
  ui->logfile = args.output_log;
  ui->solution_channel = kBestSol;
  ui->truth_channel = "/sensor/ins-d/pva";

  auto imu_rotator = std::make_shared<ImuRotatorConfig>();
  imu_rotator->group_ = "config/imu_rotator";
  imu_rotator->C_imu_to_platform = C_imu_to_platform();
  imu_rotator->channels = std::vector<std::string>{kImuChannel};

  auto time_bias = std::make_shared<TimeBiasConfig>();
  time_bias->group_ = "config/time_bias";
  time_bias->channels = with_velocity ? std::vector<std::string>{kPosChannel, kVelChannel} : std::vector<std::string>{kPosChannel};
  time_bias->time_bias = 150'000'000;

  return {transport, std::make_shared<ControllerConfig>(), std::make_shared<FusionEngineConfig>(), imu, align, mounting,
          inertial, fogm, orch, time_adjuster, ui, imu_rotator, time_bias};
}

inline int run_tutorial_app(const char* name, const Args& args, bool with_velocity) {
  AppConfig config;
  config.app.name = name;
  config.app.initialization = "manual";
  config.app.state_modeling = "tutorial";
  config.app.orchestration = with_velocity ? "tutorial_pos_vel" : "tutorial_pos";
  config.app.ui_log_plotting = true;
  config.app.joseph_form = args.joseph;
  config.app.legacy_q_rotation = args.legacy_q;
  config.configs = tutorial_configs(args, with_velocity);
  try {
    return app::run_app(config, args.options);
  } catch (const std::exception& e) {
    std::cerr << name << ": " << e.what() << "\n";
    return 2;
  }
}

}  // namespace pntos::apps
