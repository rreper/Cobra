// JSON config files, presets and the app builder (roadmap Phase 1).
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/app/AppBuilder.hpp>
#include <pntos/cobra/config/JsonConfig.hpp>
#include <pntos/cobra/diagnostics/DiagnosticLogPlugin.hpp>
#include <pntos/cobra/orchestration/StandardOrchestrationPlugin.hpp>
#include <pntos/cobra/presets/Presets.hpp>

#include "test_support.hpp"

#include <cmath>

using namespace pntos;
using namespace pntos::test;
using cobra::jsoncfg::json;

namespace {
cobra::AppConfig sample_app_config() {
  using namespace cobra;
  AppConfig a;
  a.app.name = "sample";
  a.app.diagnostic_log = true;
  auto transport = std::make_shared<LcmLogTransportConfig>();
  transport->input_file = "in.log";
  transport->output_file = "out.log";
  transport->channels_to_process = std::vector<std::string>{"/imu", "/pos"};
  auto orch = std::make_shared<StandardOrchestrationConfig>();
  orch->best_sol_channel = "/solution/pntos/pva";
  orch->imu_sol_channel = "/solution/pntos-imu/pva";
  orch->alignment_channels = {"/pos", "/imu"};
  orch->pinson_sb_config.group_ = "config/pinson_block";
  orch->pinson_sb_config.label = "pinson15";
  orch->pinson_sb_config.imu_model = *presets::imu_preset("vn100", "config/inertial_state");
  orch->pinson_sb_config.legacy_q_rotation = true;
  auto fogm = std::make_shared<FogmStateBlockConfig>();
  fogm->group_ = "config/pos_fogm_block";
  fogm->label = "pos_sensor_error";
  fogm->estimate_with_covariance = api::EstimateWithCovariance{api::EstimateWithCovarianceType::EWC_GENERIC, api::Vector::Zero(3),
                                                               api::Matrix::Identity(3, 3) * 9.0};
  fogm->fogm_model.group_ = "config/pos_sensor_error";
  fogm->fogm_model.sigma = {1.5, 1.5, 2.0};
  fogm->fogm_model.tau = {300.0, 300.0, 200.0};
  auto cb = std::make_shared<ClockBiasStateBlockConfig>();
  cb->group_ = "config/clock";
  cb->label = "clock";
  cb->h_0 = 1e-21;
  cb->h_neg2 = 1e-22;
  api::Matrix full(2, 2);
  full << 1, 0.5, 0.5, 2;
  cb->estimate_with_covariance = api::EstimateWithCovariance{api::EstimateWithCovarianceType::EWC_GENERIC, api::Vector::Zero(2), full};
  orch->additional_sb_configs = std::vector<std::shared_ptr<const StateBlockConfig>>{fogm, cb};
  auto mp = std::make_shared<LeverArmMPConfig>(mp::PinsonWithNedFogmPositionMPConfig());
  mp->group_ = "config/pos_measurement_processor";
  mp->label = "pos";
  mp->channel = "/pos";
  mp->state_block_labels = {"pinson15", "pos_sensor_error"};
  mp->lever_arm = {-0.5, 0.38, -0.05};
  mp->innovation_gate_probability = 0.999;
  mp->geoid_file = "data/egm96_15min.bin";
  auto bv = std::make_shared<LeverArmOrientationMPConfig>(mp::PinsonBodyVelocityMPConfig());
  bv->group_ = "config/bv";
  bv->label = "bv";
  bv->channel = "/bv";
  bv->state_block_labels = {"pinson15"};
  bv->orientation = {0.7, 0, 0.7, 0};
  auto vel = std::make_shared<PlainMPConfig>(mp::PinsonVelocityMPConfig());
  vel->group_ = "config/vel";
  vel->label = "vel";
  vel->channel = "/vel";
  vel->state_block_labels = {"pinson15"};
  orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{mp, bv, vel};
  auto pes = std::make_shared<PinsonErrorToStandardVSBConfig>();
  pes->group_ = "config/pes";
  pes->source = "pinson15";
  pes->target = "platform_pva";
  auto se = std::make_shared<StateExtractorConfig>();
  se->group_ = "config/se";
  se->source = "pinson15";
  se->target = "pos_only";
  se->incoming_state_size = 15;
  se->indices_to_extract = {0, 1, 2};
  orch->vsb_configs = std::vector<std::shared_ptr<const VirtualStateBlockConfig>>{pes, se};
  orch->inertial_config.group_ = "config/inertial";
  orch->inertial_config.channels = {"/imu"};
  orch->inertial_config.C_imu_to_platform = {{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}};
  FeedbackConfig fb;
  fb.group_ = "config/inertial_feedback";
  fb.pos_error_threshold = 100.0;
  orch->feedback_config = fb;
  auto align = std::make_shared<ManualHeadingAlignmentConfig>();
  align->group_ = "config/default/alignment";
  align->static_time = 10.0;
  align->imu_model = orch->pinson_sb_config.imu_model;
  align->heading = 0.069;
  align->heading_sigma = 0.0224;
  orch->alignment_config = align;
  auto ds = std::make_shared<DownsamplerConfig>();
  ds->group_ = "config/downsampler";
  ds->channels = std::vector<std::string>{"/bv"};
  ds->downsampling_factors = {10};
  auto out = std::make_shared<OutageConfig>();
  out->group_ = "config/outage";
  out->channels = std::vector<std::string>{"/pos"};
  out->start_time = 1000;
  out->end_time = 1600;
  auto baro = std::make_shared<BarometerToAltitudeConfig>();
  baro->group_ = "config/baro";
  baro->channels = std::vector<std::string>{"/baro"};
  baro->alt_sigma = 30.0;
  auto zv = std::make_shared<ZeroVelocity2dGeneratorConfig>();
  zv->group_ = "config/zv";
  zv->channels = std::vector<std::string>{"/pos"};
  zv->trigger_dt_sec = 30;
  zv->lateral_vel_sigma = 0.5;
  zv->vertical_vel_sigma = 1.0;
  zv->output_channel = "/generated/zero/velocity2d";
  auto deg = std::make_shared<SensorDegradationConfig>();
  deg->group_ = "config/degradation";
  deg->channels = std::vector<std::string>{"/imu", "/pos"};
  deg->seed = 7;
  deg->accel_noise_density = {1e-3, 1e-3, 1e-3};
  deg->gyro_bias = {1e-4, 0, 0};
  deg->position_noise_sigma_ned = {1, 1, 2};
  deg->position_covariance_scale = 4.0;
  deg->position_jumps = {{100.0, 50.0, 0.0, 0.0}, {200.0, 0.0, -50.0, 0.0}};
  deg->position_ramps = {{600.0, 1.4, 1.4, 0.0, 0.0}};
  deg->derived_position_channel = "/synthetic/cell/position";
  deg->derived_position_sigma_ned = {5, 5, 8};
  orch->preprocessor_configs = std::vector<std::shared_ptr<const PreprocessorConfig>>{ds, out, baro, zv, deg};
  orch->max_prop_interval = 1.0;
  Stream st;
  st.group_ = "config/stream0";
  st.message_type = ASPN_MEASUREMENT_POSITION;
  st.source_identifier = "/pos";
  orch->stream_config.override_streams->push_back(st);
  auto fusion = std::make_shared<FusionEngineConfig>();
  fusion->save_x_and_p_after_prop = true;
  fusion->innovation_gate_probability = 0.99;
  auto ctrl = std::make_shared<ControllerConfig>();
  ctrl->publish_interval = std::nullopt;
  auto ui = std::make_shared<UiLogPlottingConfig>();
  ui->logfile = "out.log";
  ui->solution_channel = "/solution/pntos/pva";
  ui->truth_channel = "/truth";
  auto pva_init = std::make_shared<PvaMessageInitializationConfig>();
  pva_init->group_ = "config/pva_init";
  pva_init->initial_pva_channel = "/pva";
  pva_init->initial_accel_bias_sigma = {0.07, 0.07, 0.07};
  pva_init->initial_gyro_bias_sigma = {0.003, 0.003, 0.003};
  pva_init->start_time = 5.0;
  a.configs = {transport, ctrl, fusion, orch, ui, pva_init};
  return a;
}

json registry_of(const std::vector<std::shared_ptr<const cobra::BaseConfig>>& configs) {
  TestMediator med;
  cobra::StandardRegistryPlugin reg("r", configs);
  reg.init_plugin(std::nullopt, &med);
  return cobra::jsoncfg::dump_registry(*reg.new_registry());
}
}  // namespace

TEST(JsonConfig, RoundTripPreservesTheRegistryContents) {
  auto a = sample_app_config();
  json j = cobra::jsoncfg::app_config_to_json(a);
  auto back = cobra::jsoncfg::app_config_from_json(j);
  ASSERT_EQ(back.configs.size(), a.configs.size());
  EXPECT_EQ(registry_of(back.configs), registry_of(a.configs));
  EXPECT_EQ(cobra::jsoncfg::app_config_to_json(back), j);
  EXPECT_EQ(back.app.name, "sample");
  EXPECT_TRUE(back.app.diagnostic_log);
  // every config class has a Python type name
  std::vector<std::string> names;
  for (const auto& c : j["configs"]) names.push_back(c["type"]);
  EXPECT_EQ(names, (std::vector<std::string>{"LcmLogTransportConfig", "ControllerConfig", "FusionEngineConfig",
                                             "StandardOrchestrationConfig", "UiLogPlottingConfig", "PvaMessageInitializationConfig"}));
  EXPECT_EQ(j["configs"][3]["mp_configs"][0]["type"], "PinsonWithNedFogmPositionMPConfig");
  EXPECT_EQ(j["configs"][3]["mp_configs"][1]["type"], "PinsonBodyVelocityMPConfig");
  EXPECT_EQ(j["configs"][3]["mp_configs"][2]["type"], "PinsonVelocityMPConfig");
  EXPECT_EQ(j["configs"][3]["stream_config"]["override_streams"][1]["message_type"], "MEASUREMENT_POSITION");
  EXPECT_TRUE(j["configs"][1]["publish_interval"].is_null());
}

TEST(JsonConfig, PresetsAndOverrides) {
  json jc = R"({"type":"PinsonStateBlockConfig","group":"config/pinson_block","label":"pinson15",
                "imu_model":{"preset":"stim300","group":"config/inertial_state","gyro_bias_tau":[100,100,100]}})"_json;
  auto c = std::dynamic_pointer_cast<cobra::PinsonStateBlockConfig>(cobra::jsoncfg::config_from_json(jc));
  ASSERT_TRUE(c);
  auto stim = cobra::presets::imu_preset("stim300", "config/inertial_state");
  ASSERT_TRUE(stim);
  EXPECT_EQ(c->imu_model.gyro_random_walk_sigma, stim->gyro_random_walk_sigma);
  EXPECT_EQ(c->imu_model.gyro_bias_tau[0], 100.0);
  EXPECT_EQ(c->imu_model.group_, "config/inertial_state");
  EXPECT_FALSE(c->legacy_q_rotation);
  jc["imu_model"]["preset"] = "nope";
  EXPECT_THROW(cobra::jsoncfg::config_from_json(jc), std::runtime_error);
}

TEST(JsonConfig, ErrorsNameTheField) {
  auto expect_error = [](const char* text, const char* needle) {
    try {
      cobra::jsoncfg::config_from_json(json::parse(text));
      FAIL() << "expected an error for " << text;
    } catch (const std::runtime_error& e) {
      EXPECT_NE(std::string(e.what()).find(needle), std::string::npos) << e.what();
    }
  };
  expect_error(R"({"group":"g"})", "type");
  expect_error(R"({"type":"Bogus"})", "Bogus");
  expect_error(R"({"type":"TimeBiasConfig","group":"g"})", "time_bias");
  expect_error(R"({"type":"ImuRotatorConfig","group":"g","C_imu_to_platform":[1,2,3]})", "3x3");
  expect_error(R"({"type":"Stream","group":"g","message_type":"MEASUREMENT_FOO"})", "MEASUREMENT_FOO");
  EXPECT_THROW(cobra::jsoncfg::load_app_config("/nonexistent/x.json"), std::runtime_error);
}

TEST(Presets, ImuTablesAreConsistent) {
  using namespace cobra::presets;
  auto names = imu_preset_names();
  EXPECT_GE(names.size(), 7u);
  EXPECT_NE(std::find(names.begin(), names.end(), "vn100"), names.end());
  for (const auto& p : imu_presets()) {
    SCOPED_TRACE(p.name);
    for (std::size_t i = 0; i < 3; ++i) {
      EXPECT_GT(p.config.gyro_random_walk_sigma[i], 0);
      EXPECT_GT(p.config.gyro_bias_sigma[i], 0);
      EXPECT_GT(p.config.gyro_bias_tau[i], 0);
      EXPECT_GT(p.config.accel_random_walk_sigma[i], 0);
      EXPECT_GT(p.config.accel_bias_sigma[i], 0);
      EXPECT_GE(p.config.accel_bias_initial_sigma[i], p.config.accel_bias_sigma[i]);
      EXPECT_GE(p.config.gyro_bias_initial_sigma[i], p.config.gyro_bias_sigma[i]);
    }
    EXPECT_FALSE(p.description.empty());
    EXPECT_FALSE(p.source.empty());
  }
  // the VN-100 preset is byte-for-byte the Cobra app model
  auto vn = imu_preset("vn100", "g");
  ASSERT_TRUE(vn);
  EXPECT_EQ(vn->gyro_random_walk_sigma[2], 6.7e-5);
  EXPECT_EQ(vn->group_, "g");
  EXPECT_FALSE(imu_preset("unknown", "g"));
  // datasheet conversion: 1 deg/sqrt(h) ARW = 2.909e-4 rad/sqrt(s); 1 deg/h = 4.848e-6 rad/s; 1 mg = 9.80665e-3 m/s^2
  auto d = imu_from_datasheet({1.0, 1.0, 10.0, 60.0, 1.0, 10.0, 500.0}, "g");
  EXPECT_NEAR(d.gyro_random_walk_sigma[0], 2.9089e-4, 1e-7);
  EXPECT_NEAR(d.gyro_bias_sigma[0], 4.8481e-6, 1e-9);
  EXPECT_NEAR(d.gyro_bias_initial_sigma[0], 4.8481e-5, 1e-8);
  EXPECT_NEAR(d.accel_random_walk_sigma[0], 1.0, 1e-12);
  EXPECT_NEAR(d.accel_bias_sigma[0], 9.80665e-3, 1e-9);
  EXPECT_NEAR(d.accel_bias_initial_sigma[0], 9.80665e-2, 1e-9);
  // tactical grades are quieter than consumer ones
  EXPECT_LT(imu_preset("hg4930", "g")->gyro_random_walk_sigma[0], imu_preset("consumer_mems", "g")->gyro_random_walk_sigma[0]);
  EXPECT_LT(imu_preset("stim300", "g")->gyro_bias_sigma[0], imu_preset("adis16488", "g")->gyro_bias_sigma[0]);
  // GNSS
  auto g = gnss_preset("ublox_f9");
  ASSERT_TRUE(g);
  EXPECT_TRUE(g->position && g->velocity && g->pva);
  EXPECT_EQ(g->time_bias_sec, 0.15);
  EXPECT_FALSE(gnss_preset("unknown"));
  EXPECT_GE(gnss_preset_names().size(), 5u);
}

TEST(AppBuilder, OptionsOverridesAndPluginSets) {
  using namespace cobra;
  const char* argv[] = {"app", "config.json", "out.log", "in.log", "--corrected-q", "--no-joseph", "--dump-registry", "r.json", "--quiet", "--no-record-input"};
  auto o = app::parse_run_options(10, const_cast<char**>(argv), 2);
  EXPECT_FALSE(*o.record_input);
  EXPECT_EQ(*o.output_log, "out.log");
  EXPECT_EQ(*o.input_log, "in.log");
  EXPECT_FALSE(*o.legacy_q_rotation);
  EXPECT_FALSE(*o.joseph_form);
  EXPECT_EQ(*o.dump_registry, "r.json");
  EXPECT_FALSE(o.progress);
  const char* bad[] = {"app", "--bogus"};
  EXPECT_THROW(app::parse_run_options(2, const_cast<char**>(bad), 1), std::runtime_error);

  auto a = sample_app_config();
  auto b = app::apply_overrides(a, o);
  bool saw_transport = false, saw_orch = false, saw_ui = false;
  for (const auto& c : b.configs) {
    if (auto* t = dynamic_cast<const LcmLogTransportConfig*>(c.get())) {
      saw_transport = true;
      EXPECT_EQ(*t->input_file, "in.log");
      EXPECT_EQ(*t->output_file, "out.log");
      EXPECT_FALSE(t->record_input_channels);
    }
    if (auto* orch = dynamic_cast<const StandardOrchestrationConfig*>(c.get())) {
      saw_orch = true;
      EXPECT_FALSE(orch->pinson_sb_config.legacy_q_rotation);
    }
    if (auto* ui = dynamic_cast<const UiLogPlottingConfig*>(c.get())) {
      saw_ui = true;
      EXPECT_EQ(ui->logfile, "out.log");
    }
  }
  EXPECT_TRUE(saw_transport && saw_orch && saw_ui);
  EXPECT_FALSE(b.app.joseph_form);
  EXPECT_FALSE(b.app.legacy_q_rotation);
  EXPECT_TRUE(b.app.diagnostic_log_file.empty());  // derived from the output log when the plugin is built
  // the Pinson-Q mode also selects the matching VN-100 tuning (vn100 <-> vn100_corrected)
  for (const auto& c : b.configs)
    if (auto* orch = dynamic_cast<const StandardOrchestrationConfig*>(c.get()))
      EXPECT_EQ(presets::matching_imu_preset(orch->pinson_sb_config.imu_model), "vn100_corrected");
  app::RunOptions legacy;
  legacy.legacy_q_rotation = true;
  for (const auto& c : app::apply_overrides(b, legacy).configs)
    if (auto* orch = dynamic_cast<const StandardOrchestrationConfig*>(c.get())) {
      EXPECT_TRUE(orch->pinson_sb_config.legacy_q_rotation);
      EXPECT_EQ(presets::matching_imu_preset(orch->pinson_sb_config.imu_model), "vn100");
    }

  // plugin sets
  auto plugins = app::build_plugins(b);
  EXPECT_EQ(plugins.size(), 11u);  // transport, ekf, fusion, sm, inertial, init, logging, registry, preprocessor, orchestration, diagnostics
  bool saw_diag = false;
  for (const auto& p : plugins)
    if (auto d = std::dynamic_pointer_cast<DiagnosticLogPlugin>(p)) {
      saw_diag = true;
      EXPECT_EQ(d->output_file(), "out.hdf5");
    }
  EXPECT_TRUE(saw_diag);
  AppSpec tut;
  tut.initialization = "manual";
  tut.state_modeling = "tutorial";
  tut.orchestration = "tutorial_pos_vel";
  tut.ui_log_plotting = true;
  tut.preprocessors = {"standard", "advanced"};
  AppConfig tc{tut, b.configs};
  EXPECT_EQ(app::build_plugins(tc).size(), 12u);
  AppSpec bad_spec;
  bad_spec.orchestration = "nope";
  EXPECT_THROW(app::build_plugins(AppConfig{bad_spec, b.configs}), std::runtime_error);
  bad_spec = AppSpec{};
  bad_spec.initialization = "nope";
  EXPECT_THROW(app::build_plugins(AppConfig{bad_spec, b.configs}), std::runtime_error);
  bad_spec = AppSpec{};
  bad_spec.logging_level = "LOUD";
  EXPECT_THROW(app::build_plugins(AppConfig{bad_spec, b.configs}), std::runtime_error);
}

TEST(AppBuilder, RegisteredOrchestrationsAndExtraPlugins) {
  using namespace cobra;
  auto a = sample_app_config();
  int built = 0;
  app::register_orchestration("test_counting", [&](const AppConfig&) {
    ++built;
    return std::make_shared<StandardOrchestrationPlugin>("registered orchestration");
  });
  app::register_extra_plugin("test_extra", [&](const AppConfig&) -> std::shared_ptr<api::CommonPlugin> {
    ++built;
    return std::make_shared<DiagnosticLogPlugin>("registered extra", "x.hdf5");
  });
  auto names = app::registered_orchestrations();
  EXPECT_NE(std::find(names.begin(), names.end(), "test_counting"), names.end());
  a.app.orchestration = "test_counting";
  a.app.extra_plugins = {"test_extra"};
  a.app.extra_plugins = {};
  const auto baseline = app::build_plugins(a).size();
  a.app.extra_plugins = {"test_extra"};
  auto plugins = app::build_plugins(a);
  EXPECT_EQ(built, 3);
  EXPECT_EQ(plugins.size(), baseline + 1);
  json j = jsoncfg::app_spec_to_json(a.app);
  EXPECT_EQ(j["extra_plugins"][0], "test_extra");
  EXPECT_EQ(jsoncfg::app_spec_from_json(j).extra_plugins.size(), 1u);
  a.app.extra_plugins = {"nope"};
  EXPECT_THROW(app::build_plugins(a), std::runtime_error);
}

TEST(JsonConfig, RegistryConfigRoundTrip) {
  using namespace cobra;
  auto jc = json::parse(R"({"type":"RegistryConfig","group":"config/integrity","values":{
      "sources":["/a","/b"],"probability":0.999,"consecutive":3,"enabled":true,"name":"x",
      "sigma":[1.0,2.0,3.0],"m":[[1.0,0.0],[0.0,1.0]]}})");
  auto c = jsoncfg::config_from_json(jc);
  auto* r = dynamic_cast<const RegistryConfig*>(c.get());
  ASSERT_NE(r, nullptr);
  EXPECT_EQ(r->group(), "config/integrity");
  EXPECT_EQ(std::get<api::StringArray>(r->values.at("sources")).size(), 2u);
  EXPECT_EQ(std::get<std::int64_t>(r->values.at("consecutive")), 3);
  EXPECT_EQ(std::get<api::Matrix>(r->values.at("sigma")).rows(), 3);
  EXPECT_EQ(std::get<api::Matrix>(r->values.at("m")).cols(), 2);
  json back = jsoncfg::config_to_json(*c);
  EXPECT_EQ(back["values"]["sigma"], json::parse("[1.0,2.0,3.0]"));
  EXPECT_EQ(back["values"]["m"][1][1], 1.0);
  EXPECT_EQ(back["values"]["name"], "x");
  // through the registry and back
  TestMediator med;
  StandardRegistryPlugin reg("r", {c});
  reg.init_plugin(std::nullopt, &med);
  med.set_registry(reg.new_registry());
  auto got = RegistryConfig::from_registry(med, "config/integrity");
  ASSERT_TRUE(got);
  EXPECT_EQ(std::get<double>(got->values.at("probability")), 0.999);
  EXPECT_EQ(std::get<api::StringArray>(got->values.at("sources"))[1], "/b");
  EXPECT_FALSE(RegistryConfig::from_registry(med, "config/nope"));
}
