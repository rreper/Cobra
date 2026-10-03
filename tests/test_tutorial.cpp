// Tests for the tutorial plugin set (Python: the tutorial cases of test_orchestration.py and the
// tutorial integration apps).
#include <pntos/cobra/EkfFusionStrategyPlugin.hpp>
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/fusion/StandardFusionPlugin.hpp>
#include <pntos/cobra/inertial/StandardInertialPlugin.hpp>
#include <pntos/cobra/preprocessing/StandardPreprocessorPlugin.hpp>
#include <pntos/cobra/state_modeling/Pinson15NedBlock.hpp>
#include <pntos/cobra/tutorial/TutorialPlugins.hpp>
#include <pntos/cobra/utils/navutils.hpp>
#include <pntos/cobra/controller/StandardMessageStreamConfig.hpp>

#include "test_support.hpp"

using namespace pntos;
using namespace pntos::test;
using api::EstimateWithCovariance;
using api::EstimateWithCovarianceType;
using api::Matrix;
using api::Matrix3;
using api::Message;
using api::Timestamp;
using api::Vector;
using api::Vector3;

namespace {
const std::string kImu = "/sensor/imu";
const std::string kPos = "/sensor/pos";
const std::string kVel = "/sensor/vel";
constexpr std::int64_t kS = 1'000'000'000;
constexpr std::int64_t kT0 = 1'700'000'000 * kS;

cobra::ImuConfig imu_model() {
  cobra::ImuConfig m;
  m.group_ = "config/inertial_state";
  m.accel_bias_sigma = {2.4e-3, 2.4e-3, 2.4e-3};
  m.accel_bias_tau = {300.0, 300.0, 300.0};
  m.accel_random_walk_sigma = {3.887e-6, 3.887e-6, 3.887e-6};
  m.gyro_bias_sigma = {2e-4, 2e-4, 2e-4};
  m.gyro_bias_tau = {500.0, 500.0, 500.0};
  m.gyro_random_walk_sigma = {9.9e-4, 9.9e-4, 6.7e-5};
  return m;
}

std::vector<std::shared_ptr<const cobra::BaseConfig>> tutorial_configs(bool with_velocity) {
  auto imu = std::make_shared<cobra::ImuConfig>(imu_model());
  auto align = std::make_shared<cobra::ManualAlignmentConfig>();
  align->group_ = "config/default/alignment";
  align->initial_pos_var = {0.1, 0.1, 0.1};
  align->initial_vel_var = {1e-3, 1e-3, 1e-3};
  align->initial_tilt_var = {5e-4, 5e-4, 5e-4};
  align->initial_accel_bias_var = {5.2e-3, 5.2e-3, 5.2e-3};
  align->initial_gyro_bias_var = {9e-6, 9e-6, 9e-6};
  align->initial_pos = {0.6938996038254822, -1.4679920679462133, 225.493};
  align->initial_rpy = {0.0, 0.0, 0.1};
  align->initial_time = kT0 * 1e-9;
  auto mount = std::make_shared<cobra::MountingConfig>();
  mount->group_ = "config/gp3d_state_modeling";
  mount->lever_arm = {-0.5, 0.38, -0.05};
  auto inertial = std::make_shared<cobra::InertialConfig>();
  inertial->group_ = "config/inertial";
  inertial->expected_dt = 0.01;
  inertial->channels = {kImu};
  inertial->inertial_buffer_length = 10.0;
  auto fogm = std::make_shared<cobra::FogmConfig>();
  fogm->group_ = "config/pos_sensor_error";
  fogm->sigma = {1.5, 1.5, 2.0};
  fogm->tau = {300.0, 300.0, 200.0};
  auto orch = std::make_shared<cobra::TutorialOrchestrationConfig>();
  orch->position_channel = kPos;
  if (with_velocity) orch->velocity_channel = kVel;
  auto ta = std::make_shared<cobra::TimeAdjusterConfig>();
  ta->group_ = "config/time_adjuster";
  ta->channels = std::vector<std::string>{kImu};
  ta->expected_dt_nsec = 10'000'000;
  auto rot = std::make_shared<cobra::ImuRotatorConfig>();
  rot->group_ = "config/imu_rotator";
  rot->channels = std::vector<std::string>{kImu};
  auto tb = std::make_shared<cobra::TimeBiasConfig>();
  tb->group_ = "config/time_bias";
  tb->channels = std::vector<std::string>{kPos, kVel};
  tb->time_bias = 0;
  auto ui = std::make_shared<cobra::UiLogPlottingConfig>();
  ui->logfile = "/nonexistent/tutorial.log";
  ui->solution_channel = "/solution/pntos/pva";
  ui->truth_channel = "/sensor/ins-d/pva";
  return {imu, align, mount, inertial, fogm, orch, ta, rot, tb, ui, std::make_shared<cobra::FusionEngineConfig>()};
}

class TutorialTest : public ::testing::Test {
 protected:
  void set_up(bool with_velocity) {
    registry = std::make_shared<cobra::StandardRegistryPlugin>("registry", tutorial_configs(with_velocity));
    registry->init_plugin(std::nullopt, &med);
    med.set_registry(registry->new_registry());
    init = std::make_shared<cobra::TutorialInitializationPlugin>("init");
    inertial = std::make_shared<cobra::StandardInertialPlugin>("inertial");
    fusion = std::make_shared<cobra::StandardFusionPlugin>("fusion");
    strategy = std::make_shared<cobra::EkfFusionStrategyPlugin>("ekf");
    sm = std::make_shared<cobra::TutorialPosInsStateModelingPlugin>("sm");
    pp = std::make_shared<cobra::StandardPreprocessorPlugin>("pp");
    for (auto& p : api::PluginList{init, inertial, fusion, strategy, sm, pp}) p->init_plugin(std::nullopt, &med);
    if (with_velocity)
      orch = std::make_unique<cobra::TutorialPosVelOrchestrationPlugin>("orchestration");
    else
      orch = std::make_unique<cobra::TutorialPosOrchestrationPlugin>("orchestration");
    orch->init_plugin(std::nullopt, &med);
    orch->init_orchestration_plugin(api::PluginList{fusion, strategy, inertial, init, sm, pp}, stream_config);
  }
  Message imu_msg(std::int64_t tov) {  // integrated (delta-v / delta-theta) samples at 100 Hz
    auto imu = make_imu(tov, Vector3(0, 0, -9.80) * 1e-2, Vector3(0, 0, 0));
    imu->set_imu_type(ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED);
    return Message(imu, kImu);
  }
  Message pos_msg(std::int64_t tov) {
    return Message(make_position(tov, 0.6938996038254822, -1.4679920679462133, 225.493, Matrix::Identity(3, 3)), kPos);
  }
  Message vel_msg(std::int64_t tov) {
    auto v = std::make_shared<aspn23_eigen::MeasurementVelocity>(
        header(ASPN_MEASUREMENT_VELOCITY), aspn23_eigen::TypeTimestamp(tov), ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED,
        0.0, 0.0, 0.0, rm(Matrix::Identity(3, 3) * 0.01), ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE, DynVector(0),
        std::vector<aspn23_eigen::TypeIntegrity>{});
    return Message(v, kVel);
  }

  TestMediator med;
  cobra::StandardMessageStreamConfig stream_config;
  std::shared_ptr<cobra::StandardRegistryPlugin> registry;
  std::shared_ptr<cobra::TutorialInitializationPlugin> init;
  std::shared_ptr<cobra::StandardInertialPlugin> inertial;
  std::shared_ptr<cobra::StandardFusionPlugin> fusion;
  std::shared_ptr<cobra::EkfFusionStrategyPlugin> strategy;
  std::shared_ptr<cobra::TutorialPosInsStateModelingPlugin> sm;
  std::shared_ptr<cobra::StandardPreprocessorPlugin> pp;
  std::unique_ptr<cobra::TutorialOrchestrationPlugin> orch;
};
}  // namespace

TEST(TutorialConfigs, RoundTrip) {
  TestMediator med;
  auto reg = std::make_shared<cobra::StandardRegistryPlugin>("registry", tutorial_configs(true));
  reg->init_plugin(std::nullopt, &med);
  med.set_registry(reg->new_registry());
  auto o = cobra::TutorialOrchestrationConfig::from_registry(med);
  ASSERT_TRUE(o);
  EXPECT_EQ(o->position_channel, kPos);
  EXPECT_EQ(o->velocity_channel, kVel);
  auto u = cobra::UiLogPlottingConfig::from_registry(med);
  ASSERT_TRUE(u);
  EXPECT_EQ(u->solution_channel, "/solution/pntos/pva");
  EXPECT_EQ(u->truth_channel, "/sensor/ins-d/pva");
  EXPECT_FALSE(cobra::TutorialOrchestrationConfig::from_registry(med, "config/missing"));
}

TEST(TutorialStateModeling, ProviderAndModels) {
  TestMediator med;
  auto reg = std::make_shared<cobra::StandardRegistryPlugin>("registry", tutorial_configs(false));
  reg->init_plugin(std::nullopt, &med);
  med.set_registry(reg->new_registry());
  cobra::TutorialPosInsStateModelingPlugin plugin("sm", /*legacy_q_rotation=*/false);
  plugin.init_plugin(std::nullopt, &med);
  auto provider = plugin.new_state_model_provider(api::FusionType::STANDARD);
  ASSERT_TRUE(provider);
  EXPECT_EQ(provider->processor_identifiers(), (std::vector<std::string>{"pinson_velocity", "pinson_with_ned_fogm_position"}));
  EXPECT_EQ(provider->block_identifiers(), (std::vector<std::string>{"pinson15", "fogm"}));
  EXPECT_TRUE(provider->virtual_block_identifiers().empty());
  EXPECT_EQ(provider->new_block(5, nullptr, "x", std::string("config/inertial_state")), nullptr);
  EXPECT_EQ(provider->new_block(0, nullptr, "x", std::nullopt), nullptr);
  EXPECT_EQ(provider->new_processor(1, nullptr, "x", {"a", "b"}, std::string("config/missing")), nullptr);
  EXPECT_TRUE(med.has_error());  // the three negative cases above log ERRORs
  med.logs.clear();

  auto pinson = provider->new_block(0, nullptr, "pinson15", std::string("config/inertial_state"));
  ASSERT_TRUE(pinson);
  EXPECT_EQ(pinson->num_states(), 15u);
  auto* pb = dynamic_cast<cobra::Pinson15NedBlock*>(pinson.get());
  ASSERT_TRUE(pb);
  EXPECT_TRUE(pb->tutorial_model());
  EXPECT_FALSE(pb->legacy_q_rotation());
  auto fogm = provider->new_block(1, nullptr, "pos_fogm", std::string("config/pos_sensor_error"));
  ASSERT_TRUE(fogm);
  EXPECT_EQ(fogm->num_states(), 3u);

  // velocity processor: z = meas - inertial, H = [0 I 0 0 0]
  auto vel = provider->new_processor(0, nullptr, "vel", {"pinson15"}, std::nullopt);
  ASSERT_TRUE(vel);
  Vector q(4);
  q << 1, 0, 0, 0;
  auto pva = make_pva(kT0, 0.7, -1.4, 200, 1, 2, 3, q);
  vel->receive_aux_data({Message(pva, "pva")});
  auto vmsg = std::make_shared<aspn23_eigen::MeasurementVelocity>(
      header(ASPN_MEASUREMENT_VELOCITY), aspn23_eigen::TypeTimestamp(kT0), ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED,
      1.5, 2.5, 3.5, rm(Matrix::Identity(3, 3) * 0.04), ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE, DynVector(0),
      std::vector<aspn23_eigen::TypeIntegrity>{});
  auto gen = [](const std::vector<std::string>&) -> std::optional<EstimateWithCovariance> {
    return EstimateWithCovariance{EstimateWithCovarianceType::EWC_GENERIC, Vector::Zero(18), Matrix::Identity(18, 18)};
  };
  auto vm = vel->generate_model(Message(vmsg, kVel), gen);
  ASSERT_TRUE(vm);
  EXPECT_ALLCLOSE(vm->z, Vector3(0.5, 0.5, 0.5));
  EXPECT_EQ(vm->H.rows(), 3);
  EXPECT_EQ(vm->H.cols(), 15);
  EXPECT_ALLCLOSE(Matrix(vm->H.block(0, 3, 3, 3)), Matrix(Matrix3::Identity()));
  EXPECT_ALLCLOSE(vm->R, Matrix::Identity(3, 3) * 0.04);
  Vector x = Vector::Zero(15);
  x(4) = 2.0;
  EXPECT_ALLCLOSE(vm->h(x), Vector3(0, 2, 0));
  EXPECT_FALSE(vel->generate_model(Message(pva, kVel), gen));

  // position processor: z = NED(meas - inertial), H = [I 0 (C l)^T rows 0 0 | -I]
  auto pos = provider->new_processor(1, nullptr, "pos", {"pinson15", "pos_fogm"}, std::string("config/gp3d_state_modeling"));
  ASSERT_TRUE(pos);
  pos->receive_aux_data({Message(pva, "pva")});
  const double dlat = 1.0 / cobra::nav::delta_lat_to_north(1.0, 0.7, 200);  // 1 m north
  auto pmsg = make_position(kT0, 0.7 + dlat, -1.4, 201, Matrix::Identity(3, 3) * 4);
  auto pm = pos->generate_model(Message(pmsg, kPos), gen);
  ASSERT_TRUE(pm);
  EXPECT_NEAR(pm->z(0), 1.0, 1e-6);
  EXPECT_NEAR(pm->z(1), 0.0, 1e-9);
  EXPECT_NEAR(pm->z(2), -1.0, 1e-9);
  EXPECT_EQ(pm->H.cols(), 18);
  EXPECT_ALLCLOSE(Matrix(pm->H.block(0, 0, 3, 3)), Matrix(Matrix3::Identity()));
  EXPECT_ALLCLOSE(Matrix(pm->H.block(0, 15, 3, 3)), Matrix(-Matrix3::Identity()));
  const Vector3 Cl(-0.5, 0.38, -0.05);  // identity attitude: C l = l, broadcast into each row
  for (int r = 0; r < 3; ++r) EXPECT_ALLCLOSE(Vector(pm->H.row(r).segment(6, 3).transpose()), Vector(Cl));
  Vector x18 = Vector::Zero(18);
  x18(0) = 1;
  x18(15) = 0.25;
  EXPECT_ALLCLOSE(pm->h(x18), Vector3(1 - 0.5 - 0.25, 0.38, -0.05));
  // numeric Jacobian of h in the position / sensor-error columns matches H
  for (int c : {0, 1, 2, 15, 16, 17}) {
    Vector xp = Vector::Zero(18);
    xp(c) = 1e-6;
    EXPECT_ALLCLOSE((pm->h(xp) - pm->h(Vector::Zero(18))) / 1e-6, Vector(pm->H.col(c)));
  }
  EXPECT_FALSE(med.has_error()) << med.last_message();
}

TEST_F(TutorialTest, InitAndStreamConfig) {
  set_up(false);
  EXPECT_FALSE(med.has_error()) << med.last_message();
  ASSERT_TRUE(orch->ready());
  ASSERT_TRUE(orch->fusion_engine());
  EXPECT_EQ(*orch->fusion_engine()->state_block_labels(), (std::vector<std::string>{"pinson15", "pos_fogm"}));
  EXPECT_EQ(orch->fusion_engine()->num_states(), 18u);
  EXPECT_EQ(*orch->fusion_engine()->measurement_processor_labels(), (std::vector<std::string>{"pos"}));
  EXPECT_EQ(orch->fusion_engine()->time().elapsed_nsec, kT0);
  EXPECT_FALSE(stream_config.is_sequenced(ASPN_MEASUREMENT_IMU));
  EXPECT_TRUE(stream_config.is_sequenced(ASPN_MEASUREMENT_POSITION));
  auto P = orch->fusion_engine()->get_state_block_covariance("pinson15");
  ASSERT_TRUE(P);
  EXPECT_NEAR((*P)(0, 0), 0.1, 1e-15);
  EXPECT_NEAR((*P)(14, 14), 9e-6, 1e-20);
  EXPECT_EQ(orch->filter_description_list().size(), 1u);
}

TEST_F(TutorialTest, PosVelAddsVelocityProcessor) {
  set_up(true);
  EXPECT_FALSE(med.has_error()) << med.last_message();
  EXPECT_EQ(*orch->fusion_engine()->measurement_processor_labels(), (std::vector<std::string>{"pos", "vel"}));
}

TEST_F(TutorialTest, MissingPluginsIsAnError) {
  registry = std::make_shared<cobra::StandardRegistryPlugin>("registry", tutorial_configs(false));
  registry->init_plugin(std::nullopt, &med);
  med.set_registry(registry->new_registry());
  cobra::TutorialPosOrchestrationPlugin o("orchestration");
  o.init_plugin(std::nullopt, &med);
  o.init_orchestration_plugin(api::PluginList{}, stream_config);
  EXPECT_FALSE(o.ready());
  EXPECT_TRUE(med.has_error());
  EXPECT_FALSE(o.request_solutions({Timestamp{kT0}}));
}

TEST_F(TutorialTest, ProcessesImuAndMeasurementsWithFeedback) {
  set_up(true);
  ASSERT_TRUE(orch->ready());
  // 2 s of IMU at 100 Hz, then a position fix and a velocity fix 1 s and 2 s in
  for (int i = 0; i <= 200; ++i) orch->process_pntos_message(imu_msg(kT0 + i * 10'000'000), false);
  const Timestamp t1{kT0 + 1 * kS};
  orch->process_pntos_message(pos_msg(t1.elapsed_nsec), true);
  EXPECT_EQ(orch->fusion_engine()->time().elapsed_nsec, t1.elapsed_nsec);
  // feedback zeroes the Pinson states after every update
  auto x = orch->fusion_engine()->get_state_block_estimate("pinson15");
  ASSERT_TRUE(x);
  EXPECT_NEAR(x->cwiseAbs().maxCoeff(), 0.0, 1e-15);
  const Timestamp t2{kT0 + 2 * kS};
  auto before = orch->fusion_engine()->peek_ahead(t2, {"pinson15"});
  ASSERT_TRUE(before);
  orch->process_pntos_message(vel_msg(t2.elapsed_nsec), true);
  EXPECT_EQ(orch->fusion_engine()->time().elapsed_nsec, t2.elapsed_nsec);
  // the velocity update shrank the velocity uncertainty relative to the propagated prior
  auto P = orch->fusion_engine()->get_state_block_covariance("pinson15");
  ASSERT_TRUE(P);
  EXPECT_LT((*P)(3, 3), before->covariance(3, 3));
  EXPECT_LT((*P)(4, 4), before->covariance(4, 4));
  // solutions come from the latest inertial time
  auto sols = orch->request_solutions({t2});
  ASSERT_TRUE(sols);
  ASSERT_EQ(sols->size(), 1u);
  ASSERT_TRUE((*sols)[0]);
  EXPECT_EQ((*sols)[0]->source_identifier, "/solution/pntos/pva");
  auto pva = (*sols)[0]->as<cobra::utils::PVA>();
  ASSERT_TRUE(pva);
  EXPECT_EQ(pva->get_time_of_validity().get_elapsed_nsec(), t2.elapsed_nsec);
  EXPECT_EQ(pva->get_covariance().rows(), 9);
  // unknown channels are ignored
  orch->process_pntos_message(Message(pos_msg(t2.elapsed_nsec).wrapped_message, "/other"), true);
  EXPECT_FALSE(med.has_error()) << med.last_message();
}

TEST(UiLogPlottingPlugin, MissingAndInvalidLogsOnlyWarn) {
  TestMediator med;
  auto cfg = std::make_shared<cobra::UiLogPlottingConfig>();
  cfg->logfile = "/nonexistent/dir/out.log";
  cfg->solution_channel = "/solution/pntos/pva";
  cfg->truth_channel = "/sensor/ins-d/pva";
  auto reg = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{cfg});
  reg->init_plugin(std::nullopt, &med);
  med.set_registry(reg->new_registry());
  cobra::UiLogPlottingPlugin ui("ui");
  ui.init_plugin(std::nullopt, &med);
  EXPECT_FALSE(ui.requires_main_thread());
  ui.shutdown_plugin();
  EXPECT_EQ(med.count(api::LoggingLevel::WARN), 1u);
  EXPECT_FALSE(med.has_error());
  // no config at all -> ERROR at init, nothing at shutdown
  TestMediator med2;
  auto reg2 = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{});
  reg2->init_plugin(std::nullopt, &med2);
  med2.set_registry(reg2->new_registry());
  cobra::UiLogPlottingPlugin ui2("ui");
  ui2.init_plugin(std::nullopt, &med2);
  EXPECT_TRUE(med2.has_error());
  ui2.shutdown_plugin();
}
