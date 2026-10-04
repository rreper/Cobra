// Port of pntos-cobra/tests/test_orchestration.py (standard orchestration cases). The inertial and
// initialization plugins are mocked until the NavToolkit-backed ports land; everything else is real.
#include <pntos/cobra/EkfFusionStrategyPlugin.hpp>
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/controller/StandardMessageStreamConfig.hpp>
#include <pntos/cobra/fusion/StandardFusionPlugin.hpp>
#include <pntos/cobra/orchestration/StandardOrchestrationPlugin.hpp>
#include <pntos/cobra/state_modeling/StandardStateModelingPlugin.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include "test_support.hpp"

#include <aspn23/eigen/MeasurementAltitude.hpp>

using namespace pntos;
using namespace pntos::test;
using api::EstimateWithCovariance;
using api::EstimateWithCovarianceType;
using api::LoggingLevel;
using api::Matrix;
using api::Message;
using api::Timestamp;
using api::Vector;
using api::Vector3;
namespace nav = cobra::nav;

namespace {
constexpr std::int64_t kSec = 1'000'000'000;
const std::string kBestSol = "/solution/pntos/pva";
const std::string kImuSol = "/solution/pntos-imu/pva";
const std::string kImuChannel = "/sensor/vn-100/imu";
const std::string kPosChannel = "/sensor/ublox-ZED-F9T/position";

cobra::ManualAlignmentConfig align_config() {
  cobra::ManualAlignmentConfig a;
  a.group_ = "config/default/alignment";
  a.initial_pos_var = {0.1, 0.1, 0.1};
  a.initial_vel_var = {1e-3, 1e-3, 1e-3};
  a.initial_tilt_var = {5e-4, 5e-4, 5e-4};
  a.initial_accel_bias_var = {1e-10, 1e-10, 1e-10};
  a.initial_gyro_bias_var = {1e-15, 1e-15, 1e-15};
  a.initial_accel_bias = {-0.00212767, 0.00059081, -0.05242679};
  a.initial_gyro_bias = {-0.00165402, -0.00157491, -0.00133498};
  a.initial_pos = {0.6939183923297865, -1.4680111371692746, 222.561};
  a.initial_rpy = {0.0012876203558051373, -0.05315453753188288, 0.10972323851917268};
  a.initial_time = 0.0;
  a.initial_vel = {0.0, 0.0, 0.0};
  return a;
}

cobra::ImuConfig imu_config() {
  cobra::ImuConfig c;
  c.group_ = "config/inertial_state";
  c.accel_bias_sigma = {2.4e-3, 2.4e-3, 2.4e-3};
  c.accel_bias_tau = {300, 300, 300};
  c.accel_random_walk_sigma = {3.887e-6, 3.887e-6, 3.887e-6};
  c.gyro_bias_sigma = {2e-4, 2e-4, 2e-4};
  c.gyro_bias_tau = {500, 500, 500};
  c.gyro_random_walk_sigma = {9.9e-4, 9.9e-4, 6.7e-5};
  return c;
}

cobra::InertialConfig inertial_config() {
  cobra::InertialConfig c;
  c.group_ = "config/inertial";
  c.expected_dt = 0.01;
  c.channels = {kImuChannel};
  c.inertial_buffer_length = 10.0;
  return c;
}

/// The initial PVA the mock initializer hands out (what expected_pva_best encodes in Python).
std::shared_ptr<cobra::utils::PVA> initial_pva() {
  auto a = align_config();
  Vector3 llh(a.initial_pos[0], a.initial_pos[1], a.initial_pos[2]);
  Vector3 vel(a.initial_vel[0], a.initial_vel[1], a.initial_vel[2]);
  nav::Vector4 q = nav::rpy_to_quat(Vector3(a.initial_rpy[0], a.initial_rpy[1], a.initial_rpy[2]));
  Vector d(9);
  d << 0.1, 0.1, 0.1, 1e-3, 1e-3, 1e-3, 5e-4, 5e-4, 5e-4;
  return cobra::utils::make_pva(aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0),
                                Timestamp{static_cast<std::int64_t>(a.initial_time * 1e9)}, llh, vel, q,
                                Matrix(d.asDiagonal()));
}

// ----------------------------------------------------------------------------- mocks

/// Holds one PVA; "mechanizes" by keeping it constant. Tracks the time span of IMU data it saw.
class MockInertial final : public api::StandardInertialMechanization {
 public:
  explicit MockInertial(std::shared_ptr<cobra::utils::PVA> pva) : pva_(std::move(pva)) {
    earliest_ = latest_ = cobra::utils::time_of_validity(*pva_).value_or(Timestamp{0});
  }
  api::AspnMessageType request_solution_message_type() const override {
    return ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE;
  }
  Message request_current_solution() override { return Message(at(latest_), "mock"); }
  std::optional<Message> request_solution(Timestamp t) override { return Message(at(t), "mock"); }
  std::optional<std::vector<std::optional<Message>>> request_solutions(const std::vector<Timestamp>& ts,
                                                                       api::InertialSolutionRangeType) override {
    std::vector<std::optional<Message>> out;
    for (auto t : ts) out.push_back(request_solution(t));
    return out;
  }
  bool is_time_in_range(Timestamp t) const override {
    return t.elapsed_nsec >= earliest_.elapsed_nsec && t.elapsed_nsec <= latest_.elapsed_nsec;
  }
  Timestamp request_earliest_time() const override { return earliest_; }
  Timestamp request_latest_time() const override { return latest_; }
  std::vector<api::AspnMessageType> request_process_pntos_message_types() const override {
    return {ASPN_MEASUREMENT_IMU};
  }
  void process_pntos_message(const Message& m) override {
    ++imu_count;
    if (auto t = cobra::utils::time_of_validity(*m.wrapped_message))
      if (t->elapsed_nsec > latest_.elapsed_nsec) latest_ = *t;
  }
  std::optional<api::InertialForcesRates> request_forces_and_rates(Timestamp t) override {
    api::InertialForcesRates f;
    f.forces_and_rates = make_imu(t.elapsed_nsec, Vector3(0, 0, -9.81), Vector3::Zero());
    return f;
  }
  std::optional<api::InertialForcesRates> request_average_forces_and_rates(Timestamp t1, Timestamp) override {
    return request_forces_and_rates(t1);
  }
  std::optional<std::vector<api::AspnMessageType>> request_reset_message_types() const override {
    return std::vector<api::AspnMessageType>{ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE};
  }
  void reset_solution(const Message& m) override {
    ++resets;
    auto p = m.as<cobra::utils::PVA>();
    if (p) pva_ = cobra::utils::copy_pva(*p);
  }
  void correct_sensor_errors(Timestamp, const api::StandardInertialErrors& e) override {
    ++corrections;
    errors_ = e;
  }
  std::optional<api::StandardInertialErrors> request_sensor_errors(Timestamp) override { return errors_; }

  int imu_count = 0, resets = 0, corrections = 0;
  api::StandardInertialErrors errors_;

 private:
  std::shared_ptr<cobra::utils::PVA> at(Timestamp t) const {
    auto p = cobra::utils::copy_pva(*pva_);
    p->set_time_of_validity(aspn23_eigen::TypeTimestamp(t.elapsed_nsec));
    p->set_covariance(Matrix::Zero(9, 9));
    return p;
  }
  std::shared_ptr<cobra::utils::PVA> pva_;
  Timestamp earliest_, latest_;
};

class MockInertialPlugin final : public api::InertialPlugin {
 public:
  explicit MockInertialPlugin(std::string id) : id_(std::move(id)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* m) override { mediator = m; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return id_; }
  bool is_inertial_type_supported(api::InertialType t) const override {
    return t == api::InertialType::STANDARD_MECHANIZATION;
  }
  std::unique_ptr<api::CommonInertial> new_inertial(api::InertialType, const Message& solution,
                                                    const std::optional<std::string>& group) override {
    last_group = group;
    auto p = solution.as<cobra::utils::PVA>();
    auto inertial = std::make_unique<MockInertial>(cobra::utils::copy_pva(*p));
    last = inertial.get();
    return inertial;
  }
  api::Mediator* mediator = nullptr;
  MockInertial* last = nullptr;
  std::optional<std::string> last_group;

 private:
  std::string id_;
};

/// Immediately INITIALIZED_GOOD, or after `messages_needed` alignment messages.
class MockInitializer final : public api::InertialInitializationStrategy {
 public:
  explicit MockInitializer(int messages_needed) : needed_(messages_needed) {}
  api::InitializationMotionNeeded request_motion_needed() const override {
    return api::InitializationMotionNeeded::ANY_MOTION;
  }
  api::InitializationStatus request_current_status() const override {
    return seen_ >= needed_ ? api::InitializationStatus::INITIALIZED_GOOD : api::InitializationStatus::WAITING;
  }
  void process_pntos_message(const Message& m) override {
    ++seen_;
    if (auto t = cobra::utils::time_of_validity(*m.wrapped_message)) last_time_ = *t;
  }
  api::InitialInertialSolution request_solution() override {
    api::InitialInertialSolution s;
    auto pva = initial_pva();
    pva->set_time_of_validity(aspn23_eigen::TypeTimestamp(last_time_.elapsed_nsec));
    s.solution = Message(pva, "mock init");
    api::StandardInertialErrors e;
    auto a = align_config();
    e.accel_biases = Vector3(a.initial_accel_bias[0], a.initial_accel_bias[1], a.initial_accel_bias[2]);
    e.gyro_biases = Vector3(a.initial_gyro_bias[0], a.initial_gyro_bias[1], a.initial_gyro_bias[2]);
    s.inertial_errors = e;
    Vector d(6);
    d << 1e-10, 1e-10, 1e-10, 1e-15, 1e-15, 1e-15;
    s.inertial_error_covariance = Matrix(d.asDiagonal());
    s.status = request_current_status();
    return s;
  }

 private:
  int needed_;
  int seen_ = 0;
  Timestamp last_time_{0};
};

class MockInitializationPlugin final : public api::InitializationPlugin {
 public:
  MockInitializationPlugin(std::string id, int messages_needed = 0) : id_(std::move(id)), needed_(messages_needed) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* m) override { mediator = m; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return id_; }
  bool is_initialization_type_supported(api::InitializationType t) const override {
    return t == api::InitializationType::INERTIAL;
  }
  std::unique_ptr<api::CommonInitializationStrategy> new_initialization_strategy(
      api::InitializationType, const std::optional<std::string>& group) override {
    last_group = group;
    return std::make_unique<MockInitializer>(needed_);
  }
  api::Mediator* mediator = nullptr;
  std::optional<std::string> last_group;

 private:
  std::string id_;
  int needed_;
};

/// Python's MockMP / MockVSB / MockStateModelProvider: records calls instead of printing.
struct MockLog {
  std::vector<std::string> events;
};

class MockMP final : public api::StandardMeasurementProcessor {
 public:
  MockMP(std::string label, MockLog* log) : label_(std::move(label)), log_(log) {}
  const std::string& label() const override { return label_; }
  const std::vector<std::string>& state_block_labels() const override { return labels_; }
  void receive_aux_data(const api::AuxData&) override {}
  std::optional<api::StandardMeasurementModel> generate_model(const Message&, const api::GenXandP&) override {
    log_->events.push_back(label_ + " received message");
    return std::nullopt;
  }
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override { return std::make_unique<MockMP>(*this); }

 private:
  std::string label_;
  std::vector<std::string> labels_;
  MockLog* log_;
};

class MockVSB final : public api::VirtualStateBlock {
 public:
  MockVSB(std::string s, std::string t, MockLog* log) : source_(std::move(s)), target_(std::move(t)), log_(log) {}
  const std::string& source() const override { return source_; }
  const std::string& target() const override { return target_; }
  void receive_aux_data(const api::AuxData&) override { log_->events.push_back(target_ + " got aux"); }
  EstimateWithCovariance convert(const EstimateWithCovariance& e, Timestamp) override { return e; }
  Vector convert_estimate(const Vector& e, Timestamp) override { return e; }
  Matrix jacobian(const Vector& e, Timestamp) override { return Matrix::Identity(e.size(), e.size()); }
  std::unique_ptr<api::VirtualStateBlock> clone() const override { return std::make_unique<MockVSB>(*this); }

 private:
  std::string source_, target_;
  MockLog* log_;
};

class MockProvider final : public api::StandardStateModelProvider {
 public:
  explicit MockProvider(MockLog* log) : log_(log) {}
  const std::vector<std::string>& processor_identifiers() const override { return mps_; }
  const std::vector<std::string>& block_identifiers() const override { return blocks_; }
  const std::vector<std::string>& virtual_block_identifiers() const override { return vsbs_; }
  std::unique_ptr<api::StandardMeasurementProcessor> new_processor(std::size_t, api::StandardFusionEngine*,
                                                                   const std::string& label,
                                                                   const std::vector<std::string>&,
                                                                   const std::optional<std::string>&) override {
    return std::make_unique<MockMP>(label, log_);
  }
  std::unique_ptr<api::StandardStateBlock> new_block(std::size_t, api::StandardFusionEngine*, const std::string&,
                                                     const std::optional<std::string>&) override {
    return nullptr;
  }
  std::unique_ptr<api::VirtualStateBlock> new_virtual_block(std::size_t, const std::string& s, const std::string& t,
                                                            const std::optional<std::string>&) override {
    return std::make_unique<MockVSB>(s, t, log_);
  }

 private:
  std::vector<std::string> mps_{"mock_mp"}, blocks_{}, vsbs_{"mock_vsb"};
  MockLog* log_;
};

class MockStateModelingPlugin final : public api::StateModelingPlugin {
 public:
  explicit MockStateModelingPlugin(MockLog* log) : log_(log) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator*) override {}
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return id_; }
  bool is_fusion_type_supported(api::FusionType) const override { return false; }
  std::unique_ptr<api::StandardStateModelProvider> new_state_model_provider(api::FusionType) override {
    return std::make_unique<MockProvider>(log_);
  }

 private:
  std::string id_ = "Mock State Modeling Plugin";
  MockLog* log_;
};

// ----------------------------------------------------------------------------- config builders

std::shared_ptr<cobra::StandardOrchestrationConfig> standard_orch_config(bool manual_fogm = false,
                                                                         bool real_position_mp = false) {
  auto c = std::make_shared<cobra::StandardOrchestrationConfig>();
  c->best_sol_channel = kBestSol;
  c->imu_sol_channel = kImuSol;
  c->alignment_channels = {kPosChannel, kImuChannel};
  c->pinson_sb_config.group_ = "config/pinson_block";
  c->pinson_sb_config.label = "pinson15";
  c->pinson_sb_config.imu_model = imu_config();

  auto fogm = std::make_shared<cobra::FogmStateBlockConfig>();
  fogm->group_ = "config/pos_fogm_block";
  fogm->label = "pos_sensor_error";
  fogm->estimate_with_covariance =
      EstimateWithCovariance{EstimateWithCovarianceType::EWC_GENERIC, Vector::Zero(3),
                             Matrix::Identity(3, 3) * (manual_fogm ? 1.0 : 9.0)};
  fogm->fogm_model.group_ = "config/pos_sensor_error";
  fogm->fogm_model.sigma = {1.5, 1.5, 2.0};
  fogm->fogm_model.tau = {300, 300, 200};
  c->additional_sb_configs = std::vector<std::shared_ptr<const cobra::StateBlockConfig>>{fogm};

  std::vector<std::shared_ptr<const cobra::MeasurementProcessorConfig>> mps;
  if (real_position_mp) {
    auto mp = std::make_shared<cobra::LeverArmMPConfig>(cobra::mp::PinsonWithNedFogmPositionMPConfig());
    mp->group_ = "config/gps_mp";
    mp->label = "gps_position";
    mp->channel = kPosChannel;
    mp->state_block_labels = {"pinson15", "pos_sensor_error"};
    mp->lever_arm = {0, 0, 0};
    mps.push_back(mp);
  } else {
    auto mp1 = std::make_shared<cobra::MeasurementProcessorConfig>();
    mp1->group_ = "config/mock_mp1";
    mp1->identifier = "mock_mp";
    mp1->label = "mock_mp1";
    mp1->state_block_labels = {"mock_vsb2"};
    mp1->channel = "mock_channel";
    auto mp2 = std::make_shared<cobra::MeasurementProcessorConfig>(*mp1);
    mp2->group_ = "config/mock_mp2";
    mp2->label = "mock_mp2";
    mp2->state_block_labels = {"mock_sb"};
    mps = {mp1, mp2};
    auto v1 = std::make_shared<cobra::VirtualStateBlockConfig>();
    v1->group_ = "config/mock_vsb1";
    v1->identifier = "mock_vsb";
    v1->source = "pinson15";
    v1->target = "mock_vsb1";
    v1->aux_channels = std::vector<std::string>{cobra::kAuxInertialPva};
    auto v2 = std::make_shared<cobra::VirtualStateBlockConfig>();
    v2->group_ = "config/mock_vsb2";
    v2->identifier = "mock_vsb";
    v2->source = "mock_vsb1";
    v2->target = "mock_vsb2";
    c->vsb_configs = std::vector<std::shared_ptr<const cobra::VirtualStateBlockConfig>>{v1, v2};
  }
  c->mp_configs = mps;
  c->inertial_config = inertial_config();
  c->alignment_config = std::make_shared<cobra::ManualAlignmentConfig>(align_config());
  return c;
}

// ----------------------------------------------------------------------------- fixture

class OrchestrationTest : public ::testing::Test {
 protected:
  void set_up(std::shared_ptr<cobra::StandardOrchestrationConfig> orch_cfg, int align_messages_needed = 0) {
    std::vector<std::shared_ptr<const cobra::BaseConfig>> configs{
        std::make_shared<cobra::ControllerConfig>(), std::make_shared<cobra::FusionEngineConfig>(), orch_cfg};
    registry = std::make_shared<cobra::StandardRegistryPlugin>("registry", configs);
    registry->init_plugin(std::nullopt, &med);
    med.set_registry(registry->new_registry());

    init = std::make_shared<MockInitializationPlugin>("init", align_messages_needed);
    inertial = std::make_shared<MockInertialPlugin>("inertial");
    fusion = std::make_shared<cobra::StandardFusionPlugin>("fusion");
    strategy = std::make_shared<cobra::EkfFusionStrategyPlugin>("ekf");
    sm = std::make_shared<cobra::StandardStateModelingPlugin>("sm");
    mock_sm = std::make_shared<MockStateModelingPlugin>(&mock_log);
    orch = std::make_unique<cobra::StandardOrchestrationPlugin>("orchestration");
    for (auto& p : api::PluginList{init, inertial, fusion, strategy, sm, mock_sm}) p->init_plugin(std::nullopt, &med);
    orch->init_plugin(std::nullopt, &med);
    orch->init_orchestration_plugin(api::PluginList{fusion, strategy, inertial, init, sm, mock_sm}, stream_config);
  }

  Message imu_msg(std::int64_t tov) {
    return Message(make_imu(tov, Vector3(1e-12, 1e-12, -9.81) * 1e-2, Vector3(1e-12, 1e-4, 1e-12) * 1e-2), kImuChannel);
  }
  Message pos_msg(std::int64_t tov, double lat = 1, double lon = 2, double alt = 3) {
    return Message(make_position(tov, lat, lon, alt, Matrix::Identity(3, 3)), kPosChannel);
  }
  Message altitude_on(const std::string& channel) {
    auto alt = std::make_shared<aspn23_eigen::MeasurementAltitude>(
        header(ASPN_MEASUREMENT_ALTITUDE), aspn23_eigen::TypeTimestamp(std::int64_t{0}),
        ASPN23_MEASUREMENT_ALTITUDE_REFERENCE_MSL, 100.0, 10.0, ASPN23_MEASUREMENT_ALTITUDE_ERROR_MODEL_NONE,
        DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});
    return Message(alt, channel);
  }
  bool saw(const std::string& event) const {
    return std::find(mock_log.events.begin(), mock_log.events.end(), event) != mock_log.events.end();
  }

  TestMediator med;
  MockLog mock_log;
  cobra::StandardMessageStreamConfig stream_config;
  std::shared_ptr<cobra::StandardRegistryPlugin> registry;
  std::shared_ptr<MockInitializationPlugin> init;
  std::shared_ptr<MockInertialPlugin> inertial;
  std::shared_ptr<cobra::StandardFusionPlugin> fusion;
  std::shared_ptr<cobra::EkfFusionStrategyPlugin> strategy;
  std::shared_ptr<cobra::StandardStateModelingPlugin> sm;
  std::shared_ptr<MockStateModelingPlugin> mock_sm;
  std::unique_ptr<cobra::StandardOrchestrationPlugin> orch;
};

TEST_F(OrchestrationTest, InitStandard) {
  set_up(standard_orch_config());
  EXPECT_FALSE(med.has_error()) << med.last_message();
  ASSERT_TRUE(orch->is_initialized());
  ASSERT_TRUE(orch->fusion_engine());
  EXPECT_EQ(*orch->fusion_engine()->state_block_labels(), (std::vector<std::string>{"pos_sensor_error", "pinson15"}));
  EXPECT_EQ(orch->fusion_engine()->num_states(), 18u);
  EXPECT_EQ(orch->fusion_engine()->measurement_processor_labels()->size(), 2u);
  EXPECT_TRUE(orch->fusion_engine()->has_virtual_state_block("mock_vsb2"));
  EXPECT_EQ(init->last_group, "config/default/alignment");
  EXPECT_EQ(inertial->last_group, "config/inertial");
  EXPECT_EQ(inertial->last->corrections, 1);  // initial biases applied
  // stream config: default sequenced, IMU immediate
  EXPECT_FALSE(stream_config.is_sequenced(ASPN_MEASUREMENT_IMU));
  EXPECT_TRUE(stream_config.is_sequenced(ASPN_MEASUREMENT_POSITION));
  // Pinson covariance = blockdiag(PVA cov, bias cov)
  auto P = orch->fusion_engine()->get_state_block_covariance("pinson15");
  ASSERT_TRUE(P);
  EXPECT_NEAR((*P)(0, 0), 0.1, 1e-15);
  EXPECT_NEAR((*P)(8, 8), 5e-4, 1e-15);
  EXPECT_NEAR((*P)(9, 9), 1e-10, 1e-25);
  EXPECT_NEAR((*P)(14, 14), 1e-15, 1e-30);
}

TEST_F(OrchestrationTest, InitManualFogm) {
  set_up(standard_orch_config(/*manual_fogm=*/true));
  EXPECT_FALSE(med.has_error()) << med.last_message();
  auto P = orch->fusion_engine()->get_state_block_covariance("pos_sensor_error");
  ASSERT_TRUE(P);
  EXPECT_ALLCLOSE(*P, Matrix::Identity(3, 3));
}

TEST_F(OrchestrationTest, ConfigRoundTrip) {
  set_up(standard_orch_config());
  auto back = cobra::StandardOrchestrationConfig::from_registry(med);
  ASSERT_TRUE(back);
  EXPECT_EQ(back->best_sol_channel, kBestSol);
  EXPECT_EQ(back->alignment_channels, (std::vector<std::string>{kPosChannel, kImuChannel}));
  EXPECT_EQ(back->pinson_sb_config.label, "pinson15");
  EXPECT_DOUBLE_EQ(back->pinson_sb_config.imu_model.accel_bias_tau[0], 300);
  ASSERT_TRUE(back->additional_sb_configs);
  EXPECT_EQ((*back->additional_sb_configs)[0]->identifier, "fogm");
  ASSERT_TRUE(back->mp_configs);
  EXPECT_EQ((*back->mp_configs)[1]->state_block_labels, std::vector<std::string>{"mock_sb"});
  ASSERT_TRUE(back->vsb_configs);
  EXPECT_EQ((*back->vsb_configs)[0]->aux_channels, std::vector<std::string>{"INERTIAL_PVA"});
  EXPECT_EQ(back->alignment_config_group, "config/default/alignment");
  EXPECT_EQ(back->inertial_config.channels, std::vector<std::string>{kImuChannel});
  EXPECT_FALSE(back->feedback_config);
  EXPECT_FALSE(back->preprocessor_configs);
  EXPECT_DOUBLE_EQ(back->max_prop_interval, 2.0);
  ASSERT_TRUE(back->stream_config.override_streams);
  EXPECT_EQ((*back->stream_config.override_streams)[0].message_type, ASPN_MEASUREMENT_IMU);
}

TEST_F(OrchestrationTest, ProcessMessageOnUnknownChannel) {
  set_up(standard_orch_config());
  Vector nanq = Vector::Constant(4, std::nan(""));
  auto pva = make_pva(0, std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan(""), nanq,
                      Matrix(0, 0));
  pva->set_reference_frame(ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_ECI);
  orch->process_pntos_message(Message(pva, "Genesis planet"), false);
  EXPECT_FALSE(med.has_error()) << med.last_message();
}

TEST_F(OrchestrationTest, OneChannelMultipleMps) {
  set_up(standard_orch_config());
  orch->process_pntos_message(altitude_on("mock_channel"), false);
  EXPECT_TRUE(saw("mock_mp1 received message"));
  EXPECT_TRUE(saw("mock_mp2 received message"));
}

TEST_F(OrchestrationTest, AuxForVsbChain) {
  set_up(standard_orch_config());
  // pinson15 -> mock_vsb1 (needs PVA) -> mock_vsb2 <- mock_mp1
  EXPECT_EQ(orch->vsbs_needing_pva().at("mock_mp1"), std::vector<std::string>{"mock_vsb1"});
  EXPECT_TRUE(orch->vsbs_needing_pva().at("mock_mp2").empty());
  orch->process_pntos_message(altitude_on("mock_channel"), false);
  EXPECT_TRUE(saw("mock_vsb1 got aux"));
}

TEST_F(OrchestrationTest, OutageFilterPropagation) {
  set_up(standard_orch_config());
  const auto ic = inertial_config();
  const int n = static_cast<int>(ic.inertial_buffer_length / ic.expected_dt);
  for (int i = 0; i < n; ++i) orch->process_pntos_message(imu_msg(i * static_cast<std::int64_t>(ic.expected_dt * 1e9)), false);
  EXPECT_EQ(inertial->last->imu_count, n);
  EXPECT_EQ(orch->fusion_engine()->time().elapsed_nsec, orch->inertial_drift_prop_dt());
  EXPECT_FALSE(med.has_error()) << med.last_message();
}

TEST_F(OrchestrationTest, JustImuAndPostInitialization) {
  set_up(standard_orch_config());
  orch->process_pntos_message(imu_msg(100), false);
  EXPECT_EQ(inertial->last->imu_count, 1);
  // A PVA on the position channel: not a measurement channel in this config, so nothing happens.
  orch->process_pntos_message(Message(make_pva(100, 0.1, 0.2, 0.3, 1.1, 1.2, 1.3, vec({0, 0.1, 0.2, 0.3})), kPosChannel),
                              false);
  EXPECT_FALSE(med.has_error()) << med.last_message();
  // Old messages are dropped (DEBUG log, no error).
  orch->process_pntos_message(imu_msg(-5), false);
  EXPECT_EQ(inertial->last->imu_count, 1);
}

TEST_F(OrchestrationTest, FilterDescriptionList) {
  set_up(standard_orch_config());
  EXPECT_EQ(orch->filter_description_list(),
            (std::vector<std::string>{"POS_INS_BEST_ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ESTIMATE",
                                      "POS_INS_DEAD_RECKONING_ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ESTIMATE"}));
}

TEST_F(OrchestrationTest, RequestSolutions) {
  set_up(standard_orch_config());
  auto expected = initial_pva();
  // BEST (default) at the initial time equals the initial PVA with the alignment covariance.
  auto sols = orch->request_solutions({Timestamp{0}});
  ASSERT_TRUE(sols && sols->size() == 1 && (*sols)[0]);
  auto best = (*sols)[0]->as<cobra::utils::PVA>();
  ASSERT_TRUE(best);
  EXPECT_EQ((*sols)[0]->source_identifier, kBestSol);
  EXPECT_NEAR(best->get_p1(), expected->get_p1(), 1e-15);
  EXPECT_NEAR(best->get_p2(), expected->get_p2(), 1e-15);
  EXPECT_NEAR(best->get_p3(), expected->get_p3(), 1e-9);
  EXPECT_ALLCLOSE(Vector(best->get_quaternion()), Vector(expected->get_quaternion()));
  EXPECT_ALLCLOSE(Matrix(best->get_covariance()), Matrix(expected->get_covariance()));
  // By description
  for (const auto& d : orch->filter_description_list()) {
    auto s = orch->request_solutions({Timestamp{0}}, d);
    ASSERT_TRUE(s && (*s)[0]) << d;
    auto p = (*s)[0]->as<cobra::utils::PVA>();
    if (d.find("DEAD_RECKONING") != std::string::npos) {
      EXPECT_EQ((*s)[0]->source_identifier, kImuSol);
      EXPECT_ALLCLOSE(Matrix(p->get_covariance()), Matrix::Zero(9, 9));
    } else {
      EXPECT_EQ((*s)[0]->source_identifier, kBestSol);
    }
  }
  EXPECT_FALSE(orch->request_solutions({Timestamp{0}}, "NOPE").has_value());
  EXPECT_FALSE(orch->request_solutions({Timestamp{0}, Timestamp{1}}).has_value());
  // Out-of-range time is replaced by the latest inertial time (DEBUG only).
  orch->process_pntos_message(imu_msg(3 * kSec), false);
  auto late = orch->request_solutions({Timestamp{10 * kSec}});
  ASSERT_TRUE(late && (*late)[0]);
  EXPECT_EQ(cobra::utils::time_of_validity(*(*late)[0]->wrapped_message)->elapsed_nsec, 3 * kSec);
}

TEST_F(OrchestrationTest, AlignsAfterEnoughMessages) {
  set_up(standard_orch_config(), /*align_messages_needed=*/5);
  EXPECT_FALSE(orch->is_initialized());
  EXPECT_FALSE(orch->request_solutions({Timestamp{0}}).has_value());
  for (int i = 0; i < 4; ++i) orch->process_pntos_message(imu_msg(i * 10'000'000), false);
  EXPECT_FALSE(orch->is_initialized());
  orch->process_pntos_message(pos_msg(40'000'000), false);  // 5th alignment-channel message
  ASSERT_TRUE(orch->is_initialized());
  EXPECT_EQ(orch->initializer()->request_current_status(), api::InitializationStatus::INITIALIZED_GOOD);
  EXPECT_EQ(orch->fusion_engine()->time().elapsed_nsec, 40'000'000);
  EXPECT_FALSE(med.has_error()) << med.last_message();
}

TEST_F(OrchestrationTest, EndToEndPositionUpdateWithFeedback) {
  set_up(standard_orch_config(false, /*real_position_mp=*/true));
  ASSERT_TRUE(orch->is_initialized());
  EXPECT_EQ(orch->measurement_channels().at(kPosChannel), std::vector<std::string>{"gps_position"});
  auto p0 = initial_pva();
  // 1 s of IMU, then a position 10 m north / 5 m up of the inertial solution.
  for (int i = 1; i <= 100; ++i) orch->process_pntos_message(imu_msg(i * 10'000'000), false);
  const double lat = p0->get_p1() + nav::north_to_delta_lat(10.0, p0->get_p1(), p0->get_p3());
  orch->process_pntos_message(pos_msg(kSec, lat, p0->get_p2(), p0->get_p3() + 5.0), false);
  EXPECT_FALSE(med.has_error()) << med.last_message();
  EXPECT_EQ(orch->fusion_engine()->time().elapsed_nsec, kSec);
  // Feedback: the inertial was reset with the corrected solution and the pinson states zeroed.
  EXPECT_EQ(inertial->last->resets, 1);
  EXPECT_EQ(inertial->last->corrections, 2);
  auto x = orch->fusion_engine()->get_state_block_estimate("pinson15");
  ASSERT_TRUE(x);
  EXPECT_ALLCLOSE(*x, Vector::Zero(15));
  // The reset moved the inertial solution towards the measurement (prior 0.1 m² vs R 1 m² + FOGM 9 m²).
  auto sol = orch->request_solutions({Timestamp{kSec}});
  ASSERT_TRUE(sol && (*sol)[0]);
  auto pva = (*sol)[0]->as<cobra::utils::PVA>();
  const double moved_north = nav::delta_lat_to_north(pva->get_p1() - p0->get_p1(), p0->get_p1(), p0->get_p3());
  EXPECT_GT(moved_north, 0.05);
  EXPECT_LT(moved_north, 10.0);
  EXPECT_GT(pva->get_p3(), p0->get_p3());
  // North variance: prior 0.1 grows by ~0.012 (tilt coupling 0.5*g*t^2 = 4.9 m/rad, 5e-4 rad^2) and
  // ~0.001 (velocity) over 1 s, then shrinks in the update (R_eff = 1 + 9 FOGM).
  const double p00 = pva->get_covariance()(0, 0);
  EXPECT_GT(p00, 0.1);
  EXPECT_LT(p00, 0.113);
}

}  // namespace

// ----------------------------------------------------------------------------- extension points (C++ addition)

namespace {
class CountingOrchestration final : public cobra::StandardOrchestrationPlugin {
 public:
  using StandardOrchestrationPlugin::StandardOrchestrationPlugin;
  int updates = 0, feedbacks = 0, propagations = 0;

 protected:
  void perform_measurement_update(const Message& m, const std::string& mp) override {
    ++updates;
    StandardOrchestrationPlugin::perform_measurement_update(m, mp);
  }
  void apply_inertial_feedback() override {
    ++feedbacks;
    StandardOrchestrationPlugin::apply_inertial_feedback();
  }
  void propagate_to_time(Timestamp t) override {
    ++propagations;
    StandardOrchestrationPlugin::propagate_to_time(t);
  }
};
}  // namespace

TEST_F(OrchestrationTest, DerivedOrchestrationSeesEveryStep) {
  set_up(standard_orch_config(false, /*real_position_mp=*/true));  // registry and plugins around the standard orchestration
  CountingOrchestration counting("counting");
  counting.init_plugin(std::nullopt, &med);
  counting.init_orchestration_plugin(api::PluginList{fusion, strategy, inertial, init, sm, mock_sm}, stream_config);
  ASSERT_TRUE(counting.is_initialized());
  auto p0 = initial_pva();
  for (int i = 1; i <= 100; ++i) counting.process_pntos_message(imu_msg(i * 10'000'000), false);
  counting.process_pntos_message(pos_msg(kSec, p0->get_p1(), p0->get_p2(), p0->get_p3()), false);
  EXPECT_EQ(counting.updates, 1);
  EXPECT_EQ(counting.feedbacks, 1);
  EXPECT_GE(counting.propagations, 1);
  EXPECT_FALSE(med.has_error()) << med.last_message();
}
