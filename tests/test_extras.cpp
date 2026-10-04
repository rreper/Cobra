// Tests for the extras zero-velocity preprocessor (no Python unit test exists; the Python coverage is
// the pos_ins_zerovel2d integration app).
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/extras/AdvancedPreprocessorPlugin.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include "test_support.hpp"

#include <aspn23/eigen/MeasurementImu.hpp>

#include <cmath>
#include <numeric>

using namespace pntos;
using namespace pntos::test;
using api::Matrix;
using api::Message;
using api::Vector3;

namespace {
class ExtrasTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto zv = std::make_shared<cobra::ZeroVelocity2dGeneratorConfig>();
    zv->group_ = "config/pseudovel_generator";
    zv->channels = std::vector<std::string>{"/pos"};
    zv->trigger_dt_sec = 30.0;
    zv->lateral_vel_sigma = 0.5;
    zv->vertical_vel_sigma = 1.0;
    zv->output_channel = "/generated/zero/velocity2d";
    registry = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{zv});
    registry->init_plugin(std::nullopt, &med);
    med.set_registry(registry->new_registry());
    plugin = std::make_unique<cobra::AdvancedPreprocessorPlugin>("extras");
    plugin->init_plugin(std::nullopt, &med);
  }
  Message pos(std::int64_t t, const char* ch = "/pos") { return Message(make_position(t, 1, 2, 3, Matrix::Identity(3, 3)), ch); }
  TestMediator med;
  std::shared_ptr<cobra::StandardRegistryPlugin> registry;
  std::unique_ptr<cobra::AdvancedPreprocessorPlugin> plugin;
};
}  // namespace

TEST_F(ExtrasTest, ConfigRoundTrip) {
  auto c = cobra::ZeroVelocity2dGeneratorConfig::from_registry(med, "config/pseudovel_generator");
  ASSERT_TRUE(c);
  EXPECT_EQ(c->identifier, "zero_velocity2d_generator");
  EXPECT_EQ(c->trigger_dt_sec, 30.0);
  EXPECT_EQ(c->lateral_vel_sigma, 0.5);
  EXPECT_EQ(c->vertical_vel_sigma, 1.0);
  EXPECT_EQ(c->output_channel, "/generated/zero/velocity2d");
  ASSERT_TRUE(c->channels);
  EXPECT_EQ(c->channels->at(0), "/pos");
  // the orchestration reads the base type back
  auto base = cobra::PreprocessorConfig::from_registry(med, "config/pseudovel_generator");
  ASSERT_TRUE(base);
  EXPECT_EQ(base->identifier, cobra::ZeroVelocity2dGeneratorConfig::kIdentifier);
}

TEST_F(ExtrasTest, PluginErrors) {
  EXPECT_EQ(plugin->preprocessor_identifiers(), (std::vector<std::string>{"zero_velocity2d_generator", "sensor_degradation"}));
  EXPECT_EQ(plugin->new_preprocessor(2, std::string("config/pseudovel_generator")), nullptr);
  EXPECT_EQ(plugin->new_preprocessor(1, std::nullopt), nullptr);
  EXPECT_EQ(plugin->new_preprocessor(1, std::string("config/missing")), nullptr);
  EXPECT_EQ(plugin->new_preprocessor(0, std::nullopt), nullptr);
  EXPECT_EQ(plugin->new_preprocessor(0, std::string("config/missing")), nullptr);
  EXPECT_GE(med.count(api::LoggingLevel::ERROR), 3u);  // the missing group also logs from the config reader
}

TEST_F(ExtrasTest, GeneratesTwoDZeroVelocity) {
  auto pp = plugin->new_preprocessor(0, std::string("config/pseudovel_generator"));
  ASSERT_TRUE(pp);
  const std::int64_t s = 1'000'000'000;
  // first trigger fires immediately
  auto out = pp->process_pntos_message(pos(10 * s));
  ASSERT_TRUE(out);
  ASSERT_EQ(out->size(), 2u);
  EXPECT_EQ((*out)[0].source_identifier, "/pos");
  EXPECT_EQ((*out)[1].source_identifier, "/generated/zero/velocity2d");
  auto vel = (*out)[1].as<aspn23_eigen::MeasurementVelocity>();
  ASSERT_TRUE(vel);
  EXPECT_EQ(vel->get_time_of_validity().get_elapsed_nsec(), 10 * s);
  EXPECT_EQ(vel->get_reference_frame(), ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_SENSOR);
  EXPECT_TRUE(std::isnan(vel->get_x()));  // forward axis not measured
  EXPECT_EQ(vel->get_y(), 0.0);
  EXPECT_EQ(vel->get_z(), 0.0);
  Matrix R = vel->get_covariance();
  ASSERT_EQ(R.rows(), 2);
  EXPECT_DOUBLE_EQ(R(0, 0), 0.25);
  EXPECT_DOUBLE_EQ(R(1, 1), 1.0);
  // within trigger_dt: pass-through only
  out = pp->process_pntos_message(pos(20 * s));
  ASSERT_TRUE(out);
  EXPECT_EQ(out->size(), 1u);
  // other channels never trigger
  out = pp->process_pntos_message(pos(50 * s, "/other"));
  ASSERT_TRUE(out);
  EXPECT_EQ(out->size(), 1u);
  // after trigger_dt: fires again at the new time, and the earlier message is untouched
  out = pp->process_pntos_message(pos(40 * s));
  ASSERT_TRUE(out);
  ASSERT_EQ(out->size(), 2u);
  EXPECT_EQ((*out)[1].as<aspn23_eigen::MeasurementVelocity>()->get_time_of_validity().get_elapsed_nsec(), 40 * s);
  EXPECT_EQ(vel->get_time_of_validity().get_elapsed_nsec(), 10 * s);
}

TEST(SensorDegradation, NoiseBiasJumpsAndScales) {
  TestMediator med;
  cobra::SensorDegradationConfig c;
  c.group_ = "config/degradation";
  c.seed = 42;
  c.imu_expected_dt = 0.01;
  c.accel_noise_density = {0.02, 0.0, 0.0};  // m/s^2/sqrt(Hz): per-sample sigma 0.2 at 100 Hz (sampled)
  c.gyro_bias = {0.0, 1e-3, 0.0};
  c.position_noise_sigma_ned = {0.0, 0.0, 3.0};
  c.position_covariance_scale = 9.0;
  c.velocity_noise_sigma = {0.0, 0.5, 0.0};
  c.velocity_covariance_scale = 2.0;
  c.position_jumps = {{10.0, 50.0, 0.0, 0.0}};
  cobra::SensorDegradationPreprocessor pp(c, &med);
  // IMU, sampled type: accel x gets white noise of sigma 0.02 / sqrt(0.01) = 0.2; gyro y gets the bias
  std::vector<double> ax;
  double gy_sum = 0;
  const int N = 4000;
  for (std::int64_t i = 0; i < N; ++i) {
    auto out = pp.process_pntos_message(Message(make_imu(i * 10'000'000, Vector3(1, 2, 3), Vector3(0, 0, 0)), "/imu"));
    ASSERT_TRUE(out);
    ASSERT_EQ(out->size(), 1u);
    auto imu = (*out)[0].as<aspn23_eigen::MeasurementImu>();
    ax.push_back(imu->get_meas_accel()(0) - 1.0);
    gy_sum += imu->get_meas_gyro()(1);
    EXPECT_EQ(imu->get_meas_accel()(1), 2.0);  // untouched axes
    EXPECT_EQ(imu->get_meas_gyro()(2), 0.0);
  }
  const double mean = std::accumulate(ax.begin(), ax.end(), 0.0) / N;
  double var = 0;
  for (double v : ax) var += (v - mean) * (v - mean);
  EXPECT_NEAR(std::sqrt(var / N), 0.2, 0.02);
  EXPECT_NEAR(mean, 0.0, 0.02);
  EXPECT_NEAR(gy_sum / N, 1e-3, 1e-12);
  // integrated IMU: bias * dt, noise * sqrt(dt)
  auto integ = make_imu(0, Vector3(0, 0, 0), Vector3(0, 0, 0));
  integ->set_imu_type(ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED);
  auto oi = pp.process_pntos_message(Message(integ, "/imu"));
  EXPECT_NEAR((*oi)[0].as<aspn23_eigen::MeasurementImu>()->get_meas_gyro()(1), 1e-5, 1e-15);
  // position: down noise only, covariance x9, one jump at t >= 10 s (north +50 m)
  const double lat = 0.7, lon = -1.4, alt = 200;
  auto p0 = pp.process_pntos_message(Message(make_position(0, lat, lon, alt, Matrix::Identity(3, 3)), "/pos"));
  auto q0 = (*p0)[0].as<aspn23_eigen::MeasurementPosition>();
  EXPECT_EQ(q0->get_term1(), lat);
  EXPECT_EQ(q0->get_term2(), lon);
  EXPECT_NE(q0->get_term3(), alt);
  EXPECT_DOUBLE_EQ(Matrix(q0->get_covariance())(0, 0), 9.0);
  EXPECT_EQ(pp.jumps_applied(), 0u);
  auto p1 = pp.process_pntos_message(Message(make_position(10'000'000'000, lat, lon, alt, Matrix::Identity(3, 3)), "/pos"));
  auto q1 = (*p1)[0].as<aspn23_eigen::MeasurementPosition>();
  EXPECT_EQ(pp.jumps_applied(), 1u);
  EXPECT_NEAR(cobra::nav::delta_lat_to_north(q1->get_term1() - lat, lat, alt), 50.0, 1e-6);
  EXPECT_EQ(med.count(api::LoggingLevel::INFO), 1u);
  auto p2 = pp.process_pntos_message(Message(make_position(11'000'000'000, lat, lon, alt, Matrix::Identity(3, 3)), "/pos"));
  EXPECT_EQ((*p2)[0].as<aspn23_eigen::MeasurementPosition>()->get_term1(), lat);  // one-shot
  // velocity: y noise only, absent x stays absent, covariance x2
  auto v = std::make_shared<aspn23_eigen::MeasurementVelocity>(
      header(ASPN_MEASUREMENT_VELOCITY), aspn23_eigen::TypeTimestamp(std::int64_t{0}), ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_SENSOR,
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, rm(Matrix::Identity(2, 2) * 0.25), ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE,
      DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});
  auto ov = pp.process_pntos_message(Message(v, "/vel"));
  auto w = (*ov)[0].as<aspn23_eigen::MeasurementVelocity>();
  EXPECT_TRUE(std::isnan(w->get_x()));
  EXPECT_NE(w->get_y(), 0.0);
  EXPECT_EQ(w->get_z(), 0.0);
  EXPECT_DOUBLE_EQ(Matrix(w->get_covariance())(0, 0), 0.5);
  // deterministic: a second instance with the same seed produces the same stream
  cobra::SensorDegradationPreprocessor pp2(c, &med);
  auto a1 = (*pp2.process_pntos_message(Message(make_imu(0, Vector3(1, 2, 3), Vector3(0, 0, 0)), "/imu")))[0].as<aspn23_eigen::MeasurementImu>()->get_meas_accel()(0);
  EXPECT_NEAR(a1 - 1.0, ax[0], 1e-15);
  // registry round trip of the config
  auto reg = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{std::make_shared<cobra::SensorDegradationConfig>(c)});
  reg->init_plugin(std::nullopt, &med);
  med.set_registry(reg->new_registry());
  auto back = cobra::SensorDegradationConfig::from_registry(med, "config/degradation");
  ASSERT_TRUE(back);
  EXPECT_EQ(back->seed, 42);
  EXPECT_EQ(back->position_jumps.size(), 1u);
  EXPECT_EQ(back->position_jumps[0][1], 50.0);
  EXPECT_EQ(back->position_covariance_scale, 9.0);
  EXPECT_EQ(back->accel_noise_density[0], 0.02);
}
