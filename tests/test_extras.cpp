// Tests for the extras zero-velocity preprocessor (no Python unit test exists; the Python coverage is
// the pos_ins_zerovel2d integration app).
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/extras/AdvancedPreprocessorPlugin.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include "test_support.hpp"

#include <aspn23/eigen/MeasurementAngularVelocity.hpp>
#include <aspn23/eigen/MeasurementImu.hpp>

#include <cmath>
#include <numeric>
#include <random>

using namespace pntos;
using namespace pntos::test;
using api::Matrix;
using api::Message;
using api::Vector;
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
  EXPECT_EQ(plugin->preprocessor_identifiers(),
            (std::vector<std::string>{"zero_velocity2d_generator", "sensor_degradation", "zero_motion_generator"}));
  EXPECT_EQ(plugin->new_preprocessor(3, std::string("config/pseudovel_generator")), nullptr);
  EXPECT_EQ(plugin->new_preprocessor(2, std::nullopt), nullptr);
  EXPECT_EQ(plugin->new_preprocessor(2, std::string("config/missing")), nullptr);
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

TEST(SensorDegradation, RampsAndDerivedSource) {
  TestMediator med;
  cobra::SensorDegradationConfig c;
  c.group_ = "config/degradation";
  c.position_ramps = {{10.0, 1.0, 0.0, 0.0, 5.0}, {12.0, 0.0, 2.0, 0.0, 0.0}};  // north 1 m/s for 5 s from 10 s; east 2 m/s from 12 s on
  c.derived_position_channel = "/synthetic/cell/position";
  c.derived_position_sigma_ned = {5, 5, 8};
  c.derived_position_rate_hz = 1.0;
  cobra::SensorDegradationPreprocessor pp(c, &med);
  EXPECT_EQ(pp.ramp_offset(5.0), Vector3(0, 0, 0));
  EXPECT_EQ(pp.ramp_offset(12.0), Vector3(2, 0, 0));
  EXPECT_EQ(pp.ramp_offset(20.0), Vector3(5, 16, 0));  // north saturated at 5 m, east 8 s * 2 m/s
  const double lat = 0.7, lon = -1.4, alt = 200;
  auto at = [&](double rel_s) {
    auto out = pp.process_pntos_message(Message(make_position(static_cast<std::int64_t>(rel_s * 1e9), lat, lon, alt, Matrix::Identity(3, 3)), "/pos"));
    auto q = (*out)[0].as<aspn23_eigen::MeasurementPosition>();
    return Vector3(cobra::nav::delta_lat_to_north(q->get_term1() - lat, lat, alt), cobra::nav::delta_lon_to_east(q->get_term2() - lon, lat, alt), alt - q->get_term3());
  };
  EXPECT_NEAR(at(0.0).norm(), 0.0, 1e-9);     // first message defines t=0
  EXPECT_NEAR(at(11.0)(0), 1.0, 1e-6);         // 1 s into the first ramp
  EXPECT_NEAR(at(20.0)(0), 5.0, 1e-6);
  EXPECT_NEAR(at(20.0)(1), 16.0, 1e-6);
  // derived source: one position per second from a 10 Hz PVA stream, noise of the configured size
  Vector q(4);
  q << 1, 0, 0, 0;
  std::vector<double> dn;
  std::size_t emitted = 0;
  for (int i = 0; i < 300; ++i) {  // 30 s at 10 Hz
    auto out = pp.process_pntos_message(Message(make_pva(i * 100'000'000LL, lat, lon, alt, 0, 0, 0, q), "/sensor/ins-d/pva"));
    ASSERT_TRUE(out);
    EXPECT_EQ((*out)[0].source_identifier, "/sensor/ins-d/pva");
    if (out->size() == 2) {
      ++emitted;
      auto p = (*out)[1].as<aspn23_eigen::MeasurementPosition>();
      ASSERT_TRUE(p);
      EXPECT_EQ((*out)[1].source_identifier, "/synthetic/cell/position");
      EXPECT_EQ(p->get_time_of_validity().get_elapsed_nsec(), i * 100'000'000LL);
      EXPECT_DOUBLE_EQ(Matrix(p->get_covariance())(2, 2), 64.0);
      dn.push_back(cobra::nav::delta_lat_to_north(p->get_term1() - lat, lat, alt));
    }
  }
  EXPECT_EQ(emitted, 30u);
  EXPECT_EQ(pp.derived_messages(), 30u);
  double ss = 0;
  for (double v : dn) ss += v * v;
  EXPECT_NEAR(std::sqrt(ss / dn.size()), 5.0, 2.0);
  // registry + JSON keep the new fields
  auto reg = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{std::make_shared<cobra::SensorDegradationConfig>(c)});
  reg->init_plugin(std::nullopt, &med);
  med.set_registry(reg->new_registry());
  auto back = cobra::SensorDegradationConfig::from_registry(med, "config/degradation");
  ASSERT_TRUE(back);
  ASSERT_EQ(back->position_ramps.size(), 2u);
  EXPECT_EQ(back->position_ramps[1][2], 2.0);
  EXPECT_EQ(back->derived_position_channel, "/synthetic/cell/position");
  EXPECT_EQ(back->derived_position_rate_hz, 1.0);
}

// ---------------------------------------------------------------------------------------------------------------
// Standing still: StationarityDetector and ZeroMotionGenerator (DESIGN.md §7.25)

namespace {
constexpr std::int64_t kMs = 1'000'000;
/// A vehicle's IMU at 100 Hz: gyro bias, gravity, and the vibration of an idling engine, which is strong sample
/// by sample (5 mrad/s and 0.5 m/s^2 rms, as on the example recording) but oscillates, here at 27 Hz, so that it
/// is gone from a mean over 0.2 s. A little white noise on top.
struct ImuStream {
  std::mt19937 rng{7};
  std::normal_distribution<double> n{0.0, 1.0};
  std::int64_t t = 1'700'000'000'000'000'000;
  std::int64_t k = 0;
  Vector3 bias{-1.6e-3, -2.0e-3, -2.8e-3};
  /// One sample of 10 ms with the vehicle turning at `rate` (rad/s) and accelerating at `accel` (m/s^2).
  std::pair<Vector3, Vector3> sample(const Vector3& rate = Vector3::Zero(), const Vector3& accel = Vector3::Zero()) {
    t += 10 * kMs;
    ++k;
    const double ph = 2 * 3.14159265358979 * 27.0 * 0.01 * static_cast<double>(k);
    const Vector3 shake(std::sin(ph), std::sin(ph + 2.1), std::sin(ph + 4.2));
    const Vector3 w = bias + rate + shake * 7e-3 + Vector3(n(rng), n(rng), n(rng)) * 5e-4;
    const Vector3 f = Vector3(0, 0, -9.8) + accel + shake * 0.7 + Vector3(n(rng), n(rng), n(rng)) * 0.03;
    return {Vector3(f * 0.01), Vector3(w * 0.01)};
  }
  Message message(const Vector3& rate = Vector3::Zero(), const Vector3& accel = Vector3::Zero(), const char* channel = "/imu") {
    const auto [dv, dth] = sample(rate, accel);
    return Message(std::make_shared<aspn23_eigen::MeasurementImu>(header(ASPN_MEASUREMENT_IMU), aspn23_eigen::TypeTimestamp(t),
                                                                  ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED, DynVector(dv), DynVector(dth),
                                                                  std::vector<aspn23_eigen::TypeIntegrity>{}),
                   channel);
  }
};
}  // namespace

TEST(Stationarity, QuietThroughEngineVibrationAndNotThroughMotion) {
  cobra::inertial::StationarityDetector d;
  ImuStream imu;
  // standing: not before the window is full, then steadily
  int first = -1;
  for (int k = 0; k < 500; ++k) {
    const auto [dv, dth] = imu.sample();
    const bool s = d.add(imu.t, 0.01, dv, dth);
    if (s && first < 0) first = k;
    if (first >= 0) EXPECT_TRUE(s) << k;
  }
  EXPECT_GE(first, 195);
  EXPECT_LE(first, 205);
  EXPECT_NEAR((d.window_end_ns() - d.window_begin_ns()) * 1e-9, 2.0, 1e-6);
  EXPECT_EQ(d.window_end_ns(), imu.t);
  EXPECT_LT((d.mean_rate() - imu.bias).norm(), 3e-4);  // 5 mrad/s rms of vibration is gone from the mean
  EXPECT_NEAR(d.mean_specific_force()(2), -9.8, 0.03);
  EXPECT_NEAR(d.quiet_seconds(), 5.0, 0.21);

  // a turn of 0.6 deg/s that builds up over a second: no single sample stands out of the vibration, the means do
  bool lost = false;
  for (int k = 0; k < 300; ++k) {
    const auto [dv, dth] = imu.sample(Vector3(0, 0, 0.01 * std::min(1.0, k / 100.0)));
    lost = lost || !d.add(imu.t, 0.01, dv, dth);
  }
  EXPECT_TRUE(lost);
  // a steady turn is quiet again in the deviations, but its rate is not a bias: max_rate
  for (int k = 0; k < 300; ++k) {
    const auto [dv, dth] = imu.sample(Vector3(0, 0, 0.03));
    d.add(imu.t, 0.01, dv, dth);
  }
  EXPECT_FALSE(d.stationary());
  // pulling away at 0.5 m/s^2 after standing
  for (int k = 0; k < 300; ++k) {
    const auto [dv, dth] = imu.sample();
    d.add(imu.t, 0.01, dv, dth);
  }
  EXPECT_TRUE(d.stationary());
  bool noticed = false;
  int noticed_at = -1;
  for (int k = 0; k < 100 && !noticed; ++k) {
    const auto [dv, dth] = imu.sample(Vector3::Zero(), Vector3(0.5, 0, 0));
    noticed = !d.add(imu.t, 0.01, dv, dth);
    noticed_at = k;
  }
  EXPECT_TRUE(noticed);
  EXPECT_LT(noticed_at, 45);  // within two sub-windows
  EXPECT_EQ(d.quiet_seconds(), 0.0);
}

TEST(Stationarity, GapsAndNonsenseStartOver) {
  cobra::inertial::StationarityDetector d;
  ImuStream imu;
  for (int k = 0; k < 300; ++k) {
    const auto [dv, dth] = imu.sample();
    d.add(imu.t, 0.01, dv, dth);
  }
  ASSERT_TRUE(d.stationary());
  // a second without data
  imu.t += 1000 * kMs;
  auto [dv, dth] = imu.sample();
  EXPECT_FALSE(d.add(imu.t, 0.01, dv, dth));
  for (int k = 0; k < 150; ++k) {
    std::tie(dv, dth) = imu.sample();
    EXPECT_FALSE(d.add(imu.t, 0.01, dv, dth)) << k;  // the window has to fill again
  }
  for (int k = 0; k < 100; ++k) {
    std::tie(dv, dth) = imu.sample();
    d.add(imu.t, 0.01, dv, dth);
  }
  EXPECT_TRUE(d.stationary());
  // not finite, a zero interval, a step back
  EXPECT_FALSE(d.add(imu.t + 10 * kMs, 0.01, Vector3(0, std::nan(""), 0), dth));
  EXPECT_FALSE(d.add(imu.t + 20 * kMs, 0.0, dv, dth));
  EXPECT_FALSE(d.stationary());
  // options that make no sense
  cobra::inertial::StationarityOptions o;
  o.window = 0.3;
  EXPECT_THROW(cobra::inertial::StationarityDetector{o}, std::invalid_argument);
  o = {};
  o.gyro_deviation = 0;
  EXPECT_THROW(cobra::inertial::StationarityDetector{o}, std::invalid_argument);
}

namespace {
class ZeroMotionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto zm = std::make_shared<cobra::ZeroMotionGeneratorConfig>();
    zm->group_ = "config/zero_motion";
    zm->channels = std::vector<std::string>{"/imu"};
    zm->velocity_channel = "/generated/zero_velocity";
    zm->rate_channel = "/generated/zero_rate";
    zm->velocity_sigma = 0.04;
    zm->rate_sigma = 2e-4;
    auto rate_only = std::make_shared<cobra::ZeroMotionGeneratorConfig>(*zm);
    rate_only->group_ = "config/zero_rate_only";
    rate_only->velocity_channel.clear();
    rate_only->trigger_dt_sec = 0.5;
    auto confirmed = std::make_shared<cobra::ZeroMotionGeneratorConfig>(*zm);
    confirmed->group_ = "config/zero_motion_confirmed";
    confirmed->channels = std::vector<std::string>{"/imu", kReceiver};
    confirmed->confirmation_channels = {kReceiver};
    confirmed->velocity_sigma = 0.2;
    confirmed->confirmed_velocity_sigma = 0.03;
    confirmed->confirmed_vertical_velocity_sigma = 0.15;
    auto unseen = std::make_shared<cobra::ZeroMotionGeneratorConfig>(*confirmed);
    unseen->group_ = "config/zero_motion_unseen";
    unseen->channels = std::vector<std::string>{"/imu"};
    registry = std::make_shared<cobra::StandardRegistryPlugin>(
        "registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{zm, rate_only, confirmed, unseen});
    registry->init_plugin(std::nullopt, &med);
    med.set_registry(registry->new_registry());
    plugin = std::make_unique<cobra::AdvancedPreprocessorPlugin>("extras");
    plugin->init_plugin(std::nullopt, &med);
  }

  static constexpr const char* kReceiver = "/gnss/velocity";
  /// A receiver's velocity (NED unless said otherwise), with the sigma it claims per axis.
  static Message receiver(std::int64_t t, const Vector3& v, double sigma = 0.06, const char* channel = kReceiver,
                          Aspn23MeasurementVelocityReferenceFrame frame = ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED) {
    return Message(std::make_shared<aspn23_eigen::MeasurementVelocity>(
                       header(ASPN_MEASUREMENT_VELOCITY), aspn23_eigen::TypeTimestamp(t), frame, v(0), v(1), v(2),
                       rm(Matrix(Matrix::Identity(3, 3) * sigma * sigma)), ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE, DynVector(0),
                       std::vector<aspn23_eigen::TypeIntegrity>{}),
                   channel);
  }

  struct Said {  ///< one zero-velocity message of the generator
    std::int64_t stamp, arrival;  ///< its time of validity; the IMU's time when it came out
    double sigma;                 ///< of its velocity, north (and east), m/s
    double vertical;              ///< down
    bool with_receiver;           ///< it came out behind a receiver message (not behind an IMU message)
  };
  /// Feeds IMU samples up to `until_s` seconds after `t0`. `speed(s)` is what the receiver shows at s seconds
  /// after t0, or negative for "no message"; its epochs are at 0.5, 1.5, ... s and each arrives `delay_s`
  /// later, behind the IMU sample of that time (as it does out of the controller's buffer).
  template <typename Speed, typename Motion>
  void feed(api::Preprocessor& pp, ImuStream& imu, std::int64_t t0, double until_s, double delay_s, Speed speed, Motion motion,
            std::vector<Said>* said) {
    auto take = [&](const std::vector<Message>& out, bool with_receiver) {
      for (std::size_t i = 1; i < out.size(); ++i)
        if (auto v = out[i].as<aspn23_eigen::MeasurementVelocity>(); v && out[i].source_identifier == "/generated/zero_velocity") {
          EXPECT_EQ(Vector3(v->get_x(), v->get_y(), v->get_z()), Vector3::Zero());
          const Matrix cov(v->get_covariance());
          EXPECT_EQ(cov(0, 0), cov(1, 1));
          said->push_back({v->get_time_of_validity().get_elapsed_nsec(), imu.t, std::sqrt(cov(0, 0)), std::sqrt(cov(2, 2)), with_receiver});
          // the zero rate comes with it, with the same stamp
          ASSERT_LT(i + 1, out.size());
          auto w = out[i + 1].as<aspn23_eigen::MeasurementAngularVelocity>();
          ASSERT_TRUE(w);
          EXPECT_EQ(w->get_time_of_validity().get_elapsed_nsec(), v->get_time_of_validity().get_elapsed_nsec());
        }
    };
    while ((imu.t - t0) * 1e-9 < until_s - 1e-9) {
      const double now = (imu.t + 10 * kMs - t0) * 1e-9;
      const auto [rate, accel] = motion(now);
      const Message in = imu.message(rate, accel);
      auto out = pp.process_pntos_message(in);
      ASSERT_TRUE(out);
      ASSERT_GE(out->size(), 1u);
      EXPECT_EQ((*out)[0].wrapped_message, in.wrapped_message);
      take(*out, false);
      while (0.5 + next_epoch_ + delay_s <= now + 1e-9) {
        const double epoch = 0.5 + next_epoch_++;
        const double v = speed(epoch);
        if (v < 0) continue;
        const Message r = receiver(t0 + static_cast<std::int64_t>(std::llround(epoch * 1e9)), Vector3(0.8 * v, -0.6 * v, 0.0));
        auto o = pp.process_pntos_message(r);
        ASSERT_TRUE(o);
        ASSERT_GE(o->size(), 1u);
        EXPECT_EQ((*o)[0].wrapped_message, r.wrapped_message);  // the receiver's message passes untouched, first
        take(*o, true);
      }
    }
  }
  static std::pair<Vector3, Vector3> standing(double) { return {Vector3::Zero(), Vector3::Zero()}; }
  int next_epoch_ = 0;

  TestMediator med;
  std::shared_ptr<cobra::StandardRegistryPlugin> registry;
  std::unique_ptr<cobra::AdvancedPreprocessorPlugin> plugin;
};
}  // namespace

TEST_F(ZeroMotionTest, ConfigRoundTrip) {
  auto c = cobra::ZeroMotionGeneratorConfig::from_registry(med, "config/zero_motion");
  ASSERT_TRUE(c);
  EXPECT_EQ(c->identifier, "zero_motion_generator");
  EXPECT_EQ(c->velocity_channel, "/generated/zero_velocity");
  EXPECT_EQ(c->rate_channel, "/generated/zero_rate");
  EXPECT_DOUBLE_EQ(c->velocity_sigma, 0.04);
  EXPECT_DOUBLE_EQ(c->rate_sigma, 2e-4);
  EXPECT_DOUBLE_EQ(c->window, 2.0);
  EXPECT_DOUBLE_EQ(c->trigger_dt_sec, 1.0);
  ASSERT_TRUE(c->channels);
  EXPECT_EQ(c->channels->at(0), "/imu");
  auto base = cobra::PreprocessorConfig::from_registry(med, "config/zero_motion");
  ASSERT_TRUE(base);
  EXPECT_EQ(base->identifier, cobra::ZeroMotionGeneratorConfig::kIdentifier);
  auto r = cobra::ZeroMotionGeneratorConfig::from_registry(med, "config/zero_rate_only");
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->velocity_channel.empty());
  // the receiver as a witness: off unless channels are named
  EXPECT_TRUE(c->confirmation_channels.empty());
  EXPECT_DOUBLE_EQ(c->confirmation_speed, 0.05);
  EXPECT_DOUBLE_EQ(c->confirmation_max_sigma, 0.5);
  EXPECT_DOUBLE_EQ(c->confirmation_timeout, 2.5);
  EXPECT_DOUBLE_EQ(c->confirmed_velocity_sigma, 0.05);
  EXPECT_DOUBLE_EQ(c->confirmed_vertical_velocity_sigma, 0.2);
  auto w = cobra::ZeroMotionGeneratorConfig::from_registry(med, "config/zero_motion_confirmed");
  ASSERT_TRUE(w);
  EXPECT_EQ(w->confirmation_channels, std::vector<std::string>{kReceiver});
  EXPECT_DOUBLE_EQ(w->confirmed_velocity_sigma, 0.03);
  EXPECT_DOUBLE_EQ(w->confirmed_vertical_velocity_sigma, 0.15);
  EXPECT_DOUBLE_EQ(w->velocity_sigma, 0.2);
}

TEST_F(ZeroMotionTest, SaysOnceASecondThatTheVehicleStands) {
  auto pp = plugin->new_preprocessor(2, std::string("config/zero_motion"));
  ASSERT_TRUE(pp);
  ImuStream imu;
  const std::int64_t t0 = imu.t;
  std::vector<std::int64_t> stamps, arrivals;
  for (int k = 0; k < 800; ++k) {
    const Message in = imu.message();
    auto out = pp->process_pntos_message(in);
    ASSERT_TRUE(out);
    ASSERT_GE(out->size(), 1u);
    EXPECT_EQ((*out)[0].wrapped_message, in.wrapped_message);  // the IMU message itself comes first, untouched
    if (out->size() == 1) continue;
    ASSERT_EQ(out->size(), 3u);
    auto vel = (*out)[1].as<aspn23_eigen::MeasurementVelocity>();
    auto rate = (*out)[2].as<aspn23_eigen::MeasurementAngularVelocity>();
    ASSERT_TRUE(vel);
    ASSERT_TRUE(rate);
    EXPECT_EQ((*out)[1].source_identifier, "/generated/zero_velocity");
    EXPECT_EQ((*out)[2].source_identifier, "/generated/zero_rate");
    EXPECT_EQ(vel->get_reference_frame(), ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED);
    EXPECT_EQ(Vector3(vel->get_x(), vel->get_y(), vel->get_z()), Vector3::Zero());
    EXPECT_TRUE(allclose(Matrix(vel->get_covariance()), Matrix::Identity(3, 3) * 0.04 * 0.04, 1e-12, 0));
    EXPECT_EQ(rate->get_reference_frame(), ASPN23_MEASUREMENT_ANGULAR_VELOCITY_REFERENCE_FRAME_SENSOR);
    EXPECT_EQ(Vector(rate->get_meas()), Vector(Vector3::Zero()));
    EXPECT_TRUE(allclose(Matrix(rate->get_covariance()), Matrix::Identity(3, 3) * 2e-4 * 2e-4, 1e-12, 0));
    EXPECT_EQ(vel->get_time_of_validity().get_elapsed_nsec(), rate->get_time_of_validity().get_elapsed_nsec());
    stamps.push_back(vel->get_time_of_validity().get_elapsed_nsec());
    arrivals.push_back(imu.t);
  }
  // 8 s of standing: the first after the 2 s window, then one a second
  ASSERT_GE(stamps.size(), 6u);
  ASSERT_LE(stamps.size(), 7u);
  EXPECT_NEAR((arrivals.front() - t0) * 1e-9, 2.0, 0.05);
  for (std::size_t i = 0; i < stamps.size(); ++i) {
    EXPECT_EQ(arrivals[i] - stamps[i], 1000 * kMs) << i;  // stamped at the middle of the quiet window
    if (i) EXPECT_NEAR((stamps[i] - stamps[i - 1]) * 1e-9, 1.0, 0.01) << i;
  }

  // driving off and winding: nothing; other channels and other message types pass untouched
  for (int k = 0; k < 400; ++k) {
    auto out = pp->process_pntos_message(imu.message(Vector3(0, 0, 0.05 * std::sin(0.02 * k)), Vector3(0.8, 0, 0)));
    ASSERT_TRUE(out);
    if (k > 40) EXPECT_EQ(out->size(), 1u) << k;
  }
  for (int k = 0; k < 400; ++k) {
    auto out = pp->process_pntos_message(imu.message(Vector3::Zero(), Vector3::Zero(), "/other_imu"));
    ASSERT_TRUE(out);
    EXPECT_EQ(out->size(), 1u);
  }
  auto out = pp->process_pntos_message(Message(make_position(imu.t, 1, 2, 3, Matrix::Identity(3, 3)), "/imu"));
  ASSERT_TRUE(out);
  EXPECT_EQ(out->size(), 1u);
}

TEST_F(ZeroMotionTest, RateOnlyAndItsOwnInterval) {
  auto pp = plugin->new_preprocessor(2, std::string("config/zero_rate_only"));
  ASSERT_TRUE(pp);
  ImuStream imu;
  int generated = 0;
  for (int k = 0; k < 500; ++k) {
    auto out = pp->process_pntos_message(imu.message());
    ASSERT_TRUE(out);
    if (out->size() == 1) continue;
    ASSERT_EQ(out->size(), 2u);
    EXPECT_TRUE((*out)[1].as<aspn23_eigen::MeasurementAngularVelocity>());
    ++generated;
  }
  // 0.5 s is not a multiple of the 0.2 s sub-window the detector advances by: every 0.6 s from 2 s on
  EXPECT_GE(generated, 5);
  EXPECT_LE(generated, 6);
}

// --------------------------------------------------------------------------- the receiver as a second witness
// DESIGN.md §7.28. A car that creeps at a steady 0.3 m/s is as quiet to the IMU as one that stands: on the
// example recording 21 of the detector's 420 messages come while the car moves.

TEST_F(ZeroMotionTest, TheImuAloneCallsACreepingVehicleStanding) {
  // v0.2.12, pinned: without a witness the generator says "stands" once a second whatever the receiver shows.
  auto pp = plugin->new_preprocessor(2, std::string("config/zero_motion"));
  ASSERT_TRUE(pp);
  ImuStream imu;
  std::vector<Said> said;
  feed(*pp, imu, imu.t, 12.0, 2.0, [](double) { return 0.3; }, standing, &said);
  EXPECT_GE(said.size(), 10u);
}

TEST_F(ZeroMotionTest, TheReceiverContradictsACreepingVehicle) {
  auto pp = plugin->new_preprocessor(2, std::string("config/zero_motion_confirmed"));
  ASSERT_TRUE(pp);
  auto* zm = dynamic_cast<cobra::ZeroMotionGenerator*>(pp.get());
  ASSERT_NE(zm, nullptr);
  ImuStream imu;
  const std::int64_t t0 = imu.t;
  std::vector<Said> said;
  feed(*pp, imu, t0, 14.0, 2.0, [](double) { return 0.3; }, standing, &said);
  // One message on the IMU's word before the receiver was first heard (its first velocity is 2 s old when it
  // arrives), none after that.
  ASSERT_LE(said.size(), 1u);
  for (const auto& m : said) {
    EXPECT_LT((m.arrival - t0) * 1e-9, 2.6);
    EXPECT_NEAR(m.sigma, 0.2, 1e-12);
  }
  EXPECT_TRUE(zm->detector().stationary());  // the IMU still says "stands"
  const auto& c = zm->confirmation();
  EXPECT_EQ(c.confirmed, 0u);
  EXPECT_GE(c.contradicted, 9u);  // the epochs at 1.5 .. 10.5 s, each in the middle of 2 s the IMU called quiet
  EXPECT_NEAR(c.fastest, 0.3, 1e-9);
  // Said with the speed: the first and the tenth time for everybody, each time for whoever asks (DEBUG). It is
  // what an installation needs to know about its detector.
  std::size_t info = 0, debug = 0;
  for (const auto& l : med.logs)
    if (l.message.find("0.30 m/s") != std::string::npos) {
      if (l.level == api::LoggingLevel::INFO) {
        ++info;
      } else {
        EXPECT_EQ(l.level, api::LoggingLevel::DEBUG);
        ++debug;
      }
    }
  EXPECT_GE(info, 1u);
  EXPECT_LE(info, 2u);
  EXPECT_EQ(info + debug, c.contradicted);
  EXPECT_EQ(med.count(api::LoggingLevel::ERROR), 0u);
}

TEST_F(ZeroMotionTest, TheReceiverConfirmsAStandstillAtItsEpochs) {
  auto pp = plugin->new_preprocessor(2, std::string("config/zero_motion_confirmed"));
  ASSERT_TRUE(pp);
  auto* zm = dynamic_cast<cobra::ZeroMotionGenerator*>(pp.get());
  ASSERT_NE(zm, nullptr);
  ImuStream imu;
  const std::int64_t t0 = imu.t;
  std::vector<Said> said;
  feed(*pp, imu, t0, 14.0, 2.0, [](double) { return 0.012; }, standing, &said);  // a receiver at rest: 6 mm/s median on the recording
  ASSERT_GE(said.size(), 9u);
  std::size_t alone = 0, confirmed = 0;
  for (std::size_t i = 0; i < said.size(); ++i) {
    const auto& m = said[i];
    if (!m.with_receiver) {  // only before the receiver was heard
      ++alone;
      EXPECT_LT((m.arrival - t0) * 1e-9, 2.6) << i;
      EXPECT_NEAR(m.sigma, 0.2, 1e-12) << i;
      EXPECT_NEAR(m.vertical, 0.2, 1e-12) << i;
      continue;
    }
    ++confirmed;
    EXPECT_NEAR(m.sigma, 0.03, 1e-12) << i;                 // the weight of a confirmed standstill, north and east
    EXPECT_NEAR(m.vertical, 0.15, 1e-12) << i;              // down has its own: the receiver says nothing about the accelerometer
    const double epoch = (m.stamp - t0) * 1e-9 - 0.5;        // stamped at the receiver's epoch ...
    EXPECT_NEAR(epoch, std::round(epoch), 1e-9) << i;
    EXPECT_NEAR((m.arrival - m.stamp) * 1e-9, 2.0, 0.011) << i;  // ... and out as soon as that velocity arrives
    if (i) {
      EXPECT_GE(m.stamp - said[i - 1].stamp, 900 * kMs) << i;
    }
    if (i && said[i - 1].with_receiver) {
      EXPECT_EQ(m.stamp - said[i - 1].stamp, 1000 * kMs) << i;  // none left out
    }
  }
  EXPECT_LE(alone, 1u);
  EXPECT_GE(confirmed, 9u);  // the epochs from 2.5 s (1.5 s is too close to the one the IMU said alone) to 11.5 s
  EXPECT_EQ(zm->confirmation().confirmed, confirmed);
  EXPECT_EQ(zm->confirmation().alone, alone);
  EXPECT_EQ(zm->confirmation().contradicted, 0u);
  EXPECT_EQ(zm->generated(), said.size());
}

TEST_F(ZeroMotionTest, ASlowEpochAtTheEdgeOfAStopIsNotEnough) {
  // The car winds and brakes until 6 s, then stands. The receiver shows a standstill from 5.5 s on, but the
  // epochs at 5.5 and 6.5 s do not have a second of quiet IMU on both sides.
  auto pp = plugin->new_preprocessor(2, std::string("config/zero_motion_confirmed"));
  ASSERT_TRUE(pp);
  ImuStream imu;
  const std::int64_t t0 = imu.t;
  std::vector<Said> said;
  feed(*pp, imu, t0, 14.0, 2.0, [](double s) { return s < 5.0 ? 3.0 : 0.01; },
       [](double now) -> std::pair<Vector3, Vector3> {
         if (now < 6.0) return {Vector3(0, 0, 0.05 * std::sin(2.0 * now)), Vector3(-0.8, 0, 0)};
         return {Vector3::Zero(), Vector3::Zero()};
       },
       &said);
  ASSERT_GE(said.size(), 3u);
  EXPECT_GE((said.front().stamp - t0) * 1e-9, 7.0 - 1e-9);  // 7.5 s is the first epoch with 6.5 to 8.5 s quiet
  EXPECT_LE((said.front().stamp - t0) * 1e-9, 7.5 + 1e-9);
  for (const auto& m : said) EXPECT_NEAR(m.sigma, 0.03, 1e-12);
}

TEST_F(ZeroMotionTest, WithoutTheReceiverTheImuDecidesAlone) {
  auto pp = plugin->new_preprocessor(2, std::string("config/zero_motion_confirmed"));
  ASSERT_TRUE(pp);
  auto* zm = dynamic_cast<cobra::ZeroMotionGenerator*>(pp.get());
  ASSERT_NE(zm, nullptr);
  ImuStream imu;
  const std::int64_t t0 = imu.t;
  std::vector<Said> said;
  // the receiver is there until its epoch at 7.5 s, gone until 18.5 s, and back
  feed(*pp, imu, t0, 28.0, 2.0, [](double s) { return (s > 8.0 && s < 18.0) ? -1.0 : 0.01; }, standing, &said);
  std::vector<double> alone_arrivals, confirmed_stamps;
  for (std::size_t i = 0; i < said.size(); ++i) {
    const auto& m = said[i];
    if (i) {
      EXPECT_GE(m.stamp - said[i - 1].stamp, 900 * kMs) << i;  // in order and never two for one second
    }
    if (std::abs(m.sigma - 0.2) < 1e-12) {
      if ((m.arrival - t0) * 1e-9 > 3.0) alone_arrivals.push_back((m.arrival - t0) * 1e-9);
      EXPECT_EQ((m.arrival - m.stamp), 1000 * kMs) << i;  // as before: the middle of the detector's window
    } else {
      EXPECT_NEAR(m.sigma, 0.03, 1e-12) << i;
      confirmed_stamps.push_back((m.stamp - t0) * 1e-9);
    }
  }
  // The last velocity (7.5 s) arrives at 9.5 s; 2.5 s later the IMU decides alone again, until the first velocity
  // after the gap (18.5 s) arrives at 20.5 s.
  ASSERT_GE(alone_arrivals.size(), 7u);
  EXPECT_NEAR(alone_arrivals.front(), 12.0, 0.25);
  EXPECT_LE(alone_arrivals.back(), 20.5 + 1e-9);
  ASSERT_FALSE(confirmed_stamps.empty());
  EXPECT_NEAR(confirmed_stamps.back(), 25.5, 1e-6);
  std::size_t before = 0, after = 0;
  for (double t : confirmed_stamps) (t < 8.0 ? before : after) += 1;
  EXPECT_GE(before, 5u);
  EXPECT_GE(after, 4u);  // 21.5 s (the first that is clear of the last one the IMU said alone) to 25.5 s
  for (double t : confirmed_stamps) EXPECT_FALSE(t > 8.0 && t < 18.0) << t;
  // both changes are said
  std::size_t lost = 0, back = 0;
  for (const auto& l : med.logs) {
    if (l.message.find("decides alone") != std::string::npos) ++lost;
    if (l.message.find("confirms again") != std::string::npos) ++back;
  }
  EXPECT_EQ(lost, 1u);
  EXPECT_EQ(back, 1u);
}

TEST_F(ZeroMotionTest, AVelocityThatArrivesAtOnceWaitsForTheImu) {
  // Without the controller's buffer the receiver's velocity is there 0.1 s after its epoch; whether the IMU
  // stays quiet for the second after it is not known yet. The message comes when it is, behind an IMU sample.
  auto pp = plugin->new_preprocessor(2, std::string("config/zero_motion_confirmed"));
  ASSERT_TRUE(pp);
  ImuStream imu;
  const std::int64_t t0 = imu.t;
  std::vector<Said> said;
  feed(*pp, imu, t0, 12.0, 0.1, [](double) { return 0.01; }, standing, &said);
  ASSERT_GE(said.size(), 8u);
  for (std::size_t i = 0; i < said.size(); ++i) {
    const auto& m = said[i];
    EXPECT_FALSE(m.with_receiver) << i;
    EXPECT_NEAR(m.sigma, 0.03, 1e-12) << i;  // the receiver was heard before the detector's first window was full
    const double epoch = (m.stamp - t0) * 1e-9 - 0.5;
    EXPECT_NEAR(epoch, std::round(epoch), 1e-9) << i;
    EXPECT_GE((m.arrival - m.stamp) * 1e-9, 1.0) << i;  // half a window, and the detector's next 0.2 s verdict
    EXPECT_LE((m.arrival - m.stamp) * 1e-9, 1.21) << i;
  }
  // and when the car pulls away half a second after an epoch that showed a standstill, that epoch is dropped
  std::vector<Said> more;
  const double base = (imu.t - t0) * 1e-9;  // 12 s: the epoch at 11.5 s is waiting
  feed(*pp, imu, t0, 16.0, 0.1, [](double s) { return s < 12.0 ? 0.01 : 2.0; },
       [base](double now) -> std::pair<Vector3, Vector3> { return {Vector3::Zero(), now > base ? Vector3(0.8, 0, 0) : Vector3::Zero()}; }, &more);
  for (const auto& m : more) EXPECT_LT((m.stamp - t0) * 1e-9, 11.0) << (m.stamp - t0) * 1e-9;
}

TEST_F(ZeroMotionTest, ADoubtfulVelocityIsNotHeard) {
  auto pp = plugin->new_preprocessor(2, std::string("config/zero_motion_confirmed"));
  ASSERT_TRUE(pp);
  auto* zm = dynamic_cast<cobra::ZeroMotionGenerator*>(pp.get());
  ASSERT_NE(zm, nullptr);
  ImuStream imu;
  const Vector3 still(0.004, -0.003, 0.002);
  int alone = 0;
  std::uint64_t sent = 0;
  for (int k = 0; k < 1000; ++k) {
    auto out = pp->process_pntos_message(imu.message());
    ASSERT_TRUE(out);
    if (out->size() > 1) ++alone;
    if (k % 100 != 99) continue;
    const std::int64_t t = imu.t - 2000 * kMs;
    std::vector<Message> doubtful{
        receiver(t, still, 1.0),                                                                // claims 1 m/s: says nothing about 0.05
        receiver(t, Vector3(0.004, std::nan(""), 0.002)),                                       // not finite
        receiver(t, still, 0.06, kReceiver, ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_ECI),    // not a velocity over ground
        receiver(t, Vector3(std::nan(""), 0.0, 0.0), 0.06, kReceiver, ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_SENSOR),
        Message(make_position(t, 1, 2, 3, Matrix::Identity(3, 3)), kReceiver),                 // not a velocity at all
        receiver(t, still, 0.06, "/other/velocity")};                                           // not a confirmation channel
    for (const auto& m : doubtful) {
      auto o = pp->process_pntos_message(m);
      ASSERT_TRUE(o);
      EXPECT_EQ(o->size(), 1u);
      ++sent;
    }
  }
  EXPECT_GE(alone, 8);  // the IMU went on deciding alone, every second from 2 s on
  EXPECT_EQ(zm->confirmation().confirmed, 0u);
  EXPECT_EQ(zm->confirmation().alone, static_cast<std::uint64_t>(alone));
  EXPECT_EQ(zm->confirmation().not_heard, sent - 10u);  // all but the ones on the other channel, which is not looked at
  // An ECEF velocity and the velocity of a PVA message are heard.
  std::uint64_t confirmed_before = zm->confirmation().confirmed;
  for (int k = 0; k < 600; ++k) {
    auto out = pp->process_pntos_message(imu.message());
    ASSERT_TRUE(out);
    if (k % 100 != 99) continue;
    const std::int64_t t = imu.t - 2000 * kMs;
    const Message m = (k / 100) % 2 == 0 ? receiver(t, still, 0.06, kReceiver, ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_ECEF)
                                         : Message(make_pva(t, 0.7, -1.4, 250.0, 0.004, -0.003, 0.002, vec({1, 0, 0, 0})), kReceiver);
    auto o = pp->process_pntos_message(m);
    ASSERT_TRUE(o);
    // The first is for a time a second before the last message the IMU made alone: nothing is said twice, or
    // out of order.
    EXPECT_EQ(o->size(), k == 99 ? 1u : 3u) << k;
  }
  EXPECT_EQ(zm->confirmation().confirmed, confirmed_before + 5u);
}

TEST_F(ZeroMotionTest, AVelocityStampedInTheFutureDoesNotHoldUpTheOthers) {
  // One message of the receiver with a time stamp a quarter of an hour ahead (a corrupt stamp): it cannot be
  // judged until the IMU gets there. The epochs behind it are judged as they come.
  auto pp = plugin->new_preprocessor(2, std::string("config/zero_motion_confirmed"));
  ASSERT_TRUE(pp);
  auto* zm = dynamic_cast<cobra::ZeroMotionGenerator*>(pp.get());
  ASSERT_NE(zm, nullptr);
  ImuStream imu;
  const std::int64_t t0 = imu.t;
  std::vector<Said> said;
  feed(*pp, imu, t0, 6.0, 2.0, [](double) { return 0.01; }, standing, &said);
  auto o = pp->process_pntos_message(receiver(imu.t + 900'000 * kMs, Vector3(0.004, -0.003, 0.002)));
  ASSERT_TRUE(o);
  EXPECT_EQ(o->size(), 1u);
  const std::uint64_t before = zm->confirmation().confirmed;
  feed(*pp, imu, t0, 14.0, 2.0, [](double) { return 0.01; }, standing, &said);
  EXPECT_GE(zm->confirmation().confirmed, before + 7u);  // the epochs at 4.5 .. 11.5 s, one a second as before
  for (std::size_t i = 1; i < said.size(); ++i) {
    EXPECT_LE(said[i].stamp - said[i - 1].stamp, 1500 * kMs) << i;
  }
}

TEST_F(ZeroMotionTest, AConfirmationChannelTheGeneratorNeverSeesIsAnError) {
  // "channels" is what the orchestration hands to the preprocessor; a witness that is not in it would never speak
  // and the generator would decide alone for ever, silently.
  auto pp = plugin->new_preprocessor(2, std::string("config/zero_motion_unseen"));
  EXPECT_EQ(pp, nullptr);
  EXPECT_TRUE(med.has_error());
  EXPECT_NE(med.last_message().find("/gnss/velocity"), std::string::npos) << med.last_message();
}
