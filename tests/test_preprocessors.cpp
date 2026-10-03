// Port of pntos-cobra/tests/test_preprocessor_plugin.py
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/preprocessing/StandardPreprocessorPlugin.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include "test_support.hpp"

#include <aspn23/eigen/MeasurementAltitude.hpp>
#include <aspn23/eigen/MeasurementBarometer.hpp>

using namespace pntos;
using namespace pntos::test;
using api::Matrix;
using api::Message;
using api::Vector;
using api::Vector3;
using api::Matrix3;

namespace {
const std::vector<std::string> kIds = {"downsampler", "imu_rotator", "time_adjuster", "baro_converter", "time_bias", "outage"};

class PreprocessorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto ds = std::make_shared<cobra::DownsamplerConfig>();
    ds->group_ = "config/downsampler";
    ds->channels = std::vector<std::string>{"test1", "test2", "test3"};
    ds->downsampling_factors = {2, 3, -1};
    auto rot = std::make_shared<cobra::ImuRotatorConfig>();
    rot->group_ = "config/imu_rotator";
    rot->channels = std::vector<std::string>{"/sensor/imu"};
    rot->C_imu_to_platform = {{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}};
    auto ta = std::make_shared<cobra::TimeAdjusterConfig>();
    ta->group_ = "config/time_adjuster";
    ta->channels = std::vector<std::string>{"/sensor/imu"};
    ta->expected_dt_nsec = 10'000'000;
    auto baro = std::make_shared<cobra::BarometerToAltitudeConfig>();
    baro->group_ = "config/baro";
    baro->channels = std::vector<std::string>{"/sensor/barometer"};
    auto outage = std::make_shared<cobra::OutageConfig>();
    outage->group_ = "config/outage";
    outage->channels = std::vector<std::string>{"/sensor/imu"};
    outage->start_time = 2.0;
    outage->end_time = 3.0;
    auto tb = std::make_shared<cobra::TimeBiasConfig>();
    tb->group_ = "config/time_bias";
    tb->channels = std::vector<std::string>{"/sensor/gps"};
    tb->time_bias = 100'000'000;
    registry = std::make_shared<cobra::StandardRegistryPlugin>(
        "Standard registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{ds, rot, ta, baro, outage, tb});
    registry->init_plugin(std::nullopt, &med);
    med.set_registry(registry->new_registry());
    plugin = std::make_unique<cobra::StandardPreprocessorPlugin>("preprocessor_plugin");
    plugin->init_plugin(std::nullopt, &med);
  }
  std::size_t idx(const std::string& id) const {
    const auto& ids = plugin->preprocessor_identifiers();
    return static_cast<std::size_t>(std::find(ids.begin(), ids.end(), id) - ids.begin());
  }
  static Message altitude(const std::string& channel) {
    auto alt = std::make_shared<aspn23_eigen::MeasurementAltitude>(
        header(ASPN_MEASUREMENT_ALTITUDE), aspn23_eigen::TypeTimestamp(std::int64_t{0}),
        ASPN23_MEASUREMENT_ALTITUDE_REFERENCE_HAE, 200.0, 0.2, ASPN23_MEASUREMENT_ALTITUDE_ERROR_MODEL_NONE,
        DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});
    return Message(alt, channel);
  }
  static Message imu(std::int64_t t) {
    return Message(make_imu(t, Vector3(0.01, 0.02, 0.03), Vector3(0.04, 0.05, 0.06)), "/sensor/imu");
  }
  static std::int64_t tov(const Message& m) { return cobra::utils::time_of_validity(*m.wrapped_message)->elapsed_nsec; }

  TestMediator med;
  std::shared_ptr<cobra::StandardRegistryPlugin> registry;
  std::unique_ptr<cobra::StandardPreprocessorPlugin> plugin;
};

TEST_F(PreprocessorTest, PluginConstructor) {
  EXPECT_EQ(plugin->identifier(), "preprocessor_plugin");
  EXPECT_EQ(plugin->preprocessor_identifiers(), kIds);
}

TEST_F(PreprocessorTest, InvalidMediatorAndIndex) {
  cobra::StandardPreprocessorPlugin p("x");
  p.init_plugin(std::nullopt, nullptr);
  EXPECT_EQ(p.new_preprocessor(0, "config/downsampler"), nullptr);
  EXPECT_EQ(plugin->new_preprocessor(kIds.size(), "invalid"), nullptr);
  EXPECT_EQ(plugin->new_preprocessor(static_cast<std::size_t>(-1), "invalid"), nullptr);
}

TEST_F(PreprocessorTest, BadAndMissingConfigGroup) {
  for (const auto& id : kIds) {
    EXPECT_EQ(plugin->new_preprocessor(idx(id), "wrong_group"), nullptr) << id;
    EXPECT_EQ(plugin->new_preprocessor(idx(id), std::nullopt), nullptr) << id;
  }
}

TEST_F(PreprocessorTest, Downsampler) {
  auto ds = plugin->new_preprocessor(0, "config/downsampler");
  ASSERT_TRUE(ds);
  auto bad = altitude("bad");
  for (int i = 0; i < 2; ++i) {
    auto out = ds->process_pntos_message(bad);
    ASSERT_TRUE(out && out->size() == 1);
    EXPECT_EQ((*out)[0], bad);
  }
  auto t1 = altitude("test1");
  EXPECT_TRUE(ds->process_pntos_message(t1).has_value());
  EXPECT_FALSE(ds->process_pntos_message(t1).has_value());
  EXPECT_TRUE(ds->process_pntos_message(t1).has_value());
  EXPECT_FALSE(ds->process_pntos_message(t1).has_value());
  auto t2 = altitude("test2");
  EXPECT_TRUE(ds->process_pntos_message(t2).has_value());
  EXPECT_FALSE(ds->process_pntos_message(t2).has_value());
  EXPECT_FALSE(ds->process_pntos_message(t2).has_value());
  EXPECT_TRUE(ds->process_pntos_message(t2).has_value());
  auto t3 = altitude("test3");  // negative factor -> not downsampled
  for (int i = 0; i < 3; ++i) EXPECT_TRUE(ds->process_pntos_message(t3).has_value());
  EXPECT_GE(med.count(api::LoggingLevel::WARN), 1u);
}

TEST_F(PreprocessorTest, ImuRotation) {
  auto rot = plugin->new_preprocessor(idx("imu_rotator"), "config/imu_rotator");
  ASSERT_TRUE(rot);
  auto in = imu(1'000'000'000);
  auto out = rot->process_pntos_message(in);
  ASSERT_TRUE(out && out->size() == 1);
  auto r = (*out)[0].as<aspn23_eigen::MeasurementImu>();
  ASSERT_TRUE(r);
  Matrix3 C;
  C << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  EXPECT_ALLCLOSE(Vector(r->get_meas_accel()), Vector(C * Vector3(0.01, 0.02, 0.03)));
  EXPECT_ALLCLOSE(Vector(r->get_meas_gyro()), Vector(C * Vector3(0.04, 0.05, 0.06)));
  // the input message is untouched (immutability)
  EXPECT_ALLCLOSE(Vector(in.as<aspn23_eigen::MeasurementImu>()->get_meas_accel()), vec({0.01, 0.02, 0.03}));
  // non-IMU passes through with a warning
  auto alt = altitude("/sensor/imu");
  auto o2 = rot->process_pntos_message(alt);
  ASSERT_TRUE(o2);
  EXPECT_EQ((*o2)[0], alt);
}

TEST_F(PreprocessorTest, TimeAdjuster) {
  auto ta = plugin->new_preprocessor(idx("time_adjuster"), "config/time_adjuster");
  ASSERT_TRUE(ta);
  auto run = [&](std::int64_t t) {
    auto out = ta->process_pntos_message(imu(t));
    EXPECT_TRUE(out && out->size() == 1);
    return tov((*out)[0]);
  };
  EXPECT_EQ(run(1'000'000'000), 1'000'000'000);
  EXPECT_EQ(run(1'010'000'000), 1'010'000'000);
  EXPECT_EQ(run(1'500'000'000), 1'020'000'000);  // bad -> synthetic
  EXPECT_EQ(run(1'025'000'000), 1'030'000'000);  // low dt -> synthetic
  EXPECT_EQ(run(1'040'015'000), 1'040'015'000);  // within tolerance above
  EXPECT_EQ(run(1'049'978'000), 1'049'978'000);  // within tolerance below
  EXPECT_EQ(run(1'060'000'000), 1'060'000'000);
}

TEST_F(PreprocessorTest, BarometerToAltitude) {
  auto b2a = plugin->new_preprocessor(idx("baro_converter"), "config/baro");
  ASSERT_TRUE(b2a);
  auto baro = std::make_shared<aspn23_eigen::MeasurementBarometer>(
      aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_BAROMETER, 4, 5, 6, 7), aspn23_eigen::TypeTimestamp(std::int64_t{0}),
      90000.0, 10.0, ASPN23_MEASUREMENT_BAROMETER_ERROR_MODEL_NONE, DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});
  auto out = b2a->process_pntos_message(Message(baro, "/sensor/baro_pressure"));
  ASSERT_TRUE(out && out->size() == 1);
  auto alt = (*out)[0].as<aspn23_eigen::MeasurementAltitude>();
  ASSERT_TRUE(alt);
  EXPECT_NEAR(alt->get_altitude(), 988.50, 0.01);
  EXPECT_NEAR(alt->get_variance(), std::pow(988.50 / 90000, 2) * 10, 1e-6);
  EXPECT_EQ(alt->get_reference(), ASPN23_MEASUREMENT_ALTITUDE_REFERENCE_MSL);
  EXPECT_EQ((*out)[0].source_identifier, "/sensor/altitude");
  EXPECT_EQ(alt->get_vendor_id(), 4u);
  // wrong type passes through
  auto i = imu(1'000'000'000);
  auto o2 = b2a->process_pntos_message(i);
  ASSERT_TRUE(o2);
  EXPECT_EQ((*o2)[0], i);
  // alt_sigma override
  cobra::BarometerToAltitudePreprocessor fixed(&med, 2.0);
  auto o3 = fixed.process_pntos_message(Message(baro, "/b"));
  EXPECT_NEAR((*o3)[0].as<aspn23_eigen::MeasurementAltitude>()->get_variance(), 4.0, 1e-12);
}

TEST_F(PreprocessorTest, TimeBias) {
  auto tb = plugin->new_preprocessor(idx("time_bias"), "config/time_bias");
  ASSERT_TRUE(tb);
  auto out = tb->process_pntos_message(imu(1'000'000'000));
  ASSERT_TRUE(out);
  EXPECT_EQ(tov((*out)[0]), 900'000'000);
}

TEST_F(PreprocessorTest, Outage) {
  auto pp = plugin->new_preprocessor(idx("outage"), "config/outage");
  ASSERT_TRUE(pp);
  auto* outage = dynamic_cast<cobra::OutagePreprocessor*>(pp.get());
  ASSERT_TRUE(outage);
  EXPECT_TRUE(pp->process_pntos_message(imu(10'000'000'000)).has_value());
  EXPECT_EQ(outage->first_msg_time_ns(), 10'000'000'000);
  EXPECT_TRUE(pp->process_pntos_message(imu(11'000'000'000)).has_value());
  EXPECT_FALSE(pp->process_pntos_message(imu(12'000'000'000)).has_value());
  EXPECT_FALSE(pp->process_pntos_message(imu(12'500'000'000)).has_value());
  EXPECT_TRUE(pp->process_pntos_message(imu(13'000'000'000)).has_value());
  EXPECT_EQ(med.count(api::LoggingLevel::INFO), 2u);
}

}  // namespace
