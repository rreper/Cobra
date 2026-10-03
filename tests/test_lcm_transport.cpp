// LCM log reader/writer, LCM<->Eigen conversions, and the log transport plugin
// (port of the log-related parts of test_transport_plugin.py plus round-trip checks).
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/transport/LcmLog.hpp>
#include <pntos/cobra/transport/LcmLogTransportPlugin.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include "test_support.hpp"

#include <aspn23/eigen/MeasurementAltitude.hpp>
#include <aspn23/eigen/MeasurementBarometer.hpp>
#include <aspn23/eigen/MeasurementVelocity.hpp>

#include <cstdio>
#include <iostream>
#include <thread>
#include <filesystem>

using namespace pntos;
using namespace pntos::test;
using api::Matrix;
using api::Message;
using api::Vector;
using api::Vector3;
namespace fs = std::filesystem;

namespace {
const std::string kExampleLog =
    std::string(std::getenv("HOME") ? std::getenv("HOME") : "") +
    "/orin/work/pntos/Cobra/.venv/lib/python3.12/site-packages/pntos_python_datasets_lcm/cobra_gps_ins_example_data.log";

std::string temp_path(const std::string& name) {
  return (fs::temp_directory_path() / ("pntos_cpp_test_" + name + "_" + std::to_string(::getpid()) + ".log")).string();
}

TEST(LcmLog, WriteReadRoundTrip) {
  const std::string path = temp_path("rw");
  {
    cobra::lcm::LcmLogWriter w(path);
    std::vector<std::uint8_t> a{1, 2, 3}, b{};
    w.write(1000, "/a", a);
    w.write(2000, "/b", b);
    EXPECT_EQ(w.events_written(), 2);
  }
  cobra::lcm::LcmLogReader r(path);
  auto e1 = r.next();
  ASSERT_TRUE(e1);
  EXPECT_EQ(e1->event_number, 0);
  EXPECT_EQ(e1->timestamp_us, 1000);
  EXPECT_EQ(e1->channel, "/a");
  EXPECT_EQ(e1->data, (std::vector<std::uint8_t>{1, 2, 3}));
  auto e2 = r.next();
  ASSERT_TRUE(e2);
  EXPECT_EQ(e2->channel, "/b");
  EXPECT_TRUE(e2->data.empty());
  EXPECT_FALSE(r.next());
  fs::remove(path);
}

TEST(LcmConversions, EncodeDecodeAllSupportedTypes) {
  auto imu = make_imu(123, Vector3(0.1, 0.2, 0.3), Vector3(0.4, 0.5, 0.6));
  imu->set_imu_type(ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED);
  auto pos = make_position(456, 0.7, -1.4, 200, Matrix::Identity(3, 3) * 2.5);
  auto pva = make_pva(789, 0.7, -1.4, 200, 1, 2, 3, vec({0.5, 0.5, 0.5, 0.5}), Matrix::Identity(9, 9) * 0.1);
  auto vel = std::make_shared<aspn23_eigen::MeasurementVelocity>(
      header(ASPN_MEASUREMENT_VELOCITY), aspn23_eigen::TypeTimestamp(std::int64_t{10}),
      ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED, 1.0, 2.0, 3.0, rm(Matrix::Identity(3, 3)),
      ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE, DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});
  auto alt = std::make_shared<aspn23_eigen::MeasurementAltitude>(
      header(ASPN_MEASUREMENT_ALTITUDE), aspn23_eigen::TypeTimestamp(std::int64_t{11}), ASPN23_MEASUREMENT_ALTITUDE_REFERENCE_HAE,
      123.0, 4.0, ASPN23_MEASUREMENT_ALTITUDE_ERROR_MODEL_NONE, DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});
  auto baro = std::make_shared<aspn23_eigen::MeasurementBarometer>(
      aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_BAROMETER, 4, 5, 6, 7), aspn23_eigen::TypeTimestamp(std::int64_t{12}), 90000.0,
      10.0, ASPN23_MEASUREMENT_BAROMETER_ERROR_MODEL_NONE, DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});

  auto rt = [](const api::AspnBase& m) {
    auto bytes = cobra::lcm::encode(m);
    EXPECT_TRUE(bytes);
    auto back = cobra::lcm::decode(*bytes);
    EXPECT_TRUE(back);
    return back;
  };
  auto imu2 = std::dynamic_pointer_cast<aspn23_eigen::MeasurementImu>(rt(*imu));
  ASSERT_TRUE(imu2);
  EXPECT_EQ(imu2->get_time_of_validity().get_elapsed_nsec(), 123);
  EXPECT_EQ(imu2->get_imu_type(), ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED);
  EXPECT_ALLCLOSE(Vector(imu2->get_meas_gyro()), vec({0.4, 0.5, 0.6}));
  auto pos2 = std::dynamic_pointer_cast<aspn23_eigen::MeasurementPosition>(rt(*pos));
  ASSERT_TRUE(pos2);
  EXPECT_EQ(pos2->get_reference_frame(), ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC);
  EXPECT_DOUBLE_EQ(pos2->get_term2(), -1.4);
  EXPECT_ALLCLOSE(Matrix(pos2->get_covariance()), Matrix(Matrix::Identity(3, 3) * 2.5));
  auto pva2 = std::dynamic_pointer_cast<aspn23_eigen::MeasurementPositionVelocityAttitude>(rt(*pva));
  ASSERT_TRUE(pva2);
  EXPECT_ALLCLOSE(Vector(pva2->get_quaternion()), vec({0.5, 0.5, 0.5, 0.5}));
  EXPECT_EQ(pva2->get_covariance().rows(), 9);
  EXPECT_DOUBLE_EQ(pva2->get_v3(), 3);
  auto vel2 = std::dynamic_pointer_cast<aspn23_eigen::MeasurementVelocity>(rt(*vel));
  ASSERT_TRUE(vel2);
  EXPECT_EQ(vel2->get_reference_frame(), ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED);
  auto alt2 = std::dynamic_pointer_cast<aspn23_eigen::MeasurementAltitude>(rt(*alt));
  ASSERT_TRUE(alt2);
  EXPECT_EQ(alt2->get_reference(), ASPN23_MEASUREMENT_ALTITUDE_REFERENCE_HAE);
  EXPECT_DOUBLE_EQ(alt2->get_altitude(), 123.0);
  auto baro2 = std::dynamic_pointer_cast<aspn23_eigen::MeasurementBarometer>(rt(*baro));
  ASSERT_TRUE(baro2);
  EXPECT_DOUBLE_EQ(baro2->get_pressure(), 90000.0);
  EXPECT_EQ(baro2->get_vendor_id(), 4u);
  // PVA without quaternion -> NaN on the wire -> absent after decode
  Vector nanq = Vector::Constant(4, std::nan(""));
  auto noq = make_pva(1, 0, 0, 0, 0, 0, 0, nanq);
  auto noq2 = std::dynamic_pointer_cast<aspn23_eigen::MeasurementPositionVelocityAttitude>(rt(*noq));
  ASSERT_TRUE(noq2);
  EXPECT_FALSE(cobra::utils::quaternion(*noq2).has_value());
  // unsupported bytes
  std::vector<std::uint8_t> junk(16, 0);
  EXPECT_EQ(cobra::lcm::decode(junk), nullptr);
  EXPECT_FALSE(cobra::lcm::type_name_for(junk.data(), junk.size()).has_value());
}

TEST(LcmConversions, DecodesTheRealExampleLog) {
  if (!fs::exists(kExampleLog)) GTEST_SKIP() << "example log not present";
  cobra::lcm::LcmLogReader r(kExampleLog);
  std::map<std::string, int> seen, unsupported;
  std::optional<double> imu_dv, pva_lat;
  for (int i = 0; i < 3000; ++i) {
    auto ev = r.next();
    ASSERT_TRUE(ev);
    auto msg = cobra::lcm::decode(ev->data);
    if (!msg) {
      unsupported[ev->channel]++;
      continue;
    }
    seen[ev->channel]++;
    if (auto imu = std::dynamic_pointer_cast<aspn23_eigen::MeasurementImu>(msg); imu && !imu_dv)
      imu_dv = Vector(imu->get_meas_accel()).norm();
    if (auto pva = std::dynamic_pointer_cast<aspn23_eigen::MeasurementPositionVelocityAttitude>(msg); pva && !pva_lat)
      pva_lat = pva->get_p1();
  }
  for (const auto& [ch, n] : unsupported) std::cerr << "unsupported: " << ch << " x" << n << "\n";
  // Only the simulated direction-to-points channel is expected to be unsupported.
  for (const auto& [ch, n] : unsupported) EXPECT_EQ(ch, "/sensor/simulated/directiontoknownfeature");
  EXPECT_GT(seen["/sensor/vn-100/imu"], 500);
  EXPECT_GT(seen["/sensor/ins-d/pva"], 500);
  ASSERT_TRUE(imu_dv);
  EXPECT_NEAR(*imu_dv, 9.81 * 0.01, 0.01);  // integrated over 10 ms
  ASSERT_TRUE(pva_lat);
  EXPECT_NEAR(*pva_lat, 0.694, 0.01);
}

class LogTransportTest : public ::testing::Test {
 protected:
  void SetUp() override {
    input = temp_path("in");
    output = temp_path("out");
    cobra::lcm::LcmLogWriter w(input);
    for (int i = 0; i < 10; ++i) {
      auto imu = make_imu(i * 10'000'000, Vector3(0, 0, -0.0981), Vector3::Zero());
      imu->set_imu_type(ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED);
      w.write(i, "/imu", *cobra::lcm::encode(*imu));
      if (i % 5 == 0) w.write(i, "/pos", *cobra::lcm::encode(*make_position(i * 10'000'000, 0.7, -1.4, 100, Matrix::Identity(3, 3))));
    }
    w.write(99, "/junk", std::vector<std::uint8_t>(12, 7));
  }
  void TearDown() override {
    fs::remove(input);
    fs::remove(output);
  }
  void make_plugin(std::optional<std::vector<std::string>> channels, bool with_output, bool record_input = true) {
    auto cfg = std::make_shared<cobra::LcmLogTransportConfig>();
    cfg->input_file = input;
    if (with_output) cfg->output_file = output;
    cfg->channels_to_process = channels;
    cfg->record_input_channels = record_input;
    registry = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{cfg});
    registry->init_plugin(std::nullopt, &med);
    med.set_registry(registry->new_registry());
    plugin = std::make_unique<cobra::LcmLogTransportPlugin>("lcm log");
    plugin->init_plugin(std::nullopt, &med);
  }
  std::string input, output;
  TestMediator med;
  std::shared_ptr<cobra::StandardRegistryPlugin> registry;
  std::unique_ptr<cobra::LcmLogTransportPlugin> plugin;
};

TEST_F(LogTransportTest, ReplaysEverythingAndFlagsShutdown) {
  make_plugin(std::nullopt, false);
  plugin->read_log();
  EXPECT_EQ(med.processed.size(), 12u);
  EXPECT_EQ(plugin->channels_found().size(), 2u);
  EXPECT_EQ(med.processed[0].source_identifier, "/imu");
  EXPECT_GE(med.count(api::LoggingLevel::WARN), 1u);  // junk event
  auto flag = med.registry().batch("controller/flags")->get_value<bool>("ready_to_shutdown");
  ASSERT_TRUE(flag);
  EXPECT_TRUE(*flag);
  EXPECT_TRUE(med.registry().has_group("ui/channel//imu"));
}

TEST_F(LogTransportTest, ChannelFilterAndOutputRecording) {
  make_plugin(std::vector<std::string>{"/pos"}, true);
  plugin->read_log();
  EXPECT_EQ(med.processed.size(), 2u);
  // broadcast a solution and a non-encodable message
  plugin->broadcast_message(Message(make_pva(5, 0.7, -1.4, 100, 0, 0, 0, vec({1, 0, 0, 0})), "/solution/pntos/pva"),
                            "/solution/pntos/pva");
  plugin->broadcast_message(Message(make_pva(5, 0.7, -1.4, 100, 0, 0, 0, vec({1, 0, 0, 0})), "x"), std::nullopt);  // WARN
  plugin->shutdown_plugin();
  cobra::lcm::LcmLogReader r(output);
  std::map<std::string, int> counts;
  while (auto ev = r.next()) counts[ev->channel]++;
  EXPECT_EQ(counts["/imu"], 10);   // input recorded regardless of the processing filter
  EXPECT_EQ(counts["/pos"], 2);
  EXPECT_EQ(counts["/junk"], 1);
  EXPECT_EQ(counts["/solution/pntos/pva"], 1);
}

TEST_F(LogTransportTest, ThreadedListenAndStop) {
  make_plugin(std::nullopt, true, /*record_input=*/false);
  plugin->start_listening();
  for (int i = 0; i < 500 && plugin->messages_processed() < 12; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(2));
  plugin->shutdown_plugin();  // joins the reader thread
  EXPECT_EQ(med.processed.size(), 12u);
  cobra::lcm::LcmLogReader r(output);
  EXPECT_FALSE(r.next());  // nothing recorded
}

TEST_F(LogTransportTest, SameInputAndOutputIsAnError) {
  auto cfg = std::make_shared<cobra::LcmLogTransportConfig>();
  cfg->input_file = input;
  cfg->output_file = input;
  registry = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{cfg});
  registry->init_plugin(std::nullopt, &med);
  med.set_registry(registry->new_registry());
  cobra::LcmLogTransportPlugin p("lcm log");
  p.init_plugin(std::nullopt, &med);
  EXPECT_TRUE(med.has_error());
  // no output configured -> broadcast is an error
  p.broadcast_message(Message(make_pva(5, 0, 0, 0, 0, 0, 0, vec({1, 0, 0, 0})), "x"), "x");
  auto back = cobra::LcmLogTransportConfig::from_registry(med);
  ASSERT_TRUE(back);
  EXPECT_EQ(back->input_file, input);
  EXPECT_TRUE(back->record_input_channels);
}

}  // namespace
