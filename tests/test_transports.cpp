// Network LCM (UDP multicast) and CSV transports (roadmap Phase 3).
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/app/AppBuilder.hpp>
#include <pntos/cobra/transport/CsvTransportPlugin.hpp>
#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/transport/LcmLog.hpp>
#include <pntos/cobra/transport/LcmUdpTransportPlugin.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include "test_support.hpp"

#include <aspn23/eigen/MeasurementImu.hpp>
#include <aspn23/eigen/MeasurementPosition.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>

using namespace pntos;
using namespace pntos::test;

namespace {
std::string src(const char* rel) {
  const char* s = std::getenv("PNTOS_TEST_SRCDIR");
  return std::string(s ? s : "..") + "/" + rel;
}
std::string out_dir() {
  const char* d = std::getenv("PNTOS_TEST_OUT");
  return d ? d : std::filesystem::temp_directory_path().string();
}
}  // namespace

TEST(LcmUdp, UrlAndDatagrams) {
  bool ok = false;
  auto u = cobra::parse_udpm_url("udpm://239.255.76.67:7667?ttl=1", &ok);
  EXPECT_TRUE(ok);
  EXPECT_EQ(u.address, "239.255.76.67");
  EXPECT_EQ(u.port, 7667);
  EXPECT_EQ(u.ttl, 1);
  auto d = cobra::parse_udpm_url("tcpq://", &ok);
  EXPECT_FALSE(ok);
  EXPECT_EQ(d.port, 7667);
  // short message round trip
  std::vector<std::uint8_t> payload(1000);
  for (std::size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<std::uint8_t>(i * 7);
  auto dgrams = cobra::lcm::lcm_datagrams("/chan", payload, 42);
  ASSERT_EQ(dgrams.size(), 1u);
  cobra::lcm::LcmReassembler ra;
  auto got = ra.feed(dgrams[0].data(), dgrams[0].size(), "a");
  ASSERT_TRUE(got);
  EXPECT_EQ(got->first, "/chan");
  EXPECT_EQ(got->second, payload);
  // fragmented round trip, fragments delivered out of order
  std::vector<std::uint8_t> big(150'000);
  for (std::size_t i = 0; i < big.size(); ++i) big[i] = static_cast<std::uint8_t>(i % 251);
  auto frags = cobra::lcm::lcm_datagrams("/big", big, 7);
  ASSERT_EQ(frags.size(), 3u);
  EXPECT_FALSE(ra.feed(frags[2].data(), frags[2].size(), "a"));
  EXPECT_FALSE(ra.feed(frags[0].data(), frags[0].size(), "a"));
  auto whole = ra.feed(frags[1].data(), frags[1].size(), "a");
  ASSERT_TRUE(whole);
  EXPECT_EQ(whole->first, "/big");
  EXPECT_EQ(whole->second, big);
  // garbage is ignored
  const std::uint8_t junk[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  EXPECT_FALSE(ra.feed(junk, sizeof junk, "a"));
}

TEST(LcmUdp, LoopbackDelivery) {
  // two plugins on the same multicast group: what one broadcasts, the other receives and decodes
  auto cfg = std::make_shared<cobra::LcmTransportConfig>();
  cfg->url = "udpm://239.255.76.67:7699?ttl=0";  // not the default port: other runs may be on it
  cfg->subscribe_to = "^/sensor/";
  cfg->output_file = out_dir() + "/udp_received.log";
  TestMediator med_rx, med_tx;
  auto reg = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{cfg});
  reg->init_plugin(std::nullopt, &med_rx);
  med_rx.set_registry(reg->new_registry());
  med_tx.set_registry(reg->new_registry());
  cobra::LcmUdpTransportPlugin rx("rx"), tx("tx");
  rx.init_plugin(std::nullopt, &med_rx);
  tx.init_plugin(std::nullopt, &med_tx);
  rx.start_listening();
  if (!rx.socket_open()) GTEST_SKIP() << "multicast socket unavailable here: " << med_rx.last_message();
  tx.start_listening();
  ASSERT_TRUE(tx.socket_open());
  auto pos = make_position(1'000'000'000, 0.7, -1.4, 100.0, api::Matrix::Identity(3, 3));
  tx.broadcast_message(api::Message(pos, "/sensor/pos"), std::nullopt);
  tx.broadcast_message(api::Message(pos, "/other/pos"), std::nullopt);  // filtered out by the regex
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (med_rx.processed.empty() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  rx.stop_listening();
  tx.stop_listening();
  ASSERT_EQ(med_rx.processed.size(), 1u) << "no multicast loopback delivery";
  EXPECT_EQ(med_rx.processed[0].source_identifier, "/sensor/pos");
  auto back = med_rx.processed[0].as<aspn23_eigen::MeasurementPosition>();
  ASSERT_TRUE(back);
  EXPECT_DOUBLE_EQ(back->get_term1(), 0.7);
  EXPECT_EQ(rx.messages_received(), 1u);
  EXPECT_EQ(tx.messages_sent(), 2u);
  rx.shutdown_plugin();
  // the received message was recorded
  cobra::lcm::LcmLogReader r(*cfg->output_file);
  auto ev = r.next();
  ASSERT_TRUE(ev);
  EXPECT_EQ(ev->channel, "/sensor/pos");
}

TEST(Csv, ReadsFilesAndWritesSolutions) {
  const std::string dir = out_dir();
  {
    std::ofstream imu(dir + "/t_imu.csv"), pos(dir + "/t_position.csv"), vel(dir + "/t_velocity.csv");
    imu << "# comment\ntime,gx,gy,gz,ax,ay,az\n1.00,0.01,0,0,0,0,-9.8\n1.01,0.01,0,0,0,0,-9.8\n1.02,,0,0,0,0,-9.8\n";
    pos << "time,lat_deg,lon_deg,alt_m\n1.005,40.0,-105.0,1600\n";
    vel << "time,vn,ve,vd,sigma_n,sigma_e,sigma_d\n1.015,1,2,3,0.1,0.2,0.3\n";
  }
  auto cfg = std::make_shared<cobra::CsvTransportConfig>();
  cfg->imu_file = dir + "/t_imu.csv";
  cfg->position_file = dir + "/t_position.csv";
  cfg->velocity_file = dir + "/t_velocity.csv";
  cfg->imu_channel = "/imu";
  cfg->position_channel = "/pos";
  cfg->velocity_channel = "/vel";
  cfg->default_position_sigma = {3, 3, 6};
  cfg->output_file = dir + "/t_solutions.csv";
  TestMediator med;
  auto reg = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{cfg});
  reg->init_plugin(std::nullopt, &med);
  med.set_registry(reg->new_registry());
  // registry round trip of the config
  auto back = cobra::CsvTransportConfig::from_registry(med);
  ASSERT_TRUE(back);
  EXPECT_EQ(back->position_file, cfg->position_file);
  EXPECT_EQ(back->default_position_sigma[2], 6.0);
  cobra::CsvTransportPlugin t("csv");
  t.init_plugin(std::nullopt, &med);
  t.read_all();
  ASSERT_EQ(med.processed.size(), 5u);  // merged in time order: imu 1.00, pos 1.005, imu 1.01, vel 1.015, imu 1.02
  EXPECT_EQ(med.processed[0].source_identifier, "/imu");
  EXPECT_EQ(med.processed[1].source_identifier, "/pos");
  EXPECT_EQ(med.processed[2].source_identifier, "/imu");
  EXPECT_EQ(med.processed[3].source_identifier, "/vel");
  auto imu0 = med.processed[0].as<aspn23_eigen::MeasurementImu>();
  ASSERT_TRUE(imu0);
  EXPECT_EQ(imu0->get_time_of_validity().get_elapsed_nsec(), 1'000'000'000);
  EXPECT_DOUBLE_EQ(imu0->get_meas_gyro()(0), 0.01);  // columns matched by name, not position
  EXPECT_DOUBLE_EQ(imu0->get_meas_accel()(2), -9.8);
  EXPECT_EQ(imu0->get_imu_type(), ASPN23_MEASUREMENT_IMU_IMU_TYPE_SAMPLED);
  auto p = med.processed[1].as<aspn23_eigen::MeasurementPosition>();
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->get_term1(), 40.0 * M_PI / 180.0, 1e-12);
  EXPECT_DOUBLE_EQ(api::Matrix(p->get_covariance())(2, 2), 36.0);  // default sigma used: no sigma columns
  auto v = med.processed[3].as<aspn23_eigen::MeasurementVelocity>();
  ASSERT_TRUE(v);
  EXPECT_DOUBLE_EQ(api::Matrix(v->get_covariance())(1, 1), 0.04);
  EXPECT_TRUE(std::isnan(med.processed[4].as<aspn23_eigen::MeasurementImu>()->get_meas_gyro()(0)));  // empty cell
  EXPECT_TRUE(med.registry().batch("controller/flags")->get_value<bool>("ready_to_shutdown").value_or(false));
  // a solution broadcast lands in the CSV
  api::Vector q(4);
  q << 1, 0, 0, 0;
  t.broadcast_message(api::Message(make_pva(2'000'000'000, 0.7, -1.4, 100, 1, 2, 3, q, api::Matrix::Identity(9, 9) * 4.0), "/solution/pntos/pva"), std::nullopt);
  t.shutdown_plugin();
  std::ifstream sol(dir + "/t_solutions.csv");
  std::string header, row;
  std::getline(sol, header);
  std::getline(sol, row);
  EXPECT_NE(header.find("yaw_deg"), std::string::npos);
  EXPECT_EQ(row.substr(0, 12), "2.000000000,");
  EXPECT_NE(row.find(",2,"), std::string::npos);  // sigma 2 m
  EXPECT_EQ(t.solutions_written(), 1u);
  // missing column is an error
  {
    std::ofstream bad(dir + "/t_bad.csv");
    bad << "time,ax,ay\n1,0,0\n";
  }
  cfg->imu_file = dir + "/t_bad.csv";
  TestMediator med2;
  auto reg2 = std::make_shared<cobra::StandardRegistryPlugin>("registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{cfg});
  reg2->init_plugin(std::nullopt, &med2);
  med2.set_registry(reg2->new_registry());
  cobra::CsvTransportPlugin t2("csv");
  t2.init_plugin(std::nullopt, &med2);
  t2.read_all();
  EXPECT_TRUE(med2.has_error());
  EXPECT_TRUE(med2.processed.empty());
}

TEST(Csv, FullRunMatchesTheLcmRun) {
  // export the 60 s log to CSV the way tools/lcm_to_csv does, run pos_ins on it, compare with the LCM run
  const std::string dir = out_dir();
  std::ofstream imu(dir + "/e_imu.csv"), pos(dir + "/e_position.csv");
  imu << std::setprecision(17) << "time,ax,ay,az,gx,gy,gz\n";
  pos << std::setprecision(17) << "time,lat_deg,lon_deg,alt_m,sigma_n_m,sigma_e_m,sigma_d_m\n";
  const double r2d = 180.0 / M_PI;
  cobra::lcm::LcmLogReader reader(src("testdata/example_60s.log"));
  bool integrated = false;
  while (auto ev = reader.next()) {
    if (ev->channel == "/sensor/vn-100/imu") {
      auto m = std::dynamic_pointer_cast<const aspn23_eigen::MeasurementImu>(cobra::lcm::decode(ev->data));
      integrated = m->get_imu_type() == ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED;
      const auto a = m->get_meas_accel(), g = m->get_meas_gyro();
      imu << m->get_time_of_validity().get_elapsed_nsec() * 1e-9 << ',' << a(0) << ',' << a(1) << ',' << a(2) << ',' << g(0) << ',' << g(1) << ',' << g(2) << '\n';
    } else if (ev->channel == "/sensor/ublox-ZED-F9T/position") {
      auto m = std::dynamic_pointer_cast<const aspn23_eigen::MeasurementPosition>(cobra::lcm::decode(ev->data));
      const auto c = m->get_covariance();
      pos << m->get_time_of_validity().get_elapsed_nsec() * 1e-9 << ',' << m->get_term1() * r2d << ',' << m->get_term2() * r2d << ',' << m->get_term3()
          << ',' << std::sqrt(c(0, 0)) << ',' << std::sqrt(c(1, 1)) << ',' << std::sqrt(c(2, 2)) << '\n';
    }
  }
  imu.close();
  pos.close();
  auto config = cobra::jsoncfg::load_app_config(src("configs/pos_ins_csv.json"));
  // point the CSV transport at the exported files
  for (auto& c : config.configs)
    if (auto* t = dynamic_cast<const cobra::CsvTransportConfig*>(c.get())) {
      auto mod = std::make_shared<cobra::CsvTransportConfig>(*t);
      mod->imu_file = dir + "/e_imu.csv";
      mod->position_file = dir + "/e_position.csv";
      mod->imu_integrated = integrated;
      mod->output_file = dir + "/e_solutions.csv";
      c = mod;
    }
  cobra::app::RunOptions o;
  o.progress = false;
  ASSERT_EQ(cobra::app::run_app(config, o), 0);
  // reference: the LCM run
  auto lcm_config = cobra::jsoncfg::load_app_config(src("configs/pos_ins.json"));
  cobra::app::RunOptions lo;
  lo.progress = false;
  lo.input_log = src("testdata/example_60s.log");
  lo.output_log = dir + "/e_ref.log";
  lo.record_input = false;
  ASSERT_EQ(cobra::app::run_app(lcm_config, lo), 0);
  std::vector<std::pair<double, double>> ref;  // (t, lat deg)
  cobra::lcm::LcmLogReader rr(dir + "/e_ref.log");
  while (auto ev = rr.next())
    if (auto pva = std::dynamic_pointer_cast<const cobra::utils::PVA>(cobra::lcm::decode(ev->data)))
      ref.push_back({pva->get_time_of_validity().get_elapsed_nsec() * 1e-9, pva->get_p1() * r2d});
  std::vector<std::pair<double, double>> csv;
  std::ifstream sol(dir + "/e_solutions.csv");
  std::string line;
  std::getline(sol, line);
  while (std::getline(sol, line)) {
    std::istringstream ls(line);
    std::string t, lat;
    std::getline(ls, t, ',');
    std::getline(ls, lat, ',');
    csv.push_back({std::atof(t.c_str()), std::atof(lat.c_str())});
  }
  ASSERT_GE(ref.size(), 40u);
  ASSERT_EQ(csv.size(), ref.size());
  for (std::size_t i = 0; i < ref.size(); ++i) {
    EXPECT_NEAR(csv[i].first, ref[i].first, 1e-6);
    EXPECT_NEAR(csv[i].second, ref[i].second, 2e-7);  // the CSV keeps only the diagonal of the position covariance: ~2 cm
  }
}
