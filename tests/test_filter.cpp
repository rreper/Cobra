// Push API (cobra::Filter), the via-push log runner and the C ABI (roadmap Phase 3).
#include <pntos/cobra/app/AppBuilder.hpp>
#include <pntos/cobra/app/Filter.hpp>
#include <pntos/cobra/capi/cobra.h>
#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/transport/LcmLog.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include "test_support.hpp"

#include <cstdlib>
#include <filesystem>
#include <set>

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

struct Epoch {
  std::int64_t t;
  double lat, lon, alt;
};
std::vector<Epoch> solutions_in(const std::string& log) {
  std::vector<Epoch> out;
  cobra::lcm::LcmLogReader r(log);
  while (auto ev = r.next()) {
    if (ev->channel != "/solution/pntos/pva") continue;
    auto pva = std::dynamic_pointer_cast<const cobra::utils::PVA>(cobra::lcm::decode(ev->data));
    if (pva) out.push_back({pva->get_time_of_validity().get_elapsed_nsec(), pva->get_p1(), pva->get_p2(), pva->get_p3()});
  }
  return out;
}

cobra::app::RunOptions quiet(const std::string& out) {
  cobra::app::RunOptions o;
  o.progress = false;
  o.input_log = src("testdata/example_60s.log");
  o.output_log = out;
  o.record_input = false;
  return o;
}
}  // namespace

TEST(Filter, PushMatchesTheLogTransport) {
  auto config = cobra::jsoncfg::load_app_config(src("configs/pos_ins.json"));
  const std::string via_log = out_dir() + "/filter_via_log.log", via_push = out_dir() + "/filter_via_push.log";
  ASSERT_EQ(cobra::app::run_app(config, quiet(via_log)), 0);
  auto o = quiet(via_push);
  o.via_push = true;
  ASSERT_EQ(cobra::app::run_app(config, o), 0);
  auto a = solutions_in(via_log), b = solutions_in(via_push);
  ASSERT_GE(a.size(), 40u);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].t, b[i].t);
    EXPECT_DOUBLE_EQ(a[i].lat, b[i].lat);
    EXPECT_DOUBLE_EQ(a[i].lon, b[i].lon);
    EXPECT_DOUBLE_EQ(a[i].alt, b[i].alt);
  }
}

TEST(Filter, DirectUse) {
  auto config = cobra::jsoncfg::load_app_config(src("configs/pos_ins.json"));
  cobra::app::RunOptions o;
  o.progress = false;
  cobra::Filter f(config, o);
  EXPECT_FALSE(f.error_logged());
  std::size_t callback_count = 0;
  f.set_solution_callback([&](const api::Message&) { ++callback_count; });
  // feed the first 40 s of the example log on the two channels the app uses
  const std::set<std::string> channels{"/sensor/vn-100/imu", "/sensor/ublox-ZED-F9T/position"};
  cobra::lcm::LcmLogReader r(src("testdata/example_60s.log"));
  std::int64_t first = 0, last = 0;
  std::size_t pushed = 0;
  while (auto ev = r.next()) {
    if (!channels.count(ev->channel)) continue;
    auto msg = cobra::lcm::decode(ev->data);
    auto tov = cobra::utils::time_of_validity(*msg);
    if (!first) first = tov->elapsed_nsec;
    if (tov->elapsed_nsec - first > 40'000'000'000) break;
    last = tov->elapsed_nsec;
    f.push(msg, ev->channel);
    ++pushed;
  }
  EXPECT_GT(pushed, 3000u);
  auto sols = f.take_solutions();
  EXPECT_GE(sols.size(), 25u);  // 10 s alignment, then one per second
  EXPECT_EQ(callback_count, sols.size());
  EXPECT_TRUE(f.take_solutions().empty());
  EXPECT_EQ(sols.front().source_identifier, "/solution/pntos/pva");
  // on-demand solution: the orchestration answers at the latest inertial time when asked for a time it
  // cannot serve exactly (Python quirk 8), so expect a solution within a second of the last push
  auto s = f.solution(api::Timestamp{last});
  ASSERT_TRUE(s);
  auto pva = s->as<cobra::utils::PVA>();
  ASSERT_TRUE(pva);
  EXPECT_NEAR(pva->get_time_of_validity().get_elapsed_nsec() * 1e-9, last * 1e-9, 1.0);
  // registry reachable; stop is idempotent
  EXPECT_TRUE(f.registry().has_group("config/orchestration"));
  EXPECT_EQ(f.stop(), 0);
  EXPECT_EQ(f.stop(), 0);
  EXPECT_THROW(f.push(sols.front()), std::logic_error);
}

TEST(CApi, PushAndPoll) {
  const std::string cfg = src("configs/pos_ins.json");
  EXPECT_EQ(cobra_filter_create("/nonexistent.json", nullptr), nullptr);
  EXPECT_NE(std::string(cobra_last_error()).find("cannot open"), std::string::npos);
  cobra_filter* f = cobra_filter_create(cfg.c_str(), "{\"logging_level\": \"WARN\"}");
  ASSERT_NE(f, nullptr) << cobra_last_error();
  EXPECT_STREQ(cobra_version(), "0.2.0");
  const std::set<std::string> channels{"/sensor/vn-100/imu", "/sensor/ublox-ZED-F9T/position"};
  cobra::lcm::LcmLogReader r(src("testdata/example_60s.log"));
  std::int64_t first = 0;
  int solutions = 0;
  cobra_pva sol{};
  while (auto ev = r.next()) {
    if (!channels.count(ev->channel)) continue;
    auto tov = cobra::utils::time_of_validity(*cobra::lcm::decode(ev->data));
    if (!first) first = tov->elapsed_nsec;
    if (tov->elapsed_nsec - first > 30'000'000'000) break;
    ASSERT_EQ(cobra_filter_push_lcm(f, ev->channel.c_str(), ev->data.data(), ev->data.size()), 1) << cobra_last_error();
    while (cobra_filter_poll_solution(f, &sol)) ++solutions;
  }
  EXPECT_GE(solutions, 15);
  EXPECT_NEAR(sol.lat_rad, 0.6939, 1e-3);
  EXPECT_STREQ(sol.channel, "/solution/pntos/pva");
  EXPECT_GT(sol.cov[0], 0.0);
  // typed pushes are accepted too (a position with the identity covariance), and bad input is rejected
  const double cov[9] = {1, 0, 0, 0, 1, 0, 0, 0, 4};
  EXPECT_EQ(cobra_filter_push_position(f, "/sensor/ublox-ZED-F9T/position", sol.tov_ns + 1'000'000'000, sol.lat_rad, sol.lon_rad, sol.alt_m, cov), 1);
  EXPECT_EQ(cobra_filter_push_position(f, nullptr, 0, 0, 0, 0, cov), 0);
  EXPECT_EQ(cobra_filter_push_lcm(f, "/x", reinterpret_cast<const uint8_t*>("garbage"), 7), 0);
  EXPECT_EQ(cobra_filter_solution_at(f, sol.tov_ns, &sol), 1);
  EXPECT_EQ(cobra_filter_error_logged(f), 0);
  EXPECT_EQ(cobra_filter_stop(f), 0);
  cobra_filter_destroy(f);
  cobra_filter_destroy(nullptr);
  EXPECT_EQ(cobra_filter_push_imu(nullptr, "/imu", 0, cov, cov, 0), 0);
}
