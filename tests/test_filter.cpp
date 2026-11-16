// Push API (cobra::Filter), the via-push log runner and the C ABI (roadmap Phase 3).
#include <pntos/cobra/app/AppBuilder.hpp>
#include <pntos/cobra/app/Filter.hpp>
#include <pntos/cobra/capi/cobra.h>
#include <pntos/cobra/config/JsonConfig.hpp>
#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/transport/LcmLog.hpp>
#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/logging.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include "test_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <functional>
#include <iostream>
#include <optional>
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
  f.set_solution_callback([&](const api::Message&) { ++callback_count; }, /*also_queue=*/true);
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
  // a callback registered the default way replaces the queue: a daemon that never polls must not accumulate
  std::size_t more = 0;
  f.set_solution_callback([&](const api::Message&) { ++more; });
  while (auto ev = r.next()) {
    if (!channels.count(ev->channel)) continue;
    auto msg = cobra::lcm::decode(ev->data);
    last = cobra::utils::time_of_validity(*msg)->elapsed_nsec;
    f.push(msg, ev->channel);
  }
  EXPECT_GE(more, 10u);
  EXPECT_TRUE(f.take_solutions().empty());
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

// ----------------------------------------------------------------------------- live IMU faults, end to end

namespace {
namespace inr = cobra::inertial;
using ImuPtr = std::shared_ptr<aspn23_eigen::MeasurementImu>;
/// What to push in place of the i-th IMU message of the log (empty: the message is lost).
using ImuFault = std::function<std::vector<ImuPtr>(int i, const ImuPtr& m)>;

/// pos_ins with the recorded time stamps reaching the inertial: the TimeAdjuster preprocessor of the standard
/// config is taken out (it re-stamps late IMU samples onto a 10 ms grid, which turns a lost sample into a time
/// slip instead of a gap the inertial could see).
cobra::AppConfig live_config(const inr::ImuRobustness& robustness) {
  auto config = cobra::jsoncfg::load_app_config(src("configs/pos_ins.json"));
  for (auto& c : config.configs) {
    auto orch = std::dynamic_pointer_cast<const cobra::StandardOrchestrationConfig>(c);
    if (!orch) continue;
    auto copy = std::make_shared<cobra::StandardOrchestrationConfig>(*orch);
    copy->inertial_config.robustness = robustness;
    if (copy->preprocessor_configs) {
      auto& pp = *copy->preprocessor_configs;
      pp.erase(std::remove_if(pp.begin(), pp.end(),
                              [](const auto& x) { return x->identifier == cobra::TimeAdjusterConfig::kIdentifier; }),
               pp.end());
    }
    c = copy;
  }
  return config;
}

struct LiveEpoch {
  std::int64_t t;
  double lat, lon, alt, vn, ve, vd;
  bool finite() const { return std::isfinite(lat + lon + alt + vn + ve + vd); }
};
struct LiveRun {
  std::vector<LiveEpoch> solutions;        ///< published once per second of message time
  std::optional<LiveEpoch> final_on_demand;  ///< Filter::solution() at the last IMU time, after the whole log
  bool threw = false;
  bool error_logged = false;
  std::size_t finite_after(std::int64_t t) const {
    const auto later_and_finite = [t](const LiveEpoch& e) { return e.t > t && e.finite(); };
    return static_cast<std::size_t>(std::count_if(solutions.begin(), solutions.end(), later_and_finite));
  }
};
LiveRun run_live(const inr::ImuRobustness& robustness, const ImuFault& fault) {
  LiveRun out;
  cobra::app::RunOptions o;
  o.progress = false;
  cobra::Filter f(live_config(robustness), o);
  auto epoch = [](const api::Message& m) -> std::optional<LiveEpoch> {
    auto p = m.as<cobra::utils::PVA>();
    if (!p) return std::nullopt;
    return LiveEpoch{p->get_time_of_validity().get_elapsed_nsec(), p->get_p1(), p->get_p2(), p->get_p3(),
                     p->get_v1(), p->get_v2(), p->get_v3()};
  };
  f.set_solution_callback([&](const api::Message& m) {
    if (auto e = epoch(m)) out.solutions.push_back(*e);
  });
  std::int64_t last_imu_ns = 0;
  const std::string imu_channel = "/sensor/vn-100/imu", position_channel = "/sensor/ublox-ZED-F9T/position";
  cobra::lcm::LcmLogReader r(src("testdata/example_60s.log"));
  int i = 0;
  try {
    while (auto ev = r.next()) {
      if (ev->channel == position_channel) {
        f.push(cobra::lcm::decode(ev->data), ev->channel);
      } else if (ev->channel == imu_channel) {
        auto imu = std::dynamic_pointer_cast<aspn23_eigen::MeasurementImu>(cobra::lcm::decode(ev->data));
        last_imu_ns = imu->get_time_of_validity().get_elapsed_nsec();
        for (const auto& m : fault(i++, imu)) f.push(m, ev->channel);
      }
    }
    if (auto s = f.solution(api::Timestamp{last_imu_ns})) out.final_on_demand = epoch(*s);
  } catch (const std::exception&) {
    out.threw = true;
  }
  out.error_logged = f.error_logged();
  f.stop();
  return out;
}

/// Largest horizontal / vertical distance (m) between two runs after `from_ns`. A solution is stamped with the
/// latest IMU time, so the epochs of a run with lost samples sit a few milliseconds off the clean ones: each
/// epoch of `a` is compared with the nearest one of `b` (within 50 ms), moved to the same instant with its velocity.
struct Difference {
  double horizontal = 0, vertical = 0;
  std::size_t epochs = 0;
};
Difference difference(const std::vector<LiveEpoch>& a, const std::vector<LiveEpoch>& b, std::int64_t from_ns) {
  Difference d;
  for (const auto& x : a) {
    if (x.t < from_ns) continue;
    const LiveEpoch* near = nullptr;
    for (const auto& y : b) {
      const std::int64_t apart = std::llabs(y.t - x.t);
      if (y.finite() && apart <= 50'000'000 && (!near || apart < std::llabs(near->t - x.t))) near = &y;
    }
    if (!near) continue;
    const double dt = (x.t - near->t) * 1e-9;
    const double north = (near->lat - x.lat) * 6.37e6 + near->vn * dt;
    const double east = (near->lon - x.lon) * 6.37e6 * std::cos(x.lat) + near->ve * dt;
    d.horizontal = std::max(d.horizontal, std::hypot(north, east));
    d.vertical = std::max(d.vertical, std::abs(near->alt - near->vd * dt - x.alt));
    ++d.epochs;
  }
  return d;
}

const int kFaultsFrom = 1500;  // IMU message index: 15 s into the log, after the 10 s alignment
ImuFault no_fault() {
  return [](int, const ImuPtr& m) { return std::vector<ImuPtr>{m}; };
}
/// One IMU message in 25 lost (4 %), as on a marginal serial or UDP link.
ImuFault lossy() {
  return [](int i, const ImuPtr& m) {
    return (i > kFaultsFrom && i % 25 == 0) ? std::vector<ImuPtr>{} : std::vector<ImuPtr>{m};
  };
}
/// The lossy link plus a NaN sample and ten seconds of the same data delivered as SAMPLED rates instead of
/// increments; with `future_stamp`, also one sample stamped an hour ahead.
ImuFault everything(bool future_stamp) {
  return [future_stamp](int i, const ImuPtr& m) -> std::vector<ImuPtr> {
    if (i <= kFaultsFrom) return {m};
    if (i % 25 == 0) return {};
    auto copy = std::make_shared<aspn23_eigen::MeasurementImu>(*m);
    if (i == 2001) {
      Eigen::Matrix<double, Eigen::Dynamic, 1> a = copy->get_meas_accel();
      a(1) = std::nan("");
      copy->set_meas_accel(a);
    } else if (i == 5001 && future_stamp) {
      const std::int64_t an_hour_ahead = m->get_time_of_validity().get_elapsed_nsec() + 3'600'000'000'000LL;
      copy->set_time_of_validity(aspn23_eigen::TypeTimestamp(an_hour_ahead));
    } else if (i >= 3000 && i < 4000) {
      copy->set_imu_type(ASPN23_MEASUREMENT_IMU_IMU_TYPE_SAMPLED);
      copy->set_meas_accel(Eigen::Matrix<double, Eigen::Dynamic, 1>(m->get_meas_accel() / 0.01));
      copy->set_meas_gyro(Eigen::Matrix<double, Eigen::Dynamic, 1>(m->get_meas_gyro() / 0.01));
    }
    return {copy};
  };
}
}  // namespace

TEST(Filter, LiveImuFaults) {
  const auto clean = run_live({}, no_fault());
  ASSERT_GE(clean.solutions.size(), 40u);
  ASSERT_FALSE(clean.threw || clean.error_logged);
  // the faults start about 5 s after the first solution
  const std::int64_t from = clean.solutions.front().t + 8'000'000'000;

  // On clean data the live-sensor settings change nothing that matters (the recorded stamps jitter by up to
  // 40 % of an interval; nothing is lost, nothing is filled).
  const auto clean_live = run_live(inr::ImuRobustness::live_sensor(), no_fault());
  auto d = difference(clean.solutions, clean_live.solutions, 0);
  EXPECT_EQ(d.epochs, clean.solutions.size());
  EXPECT_LT(d.horizontal, 1e-3);
  EXPECT_LT(d.vertical, 1e-3);

  // 4 % of the IMU messages lost. Default: each lost sample is 10 ms of free fall; the position fixes hold the
  // solution within metres but it is visibly pulled down. Held rate: within centimetres of the clean run.
  const auto lossy_default = run_live({}, lossy());
  const auto lossy_live = run_live(inr::ImuRobustness::live_sensor(), lossy());
  ASSERT_FALSE(lossy_default.threw);
  ASSERT_FALSE(lossy_live.threw || lossy_live.error_logged);
  const auto dd = difference(clean.solutions, lossy_default.solutions, from);
  const auto dl = difference(clean.solutions, lossy_live.solutions, from);
  EXPECT_GE(dl.epochs, 30u);
  RecordProperty("lossy_default_vertical_m", std::to_string(dd.vertical));
  RecordProperty("lossy_default_horizontal_m", std::to_string(dd.horizontal));
  RecordProperty("lossy_live_vertical_m", std::to_string(dl.vertical));
  RecordProperty("lossy_live_horizontal_m", std::to_string(dl.horizontal));
  std::printf("4%% IMU loss vs clean: default %.3f m horizontal / %.3f m vertical, live %.3f / %.3f m (%zu epochs)\n",
              dd.horizontal, dd.vertical, dl.horizontal, dl.vertical, dl.epochs);
  EXPECT_GT(dd.vertical, 1.0);
  EXPECT_LT(dl.horizontal, 0.25);
  EXPECT_LT(dl.vertical, 0.25);
  EXPECT_LT(dl.vertical * 10, dd.vertical);

  // Loss, a NaN sample and SAMPLED data together. Default: no usable solution after the NaN sample (5 s into
  // the faults), and the first SAMPLED message throws out of push(). Live settings: no exception, no ERROR,
  // close to the clean run.
  const auto all_default = run_live({}, everything(false));
  EXPECT_TRUE(all_default.threw);
  std::printf("default with NaN + sampled: %zu solutions, %zu finite after the NaN sample\n",
              all_default.solutions.size(), all_default.finite_after(from + 6'000'000'000));
  EXPECT_EQ(all_default.finite_after(from + 6'000'000'000), 0u);
  const auto all_live = run_live(inr::ImuRobustness::live_sensor(), everything(false));
  EXPECT_FALSE(all_live.threw);
  EXPECT_FALSE(all_live.error_logged);
  const auto da = difference(clean.solutions, all_live.solutions, from);
  std::printf("loss + NaN + sampled vs clean: live %.3f m horizontal / %.3f m vertical (%zu epochs)\n", da.horizontal,
              da.vertical, da.epochs);
  EXPECT_EQ(all_live.finite_after(0), all_live.solutions.size());
  EXPECT_GE(da.epochs, 30u);
  EXPECT_LT(da.horizontal, 0.5);
  EXPECT_LT(da.vertical, 0.5);

  // And one IMU message stamped an hour ahead, 50 s into the log. The inertial refuses it and the filter carries
  // on: asked at the end, it answers within centimetres of the same run without that message. (The periodic
  // solutions do stop there: the mediator schedules them on message time and takes the stamp at face value.
  // That is the controller's to fix, not the inertial's.)
  const auto stamped = run_live(inr::ImuRobustness::live_sensor(), everything(true));
  EXPECT_FALSE(stamped.threw);
  EXPECT_FALSE(stamped.error_logged);
  ASSERT_TRUE(all_live.final_on_demand && stamped.final_on_demand);
  ASSERT_TRUE(stamped.final_on_demand->finite());
  EXPECT_EQ(stamped.final_on_demand->t, all_live.final_on_demand->t);
  const auto ds = difference({*all_live.final_on_demand}, {*stamped.final_on_demand}, 0);
  std::printf("after a sample stamped 1 h ahead: final solution %.3f m horizontal / %.3f m vertical from the run "
              "without it\n",
              ds.horizontal, ds.vertical);
  EXPECT_EQ(ds.epochs, 1u);
  EXPECT_LT(ds.horizontal, 0.1);
  EXPECT_LT(ds.vertical, 0.1);
}

TEST(CApi, PushAndPoll) {
  const std::string cfg = src("configs/pos_ins.json");
  EXPECT_EQ(cobra_filter_create("/nonexistent.json", nullptr), nullptr);
  EXPECT_NE(std::string(cobra_last_error()).find("cannot open"), std::string::npos);
  cobra_filter* f = cobra_filter_create(cfg.c_str(), "{\"logging_level\": \"WARN\"}");
  ASSERT_NE(f, nullptr) << cobra_last_error();
  EXPECT_NE(std::string(cobra_version()), "0.0.0");
  EXPECT_EQ(std::string(cobra_version()).find_first_not_of("0123456789."), std::string::npos);
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

// ---------------------------------------------------------------------------------------------------------------
// Measurements that arrive out of time order (StandardOrchestrationConfig::measurement_reorder_s)

namespace {
/// pos_ins plus lateral / vertical zero-velocity constraints generated from the IMU clock every 0.1 s: a second
/// measurement stream that is always ahead of the GNSS fixes, which carry a time bias of 150 ms in this log.
cobra::AppConfig with_fast_constraints(double reorder_s) {
  std::ifstream in(src("configs/pos_ins.json"));
  cobra::jsoncfg::json j;
  in >> j;
  j["app"]["preprocessors"] = {"standard", "advanced"};
  j["app"]["logging_level"] = "WARN";
  for (auto& c : j["configs"]) {
    if (c["type"] != "StandardOrchestrationConfig") continue;
    c["preprocessor_configs"].push_back({{"type", "ZeroVelocity2dGeneratorConfig"},
                                         {"group", "config/constraints"},
                                         {"channels", {"/sensor/vn-100/imu"}},
                                         {"trigger_dt_sec", 0.1},
                                         {"lateral_vel_sigma", 0.5},
                                         {"vertical_vel_sigma", 0.5},
                                         {"output_channel", "/generated/zero/velocity2d"}});
    c["mp_configs"].push_back({{"type", "PinsonBodyVelocityMPConfig"},
                               {"group", "config/constraints_mp"},
                               {"label", "constraints"},
                               {"channel", "/generated/zero/velocity2d"},
                               {"lever_arm", {0.0, 0.0, 0.0}},
                               {"orientation", {1.0, 0.0, 0.0, 0.0}},
                               {"state_block_labels", {"pinson15"}},
                               {"aux_channels", {"INERTIAL_PVA", "INERTIAL_FORCES_AND_RATES"}}});
    for (auto& mp : c["mp_configs"])
      if (mp["label"] == "pos") mp["innovation_gate_probability"] = 0.999999;  // only to count what reaches the filter
    if (reorder_s > 0) c["measurement_reorder_s"] = reorder_s;
  }
  return cobra::jsoncfg::app_config_from_json(j);
}
struct Fed {
  int fixes = 0;       ///< position fixes in the log after the filter aligned
  int reached = 0;     ///< of those, how many the fusion engine saw (accepted or gated)
  int warnings = 0;    ///< WARN lines about discarded measurements
  bool error = false;
};
Fed feed_with_fast_constraints(double reorder_s) {
  Fed out;
  cobra::utils::set_log_sink([&](api::LoggingLevel level, const std::string&, const std::string& m) {
    if (level == api::LoggingLevel::WARN && m.find("older than the filter") != std::string::npos) ++out.warnings;
  });
  cobra::app::RunOptions o;
  o.progress = false;
  {
    cobra::Filter f(with_fast_constraints(reorder_s), o);
    bool aligned = false;
    f.set_solution_callback([&](const api::Message&) { aligned = true; });
    cobra::lcm::LcmLogReader r(src("testdata/example_60s.log"));
    while (auto ev = r.next()) {
      if (ev->channel != "/sensor/vn-100/imu" && ev->channel != "/sensor/ublox-ZED-F9T/position") continue;
      if (aligned && ev->channel == "/sensor/ublox-ZED-F9T/position") ++out.fixes;
      f.push(cobra::lcm::decode(ev->data), ev->channel);
    }
    if (f.registry().has_group("fusion/gating")) {
      auto kv = f.registry().batch("fusion/gating");
      out.reached = static_cast<int>(kv->template get_value<std::int64_t>("pos_accepted").value_or(0) +
                                     kv->template get_value<std::int64_t>("pos_rejected").value_or(0));
    }
    out.error = f.stop() != 0;
  }
  cobra::utils::set_log_sink(nullptr);
  return out;
}
}  // namespace

TEST(Filter, FastAidingStreamStarvesADelayedReceiverByDefault) {
  // Cobra's behaviour, kept as the default: a measurement older than the filter is dropped, and the filter is
  // wherever the last measurement took it. With a second stream at 10 Hz almost every GNSS fix arrives "old".
  const Fed fed = feed_with_fast_constraints(0.0);
  ASSERT_GE(fed.fixes, 40);
  EXPECT_LT(fed.reached, fed.fixes / 4);
  EXPECT_GE(fed.warnings, 1);  // ... but no longer in silence (it was a DEBUG line)
}

TEST(Filter, ReorderWindowKeepsEveryFixOfADelayedReceiver) {
  // The window has to cover how late the slowest stream is against the fastest. The constraints here are made
  // from IMU messages, which the controller passes on at once, while the fixes go through its 2 s buffer and
  // carry a 150 ms time bias on top: they are 2.15 s late. A window of 0.5 s does not cover that, 2.5 s does.
  const Fed too_short = feed_with_fast_constraints(0.5);
  EXPECT_LT(too_short.reached, too_short.fixes / 4);
  EXPECT_GE(too_short.warnings, 1);
  const Fed fed = feed_with_fast_constraints(2.5);
  ASSERT_GE(fed.fixes, 40);
  EXPECT_FALSE(fed.error);
  EXPECT_GE(fed.reached, fed.fixes - 4);  // all but what is still waiting in the window when the log ends
  EXPECT_EQ(fed.warnings, 0);
}

namespace {
/// pos_ins plus the receiver's velocity as a second measurement. Both streams pass the controller's 2 s buffer;
/// the velocity carries the larger time bias (250 ms against 150 ms), so each velocity arrives after the position
/// of the same epoch and is 100 ms older.
struct TwoStreams {
  int fixes = 0, velocity_reached = 0, discard_warnings = 0, window_warnings = 0;
};
TwoStreams feed_position_and_velocity(double reorder_s) {
  std::ifstream in(src("configs/pos_ins.json"));
  cobra::jsoncfg::json j;
  in >> j;
  j["app"]["logging_level"] = "WARN";
  for (auto& c : j["configs"]) {
    if (c["type"] == "LcmLogTransportConfig" && c.contains("channels_to_process")) c["channels_to_process"].push_back("/sensor/ublox-ZED-F9T/velocity");
    if (c["type"] != "StandardOrchestrationConfig") continue;
    c["preprocessor_configs"].push_back({{"type", "TimeBiasConfig"},
                                         {"group", "config/time_bias_velocity"},
                                         {"channels", {"/sensor/ublox-ZED-F9T/velocity"}},
                                         {"time_bias", 250000000}});
    c["mp_configs"].push_back({{"type", "PinsonVelocityMPConfig"},
                               {"group", "config/vel_measurement_processor"},
                               {"label", "vel"},
                               {"channel", "/sensor/ublox-ZED-F9T/velocity"},
                               {"state_block_labels", {"pinson15"}},
                               {"aux_channels", {"INERTIAL_PVA"}},
                               {"innovation_gate_probability", 0.999999}});  // only to count what reaches the filter
    if (reorder_s > 0) c["measurement_reorder_s"] = reorder_s;
  }
  TwoStreams out;
  cobra::utils::set_log_sink([&](api::LoggingLevel level, const std::string&, const std::string& m) {
    if (level != api::LoggingLevel::WARN) return;
    if (m.find("older than the filter") != std::string::npos) ++out.discard_warnings;
    if (m.find("controller's buffer") != std::string::npos) ++out.window_warnings;
  });
  cobra::app::RunOptions o;
  o.progress = false;
  {
    cobra::Filter f(cobra::jsoncfg::app_config_from_json(j), o);
    bool aligned = false;
    f.set_solution_callback([&](const api::Message&) { aligned = true; });
    cobra::lcm::LcmLogReader r(src("testdata/example_60s.log"));
    while (auto ev = r.next()) {
      if (ev->channel != "/sensor/vn-100/imu" && ev->channel != "/sensor/ublox-ZED-F9T/position" && ev->channel != "/sensor/ublox-ZED-F9T/velocity")
        continue;
      if (aligned && ev->channel == "/sensor/ublox-ZED-F9T/velocity") ++out.fixes;
      f.push(cobra::lcm::decode(ev->data), ev->channel);
    }
    if (f.registry().has_group("fusion/gating")) {
      auto kv = f.registry().batch("fusion/gating");
      out.velocity_reached = static_cast<int>(kv->template get_value<std::int64_t>("vel_accepted").value_or(0) +
                                              kv->template get_value<std::int64_t>("vel_rejected").value_or(0));
    }
    f.stop();
  }
  cobra::utils::set_log_sink(nullptr);
  return out;
}
}  // namespace

TEST(Filter, ReorderWindowMustExceedTheControllerBuffer) {
  // The window counts back from the IMU's time, and a measurement that passed the controller's buffer is already
  // 2 s behind the IMU when it arrives. A window of 0.5 s therefore releases it at once and orders nothing: the
  // velocity, 100 ms older than the position before it, is dropped every time. That is said at start-up (v0.2.10
  // recommended 0.5 s for exactly this case). 2.5 s, the buffer and the largest time bias, orders them.
  const TwoStreams too_short = feed_position_and_velocity(0.5);
  ASSERT_GE(too_short.fixes, 40);
  EXPECT_LT(too_short.velocity_reached, too_short.fixes / 4);
  EXPECT_GE(too_short.discard_warnings, 1);
  EXPECT_EQ(too_short.window_warnings, 1);
  const TwoStreams ok = feed_position_and_velocity(2.5);
  ASSERT_GE(ok.fixes, 40);
  EXPECT_GE(ok.velocity_reached, ok.fixes - 4);
  EXPECT_EQ(ok.discard_warnings, 0);
  EXPECT_EQ(ok.window_warnings, 0);
}

// ---------------------------------------------------------------------------------------------------------------
// Standing still: zero-velocity and zero-rate updates (ZeroMotionGeneratorConfig, PinsonZeroRateMPConfig)

namespace {
struct Standing {
  int solutions = 0;
  double heading_drift_deg = 0;   ///< heading error at the end minus at the first solution, against the log's reference
  double speed_max = 0;           ///< largest horizontal speed of the solution from 10 s on, m/s (the car stands)
  int rate_updates = 0, velocity_updates = 0, errors = 0, discards = 0;
  double velocity_sigma = 0;      ///< horizontal velocity sigma of the last solution, m/s
};
/// What the receiver's velocity is to the zero-motion generator.
enum class Witness {
  NONE,       ///< not looked at: the IMU decides alone (v0.2.12)
  RECEIVER,   ///< the log's receiver velocity confirms
  CREEPING,   ///< the receiver's velocity with 0.3 m/s added: a car that rolls, as quiet to the IMU as this one
};
/// The 60 s log (the car stands, engine running) with the filter started from the reference solution and no
/// knowledge of the gyro bias: nothing in the position fixes of a standing vehicle shows the bias about the
/// vertical, 0.16 deg/s on this IMU.
Standing stand_for_a_minute(bool zero_motion, Witness witness = Witness::NONE) {
  std::ifstream in(src("configs/pos_ins.json"));
  cobra::jsoncfg::json j;
  in >> j;
  const char* kTruth = "/sensor/ins-d/pva";
  const char* kImu = "/sensor/vn-100/imu";
  const char* kVelocity = "/sensor/ublox-ZED-F9T/velocity";
  // Started from the reference solution at the first IMU sample (the "manual" initialization, which leaves no
  // gap between the initial solution and the IMU), with the gyro bias unknown to 3 mrad/s.
  std::int64_t first_imu_ns = 0;
  std::shared_ptr<cobra::utils::PVA> start;
  {
    cobra::lcm::LcmLogReader r(src("testdata/example_60s.log"));
    while (auto ev = r.next()) {
      if (ev->channel == kImu && !first_imu_ns)
        first_imu_ns = cobra::utils::time_of_validity(*cobra::lcm::decode(ev->data)).value_or(api::Timestamp{0}).elapsed_nsec;
      if (ev->channel == kTruth && first_imu_ns && !start) start = std::dynamic_pointer_cast<cobra::utils::PVA>(cobra::lcm::decode(ev->data));
      if (start) break;
    }
  }
  if (!start) throw std::runtime_error("no reference PVA in the test log");
  const api::Vector3 rpy = cobra::nav::quat_to_rpy(api::Vector(start->get_quaternion()));
  j["app"]["initialization"] = "manual";
  j["app"]["logging_level"] = "WARN";
  if (zero_motion) j["app"]["preprocessors"] = {"standard", "advanced"};
  for (auto& c : j["configs"]) {
    if (c["type"] != "StandardOrchestrationConfig") continue;
    c["alignment_config"] = {{"type", "ManualAlignmentConfig"},
                             {"group", "config/default/alignment"},
                             {"initial_pos", {start->get_p1(), start->get_p2(), start->get_p3()}},
                             {"initial_vel", {0.0, 0.0, 0.0}},
                             {"initial_rpy", {rpy(0), rpy(1), rpy(2)}},
                             {"initial_accel_bias", {0.0, 0.0, 0.0}},
                             {"initial_gyro_bias", {0.0, 0.0, 0.0}},
                             {"initial_time", (first_imu_ns - 10'000'000) * 1e-9},
                             {"initial_pos_var", {1.0, 1.0, 1.0}},
                             {"initial_vel_var", {0.01, 0.01, 0.01}},
                             {"initial_tilt_var", {1e-4, 1e-4, 4e-4}},
                             {"initial_accel_bias_var", {0.072 * 0.072, 0.072 * 0.072, 0.072 * 0.072}},
                             {"initial_gyro_bias_var", {9e-6, 9e-6, 9e-6}}};
    if (!zero_motion) continue;
    c["measurement_reorder_s"] = 2.5;
    cobra::jsoncfg::json generator = {{"type", "ZeroMotionGeneratorConfig"},
                                      {"group", "config/zero_motion"},
                                      {"channels", {kImu}},
                                      {"velocity_channel", "/generated/zero_velocity"},
                                      {"rate_channel", "/generated/zero_rate"}};
    if (witness != Witness::NONE) {
      generator["channels"] = {kImu, kVelocity};
      generator["confirmation_channels"] = {kVelocity};
    }
    c["preprocessor_configs"].push_back(generator);
    c["mp_configs"].push_back({{"type", "PinsonVelocityMPConfig"},
                               {"group", "config/zero_velocity_processor"},
                               {"label", "zupt"},
                               {"channel", "/generated/zero_velocity"},
                               {"state_block_labels", {"pinson15"}},
                               {"aux_channels", {"INERTIAL_PVA"}},
                               {"innovation_gate_probability", 0.999999}});
    c["mp_configs"].push_back({{"type", "PinsonZeroRateMPConfig"},
                               {"group", "config/zero_rate_processor"},
                               {"label", "zaru"},
                               {"channel", "/generated/zero_rate"},
                               {"state_block_labels", {"pinson15"}},
                               {"innovation_gate_probability", 0.999999}});
  }
  Standing out;
  cobra::utils::set_log_sink([&](api::LoggingLevel level, const std::string&, const std::string& m) {
    if (level == api::LoggingLevel::ERROR) ++out.errors;
    if (m.find("older than the filter") != std::string::npos) ++out.discards;
  });
  std::vector<std::pair<std::int64_t, double>> reference;  // time, heading (rad)
  std::vector<std::pair<std::int64_t, double>> heading;
  auto yaw = [](const cobra::utils::PVA& p) { return cobra::nav::quat_to_rpy(api::Vector(p.get_quaternion()))(2); };
  cobra::app::RunOptions o;
  o.progress = false;
  {
    cobra::Filter f(cobra::jsoncfg::app_config_from_json(j), o);
    f.set_solution_callback([&](const api::Message& m) {
      if (auto p = m.as<cobra::utils::PVA>()) {
        const std::int64_t t = p->get_time_of_validity().get_elapsed_nsec();
        heading.emplace_back(t, yaw(*p));
        if (t > first_imu_ns + 10'000'000'000) out.speed_max = std::max(out.speed_max, std::hypot(p->get_v1(), p->get_v2()));
        const auto cov = p->get_covariance();
        if (cov.rows() >= 5 && cov.cols() >= 5) out.velocity_sigma = std::sqrt(cov(3, 3) + cov(4, 4));
      }
    });
    cobra::lcm::LcmLogReader r(src("testdata/example_60s.log"));
    while (auto ev = r.next()) {
      const bool velocity = witness != Witness::NONE && ev->channel == kVelocity;
      if (ev->channel != kImu && ev->channel != "/sensor/ublox-ZED-F9T/position" && ev->channel != kTruth && !velocity) continue;
      auto msg = cobra::lcm::decode(ev->data);
      if (velocity && witness == Witness::CREEPING)
        if (auto v = std::dynamic_pointer_cast<aspn23_eigen::MeasurementVelocity>(msg)) v->set_x(v->get_x() + 0.3);
      if (ev->channel == kTruth)
        if (auto p = std::dynamic_pointer_cast<cobra::utils::PVA>(msg)) reference.emplace_back(p->get_time_of_validity().get_elapsed_nsec(), yaw(*p));
      f.push(msg, ev->channel);
    }
    if (f.registry().has_group("fusion/gating")) {
      auto kv = f.registry().batch("fusion/gating");
      out.rate_updates = static_cast<int>(kv->template get_value<std::int64_t>("zaru_accepted").value_or(0));
      out.velocity_updates = static_cast<int>(kv->template get_value<std::int64_t>("zupt_accepted").value_or(0));
    }
    f.stop();
  }
  cobra::utils::set_log_sink(nullptr);
  out.solutions = static_cast<int>(heading.size());
  auto error_at = [&](const std::pair<std::int64_t, double>& h) {
    auto it = std::lower_bound(reference.begin(), reference.end(), h.first, [](const auto& a, std::int64_t t) { return a.first < t; });
    if (it == reference.end()) --it;
    return cobra::nav::wrap_to_pi(h.second - it->second);
  };
  if (heading.size() >= 2) out.heading_drift_deg = (error_at(heading.back()) - error_at(heading.front())) * 180.0 / 3.14159265358979323846;
  return out;
}
}  // namespace

TEST(Filter, StandingStillTheHeadingDriftsWithTheGyroBias) {
  // The default, pinned: with position fixes alone a standing vehicle's heading runs away with the gyro bias
  // about the vertical (0.16 deg/s here), which nothing but a manoeuvre would show.
  const Standing s = stand_for_a_minute(false);
  ASSERT_GE(s.solutions, 40);
  EXPECT_EQ(s.errors, 0);
  EXPECT_GT(std::abs(s.heading_drift_deg), 5.0) << s.heading_drift_deg;
  std::cout << "standing for a minute, position fixes only: heading drift " << s.heading_drift_deg << " deg, speed up to " << s.speed_max << " m/s\n";
}

TEST(Filter, ZeroRateAndZeroVelocityUpdatesHoldAStandingVehicle) {
  // The generator sees that the IMU is quiet (the idling engine notwithstanding) and says once a second: this
  // vehicle neither moves nor turns. The zero rate gives the filter the gyro bias, the heading stays.
  const Standing s = stand_for_a_minute(true);
  ASSERT_GE(s.solutions, 40);
  EXPECT_EQ(s.errors, 0);
  EXPECT_EQ(s.discards, 0);
  EXPECT_GE(s.rate_updates, 40);
  EXPECT_GE(s.velocity_updates, 40);
  EXPECT_LT(std::abs(s.heading_drift_deg), 0.3) << s.heading_drift_deg;
  EXPECT_LT(s.speed_max, 0.05);  // from 10 s on: the first update needs the 2 s window and the 2.5 s reorder window
  std::cout << "with zero-rate and zero-velocity updates: heading drift " << s.heading_drift_deg << " deg, speed up to " << s.speed_max
            << " m/s, " << s.rate_updates << " rate and " << s.velocity_updates << " velocity updates\n";
}

// ---------------------------------------------------------------------------------------------------------------
// The initial solution is older than the IMU (StandardOrchestrationConfig::initialization_imu_replay_s)

namespace {
struct StartUp {
  int solutions = 0;
  double fall = 0;       ///< largest vertical speed of the solution in its first 10 s, m/s (the car stands)
  double altitude = 0;   ///< largest altitude error against the reference in those 10 s, m
  double speed = 0;      ///< largest horizontal speed in those 10 s, m/s
  int errors = 0;
  int replayed = 0;      ///< IMU messages the orchestration says it gave to the inertial after the initialization
};
/// The 60 s log (the car stands) with the filter started from the first message of the reference solution
/// ("pva_message"). That PVA passes the controller's buffer like any measurement and reaches the orchestration
/// two seconds behind the IMU.
StartUp start_from_a_pva_message(double replay_s, const char* gap_policy = nullptr) {
  std::ifstream in(src("configs/pos_ins.json"));
  cobra::jsoncfg::json j;
  in >> j;
  const char* kTruth = "/sensor/ins-d/pva";
  const char* kImu = "/sensor/vn-100/imu";
  j["app"]["initialization"] = "pva_message";
  j["app"]["logging_level"] = "INFO";
  for (auto& c : j["configs"]) {
    if (c["type"] != "StandardOrchestrationConfig") continue;
    c["alignment_channels"] = {kTruth};
    c["alignment_config"] = {{"type", "PvaMessageInitializationConfig"},
                             {"group", "config/default/alignment"},
                             {"initial_pva_channel", kTruth},
                             {"initial_accel_bias_sigma", {0.072, 0.072, 0.072}},
                             {"initial_gyro_bias_sigma", {3e-3, 3e-3, 3e-3}},
                             {"initial_pva_sigma", {1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.02}}};
    if (replay_s > 0) c["initialization_imu_replay_s"] = replay_s;
    if (gap_policy) c["inertial_config"]["imu_gap_policy"] = gap_policy;
  }
  StartUp out;
  cobra::utils::set_log_sink([&](api::LoggingLevel level, const std::string&, const std::string& m) {
    if (level == api::LoggingLevel::ERROR) ++out.errors;
    const auto at = m.find(" IMU message(s) kept since then were given to the inertial");
    if (at != std::string::npos) out.replayed = std::atoi(m.substr(m.rfind(' ', at - 1) + 1).c_str());
  });
  std::vector<std::pair<std::int64_t, double>> reference;  // time, altitude
  std::vector<std::array<double, 4>> solution;             // time (s), altitude, down speed, horizontal speed
  cobra::app::RunOptions o;
  o.progress = false;
  {
    cobra::Filter f(cobra::jsoncfg::app_config_from_json(j), o);
    f.set_solution_callback([&](const api::Message& m) {
      if (auto p = m.as<cobra::utils::PVA>())
        solution.push_back({p->get_time_of_validity().get_elapsed_nsec() * 1e-9, p->get_p3(), p->get_v3(), std::hypot(p->get_v1(), p->get_v2())});
    });
    cobra::lcm::LcmLogReader r(src("testdata/example_60s.log"));
    while (auto ev = r.next()) {
      if (ev->channel != kImu && ev->channel != "/sensor/ublox-ZED-F9T/position" && ev->channel != kTruth) continue;
      auto msg = cobra::lcm::decode(ev->data);
      if (ev->channel == kTruth)
        if (auto p = std::dynamic_pointer_cast<cobra::utils::PVA>(msg)) reference.emplace_back(p->get_time_of_validity().get_elapsed_nsec(), p->get_p3());
      f.push(msg, ev->channel);
    }
    f.stop();
  }
  cobra::utils::set_log_sink(nullptr);
  out.solutions = static_cast<int>(solution.size());
  for (const auto& row : solution) {
    if (row[0] > solution.front()[0] + 10.0) break;
    auto it = std::lower_bound(reference.begin(), reference.end(), static_cast<std::int64_t>(row[0] * 1e9),
                               [](const auto& a, std::int64_t t) { return a.first < t; });
    if (it == reference.end()) --it;
    out.fall = std::max(out.fall, std::abs(row[2]));
    out.altitude = std::max(out.altitude, std::abs(row[1] - it->second));
    out.speed = std::max(out.speed, row[3]);
  }
  return out;
}
}  // namespace

TEST(Filter, AnInitialSolutionOlderThanTheImuLeavesAHoleByDefault) {
  // Cobra's behaviour, kept as the default: IMU messages that arrive before the initialization has a solution
  // are dropped. The solution of "pva_message" is two seconds old when it arrives; the first IMU sample after it
  // is mechanized across those two seconds, as free fall.
  const StartUp s = start_from_a_pva_message(0.0);
  ASSERT_GE(s.solutions, 40);
  EXPECT_GT(s.fall, 10.0) << s.fall;
  EXPECT_GT(s.altitude, 5.0) << s.altitude;
  EXPECT_EQ(s.replayed, 0);
  std::cout << "started from a PVA message, default: the solution falls at up to " << s.fall << " m/s, altitude error up to " << s.altitude
            << " m, horizontal speed up to " << s.speed << " m/s\n";
  const StartUp h = start_from_a_pva_message(0.0, "hold");
  std::cout << "with imu_gap_policy hold: " << h.fall << " m/s, " << h.altitude << " m, " << h.speed << " m/s\n";
}

TEST(Filter, ImuKeptBeforeTheInitializationClosesTheHole) {
  // With initialization_imu_replay_s the orchestration keeps the IMU messages of that many seconds while the
  // initialization has no solution, and gives those after the solution's time to the new inertial.
  const StartUp s = start_from_a_pva_message(2.5);
  ASSERT_GE(s.solutions, 40);
  EXPECT_EQ(s.errors, 0);
  EXPECT_GE(s.replayed, 195);  // two seconds at 100 Hz, give or take the samples around the PVA
  EXPECT_LE(s.replayed, 215);
  EXPECT_LT(s.fall, 0.5) << s.fall;        // what an unknown accelerometer bias of 60 mm/s^2 does before it is found
  EXPECT_LT(s.altitude, 2.0) << s.altitude;
  EXPECT_LT(s.speed, 0.5) << s.speed;
  std::cout << "with the IMU replayed (" << s.replayed << " messages): " << s.fall << " m/s, altitude error up to " << s.altitude << " m, horizontal speed up to " << s.speed << " m/s\n";
}

TEST(Filter, AStandstillConfirmedByTheReceiverCarriesMoreWeight) {
  // The receiver's velocity as a second witness (DESIGN.md 7.28). Its messages pass the controller's buffer and
  // are 2 s old when the generator sees them; the zero-motion messages made for their epochs are that old too and
  // still have to reach the filter in order: nothing may be discarded as older than the filter.
  const Standing alone = stand_for_a_minute(true);
  const Standing s = stand_for_a_minute(true, Witness::RECEIVER);
  ASSERT_GE(s.solutions, 40);
  EXPECT_EQ(s.errors, 0);
  EXPECT_EQ(s.discards, 0);
  EXPECT_GE(s.rate_updates, 40);
  EXPECT_GE(s.velocity_updates, 40);
  EXPECT_LT(std::abs(s.heading_drift_deg), 0.3) << s.heading_drift_deg;
  EXPECT_LT(s.speed_max, 0.05);
  // 0.05 m/s where the IMU alone is trusted with 0.2. The solution is 2.5 s ahead of the last update and its
  // velocity sigma is mostly what the tilt uncertainty adds in that time; the tilt is what the firmer zero velocity
  // pins down (0.13 against 0.19 m/s).
  ASSERT_GT(alone.velocity_sigma, 0.0);
  EXPECT_LT(s.velocity_sigma, 0.8 * alone.velocity_sigma) << s.velocity_sigma << " against " << alone.velocity_sigma;
  std::cout << "standstill confirmed by the receiver: heading drift " << s.heading_drift_deg << " deg, speed up to " << s.speed_max << " m/s, "
            << s.rate_updates << " rate and " << s.velocity_updates << " velocity updates, velocity sigma " << s.velocity_sigma
            << " m/s (the IMU alone: " << alone.velocity_sigma << ")\n";
}

TEST(Filter, AReceiverThatShowsMotionSilencesTheZeroMotionUpdates) {
  // The same minute with 0.3 m/s added to the receiver's velocity: to the IMU nothing has changed, and the IMU
  // alone goes on saying "stands" (ZeroMotionTest.TheImuAloneCallsACreepingVehicleStanding). With the receiver as
  // a witness the filter is told nothing: only what the IMU said alone before the receiver was first heard gets
  // through.
  const Standing s = stand_for_a_minute(true, Witness::CREEPING);
  ASSERT_GE(s.solutions, 40);
  EXPECT_EQ(s.errors, 0);
  EXPECT_LE(s.velocity_updates, 3);
  EXPECT_LE(s.rate_updates, 3);
  std::cout << "receiver shows 0.3 m/s: " << s.velocity_updates << " velocity and " << s.rate_updates << " rate updates\n";
}
