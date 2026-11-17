// The receiver's time offsets as filter states (DESIGN.md §7.30), through the whole filter on simulated drives:
// a receiver whose fixes or whose velocity are older than their time stamps, a filter that is not told.
//
// The drives are simulated because the question is one of a fraction of a second, and on the recorded drive the
// reference solution has a time base of its own.
#include <pntos/cobra/app/AppBuilder.hpp>
#include <pntos/cobra/app/Filter.hpp>
#include <pntos/cobra/config/JsonConfig.hpp>
#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/logging.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include "sim_drive.hpp"
#include "test_support.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>

using namespace pntos;
using namespace pntos::test;
using namespace pntos::test::sim;
using api::Vector3;
namespace nav = cobra::nav;

namespace {

/// What the filter is given besides the fixes, and whether it may estimate how old things are.
struct Given {
  bool position_offset_state = false;  ///< the fixes through pinson_position_time_offset
  bool velocity = false;               ///< the receiver's velocity as a measurement
  bool velocity_offset_state = false;  ///< ... through pinson_velocity_time_offset
};

struct Result {
  int solutions = 0, errors = 0;
  double along_mean = 0;      ///< m; mean of the position error along the direction of travel (negative: behind)
  double along_rms = 0, across_rms = 0;  ///< m; about zero
  double horizontal_rms = 0;  ///< m
  double velocity_rms = 0;    ///< m/s, horizontal
  double heading_rms = 0;     ///< rad
  std::vector<std::string> reports;                ///< the INFO lines of the processors, in order
  std::optional<double> position_offset, velocity_offset;  ///< s; the last value each of them reported
};

/// A drive that never stops giving the filter something to see: the speed swings, the road winds.
Drive winding() {
  Drive d;
  d.speed = [](double t) { return 10.0 * smoothstep(t, 6.0, 14.0) + 2.5 * std::sin(0.3 * t) * smoothstep(t, 14.0, 18.0); };
  d.course = [](double t) { return 0.9 + 0.6 * smoothstep(t, 9.0, 16.0) + 0.5 * std::sin(0.2 * t) * smoothstep(t, 16.0, 20.0); };
  d.latency = 0.0;  // the controller's buffer does the waiting
  return d;
}

cobra::jsoncfg::json offset_block(const char* label) {
  return {{"type", "FogmStateBlockConfig"},
          {"group", std::string("config/") + label + "_block"},
          {"label", label},
          {"estimate_with_covariance", {{"type", "EWC_GENERIC"}, {"estimate", {0.0}}, {"covariance_diag", {0.09}}}},
          {"fogm_model", {{"group", std::string("config/") + label}, {"sigma", {0.3}}, {"tau", {1.0e6}}}}};
}

/// configs/pos_ins_align_in_motion.json without the preprocessors and the lever arm of the recording, run on the
/// drive through the push API; errors over the solutions from `from` s on.
Result run(const Drive& d, const Given& setup, double duration = 180.0, double from = 80.0) {
  const char* dir = std::getenv("PNTOS_TEST_SRCDIR");
  std::ifstream in(std::string(dir ? dir : "..") + "/configs/pos_ins_align_in_motion.json");
  cobra::jsoncfg::json j;
  in >> j;
  j["app"]["logging_level"] = "INFO";
  for (auto& c : j["configs"]) {
    if (c["type"] != "StandardOrchestrationConfig") continue;
    c["preprocessor_configs"] = cobra::jsoncfg::json::array();
    c["alignment_config"]["lever_arm"] = {0.0, 0.0, 0.0};
    for (auto& mp : c["mp_configs"]) {
      mp["lever_arm"] = {0.0, 0.0, 0.0};
      if (setup.position_offset_state && mp["label"] == "pos") {
        mp["type"] = "PinsonPositionTimeOffsetMPConfig";
        mp["state_block_labels"] = {"pinson15", "pos_sensor_error", "pos_time_offset"};
      }
    }
    if (setup.position_offset_state) c["additional_sb_configs"].push_back(offset_block("pos_time_offset"));
    if (setup.velocity) {
      cobra::jsoncfg::json mp = {{"type", setup.velocity_offset_state ? "PinsonVelocityTimeOffsetMPConfig" : "PinsonVelocityMPConfig"},
                                 {"group", "config/vel_measurement_processor"},
                                 {"label", "vel"},
                                 {"channel", kVelocityChannel}};
      if (setup.velocity_offset_state) {
        mp["state_block_labels"] = {"pinson15", "vel_time_offset"};
        c["additional_sb_configs"].push_back(offset_block("vel_time_offset"));
      } else {
        mp["state_block_labels"] = {"pinson15"};
        mp["aux_channels"] = {"INERTIAL_PVA"};
      }
      c["mp_configs"].push_back(mp);
    }
  }

  Result out;
  const std::regex report(R"re("(pos|vel)": the measurements are ([0-9.]+) s (older|newer) than their time stamps)re");
  cobra::utils::set_log_sink([&](api::LoggingLevel level, const std::string&, const std::string& m) {
    if (level == api::LoggingLevel::ERROR) ++out.errors;
    std::smatch s;
    if (std::regex_search(m, s, report)) {
      out.reports.push_back(m);
      const double v = (s[3] == "older" ? 1.0 : -1.0) * std::stod(s[2]);
      (s[1] == "pos" ? out.position_offset : out.velocity_offset) = v;
    }
  });
  double along = 0, along2 = 0, across2 = 0, h2 = 0, v2 = 0, yaw2 = 0;
  int n = 0;
  {
    cobra::app::RunOptions o;
    o.progress = false;
    cobra::Filter f(cobra::jsoncfg::app_config_from_json(j), o);
    std::vector<api::Message> solutions;
    f.set_solution_callback([&](const api::Message& m) { solutions.push_back(m); });
    const Track track = simulate(d, duration, [&](const std::shared_ptr<api::AspnBase>& message, const char* channel, double) {
      f.push(message, channel);
      return true;
    });
    f.stop();
    for (const auto& m : solutions) {
      const auto pva = m.as<cobra::utils::PVA>();
      if (!pva) continue;
      ++out.solutions;
      const double t = (pva->get_time_of_validity().get_elapsed_nsec() - kT0) * 1e-9;
      if (t < from) continue;
      const Vector3 p(nav::delta_lat_to_north(pva->get_p1() - d.lat, d.lat, d.alt), nav::delta_lon_to_east(pva->get_p2() - d.lon, d.lat, d.alt),
                      -(pva->get_p3() - d.alt));
      const Vector3 e = p - position_at(track, t);
      const Vector3 v = d.velocity(t);
      const double al = e.dot(v) / std::max(v.norm(), 1e-9);
      along += al;
      along2 += al * al;
      across2 += std::max(e.head<2>().squaredNorm() - al * al, 0.0);
      h2 += e.head<2>().squaredNorm();
      v2 += (Vector3(pva->get_v1(), pva->get_v2(), pva->get_v3()) - v).head<2>().squaredNorm();
      const double yaw = attitude_error(nav::quat_to_dcm(api::Vector(pva->get_quaternion())), d.attitude(t))(2);
      yaw2 += yaw * yaw;
      ++n;
    }
  }
  cobra::utils::set_log_sink(nullptr);
  if (n) {
    out.along_mean = along / n;
    out.along_rms = std::sqrt(along2 / n);
    out.across_rms = std::sqrt(across2 / n);
    out.horizontal_rms = std::sqrt(h2 / n);
    out.velocity_rms = std::sqrt(v2 / n);
    out.heading_rms = std::sqrt(yaw2 / n);
  }
  return out;
}

std::string describe(const Result& r) {
  std::ostringstream os;
  os << r.solutions << " solutions, " << r.errors << " errors; along track " << r.along_mean << " m (mean), " << r.along_rms << " rms, across "
     << r.across_rms << " rms, horizontal " << r.horizontal_rms << " m rms, velocity " << r.velocity_rms << " m/s rms, heading " << r.heading_rms / kDeg << " deg rms";
  if (r.position_offset) os << "; fixes found " << *r.position_offset << " s old";
  if (r.velocity_offset) os << "; velocity found " << *r.velocity_offset << " s old";
  os << "; " << r.reports.size() << " report(s)";
  return os.str();
}

}  // namespace

TEST(TimeOffset, FixesOlderThanStampedPutTheSolutionBehind) {
  // The default, pinned. Fixes that are 0.3 s old and taken as stamped: the solution trails the vehicle by the
  // distance it covers in that time, 3 m at 10 m/s, and nothing in the filter's own figures says so. Its velocity
  // suffers too: a lag that changes with the speed looks like a velocity error.
  Drive d = winding();
  const Result fresh = run(d, {});
  d.position_age = 0.3;
  const Result old = run(d, {});
  ASSERT_GE(fresh.solutions, 100);
  ASSERT_GE(old.solutions, 100);
  EXPECT_EQ(fresh.errors, 0);
  EXPECT_EQ(old.errors, 0);
  EXPECT_LT(std::abs(fresh.along_mean), 0.5) << describe(fresh);
  EXPECT_LT(old.along_mean, -2.5) << describe(old);
  EXPECT_GT(old.along_mean, -3.6) << describe(old);
  EXPECT_GT(old.velocity_rms, 1.5 * fresh.velocity_rms) << describe(old);
  EXPECT_TRUE(old.reports.empty());
  std::cout << "fixes as stamped: " << describe(fresh) << "\nfixes 0.3 s old, taken as stamped: " << describe(old) << "\n";
}

TEST(TimeOffset, TheFilterFindsHowOldTheFixesAre) {
  // The same drive with the offset of the fixes as a state (pinson_position_time_offset, one more state block).
  Drive d = winding();
  d.position_age = 0.3;
  const Result plain = run(d, {});
  Given s;
  s.position_offset_state = true;
  const Result r = run(d, s);
  ASSERT_GE(r.solutions, 100);
  EXPECT_EQ(r.errors, 0);
  ASSERT_TRUE(r.position_offset.has_value()) << describe(r);
  EXPECT_NEAR(*r.position_offset, 0.3, 0.04) << describe(r);
  EXPECT_LE(r.reports.size(), 6u);  // said when it is known and when it has moved, not with every fix
  // The velocity is right as soon as the offset is known. The position follows slowly: what the solution was
  // behind by before then sits in the receiver's bias states, which let go with their own time constant (300 s).
  EXPECT_LT(r.velocity_rms, 0.6 * plain.velocity_rms) << describe(r) << " against " << describe(plain);
  EXPECT_LT(std::abs(r.along_mean), 0.75 * std::abs(plain.along_mean)) << describe(r) << " against " << describe(plain);
  const Result late = run(d, s, 700.0, 600.0), late_plain = run(d, {}, 700.0, 600.0);
  EXPECT_LT(std::abs(late.along_mean), 0.8) << describe(late);
  EXPECT_LT(late_plain.along_mean, -2.5) << describe(late_plain);
  // and when the stamps are right, nothing is invented
  const Result f = run(winding(), s);
  ASSERT_TRUE(f.position_offset.has_value()) << describe(f);
  EXPECT_NEAR(*f.position_offset, 0.0, 0.04) << describe(f);
  EXPECT_LT(std::abs(f.along_mean), 0.5) << describe(f);
  std::cout << "fixes 0.3 s old, offset a state: " << describe(r) << "\n  after ten minutes: " << describe(late)
            << "\n  taken as stamped, after ten minutes: " << describe(late_plain) << "\nfixes as stamped, offset a state: " << describe(f) << "\n";
  for (const auto& line : r.reports) std::cout << "  " << line << "\n";
}

TEST(TimeOffset, WhatTheSolutionWasBehindByLingersInTheBiasStates) {
  // The limit of the position offset, pinned. The car drives with fixes 0.3 s old and then parks. A parked car is
  // where its fixes are, old or not. Parked after 100 s, the filter with the offset state is further off in the
  // minute that follows than the one without: it has found the offset, but the three metres it was behind before
  // are in the bias states of the receiver, which are fixed to north and east and do not vanish with the speed.
  // Parked after ten minutes they have let go, and it is the filter without the state that is worse, because its
  // position has to catch up three metres every time the car stops.
  auto parked_after = [](double park) {
    Drive d = winding();
    d.speed = [park](double t) {
      return (10.0 * smoothstep(t, 6.0, 14.0) + 2.5 * std::sin(0.3 * t) * smoothstep(t, 14.0, 18.0)) * (1.0 - smoothstep(t, park - 10.0, park));
    };
    d.position_age = 0.3;
    return d;
  };
  Given s;
  s.position_offset_state = true;
  const Result early = run(parked_after(100.0), s, 170.0, 110.0), early_plain = run(parked_after(100.0), {}, 170.0, 110.0);
  const Result late = run(parked_after(600.0), s, 670.0, 610.0), late_plain = run(parked_after(600.0), {}, 670.0, 610.0);
  EXPECT_EQ(early.errors + late.errors, 0);
  ASSERT_TRUE(early.position_offset.has_value()) << describe(early);
  EXPECT_NEAR(*early.position_offset, 0.3, 0.05) << describe(early);
  EXPECT_GT(early.horizontal_rms, early_plain.horizontal_rms) << describe(early) << " against " << describe(early_plain);
  EXPECT_LT(early.horizontal_rms, 2.5) << describe(early);
  EXPECT_LT(late.horizontal_rms, 0.6 * late_plain.horizontal_rms) << describe(late) << " against " << describe(late_plain);
  std::cout << "parked after 100 s, the minute after: with the state " << early.horizontal_rms << " m, without " << early_plain.horizontal_rms
            << " m; parked after 600 s: " << late.horizontal_rms << " m, without " << late_plain.horizontal_rms << " m\n";
}

TEST(TimeOffset, AnOldVelocityHurtsAFilterThatUsesIt) {
  // The default, pinned. The receiver's velocity as a filter measurement, 0.25 s old and taken as stamped:
  // every change of velocity reaches the filter a quarter of a second late and is read as an error of the inertial.
  Drive d = winding();
  Given s;
  s.velocity = true;
  const Result fresh = run(d, s);
  d.velocity_age = 0.25;
  const Result old = run(d, s);
  ASSERT_GE(old.solutions, 100);
  EXPECT_EQ(fresh.errors, 0);
  EXPECT_EQ(old.errors, 0);
  EXPECT_GT(old.velocity_rms, 3.0 * fresh.velocity_rms) << describe(old) << " against " << describe(fresh);
  EXPECT_GT(old.heading_rms, 2.0 * fresh.heading_rms) << describe(old) << " against " << describe(fresh);
  std::cout << "velocity as stamped: " << describe(fresh) << "\nvelocity 0.25 s old, taken as stamped: " << describe(old) << "\n";
}

TEST(TimeOffset, TheFilterFindsHowOldTheVelocityIs) {
  Drive d = winding();
  Given plain;
  plain.velocity = true;
  const Result fresh = run(d, plain);
  d.velocity_age = 0.25;
  Given s = plain;
  s.velocity_offset_state = true;
  const Result r = run(d, s);
  ASSERT_GE(r.solutions, 100);
  EXPECT_EQ(r.errors, 0);
  ASSERT_TRUE(r.velocity_offset.has_value()) << describe(r);
  EXPECT_NEAR(*r.velocity_offset, 0.25, 0.03) << describe(r);
  // as good as a velocity that was fresh to begin with: there is no bias state for this one to hide in
  EXPECT_LT(r.velocity_rms, 1.3 * fresh.velocity_rms + 0.01) << describe(r) << " against " << describe(fresh);
  EXPECT_LT(r.heading_rms, 1.3 * fresh.heading_rms + 0.1 * kDeg) << describe(r) << " against " << describe(fresh);
  // fresh, with the state: nothing invented, nothing lost
  const Result f = run(winding(), s);
  ASSERT_TRUE(f.velocity_offset.has_value()) << describe(f);
  EXPECT_NEAR(*f.velocity_offset, 0.0, 0.03) << describe(f);
  EXPECT_LT(f.velocity_rms, 1.2 * fresh.velocity_rms + 0.005) << describe(f);
  // newer than stamped is found as well
  Drive early = winding();
  early.velocity_age = -0.15;
  const Result e = run(early, s);
  ASSERT_TRUE(e.velocity_offset.has_value()) << describe(e);
  EXPECT_NEAR(*e.velocity_offset, -0.15, 0.03) << describe(e);
  std::cout << "velocity 0.25 s old, offset a state: " << describe(r) << "\nvelocity as stamped, offset a state: " << describe(f) << "\n";
  for (const auto& line : r.reports) std::cout << "  " << line << "\n";
}

TEST(TimeOffset, BothOffsetsAtOnceAndEachItsOwn) {
  // Fixes 0.1 s old, velocity 0.3 s old (the u-blox of the example recording is not far from that), both states.
  Drive d = winding();
  d.position_age = 0.1;
  d.velocity_age = 0.3;
  Given neither;
  neither.velocity = true;
  const Result plain = run(d, neither);
  Given s = neither;
  s.position_offset_state = true;
  s.velocity_offset_state = true;
  const Result r = run(d, s);
  ASSERT_GE(r.solutions, 100);
  EXPECT_EQ(r.errors, 0);
  ASSERT_TRUE(r.position_offset.has_value()) << describe(r);
  ASSERT_TRUE(r.velocity_offset.has_value()) << describe(r);
  EXPECT_NEAR(*r.position_offset, 0.1, 0.04) << describe(r);
  EXPECT_NEAR(*r.velocity_offset, 0.3, 0.03) << describe(r);
  EXPECT_LT(r.velocity_rms, 0.4 * plain.velocity_rms) << describe(r) << " against " << describe(plain);
  EXPECT_LT(r.heading_rms, 0.6 * plain.heading_rms) << describe(r) << " against " << describe(plain);
  EXPECT_LT(std::abs(r.along_mean), std::abs(plain.along_mean)) << describe(r) << " against " << describe(plain);
  std::cout << "fixes 0.1 s and velocity 0.3 s old, taken as stamped: " << describe(plain) << "\nboth offsets states: " << describe(r) << "\n";
}

TEST(TimeOffset, AStandingVehicleShowsNothingAndNothingIsClaimed) {
  // No velocity, no acceleration: both columns of the model are zero. Parked from the start, the alignment in motion
  // never hands over and the processors never run; nothing is reported and nothing breaks.
  Drive d;
  d.latency = 0.0;
  d.position_age = 0.3;
  d.velocity_age = 0.3;
  Given s;
  s.position_offset_state = true;
  s.velocity = true;
  s.velocity_offset_state = true;
  const Result r = run(d, s, 60.0, 0.0);
  EXPECT_EQ(r.errors, 0);
  EXPECT_TRUE(r.reports.empty());
}
