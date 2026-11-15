// Live-sensor robustness of the inertial (BufferedImu, StandardInertialPlugin): lost and late IMU samples,
// corrupt time stamps, SAMPLED data, NaN / infinity, the dt estimate after a gap, interpolation at vertical
// pitch. Each defect has a test that failed before the opt-in handling (inertial::ImuRobustness) existed;
// the "Default..." tests pin the NavToolkit-compatible behaviour that stays the default.
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/inertial/StandardInertialPlugin.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>

using namespace pntos;
using namespace pntos::test;
using api::Matrix;
using api::Matrix3;
using api::Message;
using api::Timestamp;
using api::Vector;
using api::Vector3;
namespace nav = cobra::nav;
namespace inr = cobra::inertial;
using AddStatus = inr::BufferedImu::AddStatus;

namespace {
constexpr double kLat = 0.69, kLon = -0.8, kAlt = 0.0;
constexpr std::int64_t kMs = 1'000'000;
constexpr double kT = 0.01;  // nominal IMU interval (s)
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

std::shared_ptr<cobra::utils::PVA> pva_at(std::int64_t tov, const Vector3& rpy = Vector3::Zero()) {
  return make_pva(tov, kLat, kLon, kAlt, 0, 0, 0, Vector(nav::rpy_to_quat(rpy)), Matrix::Zero(9, 9));
}
std::shared_ptr<cobra::utils::PVA> pva_with_dcm(std::int64_t tov, const Matrix3& C) {
  return make_pva(tov, kLat, kLon, kAlt, 0, 0, 0, Vector(nav::dcm_to_quat(C)), Matrix::Zero(9, 9));
}
std::shared_ptr<inr::Imu> integrated(std::int64_t tov, const Vector3& dv, const Vector3& dth) {
  auto m = make_imu(tov, dv, dth);
  m->set_imu_type(ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED);
  return m;
}
std::shared_ptr<inr::Imu> sampled(std::int64_t tov, const Vector3& accel, const Vector3& gyro) {
  return make_imu(tov, accel, gyro);  // make_imu builds the SAMPLED type
}

/// What a level, north-pointing IMU at rest measures: the reaction to gravity and the Earth's rotation
/// (with the rate the mechanization removes, so the reference solution does not rotate at all).
Vector3 static_force() { return -nav::calculate_gravity_schwartz(kAlt, kLat); }
Vector3 static_rate() { return Vector3(inr::kMechOmega * std::cos(kLat), 0.0, -inr::kMechOmega * std::sin(kLat)); }

struct Sample {
  std::int64_t t;
  Vector3 dv, dth;
};
/// `n` samples on the nominal grid starting one interval after t0, with constant body-frame force and rate.
std::vector<Sample> stream(int n, const Vector3& force, const Vector3& rate, std::int64_t t0 = 0) {
  std::vector<Sample> s;
  for (int i = 1; i <= n; ++i) s.push_back({t0 + i * 10 * kMs, force * kT, rate * kT});
  return s;
}
std::vector<Sample> without(const std::vector<Sample>& all, const std::function<bool(int)>& lost) {
  std::vector<Sample> s;
  for (std::size_t i = 0; i < all.size(); ++i)
    if (!lost(static_cast<int>(i) + 1)) s.push_back(all[i]);
  return s;
}
void feed(inr::BufferedImu& b, const std::vector<Sample>& s) {
  for (const auto& x : s) b.add(integrated(x.t, x.dv, x.dth));
}

Vector3 llh(const cobra::utils::PVA& p) { return cobra::utils::position(p); }
Vector3 vel(const cobra::utils::PVA& p) { return cobra::utils::velocity(p); }
Matrix3 dcm(const cobra::utils::PVA& p) { return nav::quat_to_dcm(*cobra::utils::quaternion(p)); }
/// Rotation angle between two attitudes (rad), accurate for very small angles (acos of the trace is not).
double attitude_error(const Matrix3& a, const Matrix3& b) {
  const Matrix3 e = a.transpose() * b;
  const Vector3 v(e(2, 1) - e(1, 2), e(0, 2) - e(2, 0), e(1, 0) - e(0, 1));
  return std::atan2(0.5 * v.norm(), 0.5 * (e.trace() - 1.0));
}
/// North / east / down distance (m) between two solutions.
Vector3 ned_between(const cobra::utils::PVA& a, const cobra::utils::PVA& b) {
  return Vector3(nav::delta_lat_to_north(b.get_p1() - a.get_p1(), kLat, kAlt),
                 nav::delta_lon_to_east(b.get_p2() - a.get_p2(), kLat, kAlt), -(b.get_p3() - a.get_p3()));
}
bool finite(const cobra::utils::PVA& p) {
  auto q = cobra::utils::quaternion(p);
  return llh(p).allFinite() && vel(p).allFinite() && q && q->allFinite();
}

inr::ImuRobustness hold() {
  inr::ImuRobustness r;
  r.gap_policy = inr::ImuGapPolicy::HOLD;
  return r;
}
inr::BufferedImu buffered(const inr::ImuRobustness& r, double buffer_length = 20.0) {
  return inr::BufferedImu(*pva_at(0), kT, buffer_length, {}, {}, r);
}

// One lost sample after 1.00 s and a full second lost from 1.50 s, in a 4 s stream.
bool lost_one_and_a_second(int i) { return i == 101 || (i > 150 && i <= 250); }
}  // namespace

// ----------------------------------------------------------------------------- lost samples (gaps)

TEST(InertialLive, DefaultFallsUnderGravityAcrossLostSamples) {
  // NavToolkit's behaviour, kept as the default: gravity is integrated over the stamped dt while the lost
  // increments are missing, so a static IMU gains g x (time lost) of downward velocity.
  const auto all = stream(400, static_force(), static_rate());
  auto ref = buffered({}), lossy = buffered({});
  feed(ref, all);
  feed(lossy, without(all, lost_one_and_a_second));
  const double g = -static_force()(2);
  EXPECT_LT(vel(*ref.latest_pva()).norm(), 1e-3);
  EXPECT_NEAR(vel(*lossy.latest_pva())(2), g * 1.01, 1e-3);  // 101 samples = 1.01 s lost: 9.9 m/s
  EXPECT_GT(ned_between(*ref.latest_pva(), *lossy.latest_pva())(2), 15.0);  // and 20 m lower, 2.5 s later
  EXPECT_EQ(lossy.stats().gap_filled, 0u);
}

TEST(InertialLive, HeldRateBridgesLostSamples) {
  const auto all = stream(400, static_force(), static_rate());
  auto ref = buffered({}), lossy = buffered(hold());
  feed(ref, all);
  std::vector<AddStatus> late;
  for (const auto& x : without(all, lost_one_and_a_second)) {
    const AddStatus s = lossy.add_sample(integrated(x.t, x.dv, x.dth));
    ASSERT_TRUE(inr::BufferedImu::accepted(s));
    if (s != AddStatus::ACCEPTED) late.push_back(s);
  }
  EXPECT_EQ(late, (std::vector<AddStatus>{AddStatus::GAP_FILLED, AddStatus::GAP_FILLED}));
  EXPECT_EQ(lossy.stats().gap_filled, 2u);
  EXPECT_EQ(lossy.stats().accepted, 299u);
  EXPECT_NEAR(lossy.stats().longest_gap, 1.01, 1e-9);
  // A static IMU: the late sample is mechanized as the samples lost before it would have been, so the
  // result is the one of the complete stream (to rounding; the steps are the same).
  EXPECT_LT((vel(*lossy.latest_pva()) - vel(*ref.latest_pva())).norm(), 1e-12);
  EXPECT_LT(ned_between(*ref.latest_pva(), *lossy.latest_pva()).norm(), 1e-8);
  EXPECT_LT(attitude_error(dcm(*ref.latest_pva()), dcm(*lossy.latest_pva())), 1e-12);
  EXPECT_EQ(lossy.time_span().second.elapsed_nsec, 4'000 * kMs);
}

TEST(InertialLive, HeldRateUnderDynamics) {
  // Accelerating and turning at constant body rates; one sample in ten lost for 5 s. Each lost sample equals
  // the one that follows it, so holding is exact here too, while the default loses the increments outright.
  const Vector3 force = static_force() + Vector3(1.0, 0.5, 0.0), rate = static_rate() + Vector3(0.02, -0.01, 0.1);
  const auto all = stream(500, force, rate);
  const auto lossy_stream = without(all, [](int i) { return i > 20 && i < 500 && i % 10 == 0; });
  auto ref = buffered({}), navtk = buffered({}), held = buffered(hold());
  feed(ref, all);
  feed(navtk, lossy_stream);
  feed(held, lossy_stream);
  EXPECT_EQ(held.stats().gap_filled, 47u);
  const double held_error = (vel(*held.latest_pva()) - vel(*ref.latest_pva())).norm();
  const double navtk_error = (vel(*navtk.latest_pva()) - vel(*ref.latest_pva())).norm();
  EXPECT_LT(held_error, 1e-9);
  EXPECT_GT(navtk_error, 4.0);  // 47 samples x 9.9 m/s^2 x 10 ms
  EXPECT_LT(ned_between(*ref.latest_pva(), *held.latest_pva()).norm(), 1e-8);
  EXPECT_LT(attitude_error(dcm(*ref.latest_pva()), dcm(*held.latest_pva())), 1e-12);
  EXPECT_GT(attitude_error(dcm(*ref.latest_pva()), dcm(*navtk.latest_pva())), 1e-2);  // 47 x 1 mrad of turn missing
}

TEST(InertialLive, HeldRateUnderChangingDynamics) {
  // Holding is an approximation when the motion changes between samples: 2 m/s^2 of surge and 0.3 rad/s of
  // yaw rate, both swinging at 1 Hz, one sample in ten lost for 5 s. What is lost is replaced by the sample
  // after it, so the error per gap is the change of the signal over one interval, not the signal itself.
  std::vector<Sample> all;
  for (int i = 1; i <= 500; ++i) {
    const double phase = 2 * nav::PI * 1.0 * (i - 0.5) * kT;
    all.push_back({i * 10 * kMs, (static_force() + Vector3(2.0 * std::sin(phase), 0.0, 0.0)) * kT,
                   (static_rate() + Vector3(0.0, 0.0, 0.3 * std::cos(phase))) * kT});
  }
  const auto lossy_stream = without(all, [](int i) { return i > 20 && i < 500 && i % 10 == 0; });
  auto ref = buffered({}), navtk = buffered({}), held = buffered(hold());
  feed(ref, all);
  feed(navtk, lossy_stream);
  feed(held, lossy_stream);
  const double held_velocity = (vel(*held.latest_pva()) - vel(*ref.latest_pva())).norm();
  const double navtk_velocity = (vel(*navtk.latest_pva()) - vel(*ref.latest_pva())).norm();
  const double held_position = ned_between(*ref.latest_pva(), *held.latest_pva()).norm();
  const double navtk_position = ned_between(*ref.latest_pva(), *navtk.latest_pva()).norm();
  const double held_attitude = attitude_error(dcm(*ref.latest_pva()), dcm(*held.latest_pva()));
  std::printf("47 lost samples under 1 Hz dynamics: held %.4f m/s, %.4f m, %.2e rad; default %.3f m/s, %.3f m\n",
              held_velocity, held_position, held_attitude, navtk_velocity, navtk_position);
  EXPECT_LT(held_velocity, 0.02);
  EXPECT_LT(held_position, 0.05);
  EXPECT_LT(held_attitude, 2e-3);
  EXPECT_GT(navtk_velocity, 4.0);
  EXPECT_GT(navtk_position, 10.0);
}

TEST(InertialLive, ResetReplaysHeldGaps) {
  // A reset before the gap re-mechanizes the buffered samples; the replay has to bridge the gap the same way.
  const auto all = stream(400, static_force(), static_rate());
  auto ref = buffered({}), lossy = buffered(hold());
  feed(ref, all);
  feed(lossy, without(all, lost_one_and_a_second));
  auto at_half_second = lossy.calc_pva(Timestamp{500 * kMs});
  ASSERT_TRUE(at_half_second);
  ASSERT_TRUE(lossy.reset(at_half_second.get(), nullptr));
  EXPECT_LT((vel(*lossy.latest_pva()) - vel(*ref.latest_pva())).norm(), 1e-9);
  EXPECT_LT(ned_between(*ref.latest_pva(), *lossy.latest_pva()).norm(), 1e-6);
  // a reset inside the long gap (between the samples at 1.50 s and 2.51 s) holds the rate over the rest of it
  auto in_gap = lossy.calc_pva(Timestamp{2'000 * kMs});
  ASSERT_TRUE(in_gap);
  ASSERT_TRUE(lossy.reset(in_gap.get(), nullptr));
  EXPECT_LT((vel(*lossy.latest_pva()) - vel(*ref.latest_pva())).norm(), 1e-9);
  // and the replay used for "no update within range" solutions does the same
  auto no_reset = lossy.calc_pva_no_reset_since(Timestamp{4'000 * kMs}, Timestamp{1'000 * kMs});
  ASSERT_TRUE(no_reset);
  EXPECT_LT((vel(*no_reset) - vel(*ref.latest_pva())).norm(), 1e-9);
  EXPECT_LT(ned_between(*ref.latest_pva(), *no_reset).norm(), 1e-6);
}

TEST(InertialLive, TimeStampJitterIsNotAGap) {
  // Stamps as irregular as the recorded VN-100 log at its worst (a sample 1.6 intervals late followed by
  // one 0.4 intervals later, no sample lost) must not be "filled": held and default modes agree exactly.
  auto navtk = buffered({}), held = buffered(hold());
  std::int64_t t = 0;
  for (int i = 1; i <= 400; ++i) {
    std::int64_t step = 10 * kMs + ((i * 7919) % 601 - 300) * 1000;  // +-0.3 ms
    if (i % 50 == 20) step = 16 * kMs;
    if (i % 50 == 21) step = 4 * kMs;
    t += step;
    for (auto* b : {&navtk, &held}) ASSERT_TRUE(b->add(integrated(t, static_force() * kT, static_rate() * kT)));
  }
  EXPECT_EQ(held.stats().gap_filled, 0u);
  EXPECT_EQ(llh(*held.latest_pva()), llh(*navtk.latest_pva()));
  EXPECT_EQ(vel(*held.latest_pva()), vel(*navtk.latest_pva()));
  // bounded by g x jitter, as without any robustness option
  EXPECT_LT(vel(*held.latest_pva()).norm(), 0.1);
}

TEST(InertialLive, OutOfOrderStampCostsNothingWhenHeld) {
  // Stamp jitter larger than the interval puts one sample before its predecessor. It is dropped as "not
  // newer" in every mode; by default its increments are then simply gone (0.098 m/s), while held, the
  // next sample is two intervals after the last accepted one and stands for both.
  const auto all = stream(300, static_force(), static_rate());
  auto ref = buffered({}), navtk = buffered({}), held = buffered(hold());
  feed(ref, all);
  for (auto* b : {&navtk, &held}) {
    for (int i = 0; i < 300; ++i) {
      const std::int64_t t = (i == 100) ? all[i].t - 12 * kMs : all[i].t;  // stamped before sample 99
      const AddStatus s = b->add_sample(integrated(t, all[i].dv, all[i].dth));
      if (i == 100) {
        EXPECT_EQ(s, AddStatus::NOT_NEWER);
      }
    }
    EXPECT_EQ(b->stats().not_newer, 1u);
  }
  EXPECT_NEAR(vel(*navtk.latest_pva())(2) - vel(*ref.latest_pva())(2), -static_force()(2) * kT, 1e-6);
  EXPECT_EQ(held.stats().gap_filled, 1u);
  EXPECT_LT((vel(*held.latest_pva()) - vel(*ref.latest_pva())).norm(), 1e-12);
}

// ----------------------------------------------------------------------------- corrupt time stamps

TEST(InertialLive, DefaultIsStrandedByOneFutureTimeStamp) {
  auto b = buffered({});
  const auto all = stream(300, static_force(), static_rate());
  int accepted_after = 0;
  for (int i = 0; i < 300; ++i) {
    const std::int64_t t = (i == 100) ? all[i].t + 3'600'000 * kMs : all[i].t;  // one sample stamped an hour ahead
    const bool ok = b.add(integrated(t, all[i].dv, all[i].dth));
    if (i > 100) accepted_after += ok;
  }
  EXPECT_EQ(accepted_after, 0);  // every later sample is "old"
  EXPECT_GT(vel(*b.latest_pva()).norm(), 1e4);
}

TEST(InertialLive, LoneFutureTimeStampIsRejected) {
  auto ref = buffered({}), b = buffered(inr::ImuRobustness::live_sensor());
  const auto all = stream(300, static_force(), static_rate());
  feed(ref, all);
  for (int i = 0; i < 300; ++i) {
    const std::int64_t t = (i == 100) ? all[i].t + 3'600'000 * kMs : all[i].t;
    const AddStatus s = b.add_sample(integrated(t, all[i].dv, all[i].dth));
    if (i == 100) {
      EXPECT_EQ(s, AddStatus::GAP_TOO_LONG);
    } else if (i == 101) {
      EXPECT_EQ(s, AddStatus::GAP_FILLED);  // the sample that carried the bad stamp is lost: held
    } else {
      EXPECT_EQ(s, AddStatus::ACCEPTED) << i;
    }
  }
  EXPECT_EQ(b.stats().gap_too_long, 1u);
  EXPECT_EQ(b.time_span().second.elapsed_nsec, 3'000 * kMs);
  EXPECT_LT((vel(*b.latest_pva()) - vel(*ref.latest_pva())).norm(), 1e-12);
  EXPECT_LT(ned_between(*ref.latest_pva(), *b.latest_pva()).norm(), 1e-8);
}

TEST(InertialLive, OutageBeyondMaxGapIsBridgedOnceConfirmed) {
  // The IMU really was away for 3 s (max_gap is 1 s): the first samples after it are refused, and once
  // kGapConfirmations of them agree with each other the inertial moves on, holding the rate over the outage.
  auto b = buffered(inr::ImuRobustness::live_sensor());
  feed(b, stream(100, static_force(), static_rate()));
  const auto after = stream(100, static_force(), static_rate(), 4'000 * kMs);
  std::vector<AddStatus> first;
  for (int i = 0; i < 5; ++i) first.push_back(b.add_sample(integrated(after[i].t, after[i].dv, after[i].dth)));
  EXPECT_EQ(first, (std::vector<AddStatus>{AddStatus::GAP_TOO_LONG, AddStatus::GAP_TOO_LONG, AddStatus::GAP_FILLED,
                                            AddStatus::ACCEPTED, AddStatus::ACCEPTED}));
  for (std::size_t i = 5; i < after.size(); ++i) EXPECT_TRUE(b.add(integrated(after[i].t, after[i].dv, after[i].dth)));
  EXPECT_EQ(b.time_span().second.elapsed_nsec, 5'000 * kMs);
  EXPECT_EQ(b.stats().gap_too_long, 2u);
  EXPECT_NEAR(b.stats().longest_gap, 3.03, 1e-9);
  auto whole = buffered({});  // the same five seconds with nothing missing
  feed(whole, stream(500, static_force(), static_rate()));
  EXPECT_LT((vel(*b.latest_pva()) - vel(*whole.latest_pva())).norm(), 1e-12);
  EXPECT_LT(ned_between(*whole.latest_pva(), *b.latest_pva()).norm(), 1e-8);
  // two unrelated corrupt stamps do not add up to a confirmation
  auto c = buffered(inr::ImuRobustness::live_sensor());
  feed(c, stream(100, static_force(), static_rate()));
  for (std::int64_t ahead_s : {3600, 7200, 10'800, 14'400}) {
    auto stray = integrated(ahead_s * 1000 * kMs, static_force() * kT, static_rate() * kT);
    EXPECT_EQ(c.add_sample(stray), AddStatus::GAP_TOO_LONG);
  }
  EXPECT_EQ(c.time_span().second.elapsed_nsec, 1'000 * kMs);
}

// ----------------------------------------------------------------------------- SAMPLED data

TEST(InertialLive, DefaultThrowsOnSampledImu) {
  auto b = buffered({});
  EXPECT_THROW(b.add(sampled(10 * kMs, static_force(), static_rate())), std::invalid_argument);
  EXPECT_THROW(b.add(nullptr), std::invalid_argument);
}

TEST(InertialLive, SampledImuCanBeDropped) {
  inr::ImuRobustness r;
  r.sampled_imu = inr::SampledImuPolicy::DROP;
  auto b = buffered(r);
  AddStatus s = AddStatus::ACCEPTED;
  EXPECT_NO_THROW(s = b.add_sample(sampled(10 * kMs, static_force(), static_rate())));
  EXPECT_EQ(s, AddStatus::SAMPLED_DROPPED);
  EXPECT_EQ(b.stats().sampled_dropped, 1u);
  EXPECT_EQ(b.time_span().second.elapsed_nsec, 0);
  EXPECT_THROW(b.add(nullptr), std::invalid_argument);  // still a programming error
}

TEST(InertialLive, SampledImuIsIntegrated) {
  inr::ImuRobustness r;
  r.sampled_imu = inr::SampledImuPolicy::INTEGRATE;
  const Vector3 force = static_force() + Vector3(1.0, 0.5, 0.0), rate = static_rate() + Vector3(0.02, -0.01, 0.1);
  auto ref = buffered({}), b = buffered(r);
  feed(ref, stream(300, force, rate));
  for (int i = 1; i <= 300; ++i) EXPECT_EQ(b.add_sample(sampled(i * 10 * kMs, force, rate)), AddStatus::ACCEPTED);
  EXPECT_EQ(b.stats().sampled_integrated, 300u);
  // on a regular grid, rates x dt are the increments an integrating IMU would have reported
  EXPECT_LT((vel(*b.latest_pva()) - vel(*ref.latest_pva())).norm(), 1e-9);
  EXPECT_LT(ned_between(*ref.latest_pva(), *b.latest_pva()).norm(), 1e-8);
  EXPECT_LT(attitude_error(dcm(*ref.latest_pva()), dcm(*b.latest_pva())), 1e-12);
  // forces and rates are reported for sampled records as for integrated ones
  auto f_ref = ref.calc_force_and_rate(Timestamp{2'000 * kMs}), f = b.calc_force_and_rate(Timestamp{2'000 * kMs});
  ASSERT_TRUE(f_ref && f);
  EXPECT_TRUE(allclose(Vector(f->get_meas_accel()), Vector(f_ref->get_meas_accel()), 1e-9, 1e-9));
  EXPECT_TRUE(allclose(Vector(f->get_meas_gyro()), Vector(f_ref->get_meas_gyro()), 1e-9, 1e-12));
  auto a_ref = ref.calc_force_and_rate(Timestamp{1'000 * kMs}, Timestamp{2'000 * kMs});
  auto a = b.calc_force_and_rate(Timestamp{1'000 * kMs}, Timestamp{2'000 * kMs});
  ASSERT_TRUE(a_ref && a);
  EXPECT_TRUE(allclose(Vector(a->get_meas_accel()), Vector(a_ref->get_meas_accel()), 1e-9, 1e-9));
  // a reset between two samples replays sampled records as it replays integrated ones
  ASSERT_TRUE(b.reset(b.calc_pva(Timestamp{1'505 * kMs}).get(), nullptr));
  ASSERT_TRUE(ref.reset(ref.calc_pva(Timestamp{1'505 * kMs}).get(), nullptr));
  EXPECT_LT((vel(*b.latest_pva()) - vel(*ref.latest_pva())).norm(), 1e-9);
  EXPECT_LT(attitude_error(dcm(*ref.latest_pva()), dcm(*b.latest_pva())), 1e-12);
}

TEST(InertialLive, SampledImuToleratesIrregularStampsAndGaps) {
  // Rates are integrated over the stamped interval, whatever it is: a static IMU stays put through jitter
  // and through a half-second hole, with no nominal interval involved.
  inr::ImuRobustness r;
  r.sampled_imu = inr::SampledImuPolicy::INTEGRATE;
  inr::BufferedImu b(*pva_at(0), 0.02 /* wrong on purpose */, 20.0, {}, {}, r);
  std::int64_t t = 0;
  for (int i = 1; i <= 400; ++i) {
    t += 10 * kMs + ((i * 7919) % 6001 - 3000) * 1000;  // +-3 ms
    if (i == 200) t += 500 * kMs;
    ASSERT_TRUE(b.add(sampled(t, static_force(), static_rate())));
  }
  // force x dt and gravity x dt cancel whatever dt is: no free fall. (Horizontally the mechanization's
  // 0.5 dtheta x dv term leaves the 3e-6 m/s^2 it also leaves on a perfect stream.)
  EXPECT_LT(std::abs(vel(*b.latest_pva())(2)), 1e-7);  // one lost 10 ms sample would be 0.098 m/s
  EXPECT_LT(vel(*b.latest_pva()).norm(), 1e-4);
  EXPECT_LT(std::abs(b.latest_pva()->get_p3() - kAlt), 1e-6);
  EXPECT_EQ(b.stats().gap_filled, 1u);
  auto f = b.calc_force_and_rate(b.time_span().second);
  ASSERT_TRUE(f);
  EXPECT_NEAR(f->get_meas_accel()(2), static_force()(2), 1e-6);
}

// ----------------------------------------------------------------------------- NaN / infinity

TEST(InertialLive, DefaultIsPoisonedByOneNaN) {
  auto b = buffered({});
  auto all = stream(200, static_force(), static_rate());
  all[100].dv(0) = kNaN;
  feed(b, all);
  EXPECT_FALSE(finite(*b.latest_pva()));  // and stays so: every later solution is integrated from it
}

TEST(InertialLive, NonFiniteSamplesAreRejected) {
  auto r = hold();
  r.reject_non_finite = true;
  auto ref = buffered({}), b = buffered(r);
  const auto all = stream(300, static_force(), static_rate());
  feed(ref, all);
  for (int i = 0; i < 300; ++i) {
    Sample x = all[i];
    if (i == 100) x.dv(0) = kNaN;
    if (i == 150) x.dth(2) = kInf;
    if (i == 200) x.dv(1) = -kInf;
    const AddStatus s = b.add_sample(integrated(x.t, x.dv, x.dth));
    if (i == 100 || i == 150 || i == 200) {
      EXPECT_EQ(s, AddStatus::NON_FINITE);
    } else {
      EXPECT_TRUE(inr::BufferedImu::accepted(s));
    }
  }
  EXPECT_EQ(b.stats().non_finite, 3u);
  EXPECT_EQ(b.stats().gap_filled, 3u);  // each rejected sample leaves a gap, which is held
  ASSERT_TRUE(finite(*b.latest_pva()));
  EXPECT_LT((vel(*b.latest_pva()) - vel(*ref.latest_pva())).norm(), 1e-6);
  for (std::int64_t t = 0; t <= 3'000 * kMs; t += 25 * kMs) EXPECT_TRUE(finite(*b.calc_pva(Timestamp{t})));
}

TEST(InertialLive, NonFiniteResetsAreRejected) {
  inr::ImuRobustness r;
  r.reject_non_finite = true;
  auto b = buffered(r);
  feed(b, stream(100, static_force(), static_rate()));
  const Vector3 before = vel(*b.latest_pva());

  auto nan_position = pva_at(500 * kMs);
  nan_position->set_p1(kNaN);
  EXPECT_FALSE(b.reset(nan_position.get(), nullptr));
  auto inf_velocity = pva_at(500 * kMs);
  inf_velocity->set_v3(kInf);
  EXPECT_FALSE(b.reset(inf_velocity.get(), nullptr));
  auto no_attitude = pva_at(500 * kMs);
  no_attitude->set_quaternion(dyn({kNaN, kNaN, kNaN, kNaN}));
  EXPECT_FALSE(b.reset(no_attitude.get(), nullptr));
  auto zero_quaternion = pva_at(500 * kMs);
  zero_quaternion->set_quaternion(dyn({0, 0, 0, 0}));
  EXPECT_FALSE(b.reset(zero_quaternion.get(), nullptr));
  inr::ImuErrors nan_bias;
  nan_bias.gyro_biases(1) = kNaN;
  nan_bias.time = Timestamp{500 * kMs};
  EXPECT_FALSE(b.reset(nullptr, &nan_bias));

  EXPECT_EQ(b.stats().resets_rejected, 5u);
  EXPECT_EQ(vel(*b.latest_pva()), before);
  EXPECT_TRUE(finite(*b.latest_pva()));
  EXPECT_TRUE(b.imu_errors(Timestamp{900 * kMs}).gyro_biases.allFinite());
  // a good reset still goes through
  EXPECT_TRUE(b.reset(pva_at(500 * kMs).get(), nullptr));
  // and an inertial cannot start from a solution that is not finite
  auto bad_start = pva_at(0);
  bad_start->set_p3(kNaN);
  EXPECT_THROW(inr::BufferedImu(*bad_start, kT, 20.0, {}, {}, r), std::invalid_argument);
}

TEST(InertialLive, SolutionWithoutAttitudeDoesNotBreakInterpolation) {
  // No option involved: a reset to a solution without a quaternion mechanizes from a level attitude
  // (to_standard_pva), and interpolating next to that buffer entry used to dereference an empty optional.
  auto b = buffered({});
  feed(b, stream(100, static_force(), static_rate()));
  auto no_attitude = pva_at(500 * kMs);
  no_attitude->set_quaternion(dyn({kNaN, kNaN, kNaN, kNaN}));
  ASSERT_TRUE(b.reset(no_attitude.get(), nullptr));
  for (std::int64_t t : {495 * kMs, 505 * kMs}) {
    auto p = b.calc_pva(Timestamp{t});
    ASSERT_TRUE(p);
    EXPECT_TRUE(finite(*p));
    EXPECT_LT(attitude_error(dcm(*p), Matrix3::Identity()), 1e-4);
  }
}

// ----------------------------------------------------------------------------- dt estimate

TEST(InertialLive, DefaultDtEstimateRemembersEveryGap) {
  auto b = buffered({});
  feed(b, without(stream(400, static_force(), static_rate()), [](int i) { return i > 100 && i <= 200; }));
  EXPECT_NEAR(b.estimated_dt(), 4.0 / 300, 1e-12);  // mean since construction: 13.3 ms for a 10 ms IMU
  auto f = b.calc_force_and_rate(b.time_span().second);
  ASSERT_TRUE(f);
  EXPECT_NEAR(f->get_meas_accel()(2), static_force()(2) * 0.75, 1e-3);  // reported force is 25 % low, 2 s after the gap
}

TEST(InertialLive, WindowedDtEstimateForgetsGaps) {
  inr::ImuRobustness r;
  r.dt_window = 50;
  auto b = buffered(r);
  feed(b, without(stream(400, static_force(), static_rate()), [](int i) { return i > 100 && i <= 200; }));
  EXPECT_NEAR(b.estimated_dt(), kT, 1e-12);
  auto f = b.calc_force_and_rate(b.time_span().second);
  ASSERT_TRUE(f);
  EXPECT_NEAR(f->get_meas_accel()(2), static_force()(2), 1e-6);
  auto avg = b.calc_force_and_rate(Timestamp{3'200 * kMs}, Timestamp{3'900 * kMs});
  ASSERT_TRUE(avg);
  EXPECT_NEAR(avg->get_meas_accel()(2), static_force()(2), 1e-6);
  // it also follows the real rate when the configured one is wrong (every estimate does, after 10 intervals)
  inr::BufferedImu c(*pva_at(0), 0.02, 20.0, {}, {}, r);
  feed(c, stream(8, static_force(), static_rate()));
  EXPECT_EQ(c.estimated_dt(), 0.02);
  feed(c, stream(30, static_force(), static_rate(), 80 * kMs));
  EXPECT_NEAR(c.estimated_dt(), kT, 1e-12);
}

// ----------------------------------------------------------------------------- interpolation at vertical pitch

TEST(InertialLive, DefaultInterpolationLosesTheAttitudeAtVerticalPitch) {
  // NavToolkit interpolates through roll / pitch / yaw. Pointing straight up, rolled 0.4 rad between two
  // solutions: the midpoint should be rolled 0.2 rad.
  auto a = pva_at(0, Vector3(0.0, nav::PI / 2, 0.0)), b = pva_at(10 * kMs, Vector3(0.4, nav::PI / 2, 0.0));
  auto mid = inr::linear_interp_pva(*a, *b, Timestamp{5 * kMs});
  const Matrix3 truth = nav::quat_to_dcm(nav::rpy_to_quat(Vector3(0.2, nav::PI / 2, 0.0)));
  EXPECT_GT(attitude_error(dcm(*mid), truth), 1e-2);
}

TEST(InertialLive, QuaternionInterpolationHasNoPitchSingularity) {
  const Vector3 rotation(0.012, -0.007, 0.015);  // body rotation between the two solutions (rad)
  for (double pitch : {nav::PI / 2, -nav::PI / 2, nav::PI / 2 - 1e-9, -nav::PI / 2 + 1e-9, nav::PI / 2 - 1e-7, 0.3}) {
    for (double roll : {0.0, 0.4, -2.5}) {
      const Matrix3 C1 = nav::quat_to_dcm(nav::rpy_to_quat(Vector3(roll, pitch, 1.1)));
      auto a = pva_with_dcm(0, C1), b = pva_with_dcm(10 * kMs, C1 * nav::rot_vec_to_dcm(rotation));
      for (double f : {0.25, 0.5, 0.9}) {
        auto p = inr::linear_interp_pva(*a, *b, Timestamp{static_cast<std::int64_t>(f * 10 * kMs)},
                                        inr::AttitudeInterpolation::QUATERNION);
        ASSERT_TRUE(p);
        EXPECT_LT(attitude_error(dcm(*p), C1 * nav::rot_vec_to_dcm(f * rotation)), 1e-12)
            << "pitch " << pitch << " roll " << roll << " fraction " << f;
        EXPECT_NEAR(Vector(p->get_quaternion()).norm(), 1.0, 1e-14);
      }
    }
  }
  // opposite-sign quaternions describe the same attitude; position and velocity are interpolated as before
  auto a = pva_at(0, Vector3(0.1, 0.2, 0.3)), b = pva_at(10 * kMs, Vector3(0.1, 0.2, 0.3));
  b->set_quaternion(DynVector(-Vector(b->get_quaternion())));
  b->set_p3(kAlt + 4.0);
  auto p = inr::linear_interp_pva(*a, *b, Timestamp{5 * kMs}, inr::AttitudeInterpolation::QUATERNION);
  EXPECT_LT(attitude_error(dcm(*p), dcm(*a)), 1e-15);
  EXPECT_DOUBLE_EQ(p->get_p3(), kAlt + 2.0);
}

TEST(InertialLive, BufferedSolutionsAtVerticalPitch) {
  // Nose up and rolling at 0.5 rad/s: solutions between two samples must lie on the rotation between them.
  inr::ImuRobustness r;
  r.attitude_interpolation = inr::AttitudeInterpolation::QUATERNION;
  const Matrix3 C0 = nav::quat_to_dcm(nav::rpy_to_quat(Vector3(0.0, nav::PI / 2, 0.0)));
  inr::BufferedImu b(*pva_with_dcm(0, C0), kT, 20.0, {}, {}, r);
  const Vector3 rate = C0.transpose() * static_rate() + Vector3(0.5, 0.0, 0.0);
  feed(b, stream(100, C0.transpose() * static_force(), rate));
  for (int i = 10; i < 90; i += 7) {
    auto before = b.calc_pva(Timestamp{i * 10 * kMs}), after = b.calc_pva(Timestamp{(i + 1) * 10 * kMs});
    auto mid = b.calc_pva(Timestamp{i * 10 * kMs + 5 * kMs});
    ASSERT_TRUE(before && after && mid);
    // half of the rotation between the two samples, as a rotation vector (axis x angle / 2)
    const Matrix3 step = dcm(*before).transpose() * dcm(*after);
    const Vector3 sine_axis(0.5 * (step(2, 1) - step(1, 2)), 0.5 * (step(0, 2) - step(2, 0)),
                            0.5 * (step(1, 0) - step(0, 1)));
    const double angle = std::atan2(sine_axis.norm(), 0.5 * (step.trace() - 1.0));
    ASSERT_GT(angle, 4e-3);  // it is rolling
    const Vector3 half = sine_axis / sine_axis.norm() * (0.5 * angle);
    EXPECT_LT(attitude_error(dcm(*mid), dcm(*before) * nav::rot_vec_to_dcm(half)), 1e-12) << i;
  }
}

// ----------------------------------------------------------------------------- configuration

TEST(InertialLive, ConstructorRejectsImpossibleConfiguration) {
  // buffer capacity is buffer_length / expected_dt + 2: zero, negative or NaN used to give an absurd size
  for (double dt : {0.0, -0.01, kNaN, kInf})
    EXPECT_THROW(inr::BufferedImu(*pva_at(0), dt, 5.0), std::invalid_argument) << dt;
  for (double len : {0.0, -5.0, kNaN, kInf})
    EXPECT_THROW(inr::BufferedImu(*pva_at(0), kT, len), std::invalid_argument) << len;
  inr::ImuRobustness r;
  r.gap_factor = 1.0;  // every sample would be a gap
  EXPECT_THROW(inr::BufferedImu(*pva_at(0), kT, 5.0, {}, {}, r), std::invalid_argument);
  r = {};
  r.max_gap = -1.0;
  EXPECT_THROW(inr::BufferedImu(*pva_at(0), kT, 5.0, {}, {}, r), std::invalid_argument);
  r = {};
  r.dt_window = 2;  // a median of two cannot outvote one gap
  EXPECT_THROW(inr::BufferedImu(*pva_at(0), kT, 5.0, {}, {}, r), std::invalid_argument);
  r.dt_window = 3;
  EXPECT_NO_THROW(inr::BufferedImu(*pva_at(0), kT, 5.0, {}, {}, r));
}

namespace {
struct PluginFixture {
  TestMediator med;
  cobra::StandardRegistryPlugin reg{"registry", {}};
  cobra::StandardInertialPlugin plugin{"Cobra inertial plugin"};
  PluginFixture() {
    reg.init_plugin(std::nullopt, &med);
    med.set_registry(reg.new_registry());
    plugin.init_plugin(std::nullopt, &med);
  }
  cobra::InertialConfig config(const inr::ImuRobustness& r) {
    cobra::InertialConfig cfg;
    cfg.group_ = "config/inertial";
    cfg.expected_dt = kT;
    cfg.inertial_buffer_length = 20.0;
    cfg.channels = {"/sensor/imu"};
    cfg.robustness = r;
    return cfg;
  }
  std::unique_ptr<api::CommonInertial> make(const inr::ImuRobustness& r) {
    config(r).to_registry(med);
    return plugin.new_inertial(api::InertialType::STANDARD_MECHANIZATION, Message(pva_at(0), ""), "config/inertial");
  }
};
}  // namespace

TEST(InertialLive, ConfigKeysAppearOnlyWhenSet) {
  PluginFixture fx;
  // the defaults leave the registry group exactly as Python writes it
  fx.config({}).to_registry(fx.med);
  auto keys = fx.med.registry().batch("config/inertial")->keys().value_or(std::vector<std::string>{});
  std::sort(keys.begin(), keys.end());
  EXPECT_EQ(keys, (std::vector<std::string>{"C_imu_to_platform", "channels", "expected_dt", "inertial_buffer_length"}));
  auto back = cobra::InertialConfig::from_registry(fx.med, "config/inertial");
  ASSERT_TRUE(back);
  EXPECT_TRUE(back->robustness.is_default());

  // every field survives the registry
  inr::ImuRobustness r = inr::ImuRobustness::live_sensor();
  r.gap_factor = 1.6;
  r.max_gap = 0.25;
  r.dt_window = 64;
  auto cfg = fx.config(r);
  cfg.group_ = "config/inertial_live";
  cfg.to_registry(fx.med);
  back = cobra::InertialConfig::from_registry(fx.med, "config/inertial_live");
  ASSERT_TRUE(back);
  EXPECT_EQ(back->robustness, r);
  EXPECT_EQ(*fx.med.registry().batch("config/inertial_live")->get_value<std::string>("imu_gap_policy"), "hold");
  EXPECT_EQ(*fx.med.registry().batch("config/inertial_live")->get_value<std::string>("sampled_imu"), "integrate");

  // an unknown policy name is a configuration error, not a silent default
  fx.med.registry().batch("config/inertial_live")->set("attitude_interpolation", std::string("euler"));
  EXPECT_FALSE(cobra::InertialConfig::from_registry(fx.med, "config/inertial_live"));
  const auto type = api::InertialType::STANDARD_MECHANIZATION;
  EXPECT_EQ(fx.plugin.new_inertial(type, Message(pva_at(0), ""), "config/inertial_live"), nullptr);
}

TEST(InertialLive, PluginRejectsImpossibleConfigurationWithAnError) {
  PluginFixture fx;
  auto cfg = fx.config({});
  cfg.expected_dt = 0.0;
  cfg.to_registry(fx.med);
  const std::size_t errors = fx.med.count(api::LoggingLevel::ERROR);
  const auto type = api::InertialType::STANDARD_MECHANIZATION;
  EXPECT_EQ(fx.plugin.new_inertial(type, Message(pva_at(0), ""), "config/inertial"), nullptr);
  EXPECT_EQ(fx.med.count(api::LoggingLevel::ERROR), errors + 1) << fx.med.last_message();
  EXPECT_NE(fx.med.last_message().find("expected_dt"), std::string::npos) << fx.med.last_message();
}

TEST(InertialLive, PluginSurvivesALiveSensor) {
  PluginFixture fx;
  auto common = fx.make(inr::ImuRobustness::live_sensor());
  ASSERT_TRUE(common);
  auto* inertial = dynamic_cast<cobra::StandardInertial*>(common.get());
  ASSERT_TRUE(inertial);
  auto send = [&](const std::shared_ptr<inr::Imu>& m) { inertial->process_pntos_message(Message(m, "/sensor/imu")); };

  std::int64_t t = 0;
  auto good = [&] { return integrated(t += 10 * kMs, static_force() * kT, static_rate() * kT); };
  for (int i = 0; i < 100; ++i) send(good());
  EXPECT_EQ(fx.med.count(api::LoggingLevel::WARN), 0u);

  // 450 NaN samples in 0.9 s: rejected, reported, and not 450 log lines
  for (int i = 0; i < 450; ++i)
    EXPECT_NO_THROW(send(integrated(t + (i + 1) * 2 * kMs, Vector3(kNaN, 0, 0), Vector3::Zero())));
  t += 900 * kMs;
  const std::size_t after_nan = fx.med.count(api::LoggingLevel::WARN);
  EXPECT_GE(after_nan, 1u);
  EXPECT_LE(after_nan, 4u);
  EXPECT_NE(fx.med.last_message().find("non-finite"), std::string::npos) << fx.med.last_message();
  for (int i = 0; i < 100; ++i) send(good());  // the first of these closes the 0.9 s hole
  // sampled data on the same channel is integrated, a stamp from the future is refused
  for (int i = 0; i < 100; ++i) EXPECT_NO_THROW(send(sampled(t += 10 * kMs, static_force(), static_rate())));
  send(integrated(t + 86'400'000 * kMs, static_force() * kT, static_rate() * kT));
  for (int i = 0; i < 100; ++i) send(good());

  const auto& stats = inertial->buffered().stats();
  EXPECT_EQ(stats.non_finite, 450u);
  EXPECT_EQ(stats.sampled_integrated, 100u);
  EXPECT_EQ(stats.gap_too_long, 1u);
  EXPECT_EQ(stats.gap_filled, 1u);
  EXPECT_EQ(stats.accepted, 400u);
  EXPECT_EQ(inertial->request_latest_time().elapsed_nsec, t);
  auto sol = inertial->request_current_solution().as<cobra::utils::PVA>();
  ASSERT_TRUE(sol);
  EXPECT_TRUE(finite(*sol));
  EXPECT_LT(std::abs(vel(*sol)(2)), 1e-7);  // nothing fell: one lost 10 ms sample would be 0.098 m/s
  EXPECT_LT(vel(*sol).norm(), 1e-4);
  EXPECT_LE(fx.med.count(api::LoggingLevel::WARN), 8u);
  EXPECT_FALSE(fx.med.has_error()) << fx.med.last_message();

  // a non-finite feedback from a diverged filter is refused and reported, the solution stays usable
  auto bad = pva_at(t - 50 * kMs);
  bad->set_v1(kNaN);
  const std::size_t warns = fx.med.count(api::LoggingLevel::WARN);
  inertial->reset_solution(Message(bad, ""));
  api::StandardInertialErrors errs;
  errs.accel_biases(0) = kInf;
  inertial->correct_sensor_errors(Timestamp{t - 50 * kMs}, errs);
  EXPECT_EQ(fx.med.count(api::LoggingLevel::WARN), warns + 2) << fx.med.last_message();
  EXPECT_EQ(stats.resets_rejected, 2u);
  EXPECT_TRUE(finite(*inertial->request_current_solution().as<cobra::utils::PVA>()));
}

TEST(InertialLive, PluginDefaultIsUnchanged) {
  // Without any option the plugin behaves as before: a SAMPLED message throws through process_pntos_message
  // (callers that cannot afford that set sampled_imu), a late sample is mechanized as it is, nothing is logged.
  PluginFixture fx;
  auto common = fx.make({});
  ASSERT_TRUE(common);
  auto* inertial = dynamic_cast<cobra::StandardInertial*>(common.get());
  ASSERT_TRUE(inertial);
  auto increments_at = [](std::int64_t t) {
    return Message(integrated(t, static_force() * kT, static_rate() * kT), "");
  };
  for (int i = 1; i <= 50; ++i) inertial->process_pntos_message(increments_at(i * 10 * kMs));
  inertial->process_pntos_message(increments_at(700 * kMs));
  const Message rates(sampled(710 * kMs, static_force(), static_rate()), "");
  EXPECT_THROW(inertial->process_pntos_message(rates), std::invalid_argument);
  EXPECT_EQ(fx.med.logs.size(), 0u) << fx.med.last_message();
  EXPECT_GT(vel(*inertial->request_current_solution().as<cobra::utils::PVA>())(2), 1.5);  // 0.19 s of free fall
}
