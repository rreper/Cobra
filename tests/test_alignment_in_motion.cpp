// Alignment in motion (initialization/InMotionAlignment.hpp) on simulated drives: a vehicle on level ground whose
// speed and course are smooth functions of time, an IMU with bias and noise, and a receiver that delivers the
// antenna's velocity and position once a second, two seconds late.
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/app/AppBuilder.hpp>
#include <pntos/cobra/app/Filter.hpp>
#include <pntos/cobra/config/JsonConfig.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/initialization/InMotionAlignment.hpp>
#include <pntos/cobra/initialization/InitializationPlugins.hpp>
#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/logging.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include "test_support.hpp"

#include <aspn23/eigen/MeasurementVelocity.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <random>

using namespace pntos;
using namespace pntos::test;
using api::Matrix3;
using api::Vector3;
using cobra::inertial::AlignmentStatus;
using cobra::inertial::InMotionAlignment;
using cobra::inertial::InMotionAlignmentOptions;
namespace nav = cobra::nav;

namespace {

constexpr double kDeg = 0.017453292519943295;
constexpr std::int64_t kT0 = 1'700'000'000'000'000'000;  // epoch-sized time stamps, as a receiver gives them

double smoothstep(double t, double t0, double t1) {
  if (t <= t0) return 0.0;
  if (t >= t1) return 1.0;
  const double u = (t - t0) / (t1 - t0);
  return u * u * (3.0 - 2.0 * u);
}

Vector3 rotation_vector(const Matrix3& R) {
  const Vector3 v(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  const double s = 0.5 * v.norm();
  if (s < 1e-12) return 0.5 * v;
  const double angle = std::atan2(s, 0.5 * (R.trace() - 1.0));
  return v * (0.5 * angle / s);
}

/// Rotation error of `est` against `truth` (both body to NED), as a vector in NED: (north, east, down) tilts.
Vector3 attitude_error(const Matrix3& est, const Matrix3& truth) { return rotation_vector(est * truth.transpose()); }

struct Drive {
  std::function<double(double)> speed = [](double) { return 0.0; };   ///< m/s along the course
  std::function<double(double)> course = [](double) { return 0.0; };  ///< rad
  double roll = -0.015, pitch = -0.04;  ///< rad
  double mount_yaw = 0.0;               ///< rad; the sensor's x axis relative to the direction of travel
  Vector3 gyro_bias = Vector3(-1.6e-3, -2.0e-3, -2.8e-3);  ///< rad/s, the VN-100 of the example recording
  Vector3 accel_bias = Vector3(0.03, -0.02, 0.05);         ///< m/s^2
  double gyro_noise = 1.0e-4;   ///< rad/sqrt(s)
  double accel_noise = 2.4e-3;  ///< m/s/sqrt(s)
  double velocity_sigma = 0.05; ///< m/s per axis
  double position_sigma = 0.05; ///< m per axis, the part of the fix error that changes from fix to fix
  Vector3 lever = Vector3::Zero();  ///< antenna relative to the IMU, body frame
  double lat = 0.70, lon = -1.50, alt = 220.0;
  double latency = 2.0;         ///< s between a fix and its delivery
  double velocity_age = 0.0;    ///< s; the velocity stamped t is the velocity of t - velocity_age (the receiver's filter)
  double position_age = 0.0;    ///< s; the same for the position
  double imu_dt = 0.01;
  double drop_imu = 0.0;        ///< fraction of IMU samples lost
  bool send_velocity = true, send_position = true;
  unsigned seed = 1;

  Vector3 velocity(double t) const {
    const double c = course(t);
    return speed(t) * Vector3(std::cos(c), std::sin(c), 0.0);
  }
  Matrix3 attitude(double t) const { return nav::rpy_to_dcm(Vector3(roll, pitch, course(t) + mount_yaw)); }
};

struct Outcome {
  bool aligned = false;
  double at = 0;              ///< s of the drive at which the alignment was declared (arrival time)
  double solution_time = 0;   ///< s of the drive the solution refers to
  Vector3 att_error = Vector3::Zero();  ///< rad, NED
  Vector3 vel_error = Vector3::Zero();  ///< m/s
  Vector3 pos_error = Vector3::Zero();  ///< m, NED
  Vector3 gyro_bias = Vector3::Zero();
  Vector3 accel_bias = Vector3::Zero();
  api::Matrix cov;
  InMotionAlignment::Diagnostics diag;
  std::string summary;
  double heading_sigma() const { return std::sqrt(cov(8, 8)); }
};

constexpr const char* kImuChannel = "/sensor/vn-100/imu";
constexpr const char* kPositionChannel = "/sensor/ublox-ZED-F9T/position";
constexpr const char* kVelocityChannel = "/sensor/ublox-ZED-F9T/velocity";

using Track = std::vector<std::pair<double, Vector3>>;  ///< the IMU's position (NED from the start) at every sample time
/// Takes a message at its arrival time (s of the drive); false ends the drive.
using Sink = std::function<bool(const std::shared_ptr<api::AspnBase>&, const char* channel, double arrival)>;

/// Plays the drive: IMU samples as they are taken, the receiver's fixes `latency` s after their time.
Track simulate(const Drive& d, double duration, const Sink& sink) {
  std::mt19937 rng(d.seed);
  std::normal_distribution<double> normal(0.0, 1.0);
  auto noise = [&](double sigma) -> Vector3 { return Vector3(normal(rng), normal(rng), normal(rng)) * sigma; };
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  const Vector3 g = nav::calculate_gravity_schwartz(d.alt, d.lat);
  const Vector3 wie = nav::ROTATION_RATE * Vector3(std::cos(d.lat), 0.0, -std::sin(d.lat));
  const int sub = 10;
  const double h = d.imu_dt / sub;
  auto accel = [&](double t) { return Vector3((d.velocity(t + 1e-4) - d.velocity(t - 1e-4)) / 2e-4); };
  auto ns = [&](double t) { return kT0 + static_cast<std::int64_t>(std::llround(t * 1e9)); };

  struct Fix {
    double t;
    Vector3 v, p;
  };
  std::vector<Fix> fixes;       // generated, not yet delivered
  Vector3 p = Vector3::Zero();  // of the IMU, NED from the start
  Track track{{0.0, p}};
  double next_fix = 0.5;
  api::Matrix vel_cov = api::Matrix::Identity(3, 3) * d.velocity_sigma * d.velocity_sigma;
  api::Matrix pos_cov = api::Matrix::Identity(3, 3) * 1.0;

  const int samples = static_cast<int>(duration / d.imu_dt);
  for (int k = 1; k <= samples; ++k) {
    const double t0 = (k - 1) * d.imu_dt, t1 = k * d.imu_dt;
    Vector3 dv = Vector3::Zero();
    for (int j = 0; j < sub; ++j) {
      const double tm = t0 + (j + 0.5) * h;
      const Vector3 v = d.velocity(tm);
      dv += d.attitude(tm).transpose() * (accel(tm) - g + 2.0 * wie.cross(v)) * h;
      p += v * h;
    }
    track.emplace_back(t1, p);
    Vector3 dth = rotation_vector(d.attitude(t0).transpose() * d.attitude(t1)) + d.attitude(t1).transpose() * wie * d.imu_dt;
    dv += d.accel_bias * d.imu_dt + noise(d.accel_noise * std::sqrt(d.imu_dt));
    dth += d.gyro_bias * d.imu_dt + noise(d.gyro_noise * std::sqrt(d.imu_dt));

    while (next_fix <= t1 + 1e-9) {  // the antenna's velocity and position at the fix time
      const double tf = next_fix;
      const double tv = tf - d.velocity_age, tp = tf - d.position_age;  // what the receiver shows is that old
      const Matrix3 C = d.attitude(tp);
      const Vector3 lever_rate = (d.attitude(tv + 1e-4) - d.attitude(tv - 1e-4)) * d.lever / 2e-4;
      Vector3 pf = track.back().second + d.velocity(tf) * (tf - t1);
      if (d.position_age != 0.0) pf -= d.velocity(tf - 0.5 * d.position_age) * d.position_age;
      fixes.push_back({tf, Vector3(d.velocity(tv) + lever_rate + noise(d.velocity_sigma)), Vector3(pf + C * d.lever + noise(d.position_sigma))});
      next_fix += 1.0;
    }

    const bool lost = d.drop_imu > 0 && k > 5 && uniform(rng) < d.drop_imu;
    if (!lost) {
      auto imu = std::make_shared<aspn23_eigen::MeasurementImu>(header(ASPN_MEASUREMENT_IMU), aspn23_eigen::TypeTimestamp(ns(t1)),
                                                                ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED, DynVector(dv), DynVector(dth),
                                                                std::vector<aspn23_eigen::TypeIntegrity>{});
      if (!sink(imu, kImuChannel, t1)) return track;
    }
    while (!fixes.empty() && fixes.front().t + d.latency <= t1 + 1e-9) {
      const Fix f = fixes.front();
      fixes.erase(fixes.begin());
      if (d.send_position) {
        const double lat = d.lat + nav::north_to_delta_lat(f.p(0), d.lat, d.alt);
        const double lon = d.lon + nav::east_to_delta_lon(f.p(1), d.lat, d.alt);
        if (!sink(make_position(ns(f.t), lat, lon, d.alt - f.p(2), pos_cov), kPositionChannel, t1)) return track;
      }
      if (d.send_velocity) {
        auto vel = std::make_shared<aspn23_eigen::MeasurementVelocity>(
            header(ASPN_MEASUREMENT_VELOCITY), aspn23_eigen::TypeTimestamp(ns(f.t)), ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED, f.v(0),
            f.v(1), f.v(2), rm(vel_cov), ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE, DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});
        if (!sink(vel, kVelocityChannel, t1)) return track;
      }
    }
  }
  return track;
}

Vector3 position_at(const Track& track, double t) {
  const auto it = std::lower_bound(track.begin(), track.end(), t - 1e-9, [](const auto& a, double at) { return a.first < at; });
  return it == track.end() ? track.back().second : it->second;
}

/// Feeds the drive to an aligner in arrival order until it is aligned or `duration` s have passed.
Outcome run(const Drive& d, const InMotionAlignmentOptions& options, double duration,
            const cobra::inertial::ImuModel& model = cobra::inertial::ImuModel::from_config(cobra::ImuConfig{})) {
  cobra::inertial::ImuModel m = model;
  if (m.gyro_bias_initial_sigma.isZero()) {  // the VN-100 figures of the presets
    m.gyro_bias_initial_sigma.setConstant(3e-3);
    m.accel_bias_initial_sigma.setConstant(0.072);
    m.gyro_random_walk_sigma.setConstant(1e-4);
    m.accel_random_walk_sigma.setConstant(2.4e-3);
    m.gyro_bias_sigma.setConstant(5e-5);
    m.accel_bias_sigma.setConstant(6e-4);
  }
  InMotionAlignment aligner(m, options);
  Outcome out;
  const Track track = simulate(d, duration, [&](const std::shared_ptr<api::AspnBase>& message, const char*, double arrival) {
    if (aligner.process(*message) != AlignmentStatus::ALIGNED_GOOD) return true;
    out.aligned = true;
    out.at = arrival;
    return false;
  });
  out.diag = aligner.diagnostics();
  out.summary = aligner.summary();
  const auto [have, sol] = aligner.get_computed_alignment();
  const auto [have_cov, cov] = aligner.get_computed_covariance();
  const auto [have_err, err] = aligner.get_imu_errors();
  EXPECT_EQ(have, out.aligned);
  EXPECT_EQ(have_cov, out.aligned);
  EXPECT_EQ(have_err, out.aligned);
  out.cov = cov;
  if (!out.aligned) return out;
  const double ts = (sol.time.elapsed_nsec - kT0) * 1e-9;
  out.solution_time = ts;
  out.att_error = attitude_error(sol.rot_mat.transpose(), d.attitude(ts));
  out.vel_error = sol.vel - d.velocity(ts);
  const Vector3 truth = position_at(track, ts);
  out.pos_error = Vector3(nav::delta_lat_to_north(sol.pos(0) - d.lat, d.lat, d.alt), nav::delta_lon_to_east(sol.pos(1) - d.lon, d.lat, d.alt),
                          -(sol.pos(2) - d.alt)) -
                  truth;
  out.gyro_bias = err.gyro_biases;
  out.accel_bias = err.accel_biases;
  return out;
}

/// Stands for `rest` s, then pulls away to 10 m/s in a right-hand bend.
Drive pull_away(double heading, double rest = 6.0) {
  Drive d;
  d.speed = [rest](double t) { return 10.0 * smoothstep(t, rest, rest + 8.0); };
  d.course = [heading, rest](double t) { return heading + 0.6 * smoothstep(t, rest + 3.0, rest + 10.0); };
  return d;
}

/// Already under way when switched on: the speed swings by 3 m/s and the road winds.
Drive under_way(double heading) {
  Drive d;
  d.speed = [](double t) { return 12.0 + 3.0 * std::sin(0.4 * t); };
  d.course = [heading](double t) { return heading + 0.5 * std::sin(0.25 * t); };
  return d;
}

std::string describe(const Outcome& o) {
  std::ostringstream os;
  os << "aligned " << o.aligned << " at " << o.at << " s; attitude error (deg) " << (o.att_error / kDeg).transpose() << "; sigmas ";
  if (o.aligned) os << std::sqrt(o.cov(6, 6)) / kDeg << " " << std::sqrt(o.cov(7, 7)) / kDeg << " " << o.heading_sigma() / kDeg;
  os << " deg; gyro bias " << (o.gyro_bias * 1e3).transpose() << " mrad/s, sigmas ";
  if (o.aligned) os << std::sqrt(o.cov(12, 12)) * 1e3 << " " << std::sqrt(o.cov(13, 13)) * 1e3 << " " << std::sqrt(o.cov(14, 14)) * 1e3;
  os << "; accel bias " << o.accel_bias.transpose() << "; " << o.summary;
  return os.str();
}

}  // namespace

TEST(InMotionAlignment, FindsTheHeadingWhenTheVehiclePullsAway) {
  const Drive d = pull_away(2.2);
  const Outcome o = run(d, {}, 60.0);
  ASSERT_TRUE(o.aligned) << describe(o);
  EXPECT_LT(o.at, 6.0 + 20.0) << describe(o);  // within 20 s of moving off, delivery latency included
  EXPECT_LT(std::abs(o.att_error(2)), 3.0 * kDeg) << describe(o);
  EXPECT_LT(o.att_error.head<2>().norm(), 0.5 * kDeg) << describe(o);
  EXPECT_LT(o.heading_sigma(), 3.0 * kDeg);
  EXPECT_LT(std::abs(o.att_error(2)), 3.0 * o.heading_sigma()) << describe(o);
  // the standstill gave the gyro bias, the Earth's rate aside
  EXPECT_LT((o.gyro_bias - d.gyro_bias).norm(), 2e-4) << describe(o);
  EXPECT_GE(o.diag.stationary, 3.0);
  // the solution is at the last IMU sample, two seconds past the last fix used
  EXPECT_NEAR(o.solution_time, o.at, 1e-6);
  EXPECT_LT(o.vel_error.norm(), 0.3) << o.vel_error.transpose();
  EXPECT_LT(o.pos_error.norm(), 1.5) << o.pos_error.transpose();
  EXPECT_NE(o.summary.find("Aligned in motion"), std::string::npos) << o.summary;
}

TEST(InMotionAlignment, FindsTheHeadingWhenSwitchedOnUnderWay) {
  // No standstill: the gyro bias (0.16 deg/s about the vertical) has to come out of the same data.
  const Drive d = under_way(-1.0);
  const Outcome o = run(d, {}, 90.0);
  ASSERT_TRUE(o.aligned) << describe(o);
  EXPECT_LT(o.at, 40.0) << describe(o);
  EXPECT_LT(std::abs(o.att_error(2)), 4.0 * kDeg) << describe(o);
  EXPECT_LT(o.att_error.head<2>().norm(), 0.7 * kDeg) << describe(o);
  EXPECT_LT(std::abs(o.att_error(2)), 3.0 * o.heading_sigma()) << describe(o);
  EXPECT_EQ(o.diag.stationary, 0.0);
  // The bias comes with a covariance that covers its error, and the data has told at least one level axis
  // (gravity leaking into the velocity) more than the 3 mrad/s the model started from.
  for (int i = 0; i < 3; ++i) EXPECT_LT(std::abs(o.gyro_bias(i) - d.gyro_bias(i)), 3.0 * std::sqrt(o.cov(12 + i, 12 + i))) << i << ": " << describe(o);
  EXPECT_LT(std::min(o.cov(12, 12), o.cov(13, 13)), 2.0e-3 * 2.0e-3) << describe(o);
  // the vertical accelerometer error is handed over as a bias along the vertical
  EXPECT_NEAR(o.accel_bias(2), d.accel_bias(2), 0.02) << describe(o);
}

TEST(InMotionAlignment, TheGyroBiasHasToBeEstimatedAlong) {
  // Why the bias is part of the problem. A gentle drive shows the heading slowly, so the window is long, and
  // over a long window 0.1 deg/s of bias tilts the integrated attitude by degrees. The same drives with the
  // bias pinned to zero by the model:
  cobra::inertial::ImuModel pinned;
  pinned.gyro_bias_initial_sigma.setConstant(1e-9);
  pinned.accel_bias_initial_sigma.setConstant(0.072);
  pinned.gyro_random_walk_sigma.setConstant(1e-4);
  pinned.gyro_bias_sigma.setConstant(5e-5);
  double est_heading = 0, est_tilt = 0, pin_heading = 0, pin_tilt = 0;
  int est_n = 0, pin_n = 0;
  for (unsigned seed = 1; seed <= 8; ++seed) {
    Drive d;
    d.speed = [](double t) { return 12.0 + 1.2 * std::sin(0.2 * t); };
    d.course = [seed](double t) { return 0.7 * seed + 0.12 * std::sin(0.15 * t); };
    d.seed = seed;
    const Outcome a = run(d, {}, 120.0);
    ASSERT_TRUE(a.aligned) << "seed " << seed << ": " << describe(a);
    EXPECT_LT(std::abs(a.att_error(2)), 3.5 * a.heading_sigma()) << "seed " << seed << ": " << describe(a);
    est_heading += a.att_error(2) * a.att_error(2);
    est_tilt += a.att_error.head<2>().squaredNorm();
    ++est_n;
    const Outcome b = run(d, {}, 120.0, pinned);
    if (b.aligned) {
      pin_heading += b.att_error(2) * b.att_error(2);
      pin_tilt += b.att_error.head<2>().squaredNorm();
      ++pin_n;
    }
  }
  est_heading = std::sqrt(est_heading / est_n);
  est_tilt = std::sqrt(est_tilt / est_n);
  RecordProperty("estimated_heading_rms_deg", std::to_string(est_heading / kDeg));
  RecordProperty("estimated_tilt_rms_deg", std::to_string(est_tilt / kDeg));
  EXPECT_LT(est_heading, 3.0 * kDeg);
  EXPECT_LT(est_tilt, 0.5 * kDeg);
  // Pinned, it either never claims an alignment (the misfit keeps the sigma up) or claims a worse one.
  if (pin_n > 0) {
    pin_heading = std::sqrt(pin_heading / pin_n);
    pin_tilt = std::sqrt(pin_tilt / pin_n);
    EXPECT_GT(pin_tilt, 2.0 * est_tilt) << "pinned: " << pin_n << " of 8 aligned, heading rms " << pin_heading / kDeg << " deg, tilt rms "
                                        << pin_tilt / kDeg << " deg; estimated: heading " << est_heading / kDeg << ", tilt " << est_tilt / kDeg;
  }
  std::cout << "estimated: heading rms " << est_heading / kDeg << " deg, tilt rms " << est_tilt / kDeg << " deg; pinned: " << pin_n
            << " of 8 aligned, heading rms " << (pin_n ? pin_heading / kDeg : 0.0) << " deg, tilt rms " << (pin_n ? pin_tilt / kDeg : 0.0) << " deg\n";
}

TEST(InMotionAlignment, AnyHeadingAndAnyMounting) {
  // Nothing assumes the sensor looks where the vehicle goes: sideways and backwards mountings align as well.
  for (double heading : {0.0, 1.5, 3.1, -2.0}) {
    for (double mount : {0.0, 0.5 * nav::PI, nav::PI}) {
      Drive d = pull_away(heading);
      d.mount_yaw = mount;
      const Outcome o = run(d, {}, 60.0);
      ASSERT_TRUE(o.aligned) << "heading " << heading << " mount " << mount << ": " << describe(o);
      EXPECT_LT(std::abs(o.att_error(2)), 3.0 * kDeg) << "heading " << heading << " mount " << mount << ": " << describe(o);
      EXPECT_LT(o.att_error.head<2>().norm(), 0.7 * kDeg) << "heading " << heading << " mount " << mount << ": " << describe(o);
      for (int i = 0; i < 2; ++i)  // the level accelerometer biases (0.2 deg of tilt here) are inside the sigma
        EXPECT_LT(std::abs(o.att_error(i)), 3.0 * std::sqrt(o.cov(6 + i, 6 + i))) << "heading " << heading << " mount " << mount << ": " << describe(o);
    }
  }
}

TEST(InMotionAlignment, DoesNotClaimAHeadingItCannotSee) {
  // Straight and steady: gravity levels the sensor, nothing shows the heading.
  Drive steady;
  steady.speed = [](double) { return 12.0; };
  steady.course = [](double) { return 0.8; };
  const Outcome a = run(steady, {}, 90.0);
  EXPECT_FALSE(a.aligned) << describe(a);
  EXPECT_GT(a.diag.solutions, 10u);
  EXPECT_GT(a.diag.heading_sigma, 20.0 * kDeg) << describe(a);

  // Standing: the same, and the gyro bias is known meanwhile.
  Drive parked;
  const Outcome b = run(parked, {}, 60.0);
  EXPECT_FALSE(b.aligned) << describe(b);
  EXPECT_GE(b.diag.stationary, 50.0);
  EXPECT_LT((b.diag.gyro_bias - parked.gyro_bias).norm(), 1.5e-4) << (b.diag.gyro_bias * 1e3).transpose();
}

TEST(InMotionAlignment, PositionFixesAloneAreEnough) {
  Drive d = pull_away(0.7);
  d.send_velocity = false;
  const Outcome o = run(d, {}, 80.0);
  ASSERT_TRUE(o.aligned) << describe(o);
  EXPECT_TRUE(o.diag.from_positions);
  EXPECT_LT(std::abs(o.att_error(2)), 4.0 * kDeg) << describe(o);
  EXPECT_LT(o.att_error.head<2>().norm(), 0.7 * kDeg) << describe(o);
  EXPECT_LT(std::abs(o.att_error(2)), 3.0 * o.heading_sigma()) << describe(o);
  EXPECT_NE(o.summary.find("position differences"), std::string::npos) << o.summary;

  // With a velocity channel the receiver's own velocity is used, and a position-only source can be forced.
  d.send_velocity = true;
  EXPECT_FALSE(run(d, {}, 80.0).diag.from_positions);
  InMotionAlignmentOptions forced;
  forced.velocity_source = InMotionAlignmentOptions::VelocitySource::POSITION;
  const Outcome f = run(d, forced, 80.0);
  ASSERT_TRUE(f.aligned);
  EXPECT_TRUE(f.diag.from_positions);
}

TEST(InMotionAlignment, AntennaLeverArmIsTakenOut) {
  // An antenna 3 m ahead of the IMU sweeps sideways in a bend; left in, that is read as a velocity of the IMU.
  Drive d = pull_away(-0.4);
  d.lever = Vector3(3.0, 0.0, -1.0);
  d.course = [](double t) { return -0.4 + 1.4 * smoothstep(t, 8.0, 14.0); };  // a tight bend at walking pace
  double with = 0, without = 0;
  for (unsigned seed = 1; seed <= 8; ++seed) {
    d.seed = seed;
    InMotionAlignmentOptions known;
    known.lever_arm = d.lever;
    const Outcome a = run(d, known, 60.0);
    const Outcome b = run(d, {}, 60.0);
    ASSERT_TRUE(a.aligned) << describe(a);
    EXPECT_LT(std::abs(a.att_error(2)), 3.0 * kDeg) << "seed " << seed << ": " << describe(a);
    EXPECT_LT(a.pos_error.norm(), 1.5) << a.pos_error.transpose();
    with += a.att_error(2) * a.att_error(2);
    without += b.aligned ? b.att_error(2) * b.att_error(2) : (10.0 * kDeg) * (10.0 * kDeg);
  }
  EXPECT_LT(std::sqrt(with / 8), 1.5 * kDeg);
  EXPECT_GT(std::sqrt(without / 8), 2.0 * std::sqrt(with / 8)) << "rms with " << std::sqrt(with / 8) / kDeg << " deg, without "
                                                                << std::sqrt(without / 8) / kDeg << " deg";
}

TEST(InMotionAlignment, TheReportedSigmaIsHonest) {
  // Forty drives, standing and moving starts: the heading error over its reported sigma has unit scale.
  double sum = 0, worst = 0;
  int n = 0;
  for (unsigned seed = 1; seed <= 20; ++seed) {
    for (int kind = 0; kind < 2; ++kind) {
      Drive d = kind == 0 ? pull_away(0.3 * seed) : under_way(0.3 * seed);
      d.seed = seed;
      const Outcome o = run(d, {}, 90.0);
      ASSERT_TRUE(o.aligned) << "seed " << seed << " kind " << kind << ": " << describe(o);
      const double z = o.att_error(2) / o.heading_sigma();
      sum += z * z;
      worst = std::max(worst, std::abs(z));
      ++n;
    }
  }
  const double rms = std::sqrt(sum / n);
  EXPECT_GT(rms, 0.4) << "the sigma is far too large";
  EXPECT_LT(rms, 1.5) << "the sigma is too small";
  EXPECT_LT(worst, 4.0);
}

// ----------------------------------------------------------------------------- the receiver's time offset
// DESIGN.md §7.29. A receiver's velocity comes out of the receiver's own filter and is older than its time stamp:
// 0.25 s for the u-blox of the example recording.

namespace {
struct Campaign {
  int n = 0, aligned = 0;
  double heading = 0, ratio = 0, time = 0;      ///< heading error RMS (rad), error over sigma RMS, mean time to align (s)
  double offset_mean = 0, offset_rms = 0, offset_sigma = 0, offset_worst = 0;  ///< s; rms and worst of (estimate - truth)
  double velocity = 0;                           ///< RMS of the velocity error at the hand-over, m/s
};
/// Twelve drives, six that pull away and six switched on under way, with the receiver's velocity `age` s old.
Campaign campaign(double age, const InMotionAlignmentOptions& options) {
  Campaign c;
  for (unsigned seed = 1; seed <= 6; ++seed) {
    for (int kind = 0; kind < 2; ++kind) {
      Drive d = kind == 0 ? pull_away(0.5 * seed) : under_way(0.5 * seed);
      d.seed = seed;
      d.velocity_age = age;
      const Outcome o = run(d, options, 120.0);
      ++c.n;
      if (!o.aligned) continue;
      ++c.aligned;
      c.heading += o.att_error(2) * o.att_error(2);
      const double z = o.att_error(2) / o.heading_sigma();
      c.ratio += z * z;
      c.time += o.at;
      c.velocity += o.vel_error.squaredNorm();
      const double e = o.diag.time_offset - age;
      c.offset_mean += o.diag.time_offset;
      c.offset_rms += e * e;
      c.offset_worst = std::max(c.offset_worst, std::abs(e));
      c.offset_sigma += o.diag.time_offset_sigma;
    }
  }
  if (c.aligned) {
    c.heading = std::sqrt(c.heading / c.aligned);
    c.ratio = std::sqrt(c.ratio / c.aligned);
    c.time /= c.aligned;
    c.velocity = std::sqrt(c.velocity / c.aligned);
    c.offset_mean /= c.aligned;
    c.offset_rms = std::sqrt(c.offset_rms / c.aligned);
    c.offset_sigma /= c.aligned;
  }
  return c;
}
std::string describe(const Campaign& c) {
  std::ostringstream os;
  os << c.aligned << " of " << c.n << " aligned after " << c.time << " s (mean); heading rms " << c.heading / kDeg << " deg, error / sigma rms "
     << c.ratio << "; velocity error rms " << c.velocity << " m/s; time offset mean " << c.offset_mean << " s, error rms " << c.offset_rms
     << " s, worst " << c.offset_worst << " s, sigma " << c.offset_sigma << " s";
  return os.str();
}
/// The offset estimated: nothing known about it beforehand but that it is a fraction of a second.
InMotionAlignmentOptions with_offset(double prior_sigma = 0.3) {
  InMotionAlignmentOptions o;
  o.time_offset_prior_sigma = prior_sigma;
  return o;
}
}  // namespace

TEST(InMotionAlignment, AnOldVelocityTurnsTheHeadingAndTheSigmaDoesNotSaySo) {
  // The default, pinned: the stamps are taken as they are. A quarter of a second of age more than doubles the
  // heading error, and the sigma the filter is started with no longer covers it.
  const Campaign fresh = campaign(0.0, {});
  const Campaign old = campaign(0.25, {});
  ASSERT_EQ(fresh.aligned, fresh.n) << describe(fresh);
  ASSERT_EQ(old.aligned, old.n) << describe(old);
  EXPECT_LT(fresh.heading, 3.0 * kDeg) << describe(fresh);
  EXPECT_LT(fresh.ratio, 1.3) << describe(fresh);
  EXPECT_GT(old.heading, 2.0 * fresh.heading) << describe(old);
  EXPECT_GT(old.ratio, 1.5) << describe(old);
  EXPECT_EQ(old.offset_mean, 0.0);  // nothing is estimated, nothing is reported
  std::cout << "velocity as stamped: " << describe(fresh) << "\nvelocity 0.25 s old, taken as stamped: " << describe(old) << "\n";
}

TEST(InMotionAlignment, TheTimeOffsetIsEstimatedAlong) {
  // The same drives with the offset as one more unknown (prior: zero, 0.3 s; handed over once it is known to 0.08 s).
  const Campaign old = campaign(0.25, with_offset());
  ASSERT_EQ(old.aligned, old.n) << describe(old);
  EXPECT_NEAR(old.offset_mean, 0.25, 0.04) << describe(old);
  EXPECT_LT(old.offset_rms, 0.08) << describe(old);
  EXPECT_LT(old.offset_worst, 0.16) << describe(old);
  EXPECT_LE(old.offset_sigma, 0.08) << describe(old);
  EXPECT_LT(old.heading, 2.5 * kDeg) << describe(old);
  EXPECT_LT(old.ratio, 1.2) << describe(old);
  EXPECT_LT(old.velocity, 0.3) << describe(old);  // the hand-over is at the IMU's time, with the velocity of that time
  // and it is found whatever it is: newer than stamped, or nearly half a second old
  for (double age : {-0.15, 0.1, 0.4}) {
    const Campaign c = campaign(age, with_offset());
    ASSERT_EQ(c.aligned, c.n) << "age " << age << ": " << describe(c);
    EXPECT_NEAR(c.offset_mean, age, 0.04) << "age " << age << ": " << describe(c);
    EXPECT_LT(c.offset_rms, 0.08) << "age " << age << ": " << describe(c);
    EXPECT_LT(c.heading, 2.5 * kDeg) << "age " << age << ": " << describe(c);
    EXPECT_LT(c.ratio, 1.2) << "age " << age << ": " << describe(c);
    std::cout << "velocity " << age << " s old, offset estimated: " << describe(c) << "\n";
  }
  std::cout << "velocity 0.25 s old, offset estimated: " << describe(old) << "\n";
}

TEST(InMotionAlignment, TheOffsetHasToBeKnownBeforeTheHandOver) {
  // Why time_offset_sigma exists. Handed over as soon as the heading sigma allows (the limit on the offset's sigma
  // out of the way), the offset is still half the prior's zero: too small by a quarter, and the heading with it.
  InMotionAlignmentOptions early = with_offset();
  early.time_offset_sigma = 1.0;
  const Campaign e = campaign(0.25, early);
  const Campaign c = campaign(0.25, with_offset());
  ASSERT_EQ(e.aligned, e.n) << describe(e);
  ASSERT_EQ(c.aligned, c.n) << describe(c);
  EXPECT_LT(e.offset_mean, 0.22) << describe(e);
  EXPECT_GT(e.heading, 1.5 * c.heading) << describe(e) << " against " << describe(c);
  EXPECT_GT(e.ratio, 1.2) << describe(e);
  EXPECT_LT(c.time, e.time + 4.0) << describe(c) << " against " << describe(e);  // what waiting costs
  std::cout << "handed over without waiting for the offset: " << describe(e) << "\n";
}

TEST(InMotionAlignment, AnOffsetThatIsNotThereIsNotInvented) {
  // What the extra unknown costs when the stamps were right: nothing in the heading, a few seconds.
  const Campaign plain = campaign(0.0, {});
  const Campaign est = campaign(0.0, with_offset());
  ASSERT_EQ(est.aligned, est.n) << describe(est);
  EXPECT_LT(std::abs(est.offset_mean), 0.03) << describe(est);
  EXPECT_LT(est.offset_rms, 0.08) << describe(est);
  EXPECT_LT(est.heading, 1.1 * plain.heading) << describe(est) << " against " << describe(plain);
  EXPECT_LT(est.ratio, 1.2) << describe(est);
  EXPECT_LT(est.time, plain.time + 6.0) << describe(est) << " against " << describe(plain);
  std::cout << "velocity as stamped, offset estimated: " << describe(est) << "\n";
}

TEST(InMotionAlignment, TheTimeOffsetIsReportedAndCarriedToTheHandOver) {
  Drive d = under_way(0.4);
  d.velocity_age = 0.3;
  const Outcome o = run(d, with_offset(), 120.0);
  ASSERT_TRUE(o.aligned) << describe(o);
  EXPECT_NEAR(o.diag.time_offset, 0.3, 0.12) << describe(o);
  EXPECT_GT(o.diag.time_offset_sigma, 0.0);
  EXPECT_LE(o.diag.time_offset_sigma, 0.08);
  EXPECT_NE(o.summary.find("the receiver's velocity is 0."), std::string::npos) << o.summary;
  EXPECT_NE(o.summary.find("s older than its time stamps (sigma 0.0"), std::string::npos) << o.summary;
  // the solution is still at the last IMU sample, with that moment's velocity and position
  EXPECT_NEAR(o.solution_time, o.at, 1e-6);
  EXPECT_LT(o.vel_error.norm(), 0.3) << o.vel_error.transpose();
  EXPECT_LT(o.pos_error.norm(), 1.5) << o.pos_error.transpose();
  EXPECT_LT(std::abs(o.att_error(2)), 3.0 * o.heading_sigma()) << describe(o);
  // Without the option the line says nothing about an offset.
  const Outcome plain = run(d, {}, 120.0);
  ASSERT_TRUE(plain.aligned);
  EXPECT_EQ(plain.summary.find("time stamps"), std::string::npos) << plain.summary;
  EXPECT_EQ(plain.diag.time_offset, 0.0);
  // A velocity that arrives almost at once (no controller buffer) and is newer than its stamp: the IMU has to
  // get past the epoch first, and does.
  Drive quick = under_way(-0.8);
  quick.latency = 0.05;
  quick.velocity_age = -0.2;
  const Outcome q = run(quick, with_offset(), 120.0);
  ASSERT_TRUE(q.aligned) << describe(q);
  EXPECT_NEAR(q.diag.time_offset, -0.2, 0.12) << describe(q);
  EXPECT_NE(q.summary.find("s newer than its time stamps"), std::string::npos) << q.summary;
  EXPECT_LT(std::abs(q.att_error(2)), 3.0 * q.heading_sigma()) << describe(q);
}

TEST(InMotionAlignment, TheOffsetOfPositionFixesIsFoundTheSameWay) {
  // No velocity channel: the velocity is the difference of consecutive fixes, and the offset is that of the fixes.
  double sum = 0, worst = 0;
  int n = 0;
  for (unsigned seed = 1; seed <= 6; ++seed) {
    Drive d = under_way(0.7 * seed);
    d.seed = seed;
    d.send_velocity = false;
    d.position_age = 0.2;
    const Outcome o = run(d, with_offset(), 200.0);
    ASSERT_TRUE(o.aligned) << "seed " << seed << ": " << describe(o);
    EXPECT_TRUE(o.diag.from_positions);
    EXPECT_NE(o.summary.find("the position fixes are 0."), std::string::npos) << o.summary;
    EXPECT_LT(std::abs(o.att_error(2)), 3.5 * o.heading_sigma()) << "seed " << seed << ": " << describe(o);
    // The position handed over starts from the last fix as it is stamped, 0.2 s and 2.4 m behind at this speed:
    // where the filter that takes over will find the next fixes. The offset is reported, not applied to the fix.
    EXPECT_GT(o.pos_error.norm(), 1.5) << "seed " << seed << ": " << o.pos_error.transpose();
    EXPECT_LT(o.pos_error.norm(), 3.5) << "seed " << seed << ": " << o.pos_error.transpose();
    sum += o.diag.time_offset;
    worst = std::max(worst, std::abs(o.diag.time_offset - 0.2));
    ++n;
  }
  EXPECT_NEAR(sum / n, 0.2, 0.06);
  EXPECT_LT(worst, 0.2);
  std::cout << "position fixes 0.2 s old: offset mean " << sum / n << " s, worst error " << worst << " s\n";
}

TEST(InMotionAlignment, WithoutAManoeuvreTheOffsetStaysWhereThePriorPutsIt) {
  // Straight and steady: no acceleration, nothing shows how old the velocity is, and no heading is claimed either.
  Drive steady;
  steady.speed = [](double) { return 12.0; };
  steady.course = [](double) { return 0.8; };
  steady.velocity_age = 0.3;
  const Outcome a = run(steady, with_offset(), 90.0);
  EXPECT_FALSE(a.aligned) << describe(a);
  EXPECT_GT(a.diag.solutions, 10u);
  EXPECT_LT(std::abs(a.diag.time_offset), 0.15) << a.diag.time_offset;
  EXPECT_GT(a.diag.time_offset_sigma, 0.2) << a.diag.time_offset_sigma;  // of the 0.3 s it started with
}

TEST(InMotionAlignment, LostImuSamplesAreHeldNotIntegratedAsFreeFall) {
  // 4 % of the samples lost, as a serial line under load loses them (the inertial's imu_gap_policy "hold").
  Drive d = pull_away(1.1);
  d.drop_imu = 0.04;
  const Outcome o = run(d, {}, 60.0);
  ASSERT_TRUE(o.aligned) << describe(o);
  EXPECT_GT(o.diag.held, 20u);
  EXPECT_LT(std::abs(o.att_error(2)), 3.0 * kDeg) << describe(o);
  EXPECT_LT(o.att_error.head<2>().norm(), 0.5 * kDeg) << describe(o);
  EXPECT_LT(o.vel_error.norm(), 0.4) << o.vel_error.transpose();
}

TEST(InMotionAlignment, ALongImuGapStartsTheWindowAgain) {
  InMotionAlignment a;
  const api::Matrix cov = api::Matrix::Identity(3, 3);
  std::int64_t t = kT0;
  auto imu = [&](std::int64_t at) {
    return aspn23_eigen::MeasurementImu(header(ASPN_MEASUREMENT_IMU), aspn23_eigen::TypeTimestamp(at),
                                        ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED, DynVector(Vector3(0, 0, -0.098)),
                                        DynVector(Vector3::Zero()), std::vector<aspn23_eigen::TypeIntegrity>{});
  };
  for (int k = 0; k < 300; ++k) a.process(imu(t += 10'000'000));
  a.process(*make_position(t - 1'000'000'000, 0.7, -1.5, 200.0, cov));
  EXPECT_EQ(a.diagnostics().restarts, 0u);
  a.process(imu(t += 900'000'000));  // 0.9 s without data
  EXPECT_EQ(a.diagnostics().restarts, 1u);
  // out of order, and not finite: counted, not used, no exception
  a.process(imu(t - 5'000'000));
  aspn23_eigen::MeasurementImu bad(header(ASPN_MEASUREMENT_IMU), aspn23_eigen::TypeTimestamp(t + 10'000'000),
                                   ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED, DynVector(Vector3(0, std::nan(""), 0)),
                                   DynVector(Vector3::Zero()), std::vector<aspn23_eigen::TypeIntegrity>{});
  a.process(bad);
  EXPECT_GE(a.diagnostics().ignored, 1u);
  EXPECT_EQ(a.check_alignment_status(), AlignmentStatus::ALIGNING_COARSE);
  EXPECT_FALSE(a.get_computed_alignment().first);
  EXPECT_TRUE(a.summary().empty());
}

TEST(InMotionAlignment, StartTimeEmulatesASwitchOnInsideARecording) {
  // Everything before the start time is ignored, so the alignment is that of a system switched on under way.
  Drive d = pull_away(0.9, 6.0);
  InMotionAlignmentOptions late;
  late.start_time = kT0 * 1e-9 + 12.0;  // the car left at 6 s
  const Outcome o = run(d, late, 80.0);
  const Outcome whole = run(d, {}, 80.0);
  ASSERT_TRUE(whole.aligned);
  EXPECT_GE(whole.diag.stationary, 3.0);
  if (o.aligned) {
    EXPECT_EQ(o.diag.stationary, 0.0);
    EXPECT_GT(o.at, whole.at);
  }
}

TEST(InMotionAlignment, RejectsOptionsThatMakeNoSense) {
  InMotionAlignmentOptions o;
  o.heading_sigma = 0;
  EXPECT_THROW(InMotionAlignment(cobra::inertial::stim300_model(), o), std::invalid_argument);
  o = {};
  o.min_epochs = 0;
  EXPECT_THROW(InMotionAlignment(cobra::inertial::stim300_model(), o), std::invalid_argument);
  o = {};
  o.max_window = -1;
  EXPECT_THROW(InMotionAlignment(cobra::inertial::stim300_model(), o), std::invalid_argument);
  o = {};
  o.time_offset_prior_sigma = -0.1;
  EXPECT_THROW(InMotionAlignment(cobra::inertial::stim300_model(), o), std::invalid_argument);
  o.time_offset_prior_sigma = 0.6;  // an offset that could be anything is not that of a receiver's filter
  EXPECT_THROW(InMotionAlignment(cobra::inertial::stim300_model(), o), std::invalid_argument);
  o.time_offset_prior_sigma = std::nan("");
  EXPECT_THROW(InMotionAlignment(cobra::inertial::stim300_model(), o), std::invalid_argument);
  o = {};
  o.time_offset_sigma = 0.0;
  EXPECT_THROW(InMotionAlignment(cobra::inertial::stim300_model(), o), std::invalid_argument);
}

// ----------------------------------------------------------------------------- plugin and configuration

namespace {
struct Env {
  TestMediator med;
  cobra::StandardRegistryPlugin reg{"registry", {}};
  Env() {
    reg.init_plugin(std::nullopt, &med);
    med.set_registry(reg.new_registry());
  }
};
}  // namespace

TEST(InMotionAlignPlugin, BuildsTheStrategyFromItsConfig) {
  Env env;
  TestMediator& med = env.med;
  cobra::InMotionAlignInitializationPlugin plugin("in motion");
  plugin.init_plugin(std::nullopt, &med);
  EXPECT_TRUE(plugin.is_initialization_type_supported(api::InitializationType::INERTIAL));

  cobra::InMotionAlignmentConfig c;
  c.group_ = "config/alignment";
  c.imu_model = cobra::inertial::stim300_model().to_config("config/alignment/imu");
  c.heading_sigma = 2.0 * kDeg;
  c.lever_arm = {-0.5, 0.38, -0.05};
  c.velocity_source = "position";
  c.start_time = 12.5;
  c.to_registry(med);
  const auto back = cobra::InMotionAlignmentConfig::from_registry(med, c.group_);
  ASSERT_TRUE(back);
  EXPECT_DOUBLE_EQ(back->heading_sigma, c.heading_sigma);
  EXPECT_EQ(back->lever_arm, c.lever_arm);
  EXPECT_EQ(back->velocity_source, "position");
  EXPECT_EQ(back->min_epochs, 5);
  ASSERT_TRUE(back->start_time);
  EXPECT_DOUBLE_EQ(*back->start_time, 12.5);
  // the time offset is not estimated unless a prior is given, and then both of its keys travel
  EXPECT_EQ(back->time_offset_prior_sigma, 0.0);
  EXPECT_DOUBLE_EQ(back->time_offset_sigma, 0.08);
  {
    cobra::InMotionAlignmentConfig e = c;
    e.group_ = "config/alignment_with_offset";
    e.time_offset_prior_sigma = 0.25;
    e.time_offset_sigma = 0.05;
    e.to_registry(med);
    const auto b = cobra::InMotionAlignmentConfig::from_registry(med, e.group_);
    ASSERT_TRUE(b);
    EXPECT_DOUBLE_EQ(b->time_offset_prior_sigma, 0.25);
    EXPECT_DOUBLE_EQ(b->time_offset_sigma, 0.05);
    EXPECT_TRUE(plugin.new_initialization_strategy(api::InitializationType::INERTIAL, e.group_));
    e.time_offset_prior_sigma = 0.7;  // refused when the strategy is built
    e.to_registry(med);
    EXPECT_FALSE(plugin.new_initialization_strategy(api::InitializationType::INERTIAL, e.group_));
    EXPECT_TRUE(med.has_error());
    med.logs.clear();
  }

  auto strategy = plugin.new_initialization_strategy(api::InitializationType::INERTIAL, c.group_);
  ASSERT_TRUE(strategy);
  auto* inertial = dynamic_cast<api::InertialInitializationStrategy*>(strategy.get());
  ASSERT_TRUE(inertial);
  EXPECT_EQ(inertial->request_motion_needed(), api::InitializationMotionNeeded::MOTION_NEEDED);
  EXPECT_EQ(inertial->request_current_status(), api::InitializationStatus::INITIALIZING_COARSE);
  // a velocity message is input for this strategy (the static alignments report it as an error)
  aspn23_eigen::MeasurementVelocity vel(header(ASPN_MEASUREMENT_VELOCITY), aspn23_eigen::TypeTimestamp(kT0),
                                        ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED, 1.0, 0.0, 0.0, rm(api::Matrix::Identity(3, 3)),
                                        ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE, DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});
  inertial->process_pntos_message(api::Message(std::make_shared<aspn23_eigen::MeasurementVelocity>(vel), "/velocity"));
  EXPECT_FALSE(med.has_error());
  EXPECT_FALSE(inertial->request_solution().solution);

  // an unknown velocity source is refused when the strategy is built
  c.velocity_source = "doppler";
  c.to_registry(med);
  EXPECT_FALSE(plugin.new_initialization_strategy(api::InitializationType::INERTIAL, c.group_));
  EXPECT_TRUE(med.has_error());
  // and no config at all, too
  EXPECT_FALSE(plugin.new_initialization_strategy(api::InitializationType::INERTIAL, std::string("config/none")));
}

TEST(InMotionAlignPlugin, StaticAlignmentsStillRefuseAVelocityMessage) {
  Env env;
  TestMediator& med = env.med;
  cobra::AlignmentStrategy strategy(std::make_unique<cobra::inertial::StaticAlignment>(), &med);
  aspn23_eigen::MeasurementVelocity vel(header(ASPN_MEASUREMENT_VELOCITY), aspn23_eigen::TypeTimestamp(kT0),
                                        ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED, 1.0, 0.0, 0.0, rm(api::Matrix::Identity(3, 3)),
                                        ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE, DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});
  strategy.process_pntos_message(api::Message(std::make_shared<aspn23_eigen::MeasurementVelocity>(vel), "/velocity"));
  EXPECT_TRUE(med.has_error());
  EXPECT_EQ(med.last_message(), "Could not convert message");
}

TEST(InMotionAlignPlugin, JsonRoundTripAndTheExampleConfigurations) {
  using namespace cobra;
  auto src = [](const char* rel) {
    const char* dir = std::getenv("PNTOS_TEST_SRCDIR");
    return std::string(dir ? dir : "..") + "/" + rel;
  };
  // written by tools/run_alignment_matrix.py --write-configs
  for (const char* name : {"configs/pos_ins_align_in_motion.json", "configs/pos_ins_ground_vehicle.json"}) {
    const auto app = jsoncfg::load_app_config(src(name));
    EXPECT_EQ(app.app.initialization, "in_motion_align") << name;
    const StandardOrchestrationConfig* orch = nullptr;
    for (const auto& c : app.configs)
      if (auto* o = dynamic_cast<const StandardOrchestrationConfig*>(c.get())) orch = o;
    ASSERT_NE(orch, nullptr) << name;
    const auto* align = dynamic_cast<const InMotionAlignmentConfig*>(orch->alignment_config.get());
    ASSERT_NE(align, nullptr) << name;
    EXPECT_EQ(align->velocity_source, "velocity");
    EXPECT_DOUBLE_EQ(align->heading_sigma, 3.0 * kDeg);
    EXPECT_EQ(align->lever_arm, (Vec3{-0.5, 0.38, -0.05}));
    EXPECT_FALSE(align->start_time.has_value());
    // the receiver's time offset is estimated along: nothing assumed but that it is a fraction of a second
    EXPECT_DOUBLE_EQ(align->time_offset_prior_sigma, 0.3);
    EXPECT_DOUBLE_EQ(align->time_offset_sigma, 0.08);
    // the receiver's velocity is an alignment channel and carries its own time bias
    EXPECT_NE(std::find(orch->alignment_channels.begin(), orch->alignment_channels.end(), kVelocityChannel), orch->alignment_channels.end());
    // text -> config -> text -> config is stable
    const auto j = jsoncfg::app_config_to_json(app);
    const auto again = jsoncfg::app_config_from_json(j);
    EXPECT_EQ(jsoncfg::app_config_to_json(again).dump(), j.dump()) << name;
    // and the plugin set builds
    EXPECT_NO_THROW(app::build_plugins(app)) << name;
  }
  // the ground-vehicle example has the constraints and a window that covers the controller's buffer
  {
    const auto app = jsoncfg::load_app_config(src("configs/pos_ins_ground_vehicle.json"));
    for (const auto& c : app.configs)
      if (auto* o = dynamic_cast<const StandardOrchestrationConfig*>(c.get())) EXPECT_DOUBLE_EQ(o->measurement_reorder_s, 2.5);
  }
  // values that make no sense are refused when the text is read
  std::ifstream in(src("configs/pos_ins_align_in_motion.json"));
  jsoncfg::json j;
  in >> j;
  auto with = [&](const char* key, jsoncfg::json value) {
    jsoncfg::json k = j;
    for (auto& c : k["configs"])
      if (c["type"] == "StandardOrchestrationConfig") c["alignment_config"][key] = value;
    return k;
  };
  EXPECT_THROW(jsoncfg::app_config_from_json(with("velocity_source", "doppler")), std::runtime_error);
  EXPECT_THROW(jsoncfg::app_config_from_json(with("heading_sigma", 0.0)), std::runtime_error);
  EXPECT_THROW(jsoncfg::app_config_from_json(with("min_epochs", 0)), std::runtime_error);
  EXPECT_THROW(jsoncfg::app_config_from_json(with("time_offset_prior_sigma", 0.6)), std::runtime_error);
  EXPECT_THROW(jsoncfg::app_config_from_json(with("time_offset_prior_sigma", -0.1)), std::runtime_error);
  EXPECT_THROW(jsoncfg::app_config_from_json(with("time_offset_sigma", 0.0)), std::runtime_error);
  {  // not estimated: neither key is written back, and a configuration from before the keys reads as it did
    const auto off = jsoncfg::app_config_from_json(with("time_offset_prior_sigma", 0.0));
    EXPECT_EQ(jsoncfg::app_config_to_json(off).dump().find("time_offset"), std::string::npos);
    jsoncfg::json older = j;
    for (auto& c : older["configs"])
      if (c["type"] == "StandardOrchestrationConfig") {
        c["alignment_config"].erase("time_offset_prior_sigma");
        c["alignment_config"].erase("time_offset_sigma");
      }
    EXPECT_EQ(jsoncfg::app_config_to_json(jsoncfg::app_config_from_json(older)).dump(), jsoncfg::app_config_to_json(off).dump());
  }
  const auto timed = jsoncfg::app_config_from_json(with("start_time", 12.5));
  for (const auto& c : timed.configs)
    if (auto* o = dynamic_cast<const StandardOrchestrationConfig*>(c.get())) {
      const auto* align = dynamic_cast<const InMotionAlignmentConfig*>(o->alignment_config.get());
      ASSERT_NE(align, nullptr);
      ASSERT_TRUE(align->start_time.has_value());
      EXPECT_DOUBLE_EQ(*align->start_time, 12.5);
    }
  jsoncfg::json bad = j;
  bad["app"]["initialization"] = "in_motion";
  EXPECT_THROW(app::build_plugins(jsoncfg::app_config_from_json(bad)), std::runtime_error);
}

// ----------------------------------------------------------------------------- through the whole filter

TEST(InMotionAlignFilter, ComesUpWithoutAHeadingFromOutsideAndTracks) {
  // configs/pos_ins_align_in_motion.json through the push API, on a simulated drive: stand, pull away, keep
  // winding. The preprocessors of the recording (IMU rotation, time adjustment, time biases) and its lever arm
  // are taken out; everything else is as shipped.
  const char* dir = std::getenv("PNTOS_TEST_SRCDIR");
  std::ifstream in(std::string(dir ? dir : "..") + "/configs/pos_ins_align_in_motion.json");
  cobra::jsoncfg::json j;
  in >> j;
  j["app"]["logging_level"] = "INFO";
  for (auto& c : j["configs"]) {
    if (c["type"] != "StandardOrchestrationConfig") continue;
    c["preprocessor_configs"] = cobra::jsoncfg::json::array();
    c["alignment_config"]["lever_arm"] = {0.0, 0.0, 0.0};
    for (auto& mp : c["mp_configs"]) mp["lever_arm"] = {0.0, 0.0, 0.0};
  }
  Drive d;
  d.speed = [](double t) { return 10.0 * smoothstep(t, 6.0, 14.0) + 2.0 * std::sin(0.3 * t) * smoothstep(t, 14.0, 18.0); };
  d.course = [](double t) { return 0.9 + 0.6 * smoothstep(t, 9.0, 16.0) + 0.4 * std::sin(0.2 * t) * smoothstep(t, 16.0, 20.0); };
  d.latency = 0.0;  // the controller's buffer does the waiting here
  // The receiver's velocity is a quarter of a second old and no time bias is configured for it (the preprocessors
  // are out): the alignment has to find that by itself, which the example configuration asks it to.
  d.velocity_age = 0.25;

  int errors = 0;
  std::string summary;
  cobra::utils::set_log_sink([&](api::LoggingLevel level, const std::string&, const std::string& m) {
    if (level == api::LoggingLevel::ERROR) ++errors;
    if (m.find("Aligned in motion") != std::string::npos) summary = m;
  });
  struct Row {
    double t;
    Vector3 att_error, vel_error, pos_error;
  };
  std::vector<Row> rows;
  Track track;
  {
    cobra::app::RunOptions o;
    o.progress = false;
    cobra::Filter f(cobra::jsoncfg::app_config_from_json(j), o);
    std::vector<api::Message> solutions;
    f.set_solution_callback([&](const api::Message& m) { solutions.push_back(m); });
    track = simulate(d, 75.0, [&](const std::shared_ptr<api::AspnBase>& message, const char* channel, double) {
      f.push(message, channel);
      return true;
    });
    f.stop();
    for (const auto& m : solutions) {
      const auto pva = m.as<cobra::utils::PVA>();
      if (!pva) continue;
      const double t = (pva->get_time_of_validity().get_elapsed_nsec() - kT0) * 1e-9;
      const Vector3 p(nav::delta_lat_to_north(pva->get_p1() - d.lat, d.lat, d.alt), nav::delta_lon_to_east(pva->get_p2() - d.lon, d.lat, d.alt),
                      -(pva->get_p3() - d.alt));
      rows.push_back({t, attitude_error(nav::quat_to_dcm(api::Vector(pva->get_quaternion())), d.attitude(t)),
                      Vector3(Vector3(pva->get_v1(), pva->get_v2(), pva->get_v3()) - d.velocity(t)), Vector3(p - position_at(track, t))});
    }
  }
  cobra::utils::set_log_sink(nullptr);

  EXPECT_EQ(errors, 0);
  EXPECT_FALSE(summary.empty()) << "the alignment says what it found";
  EXPECT_NE(summary.find("the receiver's velocity is 0."), std::string::npos) << summary;
  EXPECT_NE(summary.find("s older than its time stamps"), std::string::npos) << summary;
  ASSERT_GE(rows.size(), 30u);
  // the first solution: the alignment handed over within 20 s of moving off (6 s) and a 2 s buffer
  EXPECT_LT(rows.front().t, 28.0) << summary;
  EXPECT_GT(rows.front().t, 8.0);
  EXPECT_LT(std::abs(rows.front().att_error(2)), 3.0 * kDeg) << summary;
  EXPECT_LT(rows.front().pos_error.norm(), 2.0) << rows.front().pos_error.transpose();
  // and the filter carries on from there
  double heading = 0, tilt = 0, pos = 0, vel = 0;
  int n = 0;
  for (const auto& r : rows) {
    if (r.t < rows.front().t + 20.0) continue;
    heading = std::max(heading, std::abs(r.att_error(2)));
    tilt = std::max(tilt, r.att_error.head<2>().norm());
    pos = std::max(pos, r.pos_error.norm());
    vel = std::max(vel, r.vel_error.norm());
    ++n;
  }
  ASSERT_GE(n, 20);
  std::cout << "first solution at " << rows.front().t << " s, heading error " << rows.front().att_error(2) / kDeg << " deg; from 20 s later, worst: heading "
            << heading / kDeg << " deg, tilt " << tilt / kDeg << " deg, position " << pos << " m, velocity " << vel << " m/s\n"
            << summary << "\n";
  EXPECT_LT(heading, 3.0 * kDeg) << "worst heading error from 20 s after the hand-over: " << heading / kDeg << " deg; " << summary;
  EXPECT_LT(tilt, 0.6 * kDeg) << tilt / kDeg;
  EXPECT_LT(pos, 2.0) << pos;
  EXPECT_LT(vel, 0.4) << vel;
}
