// A simulated drive for tests that need a vehicle rather than a recording: level ground, speed and course as smooth
// functions of time, an IMU with bias and noise, and a receiver that delivers the antenna's velocity and position
// once a second, `latency` seconds late and, if asked, older than stamped. Truth is known exactly, on one clock.
#pragma once

#include <pntos/api/types.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include "test_support.hpp"

#include <aspn23/eigen/MeasurementImu.hpp>
#include <aspn23/eigen/MeasurementVelocity.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <utility>
#include <vector>

namespace pntos::test::sim {

using api::Matrix3;
using api::Vector3;
namespace nav = cobra::nav;

inline constexpr double kDeg = 0.017453292519943295;
inline constexpr std::int64_t kT0 = 1'700'000'000'000'000'000;  // epoch-sized time stamps, as a receiver gives them

inline double smoothstep(double t, double t0, double t1) {
  if (t <= t0) return 0.0;
  if (t >= t1) return 1.0;
  const double u = (t - t0) / (t1 - t0);
  return u * u * (3.0 - 2.0 * u);
}

inline Vector3 rotation_vector(const Matrix3& R) {
  const Vector3 v(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  const double s = 0.5 * v.norm();
  if (s < 1e-12) return 0.5 * v;
  const double angle = std::atan2(s, 0.5 * (R.trace() - 1.0));
  return v * (0.5 * angle / s);
}

/// Rotation error of `est` against `truth` (both body to NED), as a vector in NED: (north, east, down) tilts.
inline Vector3 attitude_error(const Matrix3& est, const Matrix3& truth) { return rotation_vector(est * truth.transpose()); }

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

inline constexpr const char* kImuChannel = "/sensor/vn-100/imu";
inline constexpr const char* kPositionChannel = "/sensor/ublox-ZED-F9T/position";
inline constexpr const char* kVelocityChannel = "/sensor/ublox-ZED-F9T/velocity";

using Track = std::vector<std::pair<double, Vector3>>;  ///< the IMU's position (NED from the start) at every sample time
/// Takes a message at its arrival time (s of the drive); false ends the drive.
using Sink = std::function<bool(const std::shared_ptr<api::AspnBase>&, const char* channel, double arrival)>;

/// Plays the drive: IMU samples as they are taken, the receiver's fixes `latency` s after their time.
inline Track simulate(const Drive& d, double duration, const Sink& sink) {
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

inline Vector3 position_at(const Track& track, double t) {
  const auto it = std::lower_bound(track.begin(), track.end(), t - 1e-9, [](const auto& a, double at) { return a.first < at; });
  return it == track.end() ? track.back().second : it->second;
}

/// Stands for `rest` s, then pulls away to 10 m/s in a right-hand bend.
inline Drive pull_away(double heading, double rest = 6.0) {
  Drive d;
  d.speed = [rest](double t) { return 10.0 * smoothstep(t, rest, rest + 8.0); };
  d.course = [heading, rest](double t) { return heading + 0.6 * smoothstep(t, rest + 3.0, rest + 10.0); };
  return d;
}

/// Already under way when switched on: the speed swings by 3 m/s and the road winds.
inline Drive under_way(double heading) {
  Drive d;
  d.speed = [](double t) { return 12.0 + 3.0 * std::sin(0.4 * t); };
  d.course = [heading](double t) { return heading + 0.5 * std::sin(0.25 * t); };
  return d;
}

}  // namespace pntos::test::sim
