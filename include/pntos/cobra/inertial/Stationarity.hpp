// Whether a vehicle stands, from its IMU alone (C++ addition; DESIGN.md section 7.25).
//
// Sample by sample a standing car with its engine running is not quieter than a driving one: on the example
// recording the largest gyro deviation within a second is 0.010 rad/s at rest (median) and 0.007 rad/s in the
// quietest second of driving. Averaged over 0.2 s the vibration is gone and what remains is the vehicle's own
// motion: over a window of 2 s the 0.2 s means stay within 0.003 rad/s and 0.15 m/s^2 of their mean in 95 % of
// the standing windows and in none of 1030 windows above 0.5 m/s.
#pragma once

#include <pntos/api/types.hpp>

#include <cstdint>
#include <deque>

namespace pntos::cobra::inertial {

struct StationarityOptions {
  double window = 2.0;            ///< s the IMU has to stay quiet
  double sub_window = 0.2;        ///< s the samples are averaged over before they are compared
  double gyro_deviation = 0.003;  ///< rad/s; largest deviation of a sub-window mean from the window mean, per axis
  double accel_deviation = 0.15;  ///< m/s^2; the same for the specific force
  double max_rate = 0.02;         ///< rad/s; largest norm of the window's mean rate (the gyro bias has to fit under it)
};

class StationarityDetector {
 public:
  explicit StationarityDetector(StationarityOptions options = {});

  /// Adds the increments (m/s, rad) of the interval (t - dt, t]. Returns stationary().
  bool add(std::int64_t t_ns, double dt, const api::Vector3& dv, const api::Vector3& dth);
  /// Forgets everything (a gap in the data).
  void reset();

  /// The last `window` seconds were quiet.
  bool stationary() const { return stationary_; }
  /// Begin and end of the quiet window; meaningful while stationary().
  std::int64_t window_begin_ns() const { return begin_ns_; }
  std::int64_t window_end_ns() const { return end_ns_; }
  /// Mean rate (rad/s) and specific force (m/s^2) over that window.
  const api::Vector3& mean_rate() const { return mean_rate_; }
  const api::Vector3& mean_specific_force() const { return mean_force_; }
  /// How long the IMU has been quiet without interruption, s (0 when not stationary).
  double quiet_seconds() const;

  const StationarityOptions& options() const { return opt_; }

 private:
  struct Sub {
    std::int64_t begin, end;
    api::Vector3 rate, force;
  };
  void evaluate();

  StationarityOptions opt_;
  std::size_t needed_;
  std::deque<Sub> subs_;
  // the sub-window being filled
  std::int64_t sub_begin_ = 0, last_ns_ = 0;
  double sub_dt_ = 0;
  api::Vector3 sub_dv_ = api::Vector3::Zero(), sub_dth_ = api::Vector3::Zero();
  bool have_last_ = false;

  bool stationary_ = false;
  std::int64_t begin_ns_ = 0, end_ns_ = 0, quiet_since_ns_ = 0;
  api::Vector3 mean_rate_ = api::Vector3::Zero(), mean_force_ = api::Vector3::Zero();
};

}  // namespace pntos::cobra::inertial
