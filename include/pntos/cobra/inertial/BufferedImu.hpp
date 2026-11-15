// Eigen port of NavToolkit's BufferedPva/BufferedImu: a mechanizing inertial with time-indexed
// ring buffers of IMU input, solutions and error-model resets, supporting interpolation at any
// buffered time, resets with re-propagation, and force/rate queries.
#pragma once

#include <pntos/cobra/inertial/ImuRobustness.hpp>
#include <pntos/cobra/inertial/Mechanization.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>

#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

namespace pntos::cobra::inertial {

/// Time-sorted bounded buffer (NavToolkit `OrderedRing` + `TimestampedDataSeries`). When full, a
/// newly inserted element evicts the oldest one; an element older than the oldest is rejected.
template <class T>
class TimestampedRing {
 public:
  using Ptr = std::shared_ptr<T>;
  using TimeFn = std::int64_t (*)(const T&);
  TimestampedRing(std::size_t capacity, TimeFn time) : capacity_(capacity), time_(time) {}

  bool insert(Ptr item) {
    const std::int64_t t = time_(*item);
    if (buf_.size() >= capacity_ && !buf_.empty() && t < time_(*buf_.front())) return false;
    auto pos = std::upper_bound(buf_.begin(), buf_.end(), t, [&](std::int64_t a, const Ptr& b) { return a < time_(*b); });
    buf_.insert(pos, std::move(item));
    if (buf_.size() > capacity_) buf_.pop_front();
    return true;
  }
  /// (before, after): both equal on an exact match; before == end() if t precedes everything; after == end()
  /// if t follows everything.
  std::pair<std::size_t, std::size_t> nearest(std::int64_t t) const {
    const std::size_t n = buf_.size();
    if (n == 0) return {n, n};
    auto upper = std::upper_bound(buf_.begin(), buf_.end(), t, [&](std::int64_t a, const Ptr& b) { return a < time_(*b); });
    if (upper == buf_.begin()) return {n, 0};
    std::size_t lower = static_cast<std::size_t>(upper - buf_.begin()) - 1;
    std::size_t up = static_cast<std::size_t>(upper - buf_.begin());
    if (time_(*buf_[lower]) == t) up = lower;
    return {lower, up};
  }
  /// [first, last) indices of elements with t0 <= time <= t1 (empty => {n, n}).
  std::pair<std::size_t, std::size_t> in_range(std::int64_t t0, std::int64_t t1) const {
    auto lower = std::lower_bound(buf_.begin(), buf_.end(), t0, [&](const Ptr& a, std::int64_t b) { return time_(*a) < b; });
    auto upper = std::upper_bound(lower, buf_.end(), t1, [&](std::int64_t a, const Ptr& b) { return a < time_(*b); });
    if (lower == upper) return {buf_.size(), buf_.size()};
    return {static_cast<std::size_t>(lower - buf_.begin()), static_cast<std::size_t>(upper - buf_.begin())};
  }
  void erase(std::size_t first, std::size_t last) { buf_.erase(buf_.begin() + first, buf_.begin() + last); }
  bool valid(std::size_t i) const { return i < buf_.size() && buf_[i] != nullptr; }
  const Ptr& at(std::size_t i) const { return buf_[i]; }
  const Ptr& front() const { return buf_.front(); }
  const Ptr& back() const { return buf_.back(); }
  std::size_t size() const { return buf_.size(); }
  bool empty() const { return buf_.empty(); }
  bool full() const { return buf_.size() >= capacity_; }
  std::int64_t time_at(std::size_t i) const { return time_(*buf_[i]); }

 private:
  std::size_t capacity_;
  TimeFn time_;
  std::deque<Ptr> buf_;
};

using Imu = aspn23_eigen::MeasurementImu;

/// Linear interpolation of a PVA between two buffered solutions. Attitude: NavToolkit's slerp through
/// roll/pitch/yaw by default, which loses the attitude within about 1e-6 rad of pitch = +-90 degrees;
/// AttitudeInterpolation::QUATERNION slerps the quaternions directly. A solution without a quaternion is
/// taken as level and north-pointing, as to_standard_pva does.
std::shared_ptr<utils::PVA> linear_interp_pva(const utils::PVA& a, const utils::PVA& b, Timestamp t,
                                              AttitudeInterpolation attitude = AttitudeInterpolation::RPY);
/// Build a zero-covariance geodetic PVA message from a mechanization solution.
std::shared_ptr<utils::PVA> to_pva_message(const StandardPva& s);
StandardPva to_standard_pva(const utils::PVA& pva);
/// A SAMPLED MeasurementImu carrying specific force (NED) and rotation rate (sensor).
std::shared_ptr<Imu> to_force_rate_message(Timestamp t, const Vector3& force, const Vector3& rate);

class BufferedImu {
 public:
  /// Outcome of add_sample(). Only ACCEPTED and NOT_NEWER can occur with the default ImuRobustness.
  enum class AddStatus {
    ACCEPTED,          ///< mechanized as received
    GAP_FILLED,        ///< mechanized; it arrived late and its rate was held across the gap (HOLD, or SAMPLED data)
    NOT_NEWER,         ///< rejected: not after the latest solution (duplicate or out-of-order time stamp)
    SAMPLED_DROPPED,   ///< rejected: not INTEGRATED and SampledImuPolicy::DROP
    NON_FINITE,        ///< rejected: NaN or infinity in the measurement (reject_non_finite)
    GAP_TOO_LONG,      ///< rejected: more than max_gap ahead of the latest solution and not (yet) confirmed
  };
  static bool accepted(AddStatus s) { return s == AddStatus::ACCEPTED || s == AddStatus::GAP_FILLED; }

  /// Counters since construction, for health reporting.
  struct Stats {
    std::uint64_t accepted = 0;         ///< samples mechanized (including gap_filled and sampled_integrated)
    std::uint64_t gap_filled = 0;       ///< late samples whose rate was held across the gap
    std::uint64_t sampled_integrated = 0;
    std::uint64_t not_newer = 0;
    std::uint64_t sampled_dropped = 0;
    std::uint64_t non_finite = 0;       ///< IMU samples rejected for NaN / infinity
    std::uint64_t gap_too_long = 0;
    std::uint64_t resets_rejected = 0;  ///< reset() calls refused (out of range, or non-finite with reject_non_finite)
    double longest_gap = 0;             ///< s, longest interval between two accepted samples
  };

  /// Throws std::invalid_argument unless expected_dt and buffer_length are positive and finite, the
  /// robustness values are in range (gap_factor > 1, max_gap >= 0, dt_window 0 or >= 3) and, with
  /// robustness.reject_non_finite, `initial` holds a finite position and velocity and a unit quaternion.
  BufferedImu(const utils::PVA& initial, double expected_dt = 0.01, double buffer_length = 60.0,
              const ImuErrors& errors = {}, const MechanizationOptions& opts = {},
              const ImuRobustness& robustness = {});

  /// Mechanize one IMU message. Returns false if rejected; throws std::invalid_argument for a null message
  /// and, unless ImuRobustness::sampled_imu says otherwise, for one that is not INTEGRATED.
  bool add(std::shared_ptr<Imu> imu) { return accepted(add_sample(std::move(imu))); }
  /// As add(), telling why a sample was rejected or how it was used.
  AddStatus add_sample(std::shared_ptr<Imu> imu);
  const Stats& stats() const { return stats_; }
  const ImuRobustness& robustness() const { return robustness_; }

  std::shared_ptr<utils::PVA> calc_pva(Timestamp t) const;  ///< nullptr if out of range
  std::shared_ptr<utils::PVA> latest_pva() const;
  /// Solution at `t` as if no reset had happened after `since`.
  std::shared_ptr<utils::PVA> calc_pva_no_reset_since(Timestamp t, Timestamp since) const;

  std::pair<Timestamp, Timestamp> time_span() const;
  bool in_range(Timestamp t) const;

  std::shared_ptr<Imu> calc_force_and_rate(Timestamp t) const;
  std::shared_ptr<Imu> calc_force_and_rate(Timestamp t1, Timestamp t2) const;

  ImuErrors imu_errors(Timestamp t) const;

  /// Reset the solution and/or the error model at the time of `pva` (or `errors.time`), then
  /// re-mechanize the buffered IMU data from there. Returns false if the time is out of range.
  /// With ImuRobustness::reject_non_finite a solution or error set that is not finite (or a solution
  /// without position, velocity or a unit quaternion) is refused as well.
  bool reset(const utils::PVA* pva, const ImuErrors* errors, const utils::PVA* previous = nullptr);

  /// Nominal interval between IMU samples: expected_dt until more than 10 intervals were seen, then their
  /// mean since construction (NavToolkit) or, with ImuRobustness::dt_window, the median of the last N.
  double estimated_dt() const;

 private:
  /// How one buffered record is mechanized up to its time stamp: `steps` equal sub-steps of dv / dth each.
  struct Plan {
    Vector3 dv, dth;
    int steps = 1;      ///< more than one only when a rate is held across a gap
    bool held = false;  ///< the record arrived late and stands for the samples lost before it as well
  };
  std::pair<std::int64_t, std::int64_t> nsec_span() const;
  std::shared_ptr<utils::PVA> calc_pva_no_check(Timestamp t) const;
  void reset_ins(Inertial& ins, const utils::PVA& pva, const ImuErrors& e, const utils::PVA* old) const;
  bool reset_checked(const utils::PVA* pva, const ImuErrors* errors, const utils::PVA* previous);
  /// How buffered record `imu` is mechanized over the `dt` seconds since the record before it. The one
  /// place that decides how a record is used, shared by add_sample() and the replays of reset() and
  /// calc_pva_no_reset_since(), so that a replay bridges a gap exactly as the first pass did.
  Plan plan(const Imu& imu, double dt) const;
  /// The same for the part of the interval of record `index` that follows a reset at `from_nsec`. False if
  /// nothing is left to mechanize (`min_scale` is NavToolkit's cut-off, which differs between the callers).
  bool plan_after_reset(std::size_t index, std::int64_t from_nsec, double min_scale, Plan& out) const;
  bool late(double dt) const;
  static void run(Inertial& ins, std::int64_t to_nsec, const Plan& plan);
  bool confirmed_time_jump(std::int64_t t);
  void note_interval(double dt);

  Inertial ins_;
  TimestampedRing<utils::PVA> pva_buf_;
  TimestampedRing<Imu> imu_buf_;
  TimestampedRing<ImuErrors> err_buf_;
  double expected_dt_;
  double dt_sum_ = 0;
  long num_dt_ = 0;
  ImuRobustness robustness_;
  Stats stats_;
  // windowed dt estimate (ImuRobustness::dt_window, or kDefaultDtWindow for ImuGapPolicy::HOLD)
  std::size_t dt_window_ = 0;
  std::vector<double> recent_dt_;
  std::size_t recent_next_ = 0;
  std::vector<double> median_scratch_;
  double median_dt_ = 0;
  // samples refused for being more than max_gap ahead, while they keep agreeing with each other
  std::int64_t pending_jump_nsec_ = 0;
  int pending_jumps_ = 0;
};

}  // namespace pntos::cobra::inertial
