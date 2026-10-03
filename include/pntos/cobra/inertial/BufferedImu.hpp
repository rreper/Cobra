// Eigen port of NavToolkit's BufferedPva/BufferedImu: a mechanizing inertial with time-indexed
// ring buffers of IMU input, solutions and error-model resets, supporting interpolation at any
// buffered time, resets with re-propagation, and force/rate queries.
#pragma once

#include <pntos/cobra/inertial/Mechanization.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>

#include <deque>
#include <memory>

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

/// Linear interpolation of a PVA between two buffered solutions (RPY slerp for attitude).
std::shared_ptr<utils::PVA> linear_interp_pva(const utils::PVA& a, const utils::PVA& b, Timestamp t);
/// Build a zero-covariance geodetic PVA message from a mechanization solution.
std::shared_ptr<utils::PVA> to_pva_message(const StandardPva& s);
StandardPva to_standard_pva(const utils::PVA& pva);
/// A SAMPLED MeasurementImu carrying specific force (NED) and rotation rate (sensor).
std::shared_ptr<Imu> to_force_rate_message(Timestamp t, const Vector3& force, const Vector3& rate);

class BufferedImu {
 public:
  BufferedImu(const utils::PVA& initial, double expected_dt = 0.01, double buffer_length = 60.0,
              const ImuErrors& errors = {}, const MechanizationOptions& opts = {});

  /// Mechanize one integrated IMU message (type must be INTEGRATED). Returns false if rejected.
  bool add(std::shared_ptr<Imu> imu);

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
  bool reset(const utils::PVA* pva, const ImuErrors* errors, const utils::PVA* previous = nullptr);

  double estimated_dt() const { return num_dt_ > 10 ? dt_sum_ / num_dt_ : expected_dt_; }

 private:
  std::pair<std::int64_t, std::int64_t> nsec_span() const;
  std::shared_ptr<utils::PVA> calc_pva_no_check(Timestamp t) const;
  void reset_ins(Inertial& ins, const utils::PVA& pva, const ImuErrors& e, const utils::PVA* old) const;

  Inertial ins_;
  TimestampedRing<utils::PVA> pva_buf_;
  TimestampedRing<Imu> imu_buf_;
  TimestampedRing<ImuErrors> err_buf_;
  double expected_dt_;
  double dt_sum_ = 0;
  long num_dt_ = 0;
};

}  // namespace pntos::cobra::inertial
