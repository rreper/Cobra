#include <pntos/cobra/inertial/Stationarity.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pntos::cobra::inertial {

using api::Vector3;

StationarityDetector::StationarityDetector(StationarityOptions options) : opt_(options) {
  if (!(opt_.sub_window > 0) || !(opt_.window >= 2 * opt_.sub_window) || !(opt_.gyro_deviation > 0) ||
      !(opt_.accel_deviation > 0) || !(opt_.max_rate > 0))
    throw std::invalid_argument("StationarityDetector: sub_window, gyro_deviation, accel_deviation and max_rate must be > 0 "
                                "and window at least two sub-windows");
  needed_ = static_cast<std::size_t>(std::ceil(opt_.window / opt_.sub_window - 1e-9));
}

void StationarityDetector::reset() {
  subs_.clear();
  sub_dt_ = 0;
  sub_dv_.setZero();
  sub_dth_.setZero();
  have_last_ = false;
  stationary_ = false;
}

double StationarityDetector::quiet_seconds() const { return stationary_ ? (end_ns_ - quiet_since_ns_) * 1e-9 : 0.0; }

bool StationarityDetector::add(std::int64_t t_ns, double dt, const Vector3& dv, const Vector3& dth) {
  // A sample that does not follow the last one (a gap, a step back, nonsense) ends what was known.
  if (!(dt > 0) || !dv.allFinite() || !dth.allFinite() || dt > 2 * opt_.sub_window || (have_last_ && t_ns <= last_ns_)) {
    reset();
    return false;
  }
  const std::int64_t dt_ns = static_cast<std::int64_t>(std::llround(dt * 1e9));
  if (have_last_ && std::llabs((t_ns - dt_ns) - last_ns_) > dt_ns / 2) reset();  // the interval does not adjoin the last
  if (sub_dt_ == 0) sub_begin_ = t_ns - dt_ns;
  sub_dv_ += dv;
  sub_dth_ += dth;
  sub_dt_ += dt;
  last_ns_ = t_ns;
  have_last_ = true;
  if (sub_dt_ < opt_.sub_window - 1e-9) return stationary_;

  subs_.push_back(Sub{sub_begin_, t_ns, Vector3(sub_dth_ / sub_dt_), Vector3(sub_dv_ / sub_dt_)});
  sub_dt_ = 0;
  sub_dv_.setZero();
  sub_dth_.setZero();
  while (subs_.size() > needed_) subs_.pop_front();
  evaluate();
  return stationary_;
}

void StationarityDetector::evaluate() {
  const bool was = stationary_;
  stationary_ = false;
  if (subs_.size() < needed_) return;
  Vector3 rate = Vector3::Zero(), force = Vector3::Zero();
  for (const auto& s : subs_) {
    rate += s.rate;
    force += s.force;
  }
  rate /= static_cast<double>(subs_.size());
  force /= static_cast<double>(subs_.size());
  if (rate.norm() >= opt_.max_rate) return;
  for (const auto& s : subs_) {
    if ((s.rate - rate).cwiseAbs().maxCoeff() >= opt_.gyro_deviation) return;
    if ((s.force - force).cwiseAbs().maxCoeff() >= opt_.accel_deviation) return;
  }
  stationary_ = true;
  begin_ns_ = subs_.front().begin;
  end_ns_ = subs_.back().end;
  mean_rate_ = rate;
  mean_force_ = force;
  if (!was) quiet_since_ns_ = begin_ns_;
}

}  // namespace pntos::cobra::inertial
