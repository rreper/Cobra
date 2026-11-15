#include <pntos/cobra/inertial/BufferedImu.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pntos::cobra::inertial {

namespace {
std::int64_t pva_time(const utils::PVA& p) { return p.get_time_of_validity().get_elapsed_nsec(); }
std::int64_t imu_time(const Imu& m) { return m.get_time_of_validity().get_elapsed_nsec(); }
std::int64_t err_time(const ImuErrors& e) { return e.time.elapsed_nsec; }
Vector3 v3(const Eigen::Matrix<double, Eigen::Dynamic, 1>& v) { return Vector3(v(0), v(1), v(2)); }
Eigen::Matrix<double, Eigen::Dynamic, 1> dynv(const Vector3& v) { return Eigen::Matrix<double, Eigen::Dynamic, 1>(v); }
constexpr double kVerySmall = 1e-20;
/// A rate held across a gap of `dt` seconds is mechanized in steps of about one nominal interval: at most
/// this many of them, and none shorter than a microsecond (the steps must stay distinct in nanoseconds).
constexpr double kMaxSubSteps = 1000;
int sub_steps(double intervals, double dt) {
  const double most = std::max(1.0, std::min(kMaxSubSteps, std::floor(dt * 1e6)));
  return static_cast<int>(std::clamp(std::round(intervals), 1.0, most));
}
const char* const kOnlyIntegrated = "Only ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED is supported by BufferedImu.";
bool is_rates(const Imu& m) { return m.get_imu_type() != ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED; }

/// The attitude assumed for a solution that carries none (to_standard_pva mechanizes from the same one).
nav::Vector4 level_quaternion() { return nav::Vector4(1.0, 0.0, 0.0, 0.0); }

/// A solution the inertial can mechanize from: finite position and velocity, and a unit quaternion.
bool usable(const utils::PVA& p) {
  if (!utils::position(p).allFinite() || !utils::velocity(p).allFinite()) return false;
  const auto q = utils::quaternion(p);
  return q && q->allFinite() && std::abs(q->norm() - 1.0) < 1e-2;
}
bool usable(const ImuErrors& e) {
  return e.accel_biases.allFinite() && e.gyro_biases.allFinite() && e.accel_scale_factors.allFinite() &&
         e.gyro_scale_factors.allFinite();
}

/// buffer_length / expected_dt + 2 entries, as NavToolkit sizes its rings.
std::size_t ring_capacity(double buffer_length, double expected_dt) {
  if (!(expected_dt > 0) || !std::isfinite(expected_dt))
    throw std::invalid_argument("BufferedImu: expected_dt must be a positive, finite number of seconds");
  if (!(buffer_length > 0) || !std::isfinite(buffer_length))
    throw std::invalid_argument("BufferedImu: buffer_length must be a positive, finite number of seconds");
  const double entries = buffer_length / expected_dt + 2;
  if (!(entries < 1e9))
    throw std::invalid_argument("BufferedImu: buffer_length / expected_dt asks for more than 1e9 buffered samples");
  return static_cast<std::size_t>(entries);
}

/// Shortest-arc spherical interpolation from q1 (f = 0) to q2 (f = 1).
nav::Vector4 slerp(nav::Vector4 q1, nav::Vector4 q2, double f) {
  q1 = q1.norm() > 0 ? nav::quat_norm(q1) : level_quaternion();
  q2 = q2.norm() > 0 ? nav::quat_norm(q2) : level_quaternion();
  double d = q1.dot(q2);
  if (d < 0) {
    q2 = -q2;
    d = -d;
  }
  const nav::Vector4 perp = q2 - q1 * d;  // the part of q2 orthogonal to q1; its norm is the sine of the angle
  const double s = perp.norm();
  if (s < 1e-12) return nav::quat_norm(q1 + (q2 - q1) * f);
  const double angle = std::atan2(s, d) * f;
  return nav::quat_norm(q1 * std::cos(angle) + perp * (std::sin(angle) / s));
}

Vector3 extrapolate_rpy(std::int64_t t1, const Vector3& rpy1, std::int64_t t2, const Vector3& rpy2, std::int64_t t) {
  const double tq = static_cast<double>(t - t1) / static_cast<double>(t2 - t1);
  nav::Vector4 q1 = nav::rpy_to_quat(rpy1), q2 = nav::rpy_to_quat(rpy2);
  double d = q1.dot(q2);
  if (d < 0) {
    q2 = -q2;
    d = -d;
  }
  if (d > 0.999) return nav::quat_to_rpy(nav::quat_norm(q1 + (q2 - q1) * tq));
  const double theta = std::acos(d) * tq;
  nav::Vector4 v2 = nav::quat_norm(q2 - q1 * d);
  nav::Vector4 q = nav::quat_norm(q1 * std::cos(theta) + v2 * std::sin(theta));
  return nav::dcm_to_rpy(nav::ortho_dcm(nav::quat_to_dcm(q)));
}
}  // namespace

std::shared_ptr<utils::PVA> linear_interp_pva(const utils::PVA& a, const utils::PVA& b, Timestamp t,
                                              AttitudeInterpolation attitude) {
  const std::int64_t t1 = pva_time(a), t2 = pva_time(b);
  if (t1 > t2) return linear_interp_pva(b, a, t, attitude);
  if (t.elapsed_nsec <= t1) return utils::copy_pva(a);
  if (t.elapsed_nsec >= t2 || t1 == t2) return utils::copy_pva(b);
  const double f = static_cast<double>(t.elapsed_nsec - t1) / static_cast<double>(t2 - t1);
  Vector3 llh = utils::position(a) + (utils::position(b) - utils::position(a)) * f;
  Vector3 vel = utils::velocity(a) + (utils::velocity(b) - utils::velocity(a)) * f;
  const nav::Vector4 qa = utils::quaternion(a).value_or(level_quaternion());
  const nav::Vector4 qb = utils::quaternion(b).value_or(level_quaternion());
  nav::Vector4 q;
  if (attitude == AttitudeInterpolation::QUATERNION) {
    q = slerp(qa, qb, f);
  } else {
    Vector3 rpy = extrapolate_rpy(t1, nav::quat_to_rpy(qa), t2, nav::quat_to_rpy(qb), t.elapsed_nsec);
    q = nav::rpy_to_quat(rpy);
  }
  auto out = utils::make_pva(a.get_header(), t, llh, vel, q, api::Matrix(a.get_covariance()));
  out->set_reference_frame(a.get_reference_frame());
  return out;
}

std::shared_ptr<utils::PVA> to_pva_message(const StandardPva& s) {
  return utils::make_pva(aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0), s.time, s.llh,
                         s.vned, nav::dcm_to_quat(s.C_s_to_n), api::Matrix::Zero(9, 9));
}

StandardPva to_standard_pva(const utils::PVA& p) {
  StandardPva s;
  s.time = Timestamp{pva_time(p)};
  s.llh = utils::position(p);
  s.vned = utils::velocity(p);
  auto q = utils::quaternion(p);
  s.C_s_to_n = q ? nav::quat_to_dcm(*q) : Matrix3::Identity();
  return s;
}

std::shared_ptr<Imu> to_force_rate_message(Timestamp t, const Vector3& force, const Vector3& rate) {
  return std::make_shared<Imu>(aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0),
                               aspn23_eigen::TypeTimestamp(t.elapsed_nsec), ASPN23_MEASUREMENT_IMU_IMU_TYPE_SAMPLED,
                               dynv(force), dynv(rate), std::vector<aspn23_eigen::TypeIntegrity>{});
}

// ----------------------------------------------------------------------------- ImuRobustness names

const char* to_string(SampledImuPolicy p) {
  switch (p) {
    case SampledImuPolicy::THROW: return "throw";
    case SampledImuPolicy::DROP: return "drop";
    case SampledImuPolicy::INTEGRATE: return "integrate";
  }
  return "throw";
}
const char* to_string(ImuGapPolicy p) { return p == ImuGapPolicy::HOLD ? "hold" : "navtk"; }
const char* to_string(AttitudeInterpolation p) { return p == AttitudeInterpolation::QUATERNION ? "quaternion" : "rpy"; }

std::optional<SampledImuPolicy> sampled_imu_policy_from_string(const std::string& s) {
  if (s == "throw") return SampledImuPolicy::THROW;
  if (s == "drop") return SampledImuPolicy::DROP;
  if (s == "integrate") return SampledImuPolicy::INTEGRATE;
  return std::nullopt;
}
std::optional<ImuGapPolicy> imu_gap_policy_from_string(const std::string& s) {
  if (s == "navtk") return ImuGapPolicy::NAVTK;
  if (s == "hold") return ImuGapPolicy::HOLD;
  return std::nullopt;
}
std::optional<AttitudeInterpolation> attitude_interpolation_from_string(const std::string& s) {
  if (s == "rpy") return AttitudeInterpolation::RPY;
  if (s == "quaternion") return AttitudeInterpolation::QUATERNION;
  return std::nullopt;
}

// ----------------------------------------------------------------------------- BufferedImu

BufferedImu::BufferedImu(const utils::PVA& initial, double expected_dt, double buffer_length, const ImuErrors& errors,
                         const MechanizationOptions& opts, const ImuRobustness& robustness)
    : ins_(to_standard_pva(initial), opts),
      pva_buf_(ring_capacity(buffer_length, expected_dt), &pva_time),
      imu_buf_(ring_capacity(buffer_length, expected_dt), &imu_time),
      err_buf_(ring_capacity(buffer_length, expected_dt), &err_time),
      expected_dt_(expected_dt),
      robustness_(robustness) {
  if (!(robustness_.gap_factor > 1.0) || !std::isfinite(robustness_.gap_factor))
    throw std::invalid_argument("BufferedImu: gap_factor must be greater than 1 (late means beyond gap_factor x dt)");
  if (!(robustness_.max_gap >= 0.0) || !std::isfinite(robustness_.max_gap))
    throw std::invalid_argument("BufferedImu: max_gap must be zero (unlimited) or a positive number of seconds");
  if (robustness_.dt_window > 0 && robustness_.dt_window < ImuRobustness::kMinDtWindow)
    throw std::invalid_argument("BufferedImu: dt_window must be 0 (mean since start) or at least 3 samples");
  if (robustness_.reject_non_finite && (!usable(initial) || !usable(errors)))
    throw std::invalid_argument("BufferedImu: the initial solution or IMU errors are not finite");
  // Holding a rate across a gap needs a nominal interval that the gaps themselves cannot move.
  dt_window_ = robustness_.dt_window;
  if (dt_window_ == 0 && robustness_.gap_policy == ImuGapPolicy::HOLD) dt_window_ = ImuRobustness::kDefaultDtWindow;
  recent_dt_.reserve(dt_window_);
  median_scratch_.reserve(dt_window_);
  pva_buf_.insert(utils::copy_pva(initial));
  ImuErrors e = errors;
  e.time = Timestamp{pva_time(initial)};
  err_buf_.insert(std::make_shared<ImuErrors>(e));
  imu_buf_.insert(std::make_shared<Imu>(aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0),
                                        initial.get_time_of_validity(), ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED,
                                        dynv(Vector3::Zero()), dynv(Vector3::Zero()),
                                        std::vector<aspn23_eigen::TypeIntegrity>{}));
  ins_.set_imu_errors(errors);
}

std::pair<std::int64_t, std::int64_t> BufferedImu::nsec_span() const {
  if (pva_buf_.empty()) return {std::numeric_limits<int>::lowest(), std::numeric_limits<int>::lowest()};
  // When the ring is full the oldest entry may be partially overwritten history; NavToolkit excludes it.
  const std::size_t first = pva_buf_.full() ? 1 : 0;
  return {pva_buf_.time_at(first), pva_buf_.time_at(pva_buf_.size() - 1)};
}

std::pair<Timestamp, Timestamp> BufferedImu::time_span() const {
  auto s = nsec_span();
  return {Timestamp{s.first}, Timestamp{s.second}};
}

bool BufferedImu::in_range(Timestamp t) const {
  auto s = nsec_span();
  return t.elapsed_nsec >= s.first && t.elapsed_nsec <= s.second;
}

std::shared_ptr<utils::PVA> BufferedImu::calc_pva_no_check(Timestamp t) const {
  auto [lo, hi] = pva_buf_.nearest(t.elapsed_nsec);
  if (pva_buf_.valid(lo) && pva_buf_.valid(hi))
    return linear_interp_pva(*pva_buf_.at(lo), *pva_buf_.at(hi), t, robustness_.attitude_interpolation);
  return nullptr;
}

std::shared_ptr<utils::PVA> BufferedImu::calc_pva(Timestamp t) const {
  return in_range(t) ? calc_pva_no_check(t) : nullptr;
}

std::shared_ptr<utils::PVA> BufferedImu::latest_pva() const { return utils::copy_pva(*pva_buf_.back()); }

double BufferedImu::estimated_dt() const {
  if (num_dt_ <= 10) return expected_dt_;
  return dt_window_ == 0 ? dt_sum_ / num_dt_ : median_dt_;
}

void BufferedImu::note_interval(double dt) {
  dt_sum_ += dt;
  ++num_dt_;
  if (dt_window_ == 0) return;
  if (recent_dt_.size() < dt_window_) {
    recent_dt_.push_back(dt);
  } else {
    recent_dt_[recent_next_] = dt;
    recent_next_ = (recent_next_ + 1) % dt_window_;
  }
  // The median is taken here, where the buffer is being written anyway, so that estimated_dt() and the
  // queries built on it stay read-only.
  median_scratch_ = recent_dt_;
  const auto mid = median_scratch_.begin() + static_cast<std::ptrdiff_t>(median_scratch_.size() / 2);
  std::nth_element(median_scratch_.begin(), mid, median_scratch_.end());
  median_dt_ = *mid;
}

bool BufferedImu::late(double dt) const {
  // "Late" needs a measured nominal interval: until estimated_dt() rests on data nothing is called a gap.
  return num_dt_ > 10 && dt > robustness_.gap_factor * estimated_dt();
}

BufferedImu::Plan BufferedImu::plan(const Imu& imu, double dt) const {
  Plan p{v3(imu.get_meas_accel()), v3(imu.get_meas_gyro()), 1, false};
  const bool rates = is_rates(imu);
  if (!rates && robustness_.gap_policy != ImuGapPolicy::HOLD) return p;  // NavToolkit: the increments as received
  const bool is_late = late(dt);
  if (rates) {
    // Rates held since the record before; across a gap in steps of about one nominal interval.
    p.steps = is_late ? sub_steps(dt / estimated_dt(), dt) : 1;
    p.dv *= dt / p.steps;
    p.dth *= dt / p.steps;
    p.held = is_late;
  } else if (is_late) {
    // The record stands for itself and for the samples lost before it: each of them is taken to have
    // carried the same increments. Counting whole samples keeps time-stamp jitter out of the increments.
    const double samples = std::max(2.0, std::round(dt / estimated_dt()));
    p.steps = sub_steps(samples, dt);
    const double per_step = samples / p.steps;  // 1 unless the gap exceeds kMaxSubSteps intervals
    p.dv *= per_step;
    p.dth *= per_step;
    p.held = true;
  }
  return p;
}

bool BufferedImu::plan_after_reset(std::size_t index, std::int64_t from_nsec, double min_scale, Plan& out) const {
  const Imu& imu = *imu_buf_.at(index);
  const double dt = (imu_time(imu) - from_nsec) * 1e-9;  // what is left of the record's interval
  if (is_rates(imu)) {
    if (!(dt > 0)) return false;
    const int steps = late(dt) ? sub_steps(dt / estimated_dt(), dt) : 1;
    out = Plan{v3(imu.get_meas_accel()), v3(imu.get_meas_gyro()), steps, false};
    out.dv *= dt / out.steps;
    out.dth *= dt / out.steps;
    return true;
  }
  // NavToolkit: the record's rate (increments / nominal interval) over the time that is left.
  const double scale = (imu_time(imu) - from_nsec) * 1e-9 / estimated_dt();
  if (!(scale > min_scale)) return false;
  out = Plan{v3(imu.get_meas_accel()) * scale, v3(imu.get_meas_gyro()) * scale, 1, false};
  if (robustness_.gap_policy == ImuGapPolicy::HOLD && index > 0) {
    const double whole = (imu_time(imu) - imu_buf_.time_at(index - 1)) * 1e-9;
    if (late(whole)) {
      // The reset fell inside a gap that plan() bridges with a whole number of samples: keep the share of
      // them that follows the reset, so that the replay continues what the first pass did.
      const double samples = std::max(2.0, std::round(whole / estimated_dt())) * dt / whole;
      out.steps = sub_steps(samples, dt);
      out.dv = v3(imu.get_meas_accel()) * (samples / out.steps);
      out.dth = v3(imu.get_meas_gyro()) * (samples / out.steps);
    }
  }
  return true;
}

void BufferedImu::run(Inertial& ins, std::int64_t to_nsec, const Plan& plan) {
  if (plan.steps > 1) {
    const std::int64_t from_nsec = ins.solution().time.elapsed_nsec;
    const double span = static_cast<double>(to_nsec - from_nsec);
    for (int k = 1; k < plan.steps; ++k)  // sub_steps() keeps these a microsecond or more apart
      ins.mechanize(Timestamp{from_nsec + std::llround(span * k / plan.steps)}, plan.dv, plan.dth);
  }
  ins.mechanize(Timestamp{to_nsec}, plan.dv, plan.dth);
}

bool BufferedImu::confirmed_time_jump(std::int64_t t) {
  const double since_last = (t - pending_jump_nsec_) * 1e-9;
  const bool agrees = pending_jumps_ > 0 && since_last > 0 && since_last <= robustness_.max_gap;
  pending_jumps_ = agrees ? pending_jumps_ + 1 : 1;
  pending_jump_nsec_ = t;
  return pending_jumps_ >= ImuRobustness::kGapConfirmations;
}

BufferedImu::AddStatus BufferedImu::add_sample(std::shared_ptr<Imu> imu) {
  if (!imu) throw std::invalid_argument(kOnlyIntegrated);
  const bool rates = is_rates(*imu);
  if (rates) {
    if (robustness_.sampled_imu == SampledImuPolicy::THROW) throw std::invalid_argument(kOnlyIntegrated);
    if (robustness_.sampled_imu == SampledImuPolicy::DROP) {
      ++stats_.sampled_dropped;
      return AddStatus::SAMPLED_DROPPED;
    }
  }
  if (robustness_.reject_non_finite &&
      !(v3(imu->get_meas_accel()).allFinite() && v3(imu->get_meas_gyro()).allFinite())) {
    ++stats_.non_finite;
    return AddStatus::NON_FINITE;
  }
  const std::int64_t t = imu_time(*imu);
  const double dt = (t - nsec_span().second) * 1e-9;
  if (dt <= 0) {
    ++stats_.not_newer;
    return AddStatus::NOT_NEWER;
  }
  if (robustness_.max_gap > 0 && dt > robustness_.max_gap && !confirmed_time_jump(t)) {
    ++stats_.gap_too_long;
    return AddStatus::GAP_TOO_LONG;
  }
  pending_jumps_ = 0;
  const Plan how = plan(*imu, dt);  // judged against the nominal interval before this one counts
  note_interval(dt);
  run(ins_, t, how);
  imu_buf_.insert(std::move(imu));
  pva_buf_.insert(to_pva_message(ins_.solution()));
  ++stats_.accepted;
  if (rates) ++stats_.sampled_integrated;
  if (how.held) ++stats_.gap_filled;
  stats_.longest_gap = std::max(stats_.longest_gap, dt);
  return how.held ? AddStatus::GAP_FILLED : AddStatus::ACCEPTED;
}

std::shared_ptr<Imu> BufferedImu::calc_force_and_rate(Timestamp t) const {
  if (!in_range(t)) return nullptr;
  auto pva = calc_pva(t);
  auto after = imu_buf_.nearest(t.elapsed_nsec).second;
  if (!pva || !imu_buf_.valid(after)) return nullptr;
  const auto& imu = *imu_buf_.at(after);
  const StandardPva s = to_standard_pva(*pva);
  Vector3 dv = v3(imu.get_meas_accel()), dth = v3(imu.get_meas_gyro());
  if (is_rates(imu)) {  // the increments this record stands for over one nominal interval
    dv *= estimated_dt();
    dth *= estimated_dt();
  }
  const Vector3 force = calc_force_ned(s.C_s_to_n, estimated_dt(), dth, dv);
  const Vector3 rate = calc_rot_rate(s, estimated_dt(), dth);
  return to_force_rate_message(t, force, rate);
}

std::shared_ptr<Imu> BufferedImu::calc_force_and_rate(Timestamp t1, Timestamp t2) const {
  if (!in_range(t1) || !in_range(t2)) return nullptr;
  auto [first, last] = imu_buf_.in_range(t1.elapsed_nsec, t2.elapsed_nsec);
  const double total = (t2.elapsed_nsec - t1.elapsed_nsec) * 1e-9;
  if (first >= imu_buf_.size() || last - first < 1) return calc_force_and_rate(t2);
  if (total < 1e-9) return calc_force_and_rate(t1);
  if (last < imu_buf_.size() && t2.elapsed_nsec != imu_buf_.time_at(last)) ++last;

  // Per-record forces, weighted by the time each record covers inside [t1, t2].
  std::vector<std::int64_t> times{t1.elapsed_nsec};
  for (std::size_t i = first; i + 1 < last; ++i) times.push_back(imu_buf_.time_at(i));
  times.push_back(t2.elapsed_nsec);
  Vector3 f_sum = Vector3::Zero(), r_sum = Vector3::Zero();
  for (std::size_t i = first, k = 0; i < last; ++i, ++k) {
    auto fr = calc_force_and_rate(Timestamp{imu_buf_.time_at(i)});
    const double w = (times[k + 1] - times[k]) * 1e-9;
    if (!fr) continue;
    f_sum += v3(fr->get_meas_accel()) * w;
    r_sum += v3(fr->get_meas_gyro()) * w;
  }
  const Timestamp mid{t1.elapsed_nsec + (t2.elapsed_nsec - t1.elapsed_nsec) / 2};
  return to_force_rate_message(mid, f_sum / total, r_sum / total);
}

ImuErrors BufferedImu::imu_errors(Timestamp t) const {
  auto [lo, hi] = err_buf_.nearest(t.elapsed_nsec);
  if (err_buf_.valid(lo)) return *err_buf_.at(lo);
  if (err_buf_.valid(hi)) return *err_buf_.at(hi);
  ImuErrors e = ins_.imu_errors();
  e.time = ins_.solution().time;
  return e;
}

void BufferedImu::reset_ins(Inertial& ins, const utils::PVA& pva, const ImuErrors& e, const utils::PVA* old) const {
  if (old)
    ins.reset(to_standard_pva(pva), to_standard_pva(*old));
  else
    ins.reset(to_standard_pva(pva));
  ins.set_imu_errors(e);
}

bool BufferedImu::reset(const utils::PVA* pva, const ImuErrors* errors, const utils::PVA* previous) {
  const bool refused = robustness_.reject_non_finite &&
                       ((pva && !usable(*pva)) || (errors && !usable(*errors)) || (previous && !usable(*previous)));
  if (refused || !reset_checked(pva, errors, previous)) {
    ++stats_.resets_rejected;
    return false;
  }
  return true;
}

bool BufferedImu::reset_checked(const utils::PVA* pva, const ImuErrors* errors, const utils::PVA* previous) {
  Timestamp rt{-1};
  if (pva && in_range(Timestamp{pva_time(*pva)}))
    rt = Timestamp{pva_time(*pva)};
  else if (errors && in_range(errors->time))
    rt = errors->time;
  else
    return false;

  std::shared_ptr<utils::PVA> reset_pva = (pva && pva_time(*pva) == rt.elapsed_nsec) ? utils::copy_pva(*pva) : calc_pva(rt);
  if (!reset_pva || !in_range(Timestamp{pva_time(*reset_pva)})) return false;
  ImuErrors reset_err = errors ? *errors : imu_errors(rt);
  reset_err.time = rt;

  reset_ins(ins_, *reset_pva, reset_err, previous);
  const std::int64_t end = nsec_span().second;
  const std::int64_t reset_t = pva_time(*reset_pva);
  auto sols = pva_buf_.in_range(reset_t, end);
  if (sols.first < pva_buf_.size()) pva_buf_.erase(sols.first, sols.second);
  auto errs = err_buf_.in_range(rt.elapsed_nsec, end);
  if (errs.first < err_buf_.size()) err_buf_.erase(errs.first, errs.second);
  pva_buf_.insert(reset_pva);

  auto [first, last] = imu_buf_.in_range(reset_t, end);
  for (std::size_t i = first; i < last; ++i) {
    const auto& imu = *imu_buf_.at(i);
    if (i == first) {
      Plan how;
      if (plan_after_reset(i, reset_t, kVerySmall, how)) run(ins_, imu_time(imu), how);
    } else {
      run(ins_, imu_time(imu), plan(imu, (imu_time(imu) - imu_buf_.time_at(i - 1)) * 1e-9));
    }
    pva_buf_.insert(to_pva_message(ins_.solution()));
  }
  err_buf_.insert(std::make_shared<ImuErrors>(reset_err));
  return true;
}

std::shared_ptr<utils::PVA> BufferedImu::calc_pva_no_reset_since(Timestamp t, Timestamp since) const {
  if (err_buf_.size() <= 1 || err_buf_.back()->time.elapsed_nsec <= since.elapsed_nsec ||
      since.elapsed_nsec > t.elapsed_nsec)
    return calc_pva(t);
  auto pva_last_reset = calc_pva(since);
  if (!pva_last_reset) return nullptr;
  const ImuErrors err_last_reset = imu_errors(since);
  Inertial clone = ins_;
  auto prior = calc_pva_no_check(Timestamp{since.elapsed_nsec - static_cast<std::int64_t>(estimated_dt() * 1e9)});
  reset_ins(clone, *pva_last_reset, err_last_reset, prior.get());

  auto [first, last] = imu_buf_.in_range(pva_time(*pva_last_reset), t.elapsed_nsec);
  const std::size_t stop = (last >= imu_buf_.size()) ? last : last + 1;
  std::shared_ptr<utils::PVA> out;
  for (std::size_t i = first; i < stop; ++i) {
    const auto& imu = *imu_buf_.at(i);
    if (i == first) {
      Plan how;
      if (plan_after_reset(i, pva_time(*pva_last_reset), 0.0, how)) run(clone, imu_time(imu), how);
      continue;
    }
    const Plan how = plan(imu, (imu_time(imu) - imu_buf_.time_at(i - 1)) * 1e-9);
    if (i == stop - 1) {
      auto pre = to_pva_message(clone.solution());
      run(clone, imu_time(imu), how);
      auto post = to_pva_message(clone.solution());
      out = linear_interp_pva(*pre, *post, t, robustness_.attitude_interpolation);
    } else {
      run(clone, imu_time(imu), how);
    }
  }
  if (!out) out = to_pva_message(clone.solution());
  return out;
}

}  // namespace pntos::cobra::inertial
