#include <pntos/cobra/inertial/BufferedImu.hpp>

#include <cmath>

namespace pntos::cobra::inertial {

namespace {
std::int64_t pva_time(const utils::PVA& p) { return p.get_time_of_validity().get_elapsed_nsec(); }
std::int64_t imu_time(const Imu& m) { return m.get_time_of_validity().get_elapsed_nsec(); }
std::int64_t err_time(const ImuErrors& e) { return e.time.elapsed_nsec; }
Vector3 v3(const Eigen::Matrix<double, Eigen::Dynamic, 1>& v) { return Vector3(v(0), v(1), v(2)); }
Eigen::Matrix<double, Eigen::Dynamic, 1> dynv(const Vector3& v) { return Eigen::Matrix<double, Eigen::Dynamic, 1>(v); }
constexpr double kVerySmall = 1e-20;

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

std::shared_ptr<utils::PVA> linear_interp_pva(const utils::PVA& a, const utils::PVA& b, Timestamp t) {
  const std::int64_t t1 = pva_time(a), t2 = pva_time(b);
  if (t1 > t2) return linear_interp_pva(b, a, t);
  if (t.elapsed_nsec <= t1) return utils::copy_pva(a);
  if (t.elapsed_nsec >= t2 || t1 == t2) return utils::copy_pva(b);
  const double f = static_cast<double>(t.elapsed_nsec - t1) / static_cast<double>(t2 - t1);
  Vector3 llh = utils::position(a) + (utils::position(b) - utils::position(a)) * f;
  Vector3 vel = utils::velocity(a) + (utils::velocity(b) - utils::velocity(a)) * f;
  Vector3 rpy = extrapolate_rpy(t1, nav::quat_to_rpy(*utils::quaternion(a)), t2, nav::quat_to_rpy(*utils::quaternion(b)),
                                t.elapsed_nsec);
  auto out = utils::make_pva(a.get_header(), t, llh, vel, nav::rpy_to_quat(rpy), api::Matrix(a.get_covariance()));
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

// ----------------------------------------------------------------------------- BufferedImu

BufferedImu::BufferedImu(const utils::PVA& initial, double expected_dt, double buffer_length, const ImuErrors& errors,
                         const MechanizationOptions& opts)
    : ins_(to_standard_pva(initial), opts),
      pva_buf_(static_cast<std::size_t>(buffer_length / expected_dt + 2), &pva_time),
      imu_buf_(static_cast<std::size_t>(buffer_length / expected_dt + 2), &imu_time),
      err_buf_(static_cast<std::size_t>(buffer_length / expected_dt + 2), &err_time),
      expected_dt_(expected_dt) {
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
  if (pva_buf_.valid(lo) && pva_buf_.valid(hi)) return linear_interp_pva(*pva_buf_.at(lo), *pva_buf_.at(hi), t);
  return nullptr;
}

std::shared_ptr<utils::PVA> BufferedImu::calc_pva(Timestamp t) const {
  return in_range(t) ? calc_pva_no_check(t) : nullptr;
}

std::shared_ptr<utils::PVA> BufferedImu::latest_pva() const { return utils::copy_pva(*pva_buf_.back()); }

bool BufferedImu::add(std::shared_ptr<Imu> imu) {
  if (!imu || imu->get_imu_type() != ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED)
    throw std::invalid_argument("Only ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED is supported by BufferedImu.");
  const std::int64_t t = imu_time(*imu);
  const double dt = (t - nsec_span().second) * 1e-9;
  if (dt <= 0) return false;
  dt_sum_ += dt;
  ++num_dt_;
  ins_.mechanize(Timestamp{t}, v3(imu->get_meas_accel()), v3(imu->get_meas_gyro()));
  imu_buf_.insert(std::move(imu));
  pva_buf_.insert(to_pva_message(ins_.solution()));
  return true;
}

std::shared_ptr<Imu> BufferedImu::calc_force_and_rate(Timestamp t) const {
  if (!in_range(t)) return nullptr;
  auto pva = calc_pva(t);
  auto after = imu_buf_.nearest(t.elapsed_nsec).second;
  if (!pva || !imu_buf_.valid(after)) return nullptr;
  const auto& imu = *imu_buf_.at(after);
  const StandardPva s = to_standard_pva(*pva);
  const Vector3 force = calc_force_ned(s.C_s_to_n, estimated_dt(), v3(imu.get_meas_gyro()), v3(imu.get_meas_accel()));
  const Vector3 rate = calc_rot_rate(s, estimated_dt(), v3(imu.get_meas_gyro()));
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
      const double scale = (imu_time(imu) - reset_t) * 1e-9 / estimated_dt();
      if (scale > kVerySmall)
        ins_.mechanize(Timestamp{imu_time(imu)}, v3(imu.get_meas_accel()) * scale, v3(imu.get_meas_gyro()) * scale);
    } else {
      ins_.mechanize(Timestamp{imu_time(imu)}, v3(imu.get_meas_accel()), v3(imu.get_meas_gyro()));
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
      const double scale = (imu_time(imu) - pva_time(*pva_last_reset)) * 1e-9 / estimated_dt();
      if (scale > 0)
        clone.mechanize(Timestamp{imu_time(imu)}, v3(imu.get_meas_accel()) * scale, v3(imu.get_meas_gyro()) * scale);
    } else if (i == stop - 1) {
      auto pre = to_pva_message(clone.solution());
      clone.mechanize(Timestamp{imu_time(imu)}, v3(imu.get_meas_accel()), v3(imu.get_meas_gyro()));
      auto post = to_pva_message(clone.solution());
      out = linear_interp_pva(*pre, *post, t);
    } else {
      clone.mechanize(Timestamp{imu_time(imu)}, v3(imu.get_meas_accel()), v3(imu.get_meas_gyro()));
    }
  }
  if (!out) out = to_pva_message(clone.solution());
  return out;
}

}  // namespace pntos::cobra::inertial
