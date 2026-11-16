#include <pntos/cobra/initialization/InMotionAlignment.hpp>

#include <aspn23/eigen/MeasurementVelocity.hpp>

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace pntos::cobra::inertial {

namespace {

constexpr double kNs = 1e-9;
constexpr std::int64_t kMaxImuGapNs = 500'000'000;        // a longer gap abandons the window
constexpr std::int64_t kRateSpanNs = 100'000'000;         // half span of the rate behind the lever-arm term
constexpr std::int64_t kMaxFixIntervalNs = 3'000'000'000; // fixes further apart give no velocity
constexpr std::size_t kMaxPending = 600;
constexpr std::size_t kDtRing = 31;
constexpr double kMinVelocityVar = 1e-4;                   // (0.01 m/s)^2
constexpr int kMinSolveEpochs = 4;
constexpr double kRadToDeg = 57.29577951308232;

Vector3 v3(const Eigen::Matrix<double, Eigen::Dynamic, 1>& v) { return Vector3(v(0), v(1), v(2)); }

/// exp([rv x])
Matrix3 rotation(const Vector3& rv) {
  const double a = rv.norm();
  const Matrix3 K = nav::skew(rv);
  if (a < 1e-8) return Matrix3::Identity() + K + 0.5 * K * K;
  return Matrix3::Identity() + (std::sin(a) / a) * K + ((1.0 - std::cos(a)) / (a * a)) * K * K;
}

Matrix3 nearest_rotation(const Matrix3& m) {
  Eigen::JacobiSVD<Matrix3> svd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3 d = Matrix3::Identity();
  if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0) d(2, 2) = -1.0;
  return svd.matrixU() * d * svd.matrixV().transpose();
}

}  // namespace

InMotionAlignment::InMotionAlignment(ImuModel model, InMotionAlignmentOptions options)
    : AlignBase(false, true, std::move(model)), opt_(std::move(options)) {
  if (!(opt_.heading_sigma > 0) || !(opt_.max_window > 0) || opt_.min_epochs < 1 || !(opt_.stationary_time > 0) ||
      !(opt_.position_difference_sigma > 0) || !opt_.lever_arm.allFinite())
    throw std::invalid_argument("InMotionAlignment: heading_sigma, max_window, stationary_time and "
                                "position_difference_sigma must be > 0, min_epochs >= 1, lever_arm finite");
  if (opt_.start_time) start_ns_ = static_cast<std::int64_t>(*opt_.start_time * 1e9);
  bias_prior_var_ = model_.gyro_bias_initial_sigma.cwiseAbs2().cwiseMax(1e-20);
}

AlignmentStatus InMotionAlignment::process(const api::AspnBase& message) {
  if (status_ == AlignmentStatus::ALIGNED_GOOD) return status_;
  const auto tov = utils::time_of_validity(message);
  if (!tov) return status_;
  const std::int64_t t = tov->elapsed_nsec;
  if (start_ns_ && t < *start_ns_) return status_;
  if (!first_ns_) first_ns_ = t;

  using Source = InMotionAlignmentOptions::VelocitySource;
  if (const auto* imu = dynamic_cast<const aspn23_eigen::MeasurementImu*>(&message)) {
    on_imu(*imu);
  } else if (const auto* vel = dynamic_cast<const aspn23_eigen::MeasurementVelocity*>(&message)) {
    if (opt_.velocity_source == Source::POSITION) return status_;
    if (vel->get_reference_frame() != ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED) {
      ++diag_.ignored;
      return status_;
    }
    const auto cov = vel->get_covariance();
    const Vector3 var = (cov.rows() == 3 && cov.cols() == 3) ? Vector3(cov(0, 0), cov(1, 1), cov(2, 2))
                                                             : Vector3::Constant(0.01);
    on_velocity(t, Vector3(vel->get_x(), vel->get_y(), vel->get_z()), var);
  } else if (const auto* pos = dynamic_cast<const aspn23_eigen::MeasurementPosition*>(&message)) {
    if (pos->get_reference_frame() != ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC) {
      ++diag_.ignored;
      return status_;
    }
    Fix f;
    f.t = t;
    f.lla = Vector3(pos->get_term1(), pos->get_term2(), pos->get_term3());
    const auto cov = pos->get_covariance();
    if (cov.rows() == 3 && cov.cols() == 3) f.cov = Matrix3(cov);
    on_fix(f);
  } else if (const auto* pva = dynamic_cast<const utils::PVA*>(&message)) {
    const auto cov = pva->get_covariance();
    if (utils::has_position(*pva)) {
      Fix f;
      f.t = t;
      f.lla = Vector3(pva->get_p1(), pva->get_p2(), pva->get_p3());
      if (cov.rows() >= 3 && cov.cols() >= 3) f.cov = Matrix3(cov.topLeftCorner(3, 3));
      on_fix(f);
    }
    if (status_ != AlignmentStatus::ALIGNED_GOOD && opt_.velocity_source != Source::POSITION && utils::has_velocity(*pva)) {
      const Vector3 var = (cov.rows() >= 6 && cov.cols() >= 6) ? Vector3(cov(3, 3), cov(4, 4), cov(5, 5))
                                                               : Vector3::Constant(0.01);
      on_velocity(t, Vector3(pva->get_v1(), pva->get_v2(), pva->get_v3()), var);
    }
  }
  return status_;
}

void InMotionAlignment::on_imu(const aspn23_eigen::MeasurementImu& imu) {
  const std::int64_t t = imu.get_time_of_validity().get_elapsed_nsec();
  if (!last_imu_ns_) {  // the first sample only sets the time base
    last_imu_ns_ = t;
    return;
  }
  const std::int64_t dt_ns = t - *last_imu_ns_;
  if (dt_ns <= 0) {
    ++diag_.ignored;
    return;
  }
  Vector3 dv = v3(imu.get_meas_accel()), dth = v3(imu.get_meas_gyro());
  if (dt_ns > kMaxImuGapNs || !dv.allFinite() || !dth.allFinite()) {  // nothing can be integrated across this
    restart();
    last_imu_ns_ = t;
    return;
  }
  last_imu_ns_ = t;
  const double dt = dt_ns * kNs;
  if (imu.get_imu_type() != ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED) {
    dv *= dt;  // a rate stands for the whole interval since the last sample
    dth *= dt;
  } else {
    // Integrated increments cover one nominal interval. After lost samples the late one stands for them too
    // (the inertial's imu_gap_policy "hold"): without it each lost sample is 10 ms of free fall.
    recent_dt_.push_back(dt_ns);
    if (recent_dt_.size() > kDtRing) recent_dt_.pop_front();
    std::vector<std::int64_t> sorted(recent_dt_.begin(), recent_dt_.end());
    std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2), sorted.end());
    const std::int64_t nominal = sorted[sorted.size() / 2];
    if (recent_dt_.size() >= 5 && dt_ns > nominal + nominal / 2) {
      const double k = static_cast<double>(dt_ns) / static_cast<double>(nominal);
      dv *= k;
      dth *= k;
      ++diag_.held;
    }
  }
  imu_.push_back(Sample{t, dt_ns, dv, dth});
  advance();
}

void InMotionAlignment::on_velocity(std::int64_t t, const Vector3& v, const Vector3& var) {
  if (!v.allFinite()) {
    ++diag_.ignored;
    return;
  }
  if (!velocity_seen_) {
    velocity_seen_ = true;
    if (diag_.from_positions) {  // the epochs so far came from position differences: start over with the receiver's own
      if (!epochs_.empty() || !pending_.empty()) ++diag_.restarts;
      epochs_.clear();
      pending_.clear();
      diag_.from_positions = false;
    }
  }
  Epoch e;
  e.t = t;
  e.v = v;
  e.var = var.allFinite() ? Vector3(var.cwiseMax(kMinVelocityVar)) : Vector3::Constant(0.01);
  queue(e);
}

void InMotionAlignment::on_fix(const Fix& f) {
  if (!f.lla.allFinite() || (fix_ && f.t <= fix_->t)) {
    ++diag_.ignored;
    return;
  }
  previous_fix_ = fix_;
  fix_ = f;
  const double lat = f.lla(0);
  gravity_ = nav::calculate_gravity_schwartz(f.lla(2), lat);
  wie_ = nav::ROTATION_RATE * Vector3(std::cos(lat), 0.0, -std::sin(lat));

  using Source = InMotionAlignmentOptions::VelocitySource;
  const bool from_positions =
      opt_.velocity_source == Source::POSITION || (opt_.velocity_source == Source::AUTO && !velocity_seen_);
  if (!from_positions || !previous_fix_) return;
  const std::int64_t dt_ns = f.t - previous_fix_->t;
  if (dt_ns > kMaxFixIntervalNs) return;
  const double dt = dt_ns * kNs;
  const Vector3& p0 = previous_fix_->lla;
  Epoch e;
  e.half_span = dt_ns / 2;
  e.t = previous_fix_->t + e.half_span;
  e.v = Vector3(nav::delta_lat_to_north(f.lla(0) - p0(0), p0(0), p0(2)), nav::delta_lon_to_east(f.lla(1) - p0(1), p0(0), p0(2)),
                -(f.lla(2) - p0(2))) /
        dt;
  const double s = opt_.position_difference_sigma / dt;
  e.var = Vector3(s * s, s * s, 4.0 * s * s);
  diag_.from_positions = true;
  queue(e);
}

void InMotionAlignment::queue(const Epoch& e) {
  if (!pending_.empty() && e.t <= pending_.back().t) {
    ++diag_.ignored;
    return;
  }
  pending_.push_back(e);
  if (pending_.size() > kMaxPending) pending_.pop_front();
  advance();
}

void InMotionAlignment::advance() {
  // An epoch is taken once the IMU has passed it by the span its lever-arm rate is averaged over.
  while (status_ != AlignmentStatus::ALIGNED_GOOD && !pending_.empty() && !imu_.empty() &&
         pending_.front().t + std::max(pending_.front().half_span, kRateSpanNs) <= imu_.back().t) {
    const Epoch e = pending_.front();
    pending_.pop_front();
    add_epoch(e);
  }
}

void InMotionAlignment::add_epoch(Epoch e) {
  if (!fix_) {  // no latitude for gravity and the Earth's rate yet
    ++diag_.ignored;
    return;
  }
  if (!epochs_.empty() && e.t <= epochs_.back().t) {
    ++diag_.ignored;
    return;
  }
  if (imu_.empty() || e.t < imu_.front().t - imu_.front().dt) return;  // older than the IMU data held

  e.accel = gravity_ - 2.0 * wie_.cross(e.v);
  if (!epochs_.empty()) {
    const Epoch& p = epochs_.back();
    e.corr = p.corr + 0.5 * (p.accel + e.accel) * ((e.t - p.t) * kNs);
  }
  epochs_.push_back(e);

  if (epochs_.size() >= 2) {
    Vector3 dth, dv;
    double seconds = 0;
    if (stationary(epochs_[epochs_.size() - 2], epochs_.back(), &dth, &dv, &seconds)) {
      static_dth_ += dth;
      static_dv_ += dv;
      static_seconds_ += seconds;
      diag_.stationary = static_seconds_;
      if (static_seconds_ >= opt_.stationary_time) set_bias_prior();
      while (epochs_.size() > 1) epochs_.pop_front();  // the window starts at the last epoch at rest
      trim();
      return;
    }
  }
  while (epochs_.size() > 2 && (epochs_.back().t - epochs_.front().t) * kNs > opt_.max_window) epochs_.pop_front();
  trim();

  const int n = static_cast<int>(epochs_.size()) - 1;
  if (n < kMinSolveEpochs) return;
  const auto s = solve();
  if (!s) return;
  ++diag_.solutions;
  bias_ = s->gyro_bias;
  diag_.epochs = n;
  diag_.window = (epochs_.back().t - epochs_.front().t) * kNs;
  diag_.heading_sigma = std::sqrt(std::max(s->P_att(2, 2), 0.0));
  diag_.chi2_per_dof = s->chi2_per_dof;
  diag_.gyro_bias = s->gyro_bias;
  diag_.accel_bias = s->accel_bias;
  if (n >= opt_.min_epochs && diag_.heading_sigma < opt_.heading_sigma) finish(*s);
}

bool InMotionAlignment::stationary(const Epoch& a, const Epoch& b, Vector3* dth_sum, Vector3* dv_sum, double* seconds) const {
  if (a.v.head<2>().norm() >= opt_.stationary_speed || b.v.head<2>().norm() >= opt_.stationary_speed) return false;
  Vector3 dth = Vector3::Zero(), dv = Vector3::Zero();
  std::int64_t dt = 0;
  int n = 0;
  for (const auto& s : imu_) {
    if (s.t <= a.t) continue;
    if (s.t > b.t) break;
    dth += s.dth;
    dv += s.dv;
    dt += s.dt;
    ++n;
  }
  if (n < 5 || dt <= 0) return false;
  const Vector3 mean = dth / (dt * kNs);
  if (mean.norm() >= opt_.stationary_rate) return false;
  for (const auto& s : imu_) {
    if (s.t <= a.t) continue;
    if (s.t > b.t) break;
    if ((s.dth / (s.dt * kNs) - mean).cwiseAbs().maxCoeff() >= opt_.stationary_rate) return false;
  }
  *dth_sum = dth;
  *dv_sum = dv;
  *seconds = dt * kNs;
  return true;
}

void InMotionAlignment::set_bias_prior() {
  // At rest the gyro shows its bias plus the Earth's rate. The vertical part of that rate needs no heading
  // (the specific force points up); the horizontal part does and stays in as an uncertainty.
  const double lat = fix_ ? fix_->lla(0) : 0.0;
  Vector3 mean = static_dth_ / static_seconds_;
  const double f = static_dv_.norm();
  if (f > 0) mean -= nav::ROTATION_RATE * std::sin(lat) * (static_dv_ / f);
  const double arw = model_.gyro_random_walk_sigma.cwiseAbs().maxCoeff();
  const double in_run = model_.gyro_bias_sigma.cwiseAbs().maxCoeff();
  const double horizontal = nav::ROTATION_RATE * std::cos(lat);
  const double var_static = arw * arw / static_seconds_ + in_run * in_run + horizontal * horizontal;
  // Combined with the model's own prior (zero, initial sigma): a gyro better than the Earth's rate keeps its zero.
  const Vector3 var0 = model_.gyro_bias_initial_sigma.cwiseAbs2().cwiseMax(1e-20);
  for (int i = 0; i < 3; ++i) {
    const double w = var0(i) / (var0(i) + var_static);
    bias_prior_(i) = w * mean(i);
    bias_prior_var_(i) = std::max(var0(i) * var_static / (var0(i) + var_static), 1e-20);
  }
  bias_ = bias_prior_;
  diag_.gyro_bias = bias_;
}

void InMotionAlignment::trim() {
  if (imu_.empty()) return;
  std::int64_t keep_after;
  if (epochs_.empty()) {
    keep_after = imu_.back().t - static_cast<std::int64_t>((opt_.max_window + 5.0) * 1e9);
  } else {
    const Epoch& e = epochs_.front();
    keep_after = e.t - std::max(e.half_span, kRateSpanNs) - 1;
  }
  while (!imu_.empty() && imu_.front().t <= keep_after) imu_.pop_front();
}

void InMotionAlignment::restart() {
  if (!epochs_.empty() || !pending_.empty() || !imu_.empty()) ++diag_.restarts;
  epochs_.clear();
  pending_.clear();
  imu_.clear();
  recent_dt_.clear();
}

Vector3 InMotionAlignment::mean_rate(std::int64_t from, std::int64_t to) const {
  Vector3 dth = Vector3::Zero();
  std::int64_t dt = 0;
  for (const auto& s : imu_) {
    if (s.t <= from) continue;
    if (s.t - s.dt >= to) break;
    dth += s.dth;
    dt += s.dt;
  }
  return dt > 0 ? Vector3(dth / (dt * kNs)) : Vector3(Vector3::Zero());
}

std::optional<InMotionAlignment::Solution> InMotionAlignment::solve() {
  const std::size_t m = epochs_.size();  // epochs 0..n
  const std::size_t n = m - 1;
  const Epoch& e0 = epochs_.front();
  const Epoch& en = epochs_.back();

  // 1. The window integrated with the current bias: x, X, G = dx/db and Theta = d(rotation)/db at every epoch.
  std::vector<Vector3> x(m, Vector3::Zero());
  std::vector<Matrix3> X(m, Matrix3::Identity()), G(m, Matrix3::Zero()), Th(m, Matrix3::Zero());
  {
    Matrix3 Xc = Matrix3::Identity(), Gc = Matrix3::Zero(), Tc = Matrix3::Zero();
    Vector3 xc = Vector3::Zero();
    std::int64_t t = e0.t;
    std::size_t e = 1;
    for (const auto& s : imu_) {
      if (e >= m) break;
      if (s.t <= t) continue;
      std::int64_t a = std::max(t, s.t - s.dt);
      while (e < m) {
        const std::int64_t b = std::min(s.t, epochs_[e].t);
        if (b > a) {
          const double fr = static_cast<double>(b - a) / static_cast<double>(s.dt);
          const double dt = fr * s.dt * kNs;
          const Vector3 dth = fr * s.dth - bias_ * dt;
          const Vector3 dv = fr * s.dv;
          const Vector3 f = Xc * (dv + 0.5 * dth.cross(dv));
          Gc -= nav::skew(f) * (Tc - 0.5 * dt * Xc);
          xc += f;
          Tc -= dt * Xc;
          Xc = Xc * rotation(dth);
          a = b;
        }
        while (e < m && epochs_[e].t <= a) {
          x[e] = xc;
          X[e] = Xc;
          G[e] = Gc;
          Th[e] = Tc;
          ++e;
        }
        if (a >= s.t) break;
      }
      t = s.t;
    }
    if (e < m) return std::nullopt;  // the IMU does not reach the last epoch
  }
  if (!opt_.lever_arm.isZero()) {
    // The receiver measures the antenna: its velocity differs from the IMU's by C (w x l).
    std::vector<Vector3> l(m);
    for (std::size_t i = 0; i < m; ++i) {
      const Epoch& ep = epochs_[i];
      const std::int64_t h = std::max(ep.half_span, kRateSpanNs);
      l[i] = X[i] * (mean_rate(ep.t - h, ep.t + h) - bias_).cross(opt_.lever_arm);
    }
    for (std::size_t i = 0; i < m; ++i) x[i] += l[i] - l[0];
  }

  // 2. What the receiver saw of it.
  std::vector<Vector3> y(m), w(m);
  std::vector<double> ws(m), tau(m);
  for (std::size_t i = 0; i < m; ++i) {
    const Epoch& ep = epochs_[i];
    y[i] = (ep.v - ep.corr) - (e0.v - e0.corr);
    w[i] = ep.var.cwiseInverse();
    ws[i] = 3.0 / ep.var.sum();
    tau[i] = (ep.t - e0.t) * kNs;
  }
  const Vector3 bias_info = bias_prior_var_.cwiseInverse();
  const Vector3 accel_info = model_.accel_bias_initial_sigma.cwiseAbs().cwiseMax(1e-4).cwiseAbs2().cwiseInverse();

  // 3. Alternate: C0 from Wahba's problem (global in the heading), then (db, ba, c) by linear least squares.
  //    An accelerometer bias ba (measured = true + ba) takes -Theta_i ba out of x_i: dx_i/dba = integral of X.
  Vector3 db = Vector3::Zero(), ba = Vector3::Zero(), c = Vector3::Zero();
  Matrix3 C0 = Matrix3::Identity();
  for (int it = 0; it < 10; ++it) {
    Vector3 xm = Vector3::Zero(), ym = Vector3::Zero();
    double wsum = 0.0;
    for (std::size_t i = 0; i < m; ++i) {
      xm += ws[i] * (x[i] + G[i] * db + Th[i] * ba);
      ym += ws[i] * y[i];
      wsum += ws[i];
    }
    xm /= wsum;
    ym /= wsum;
    Matrix3 B = Matrix3::Zero();
    for (std::size_t i = 0; i < m; ++i) B += ws[i] * (y[i] - ym) * (x[i] + G[i] * db + Th[i] * ba - xm).transpose();
    const Matrix3 C0n = nearest_rotation(B);

    Eigen::Matrix<double, 9, 9> H = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, 9, 1> g = Eigen::Matrix<double, 9, 1>::Zero();
    for (int k = 0; k < 3; ++k) {
      H(k, k) += bias_info(k);
      g(k) += bias_info(k) * (bias_prior_(k) - bias_(k));
      H(3 + k, 3 + k) += accel_info(k);
    }
    for (std::size_t i = 0; i < m; ++i) {
      Eigen::Matrix<double, 3, 9> J;
      J.leftCols<3>() = C0n * G[i];
      J.block<3, 3>(0, 3) = C0n * Th[i];
      J.rightCols<3>() = Matrix3::Identity();
      const Vector3 r = y[i] - C0n * x[i];
      H += J.transpose() * w[i].asDiagonal() * J;
      g += J.transpose() * w[i].cwiseProduct(r);
    }
    Eigen::LDLT<Eigen::Matrix<double, 9, 9>> ldlt(H);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) return std::nullopt;
    const Eigen::Matrix<double, 9, 1> sol = ldlt.solve(g);
    if (!sol.allFinite()) return std::nullopt;
    const bool done = it > 0 && (sol.head<3>() - db).norm() * tau[n] < 1e-5 && (sol.segment<3>(3) - ba).norm() * tau[n] < 1e-4;
    db = sol.head<3>();
    ba = sol.segment<3>(3);
    c = sol.tail<3>();
    C0 = C0n;
    if (done) break;
  }

  // 4. Covariance over (rotation, db, ba, c), scaled by the misfit when the residuals exceed the stated noise.
  Eigen::Matrix<double, 12, 12> H = Eigen::Matrix<double, 12, 12>::Zero();
  const double no_heading = 1.0 / (nav::PI * nav::PI);  // a rotation the data does not show: sigma 180 degrees
  for (int k = 0; k < 3; ++k) {
    H(k, k) += no_heading;
    H(3 + k, 3 + k) += bias_info(k);
    H(6 + k, 6 + k) += accel_info(k);
  }
  double chi2 = 0.0;
  for (std::size_t i = 0; i < m; ++i) {
    const Vector3 mi = C0 * (x[i] + G[i] * db + Th[i] * ba);
    Eigen::Matrix<double, 3, 12> J;
    J.leftCols<3>() = -nav::skew(mi);
    J.block<3, 3>(0, 3) = C0 * G[i];
    J.block<3, 3>(0, 6) = C0 * Th[i];
    J.rightCols<3>() = Matrix3::Identity();
    const Vector3 r = y[i] - mi - c;
    H += J.transpose() * w[i].asDiagonal() * J;
    chi2 += w[i].dot(r.cwiseAbs2());
  }
  const double dof = std::max(3.0 * static_cast<double>(m) - 12.0, 1.0);
  Eigen::LDLT<Eigen::Matrix<double, 12, 12>> ldlt(H);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) return std::nullopt;
  const Eigen::Matrix<double, 12, 12> P =
      ldlt.solve(Eigen::Matrix<double, 12, 12>::Identity()) * std::max(1.0, chi2 / dof);
  if (!P.allFinite()) return std::nullopt;

  Solution out;
  out.C = nearest_rotation(C0 * (Matrix3::Identity() + nav::skew(Th[n] * db)) * X[n]);
  Eigen::Matrix<double, 3, 6> A;
  A.leftCols<3>() = Matrix3::Identity();
  A.rightCols<3>() = C0 * Th[n];
  out.P_att = A * P.topLeftCorner<6, 6>() * A.transpose();
  out.gyro_bias = bias_ + db;
  out.P_bias = P.block<3, 3>(3, 3);
  out.accel_bias = ba;
  out.P_accel = P.block<3, 3>(6, 6);
  out.chi2_per_dof = chi2 / dof;
  out.v_antenna = e0.v + (en.corr - e0.corr) + C0 * (x[n] + G[n] * db + Th[n] * ba) + c;
  const std::int64_t h = std::max(en.half_span, kRateSpanNs);
  out.rate = mean_rate(en.t - h, en.t + h) - out.gyro_bias;
  return out;
}

void InMotionAlignment::finish(const Solution& s) {
  // The solution belongs to the last velocity epoch, which is behind the IMU by the receiver's latency. Carry it
  // to the last IMU sample so that the mechanization continues without a gap.
  const Epoch& ek = epochs_.back();
  const Fix& fix = *fix_;
  const Vector3& b = s.gyro_bias;
  Matrix3 C = s.C;
  Vector3 v = s.v_antenna - C * s.rate.cross(opt_.lever_arm);  // of the IMU
  Vector3 d = Vector3::Zero();                                 // displacement since the epoch, NED
  std::int64_t t = ek.t;
  std::optional<Vector3> d_at_fix;
  Matrix3 C_at_fix = C;  // the fix is the antenna's position: the lever arm applies with the attitude of that moment
  if (fix.t <= t) d_at_fix = Vector3(-v * ((t - fix.t) * kNs));  // the fix is older than the epoch: constant velocity
  for (const auto& smp : imu_) {
    if (smp.t <= t) continue;
    const std::int64_t a = std::max(t, smp.t - smp.dt);
    const double fr = static_cast<double>(smp.t - a) / static_cast<double>(smp.dt);
    const double dt = fr * smp.dt * kNs;
    const Vector3 dth = fr * smp.dth - b * dt;
    const Vector3 dv = fr * smp.dv - s.accel_bias * dt;
    const Vector3 dvn = C * (dv + 0.5 * dth.cross(dv)) + (gravity_ - 2.0 * wie_.cross(v)) * dt;
    const Vector3 d_new = d + (v + 0.5 * dvn) * dt;
    if (!d_at_fix && fix.t <= smp.t) {
      d_at_fix = Vector3(d + (d_new - d) * (static_cast<double>(fix.t - a) / static_cast<double>(smp.t - a)));
      C_at_fix = C;
    }
    d = d_new;
    v += dvn;
    C = C * rotation(dth);
    t = smp.t;
  }
  if (!d_at_fix) {
    d_at_fix = d;
    C_at_fix = C;
  }
  const double since_epoch = (t - ek.t) * kNs;
  const double since_fix = std::abs((t - fix.t) * kNs);

  const Vector3 disp = d - *d_at_fix - C_at_fix * opt_.lever_arm;
  Vector3 lla = fix.lla;
  lla(0) += nav::north_to_delta_lat(disp(0), fix.lla(0), fix.lla(2));
  lla(1) += nav::east_to_delta_lon(disp(1), fix.lla(0), fix.lla(2));
  lla(2) -= disp(2);
  computed_ = {true, NavSolution{lla, v, C.transpose(), Timestamp{t}}};

  covariance_ = api::Matrix::Zero(15, 15);
  const double v_var = ek.var.mean();
  const double tilt_var = std::max(s.P_att(0, 0), s.P_att(1, 1));
  const double g = gravity_.norm();
  covariance_.block<3, 3>(0, 0) = fix.cov + Matrix3::Identity() * (v_var * since_fix * since_fix);
  covariance_.block<3, 3>(3, 3) = Matrix3(ek.var.asDiagonal());
  covariance_.block<3, 3>(3, 3) += Matrix3::Identity() * (g * g * since_epoch * since_epoch * tilt_var);
  const Matrix3 P_att = s.P_att + since_epoch * since_epoch * (C * s.P_bias * C.transpose());
  covariance_.block<3, 3>(6, 6) = 0.5 * (P_att + P_att.transpose());
  Matrix3 P_accel = 0.5 * (s.P_accel + s.P_accel.transpose());
  for (int i = 0; i < 3; ++i) P_accel(i, i) = std::max(P_accel(i, i), model_.accel_bias_sigma(i) * model_.accel_bias_sigma(i));
  covariance_.block<3, 3>(9, 9) = P_accel;
  Matrix3 P_bias = 0.5 * (s.P_bias + s.P_bias.transpose());
  for (int i = 0; i < 3; ++i)  // not tighter than the bias is stable in the run
    P_bias(i, i) = std::max(P_bias(i, i), model_.gyro_bias_sigma(i) * model_.gyro_bias_sigma(i));
  covariance_.block<3, 3>(12, 12) = P_bias;

  errors_ = ImuErrors{};
  errors_.gyro_biases = b;
  errors_.accel_biases = s.accel_bias;
  errors_.time = Timestamp{t};

  diag_.elapsed = first_ns_ ? (t - *first_ns_) * kNs : 0.0;
  status_ = AlignmentStatus::ALIGNED_GOOD;
  imu_.clear();
  epochs_.clear();
  pending_.clear();
}

std::string InMotionAlignment::summary() const {
  if (!computed_.first) return {};
  std::ostringstream os;
  const Vector3 rpy = nav::dcm_to_rpy(computed_.second.rot_mat.transpose());
  os << std::fixed << std::setprecision(1) << "Aligned in motion after " << diag_.elapsed << " s (window of " << diag_.epochs
     << " epochs, " << diag_.window << " s, velocity from " << (diag_.from_positions ? "position differences" : "the receiver")
     << "): heading " << rpy(2) * kRadToDeg << " deg, sigma " << std::setprecision(2) << diag_.heading_sigma * kRadToDeg
     << " deg; gyro bias (" << std::setprecision(3) << diag_.gyro_bias(0) * 1e3 << ", " << diag_.gyro_bias(1) * 1e3 << ", "
     << diag_.gyro_bias(2) * 1e3 << ") mrad/s";
  if (diag_.stationary >= opt_.stationary_time) os << std::setprecision(1) << ", " << diag_.stationary << " s of standstill used";
  if (diag_.held) os << "; " << diag_.held << " IMU sample(s) stood for lost ones";
  return os.str();
}

}  // namespace pntos::cobra::inertial
