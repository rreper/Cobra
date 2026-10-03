#include <pntos/cobra/initialization/Alignment.hpp>

#include <cmath>
#include <limits>

namespace pntos::cobra::inertial {

using api::Matrix;
using api::Vector;

namespace {
Vector3 v3(const std::array<double, 3>& a) { return Vector3(a[0], a[1], a[2]); }
std::array<double, 3> a3(const Vector3& v) { return {v(0), v(1), v(2)}; }
Vector3 v3(const Eigen::Matrix<double, Eigen::Dynamic, 1>& v) { return Vector3(v(0), v(1), v(2)); }
}  // namespace

ImuModel ImuModel::from_config(const ImuConfig& c) {
  ImuModel m;
  m.accel_bias_sigma = v3(c.accel_bias_sigma);
  m.accel_bias_tau = v3(c.accel_bias_tau);
  m.accel_random_walk_sigma = v3(c.accel_random_walk_sigma);
  m.gyro_bias_sigma = v3(c.gyro_bias_sigma);
  m.gyro_bias_tau = v3(c.gyro_bias_tau);
  m.gyro_random_walk_sigma = v3(c.gyro_random_walk_sigma);
  m.accel_bias_initial_sigma = v3(c.accel_bias_initial_sigma);
  m.gyro_bias_initial_sigma = v3(c.gyro_bias_initial_sigma);
  return m;
}

ImuConfig ImuModel::to_config(const std::string& group) const {
  ImuConfig c;
  c.group_ = group;
  c.accel_bias_sigma = a3(accel_bias_sigma);
  c.accel_bias_tau = a3(accel_bias_tau);
  c.accel_random_walk_sigma = a3(accel_random_walk_sigma);
  c.gyro_bias_sigma = a3(gyro_bias_sigma);
  c.gyro_bias_tau = a3(gyro_bias_tau);
  c.gyro_random_walk_sigma = a3(gyro_random_walk_sigma);
  c.accel_bias_initial_sigma = a3(accel_bias_initial_sigma);
  c.gyro_bias_initial_sigma = a3(gyro_bias_initial_sigma);
  return c;
}

ImuModel hg1700_model() {
  ImuModel m;
  m.accel_random_walk_sigma.setConstant(0.065 * 0.3048 / 60);
  m.gyro_random_walk_sigma.setConstant(0.125 * nav::PI / 180 / 60);
  m.accel_bias_sigma.setConstant(9.81e-3);
  m.accel_bias_tau.setConstant(3600.0);
  m.gyro_bias_sigma.setConstant(nav::PI / 180 / 3600);
  m.gyro_bias_tau.setConstant(3600.0);
  m.accel_bias_initial_sigma.setConstant(9.81e-3);
  m.gyro_bias_initial_sigma.setConstant(nav::PI / 180 / 3600);
  return m;
}

ImuModel hg9900_model() {
  ImuModel m;
  m.accel_random_walk_sigma.setConstant(1e-12);
  m.gyro_random_walk_sigma.setConstant(0.002 * nav::PI / 180 / 60);
  m.accel_bias_sigma.setConstant(25 * 9.81e-6);
  m.accel_bias_tau.setConstant(3600);
  m.gyro_bias_sigma.setConstant(0.0035 * nav::PI / 180 / 3600);
  m.gyro_bias_tau.setConstant(3600);
  m.accel_scale_factor.setConstant(100);
  m.gyro_scale_factor.setConstant(5);
  m.accel_bias_initial_sigma.setConstant(25 * 9.81e-6);
  m.gyro_bias_initial_sigma.setConstant(0.003 * nav::PI / 180 / 3600);
  return m;
}

ImuModel stim300_model() {
  ImuModel m;
  m.accel_random_walk_sigma.setConstant(0.06 / 60);
  m.gyro_random_walk_sigma.setConstant(0.15 * nav::PI / 180 / 60);
  m.accel_bias_sigma.setConstant(0.05 * 9.81 / 1000);
  m.accel_bias_tau.setConstant(3600.0);
  m.gyro_bias_sigma.setConstant(0.5 * nav::PI / 180 / 3600);
  m.gyro_bias_tau.setConstant(3600.0);
  m.accel_scale_factor.setConstant(300);
  m.gyro_scale_factor.setConstant(500);
  m.accel_bias_initial_sigma.setConstant(0.75 * 9.81 / 1000);
  m.gyro_bias_initial_sigma.setConstant(0.5 * nav::PI / 180 / 3600);
  return m;
}

Matrix3 quaternion_static_alignment(const Vector3& accel, const Vector3& ang_rate) {
  const Vector3 z(0, 0, -1);
  Vector3 normal = accel.cross(z);
  normal /= normal.norm();
  const double z_rot = std::acos(z.dot(accel) / (accel.norm() * z.norm()));
  const Matrix3 C_sensor_to_nedprime = nav::axis_angle_to_dcm(normal, z_rot);
  const Vector3 accel_np = C_sensor_to_nedprime * accel;
  if (accel_np(0) >= 1e-10 || accel_np(1) >= 1e-10)
    throw std::runtime_error("quaternion_static_alignment: accel not aligned with the vertical after levelling");
  const Vector3 gy_np = C_sensor_to_nedprime * ang_rate;
  const Vector3 gy_proj(gy_np(0), gy_np(1), 0.0);
  const Vector3 x(1, 0, 0);
  const double xy_rot = std::acos(x.dot(gy_proj) / (gy_proj.norm() * x.norm()));
  Vector3 xy_normal = gy_proj.cross(x);
  xy_normal /= xy_normal.norm();
  const Matrix3 C_navprime_to_ned = nav::axis_angle_to_dcm(xy_normal, xy_rot);
  const Matrix3 mult = C_navprime_to_ned * C_sensor_to_nedprime;
  const Vector3 gy_n = mult * ang_rate;
  if (gy_n(1) >= 1e-10) throw std::runtime_error("quaternion_static_alignment: residual east rotation rate");
  return mult;
}

Matrix first_order_rpy_covariance(const Vector& x, const Matrix& P, const std::function<Vector3(const Vector&)>& f,
                                  double eps) {
  const Eigen::Index n = x.size();
  Matrix jac = Matrix::Zero(3, n);
  const Matrix3 cns_nom = nav::rpy_to_dcm(f(x));
  for (Eigen::Index i = 0; i < n; ++i) {
    // NavToolkit's perturbation is relative (x_i * eps), falling back to eps where x_i == 0.
    const double e = x(i) * eps != 0.0 ? x(i) * eps : eps;
    Vector xp = x, xm = x;
    xp(i) += e;
    xm(i) -= e;
    const Matrix3 csn_p = nav::rpy_to_dcm(f(xp)).transpose();
    const Matrix3 csn_m = nav::rpy_to_dcm(f(xm)).transpose();
    const Vector3 tilt_p = nav::dcm_to_rpy((cns_nom * csn_p).transpose());
    const Vector3 tilt_m = nav::dcm_to_rpy((cns_nom * csn_m).transpose());
    jac.col(i) = (tilt_p - tilt_m) / (2 * e);
  }
  return jac * P * jac.transpose();
}

// ----------------------------------------------------------------------------- AlignBase

AlignBase::AlignBase(bool supports_static, bool supports_dynamic, ImuModel model)
    : supports_static_(supports_static), supports_dynamic_(supports_dynamic), model_(std::move(model)) {}

Matrix AlignBase::bias_stats_from_model() const {
  Matrix cov = Matrix::Zero(6, 6);
  cov.block<3, 3>(0, 0) = model_.accel_bias_initial_sigma.cwiseAbs2().asDiagonal();
  cov.block<3, 3>(3, 3) = model_.gyro_bias_initial_sigma.cwiseAbs2().asDiagonal();
  return cov;
}

// ----------------------------------------------------------------------------- StaticAlignment

StaticAlignment::StaticAlignment(ImuModel model, double align_time, const Matrix3& vel_cov)
    : AlignBase(true, false, std::move(model)), align_time_(align_time), vel_cov_(vel_cov) {}

AlignmentStatus StaticAlignment::process(const api::AspnBase& message) {
  if (status_ == AlignmentStatus::ALIGNED_GOOD) return status_;
  if (const auto* pos = dynamic_cast<const aspn23_eigen::MeasurementPosition*>(&message)) {
    if (pos->get_covariance().rows() == 3) {
      gps_buffer_.push_back(*pos);
      if (sufficient_data()) calc_alignment(Timestamp{align_buffer_.back().get_time_of_validity().get_elapsed_nsec()});
    }
    return status_;
  }
  if (const auto* pva = dynamic_cast<const utils::PVA*>(&message)) {
    if (utils::has_position(*pva)) {
      aspn23_eigen::MeasurementPosition pos(aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0),
                                            pva->get_time_of_validity(), ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC,
                                            pva->get_p1(), pva->get_p2(), pva->get_p3(),
                                            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
                                                pva->get_covariance().topLeftCorner(3, 3)),
                                            ASPN23_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
                                            Eigen::Matrix<double, Eigen::Dynamic, 1>(0),
                                            std::vector<aspn23_eigen::TypeIntegrity>{});
      return process(pos);
    }
    return status_;
  }
  if (const auto* imu = dynamic_cast<const aspn23_eigen::MeasurementImu*>(&message)) {
    align_buffer_.push_back(*imu);
    if (sufficient_data()) calc_alignment(Timestamp{imu->get_time_of_validity().get_elapsed_nsec()});
    return status_;
  }
  return status_;
}

bool StaticAlignment::sufficient_data() const {
  if (gps_buffer_.empty() || align_buffer_.empty()) return false;
  const std::int64_t back = align_buffer_.back().get_time_of_validity().get_elapsed_nsec();
  const std::int64_t front = align_buffer_.front().get_time_of_validity().get_elapsed_nsec();
  return (back - front) * 1e-9 > align_time_;
}

std::pair<Vector3, Matrix3> StaticAlignment::pos_stats() const {
  if (gps_buffer_.empty()) return {Vector3::Zero(), Matrix3::Identity() * std::numeric_limits<double>::max()};
  Vector3 mean = Vector3::Zero();
  for (const auto& p : gps_buffer_) mean += Vector3(p.get_term1(), p.get_term2(), p.get_term3());
  mean /= static_cast<double>(gps_buffer_.size());
  return {mean, Matrix3(gps_buffer_.back().get_covariance())};
}

std::pair<Vector3, Vector3> StaticAlignment::calc_avg() const {
  Vector3 dv = Vector3::Zero(), dth = Vector3::Zero();
  for (const auto& imu : align_buffer_) {
    dv += v3(imu.get_meas_accel());
    dth += v3(imu.get_meas_gyro());
  }
  const double n = static_cast<double>(align_buffer_.size());
  return {dv / n, dth / n};
}

double StaticAlignment::calc_average_delta_time() const {
  const std::int64_t back = align_buffer_.back().get_time_of_validity().get_elapsed_nsec();
  const std::int64_t front = align_buffer_.front().get_time_of_validity().get_elapsed_nsec();
  return (back - front) * 1e-9 / static_cast<double>(align_buffer_.size() - 1);
}

void StaticAlignment::calc_alignment(Timestamp imu_time) {
  auto [dv_avg, dth_avg] = calc_avg();
  const Matrix3 cns = quaternion_static_alignment(dv_avg, dth_avg);  // C_sensor_to_ned despite the name
  const Matrix3 csn = cns.transpose();
  const double dt = calc_average_delta_time();
  const Vector3 pos = pos_stats().first;

  Vector x(6);
  x << dv_avg, dth_avg;
  Vector sigma(6);
  sigma << model_.accel_bias_initial_sigma * dt, model_.gyro_bias_initial_sigma * dt;
  const Matrix P = sigma.cwiseAbs2().asDiagonal();
  tilt_cov_ = first_order_rpy_covariance(x, P, [](const Vector& v) {
    return nav::dcm_to_rpy(quaternion_static_alignment(v.head<3>(), v.tail<3>()));
  });
  computed_ = {true, NavSolution{pos, Vector3::Zero(), csn, imu_time}};
  status_ = AlignmentStatus::ALIGNED_GOOD;
}

std::pair<bool, Matrix> StaticAlignment::get_computed_covariance() const {
  Matrix out = Matrix::Zero(15, 15);
  out.block<3, 3>(0, 0) = pos_stats().second;
  out.block<3, 3>(3, 3) = vel_cov_;
  out.block<3, 3>(6, 6) = tilt_cov_;
  out.block<6, 6>(9, 9) = bias_stats_from_model();
  return {status_ == AlignmentStatus::ALIGNED_GOOD, out};
}

// ----------------------------------------------------------------------------- ManualHeadingAlignment

ManualHeadingAlignment::ManualHeadingAlignment(double heading, double heading_sigma, ImuModel model, double align_time,
                                               const Matrix3& vel_cov)
    : StaticAlignment(std::move(model), align_time, vel_cov), heading_(heading), heading_sigma_(heading_sigma) {}

void ManualHeadingAlignment::calc_align() {
  auto alignment = StaticAlignment::get_computed_alignment();
  if (!alignment.first) return;
  auto [dv, dth] = calc_avg();
  const double roll = std::atan(dv(1) / dv(2));
  const double sr = std::sin(roll), cr = std::cos(roll);
  const double pitch = std::atan(-dv(0) / (sr * dv(1) + cr * dv(2)));
  alignment.second.rot_mat = nav::rpy_to_dcm(Vector3(roll, pitch, heading_)).transpose();
  computed_ = alignment;
  status_ = AlignmentStatus::ALIGNED_GOOD;
}

AlignmentStatus ManualHeadingAlignment::process(const api::AspnBase& message) {
  if (const auto* imu = dynamic_cast<const aspn23_eigen::MeasurementImu*>(&message))
    if (imu->get_imu_type() != ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED)
      throw std::invalid_argument("Only ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED is supported in ManualHeadingAlignment.");
  StaticAlignment::process(message);
  calc_align();
  return status_;
}

std::pair<bool, Matrix> ManualHeadingAlignment::get_computed_covariance() const {
  auto cov = StaticAlignment::get_computed_covariance();
  if (!cov.first) return cov;
  Matrix& P = cov.second;
  P.row(8).setZero();
  P.col(8).setZero();
  const auto& last = gps_buffer_.back();
  const double g = nav::calculate_gravity_schwartz(last.get_term3(), last.get_term1())(2);
  const Matrix3 csn = computed_.second.rot_mat;
  const Matrix3 cov_accel = P.block<3, 3>(9, 9);
  Matrix3 g_rot;
  g_rot << 0, 1.0 / g, 0, -1.0 / g, 0, 0, 0, 0, 0;
  const Matrix3 partial = csn.transpose() * (cov_accel * (csn * g_rot.transpose()));
  P.block<3, 3>(6, 6) = g_rot * partial;
  P(8, 8) = heading_sigma_ * heading_sigma_;
  const Matrix3 cross = csn * partial;
  P.block<3, 3>(9, 6) = cross;
  P.block<3, 3>(6, 9) = cross.transpose();
  return cov;
}

std::pair<bool, ImuErrors> ManualHeadingAlignment::get_imu_errors() const {
  ImuErrors e;
  if (computed_.first) {
    auto [dv, dth] = calc_avg();
    const auto& last = gps_buffer_.back();
    const Vector3 g = nav::calculate_gravity_schwartz(last.get_term3(), last.get_term1());
    const double dt = calc_average_delta_time();
    const Vector3 wie_n(std::cos(last.get_term1()) * nav::ROTATION_RATE, 0.0,
                        -std::sin(last.get_term1()) * nav::ROTATION_RATE);
    const Matrix3& rot = computed_.second.rot_mat;
    e.gyro_biases += dth / dt - rot * wie_n;
    const Vector3 dv_rot = rot.transpose() * dv;
    const Vector3 est_ned(0.0, 0.0, g(2) + dv_rot(2) / dt);
    e.accel_biases = rot * est_ned;
  }
  return {status_ == AlignmentStatus::ALIGNED_GOOD, e};
}

}  // namespace pntos::cobra::inertial
