#include <pntos/cobra/inertial/Mechanization.hpp>

#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>

namespace pntos::cobra::inertial {

Vector3 calc_force_ned(const Matrix3& C, double dt, const Vector3& dth, const Vector3& dv) {
  const Vector3 rot_corr = 0.5 * dth.cross(dv);
  return C * (dv + rot_corr) / dt;
}

Vector3 calc_force_and_acceleration_offset(double r_e, double r_n, double alt0, double cos_l, const Vector3& g,
                                           double sec_l, double sin_l, const Vector3& v, double omega) {
  const double l_dot = v(0) / (r_n + alt0);
  const double lambda_dot = v(1) * sec_l / (r_e + alt0);
  return Vector3(-v(1) * (2 * omega + lambda_dot) * sin_l + v(2) * l_dot + g(0),
                 v(0) * (2 * omega + lambda_dot) * sin_l + v(2) * (2 * omega + lambda_dot) * cos_l + g(1),
                 -v(1) * (2 * omega + lambda_dot) * cos_l - v(0) * l_dot + g(2));
}

Vector3 calc_force_ned(const StandardPva& pva1, const StandardPva& pva2) {
  const double dt = (pva2.time.elapsed_nsec - pva1.time.elapsed_nsec) * 1e-9;
  const Vector3 accel = (pva2.vned - pva1.vned) / dt;
  const double lat = pva1.llh(0), alt = pva1.llh(2);
  const Vector3 g = nav::calculate_gravity_schwartz(alt, lat);
  const Vector3 offset = calc_force_and_acceleration_offset(
      nav::transverse_radius(lat), nav::meridian_radius(lat), alt, std::cos(lat), g, 1.0 / std::cos(lat),
      std::sin(lat), pva1.vned);
  return accel - offset;
}

Vector3 calc_rot_rate(const Matrix3& C, double r_e, double r_n, double alt0, double cos_l, double dt,
                      const Vector3& dth, double sin_l, double tan_l, const Vector3& v, double omega) {
  const Vector3 omega_en_n(v(1) / (r_e + alt0), -v(0) / (r_n + alt0), -v(1) * tan_l / (r_e + alt0));
  const Vector3 omega_ie_n(omega * cos_l, 0.0, -omega * sin_l);
  return dth / dt - C.transpose() * (omega_ie_n + omega_en_n);
}

Vector3 calc_rot_rate(const StandardPva& pva, double dt, const Vector3& dth) {
  const double lat = pva.llh(0);
  return calc_rot_rate(pva.C_s_to_n, nav::transverse_radius(lat), nav::meridian_radius(lat), pva.llh(2),
                       std::cos(lat), dt, dth, std::sin(lat), std::tan(lat), pva.vned);
}

Vector3 calc_rot_rate(const StandardPva& pva1, const StandardPva& pva2) {
  const double dt = (pva1.time.elapsed_nsec - pva2.time.elapsed_nsec) * 1e-9;
  const Matrix3 C_s1_to_s2 = pva2.C_s_to_n.transpose() * pva1.C_s_to_n;
  return nav::dcm_to_rpy(C_s1_to_s2) / dt;
}

std::tuple<Vector3, Vector3, Matrix3> mechanization_standard(const Vector3& dv, const Vector3& dth, double dt,
                                                             const Vector3& llh0, const Matrix3& C0,
                                                             const Vector3& v0, const Vector3& v_prev,
                                                             const MechanizationOptions& o) {
  const double omega = kMechOmega;
  const double lat0 = llh0(0), lon0 = llh0(1), alt0 = llh0(2);
  const double sin_l = std::sin(lat0), cos_l = std::cos(lat0);
  const double sec_l = 1 / cos_l, tan_l = sin_l / cos_l;
  const double r_n = nav::meridian_radius(lat0), r_e = nav::transverse_radius(lat0);
  const double r_zero = std::sqrt(r_n * r_e);

  Vector3 g;
  switch (o.grav_model) {
    case GravModel::SCHWARTZ: g = nav::calculate_gravity_schwartz(alt0, lat0); break;
    case GravModel::TITTERTON: g = nav::calculate_gravity_titterton(alt0, lat0, r_zero); break;
  }

  // Attitude: integrate the sensor rotation w.r.t. the nav frame.
  const Vector3 sigma = calc_rot_rate(C0, r_e, r_n, alt0, cos_l, dt, dth, sin_l, tan_l, v0, omega) * dt;
  Matrix3 A;
  switch (o.dcm_method) {
    case DcmIntegrationMethod::SIXTH_ORDER: A = nav::rot_vec_to_dcm(sigma); break;
    case DcmIntegrationMethod::EXPONENTIAL: A = nav::skew(sigma).exp(); break;
    case DcmIntegrationMethod::FIRST_ORDER: A = Matrix3::Identity() + nav::skew(sigma); break;
  }
  const Matrix3 C1 = C0 * A;

  // Velocity.
  const Vector3 f_ned = calc_force_ned(C0, dt, dth, dv);
  const Vector3 accel = f_ned + calc_force_and_acceleration_offset(r_e, r_n, alt0, cos_l, g, sec_l, sin_l, v0, omega);
  const Vector3 v1 = accel * dt + v0;

  // Position.
  Vector3 dp;
  switch (o.int_method) {
    case IntegrationMethod::TRAPEZOIDAL: dp = (v1 + v0) / 2 * dt; break;
    case IntegrationMethod::SIMPSONS_RULE: dp = (v_prev + 4 * v0 + v1) / 6 * dt; break;
    case IntegrationMethod::RECTANGULAR: dp = v0 * dt; break;
  }
  const Vector3 llh1(lat0 + dp(0) / (r_n + alt0), lon0 + dp(1) * sec_l / (r_e + alt0), alt0 - dp(2));
  return {llh1, v1, C1};
}

Inertial::Inertial(StandardPva start, MechanizationOptions opts) : pva_(start), pva_old_(start), opts_(opts) {}

void Inertial::reset(const StandardPva& new_pva, const std::optional<StandardPva>& old_pva) {
  pva_ = new_pva;
  pva_old_ = old_pva.value_or(new_pva);
}

void Inertial::mechanize(Timestamp time, const Vector3& delta_v, const Vector3& delta_theta) {
  const double dt = (time.elapsed_nsec - pva_.time.elapsed_nsec) * 1e-9;
  Vector3 dv, dth;
  for (int i = 0; i < 3; ++i) {
    dv(i) = (errors_.accel_biases(i) * -dt + delta_v(i)) / (1.0 + errors_.accel_scale_factors(i));
    dth(i) = (errors_.gyro_biases(i) * -dt + delta_theta(i)) / (1.0 + errors_.gyro_scale_factors(i));
  }
  auto [llh1, v1, C1] = mechanization_standard(dv, dth, dt, pva_.llh, pva_.C_s_to_n, pva_.vned, pva_old_.vned, opts_);
  pva_old_ = pva_;
  pva_ = StandardPva{time, llh1, v1, C1};
}

}  // namespace pntos::cobra::inertial
