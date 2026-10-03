#include <pntos/cobra/utils/navutils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace pntos::cobra::nav {

double meridian_radius(double latitude) {
  const double s = std::sin(latitude);
  return SEMI_MAJOR_RADIUS * (1 - ECCENTRICITY_SQUARED) / std::pow(1 - ECCENTRICITY_SQUARED * s * s, 1.5);
}

double transverse_radius(double latitude) {
  const double s = std::sin(latitude);
  return SEMI_MAJOR_RADIUS / std::sqrt(1 - ECCENTRICITY_SQUARED * s * s);
}

double delta_lat_to_north(double delta_lat, double approx_lat, double altitude) {
  return (meridian_radius(approx_lat) + altitude) * delta_lat;
}
double delta_lon_to_east(double delta_lon, double approx_lat, double altitude) {
  return (transverse_radius(approx_lat) + altitude) * delta_lon * std::cos(approx_lat);
}
double north_to_delta_lat(double north_distance, double approx_lat, double altitude) {
  return north_distance / (meridian_radius(approx_lat) + altitude);
}
double east_to_delta_lon(double east_distance, double approx_lat, double altitude) {
  return east_distance / ((transverse_radius(approx_lat) + altitude) * std::cos(approx_lat));
}

Matrix3 skew(const Vector3& v) {
  Matrix3 m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

Matrix3 quat_to_dcm(const Vector4& q) {
  const double q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);
  const double a2 = q0 * q0, b2 = q1 * q1, c2 = q2 * q2, d2 = q3 * q3;
  const double ab = q0 * q1, ac = q0 * q2, ad = q0 * q3, bc = q1 * q2, bd = q1 * q3, cd = q2 * q3;
  Matrix3 m;
  m << a2 + b2 - c2 - d2, 2 * (bc - ad), 2 * (bd + ac),  //
      2 * (bc + ad), a2 - b2 + c2 - d2, 2 * (cd - ab),   //
      2 * (bd - ac), 2 * (cd + ab), a2 - b2 - c2 + d2;
  return m;
}

Matrix3 quat_to_dcm(const Vector& q) { return quat_to_dcm(Vector4(q(0), q(1), q(2), q(3))); }

Vector4 dcm_to_quat(const Matrix3& dcm) {
  const double d0 = dcm(0, 0), d1 = dcm(1, 1), d2 = dcm(2, 2);
  const double pa = std::fabs(1 + d0 + d1 + d2);
  const double pb = std::fabs(1 + d0 - d1 - d2);
  const double pc = std::fabs(1 - d0 + d1 - d2);
  const double pd = std::fabs(1 - d0 - d1 + d2);
  double q0, q1, q2, q3;
  if (pa >= pb && pa >= pc && pa >= pd) {
    q0 = 0.5 * std::sqrt(pa);
    const double t = 4 * q0;
    q1 = (dcm(2, 1) - dcm(1, 2)) / t;
    q2 = (dcm(0, 2) - dcm(2, 0)) / t;
    q3 = (dcm(1, 0) - dcm(0, 1)) / t;
  } else if (pb >= pa && pb >= pc && pb >= pd) {
    q1 = 0.5 * std::sqrt(pb);
    const double t = 4 * q1;
    q0 = (dcm(2, 1) - dcm(1, 2)) / t;
    q2 = (dcm(1, 0) + dcm(0, 1)) / t;
    q3 = (dcm(0, 2) + dcm(2, 0)) / t;
  } else if (pc >= pa && pc >= pb && pc >= pd) {
    q2 = 0.5 * std::sqrt(pc);
    const double t = 4 * q2;
    q0 = (dcm(0, 2) - dcm(2, 0)) / t;
    q1 = (dcm(1, 0) + dcm(0, 1)) / t;
    q3 = (dcm(2, 1) + dcm(1, 2)) / t;
  } else {
    q3 = 0.5 * std::sqrt(pd);
    const double t = 4 * q3;
    q0 = (dcm(1, 0) - dcm(0, 1)) / t;
    q1 = (dcm(0, 2) + dcm(2, 0)) / t;
    q2 = (dcm(2, 1) + dcm(1, 2)) / t;
  }
  if (std::signbit(q0)) return Vector4(-q0, -q1, -q2, -q3);
  return Vector4(q0, q1, q2, q3);
}

Matrix3 rpy_to_dcm(const Vector3& rpy) {
  const double cph = std::cos(rpy(0)), sph = std::sin(rpy(0));
  const double cth = std::cos(rpy(1)), sth = std::sin(rpy(1));
  const double cps = std::cos(rpy(2)), sps = std::sin(rpy(2));
  Matrix3 m;
  m << cps * cth, -sps * cph + cps * sth * sph, sps * sph + cps * sth * cph,  //
      sps * cth, cps * cph + sps * sth * sph, -cps * sph + sps * sth * cph,   //
      -sth, cth * sph, cth * cph;
  return m;
}

Vector3 dcm_to_rpy(const Matrix3& dcm) {
  const double asin_arg = std::min(1.0, std::max(dcm(2, 0), -1.0));
  const double r = std::atan2(dcm(2, 1), dcm(2, 2));
  const double p = -std::asin(asin_arg);
  double y = std::atan2(dcm(1, 0), dcm(0, 0));
  if (asin_arg <= -1 + 1e-12) {
    const double y_min_r = std::atan2(dcm(1, 2) - dcm(0, 1), dcm(0, 2) + dcm(1, 1));
    y = y_min_r + r;
  }
  if (asin_arg >= 1 - 1e-12) {
    const double y_pls_r = std::atan2(dcm(1, 2) + dcm(0, 1), dcm(0, 2) - dcm(1, 1)) + PI;
    y = std::remainder(y_pls_r - r, 2.0 * PI);
  }
  return Vector3(r, p, y);
}

Vector4 rpy_to_quat(const Vector3& rpy) {
  const double cr = std::cos(rpy(0) / 2), cp = std::cos(rpy(1) / 2), cy = std::cos(rpy(2) / 2);
  const double sr = std::sin(rpy(0) / 2), sp = std::sin(rpy(1) / 2), sy = std::sin(rpy(2) / 2);
  return Vector4(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                 cr * cp * sy - sr * sp * cy);
}

Matrix3 correct_dcm_with_tilt(const Matrix3& dcm, const Vector3& tilt) {
  const double sum_squares = tilt.squaredNorm();
  if (sum_squares > 0) {
    const double m = std::sqrt(sum_squares);
    const Matrix3 s = skew(tilt);
    const Matrix3 B = Matrix3::Identity() - (std::sin(m) / m) * s + ((1 - std::cos(m)) / sum_squares) * (s * s);
    return B * dcm;
  }
  return dcm;
}

Matrix3 ortho_dcm(const Matrix3& dcm) {
  Matrix3 out = dcm;
  const double delta = 1e-15;
  const Matrix3 dcm_trans = dcm.transpose();
  for (int k = 0; k < 20; ++k) {
    Matrix3 delta_mat = 0.5 * (out * (dcm_trans * out) - dcm);
    out -= delta_mat;
    // xt::allclose(delta_mat, 0, rtol=delta, atol=delta) -> |d| <= delta + delta*0
    if ((delta_mat.array().abs() <= delta).all()) break;
  }
  return out;
}

Matrix3 d_ortho_dcm_wrt_tilt(const Matrix3& C_nav_to_platform, const Vector3& tilts, const Matrix3& dtilt) {
  const Matrix3 c = C_nav_to_platform * (Matrix3::Identity() + skew(tilts));
  const Matrix3 dc = C_nav_to_platform * dtilt;
  const Matrix3 cdc = dc.transpose() * c;
  // fma(-0.5, X, dc) = dc - 0.5 * X
  return dc - 0.5 * (dc * (c.transpose() * c) + c * cdc + c * cdc.transpose() - dc);
}

Matrix3 d_dcm_to_rpy(const Matrix3& ab, const Matrix3& dx, const Matrix3& dy, const Matrix3& dz) {
  const double d_min = std::numeric_limits<double>::min();
  Matrix3 jac;
  const double den1 = std::max(ab(2, 2) * ab(2, 2) + ab(1, 2) * ab(1, 2), d_min);
  jac(0, 0) = (dx(1, 2) * ab(2, 2) - ab(1, 2) * dx(2, 2)) / den1;
  jac(0, 1) = (dy(1, 2) * ab(2, 2) - ab(1, 2) * dy(2, 2)) / den1;
  jac(0, 2) = (dz(1, 2) * ab(2, 2) - ab(1, 2) * dz(2, 2)) / den1;

  const double den2 = std::max(std::sqrt(1.0 - ab(0, 2) * ab(0, 2)), d_min);
  jac(1, 0) = -dx(0, 2) / den2;
  jac(1, 1) = -dy(0, 2) / den2;
  jac(1, 2) = -dz(0, 2) / den2;

  const Vector3 rpy = dcm_to_rpy(ab.transpose());
  if (std::fabs(rpy(1) - PI / 2.0) < 1e-12) {
    const double den3 =
        std::max(std::pow(ab(2, 0) + ab(1, 1), 2.0) + std::pow(ab(2, 1) - ab(1, 0), 2.0), d_min);
    jac(2, 0) = ((dx(2, 1) - dx(1, 0)) * (ab(2, 0) + ab(1, 1)) - (ab(2, 1) - ab(1, 0)) * (dx(2, 0) + dx(1, 1))) / den3 +
                jac(0, 0);
    jac(2, 1) = ((dy(2, 1) - dy(1, 0)) * (ab(2, 0) + ab(1, 1)) - (ab(2, 1) - ab(1, 0)) * (dy(2, 0) + dy(1, 1))) / den3 +
                jac(0, 1);
    jac(2, 2) = ((dz(2, 1) - dz(1, 0)) * (ab(2, 0) + ab(1, 1)) - (ab(2, 1) - ab(1, 0)) * (dz(2, 0) + dz(1, 1))) / den3 +
                jac(0, 2);
  } else if (std::fabs(rpy(1) + PI / 2.0) < 1e-12) {
    const double den3 =
        std::max(std::pow(ab(2, 0) - ab(1, 1), 2.0) + std::pow(ab(2, 1) + ab(1, 0), 2.0), d_min);
    jac(2, 0) = ((dx(2, 1) + dx(1, 0)) * (ab(2, 0) - ab(1, 1)) - (ab(2, 1) + ab(1, 0)) * (dx(2, 0) - dx(1, 1))) / den3 -
                jac(0, 0);
    jac(2, 1) = ((dy(2, 1) + dy(1, 0)) * (ab(2, 0) - ab(1, 1)) - (ab(2, 1) + ab(1, 0)) * (dy(2, 0) - dy(1, 1))) / den3 -
                jac(0, 1);
    jac(2, 2) = ((dz(2, 1) + dz(1, 0)) * (ab(2, 0) - ab(1, 1)) - (ab(2, 1) + ab(1, 0)) * (dz(2, 0) - dz(1, 1))) / den3 -
                jac(0, 2);
  } else {
    const double den3 = std::max(ab(0, 0) * ab(0, 0) + ab(0, 1) * ab(0, 1), d_min);
    jac(2, 0) = (dx(0, 1) * ab(0, 0) - ab(0, 1) * dx(0, 0)) / den3;
    jac(2, 1) = (dy(0, 1) * ab(0, 0) - ab(0, 1) * dy(0, 0)) / den3;
    jac(2, 2) = (dz(0, 1) * ab(0, 0) - ab(0, 1) * dz(0, 0)) / den3;
  }
  return jac;
}

Matrix3 d_dcm_to_rpy(const Matrix3& a, const Matrix3& dadx, const Matrix3& dady, const Matrix3& dadz,
                     const Matrix3& b, const Matrix3& dbdx, const Matrix3& dbdy, const Matrix3& dbdz) {
  const Matrix3 ab = a * b;
  const Matrix3 dx = dadx * b + a * dbdx;
  const Matrix3 dy = dady * b + a * dbdy;
  const Matrix3 dz = dadz * b + a * dbdz;
  return d_dcm_to_rpy(ab, dx, dy, dz);
}

namespace {
struct Trig {
  double cr, sr, cp, sp, cy, sy;
  explicit Trig(const Vector3& rpy)
      : cr(std::cos(rpy(0))),
        sr(std::sin(rpy(0))),
        cp(std::cos(rpy(1))),
        sp(std::sin(rpy(1))),
        cy(std::cos(rpy(2))),
        sy(std::sin(rpy(2))) {}
};
}  // namespace

Matrix3 d_rpy_to_dcm_wrt_r(const Vector3& rpy) {
  Trig t(rpy);
  Matrix3 m;
  m << 0, 0, 0,  //
      t.cr * t.sp * t.cy + t.sr * t.sy, t.cr * t.sp * t.sy - t.sr * t.cy, t.cr * t.cp,  //
      -t.sr * t.sp * t.cy + t.cr * t.sy, -t.sr * t.sp * t.sy - t.cr * t.cy, -t.sr * t.cp;
  return m;
}
Matrix3 d_rpy_to_dcm_wrt_p(const Vector3& rpy) {
  Trig t(rpy);
  Matrix3 m;
  m << -t.sp * t.cy, -t.sp * t.sy, -t.cp,  //
      t.sr * t.cp * t.cy, t.sr * t.cp * t.sy, -t.sr * t.sp,  //
      t.cr * t.cp * t.cy, t.cr * t.cp * t.sy, -t.cr * t.sp;
  return m;
}
Matrix3 d_rpy_to_dcm_wrt_y(const Vector3& rpy) {
  Trig t(rpy);
  Matrix3 m;
  m << -t.cp * t.sy, t.cp * t.cy, 0,  //
      -t.sr * t.sp * t.sy - t.cr * t.cy, t.sr * t.sp * t.cy - t.cr * t.sy, 0,  //
      -t.cr * t.sp * t.sy + t.sr * t.cy, t.cr * t.sp * t.cy + t.sr * t.sy, 0;
  return m;
}

Vector3 calculate_gravity_schwartz(double alt, double lat) {
  const double a1 = 9.7803267715, a2 = 0.0052790414, a3 = 0.0000232718;
  const double a4 = -3.0876910891E-6, a5 = 4.3977311E-9, a6 = 7.211E-13;
  const double s = std::sin(lat), s2 = s * s, s4 = s2 * s2;
  double g;
  if (alt >= 0) {
    g = a1 * (1 + a2 * s2 + a3 * s4) + (a4 + a5 * s2) * alt + a6 * alt * alt;
  } else {
    const double g0 = a1 * (1 + a2 * s2 + a3 * s4);
    const double R0 = std::sqrt(meridian_radius(lat) * transverse_radius(lat));
    g = g0 * (1 + alt / R0);
  }
  return Vector3(0.0, 0.0, g);
}

EarthModel::EarthModel(const Vector3& pos_llh, const Vector3& vel_ned) {
  constexpr double RAD_E = 6378137.0;
  constexpr double F = 1.0 / 298.257223563;
  constexpr double ECC_SQUARE = F * (2 - F);
  lat = pos_llh(0);
  alt = pos_llh(2);
  v_n = vel_ned(0);
  v_e = vel_ned(1);
  sin_l = std::sin(lat);
  cos_l = std::cos(lat);
  tan_l = sin_l / cos_l;
  sec_l = 1 / cos_l;
  sin_2l = std::sin(2 * lat);
  const double ell = 1.0 - ECC_SQUARE * sin_l * sin_l;
  r_n = RAD_E * (1.0 - ECC_SQUARE) / std::pow(ell, 1.5);
  r_e = RAD_E / std::sqrt(ell);
  r_zero = std::sqrt(r_n * r_e);
  lat_factor = r_n + alt;
  lon_factor = cos_l * (r_e + alt);
  omega_en_n = Vector3(v_e / (r_e + alt), -v_n / (r_n + alt), -v_e * tan_l / (r_e + alt));
  omega_ie_n = Vector3(ROTATION_RATE * cos_l, 0.0, -ROTATION_RATE * sin_l);
  omega_in_n = omega_ie_n + omega_en_n;
  g_n = calculate_gravity_schwartz(alt, lat);
}

}  // namespace pntos::cobra::nav
