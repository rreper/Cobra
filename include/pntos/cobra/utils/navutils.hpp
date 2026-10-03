// Navigation math used by the Cobra plugins. These are C++/Eigen re-implementations of the
// NavToolkit `navtk.navutils` functions Cobra calls (same formulas and conventions, so results
// match the Python original to rounding). See NavToolkit's coordinate-frames documentation.
//
// Conventions: quaternion q = [a, b, c, d] with a = cos(phi/2) (scalar first); quat_to_dcm(q)
// returns C_B^A ("sensor to nav" when q is the PVA attitude quaternion); rpy are 3-2-1 Euler
// angles describing the frame rotation A -> B, and rpy_to_dcm returns C_B^A.
#pragma once

#include <pntos/api/types.hpp>

namespace pntos::cobra::nav {

// WGS-84
constexpr double SEMI_MAJOR_RADIUS = 6378137.0;
constexpr double FLATTENING = 1.0 / 298.257223563;
constexpr double ECCENTRICITY_SQUARED = 0.00669437999013;
constexpr double ROTATION_RATE = 7.2921151467e-5;  ///< Earth spin rate (rad/s), a.k.a. OMEGA_E
constexpr double PI = 3.14159265358979323846;

using api::Matrix;
using api::Matrix3;
using api::Vector;
using api::Vector3;
using Vector4 = Eigen::Vector4d;

double meridian_radius(double latitude);
double transverse_radius(double latitude);

/// Small latitude difference (rad) -> north distance (m) at approx_lat / altitude.
double delta_lat_to_north(double delta_lat, double approx_lat, double altitude);
double delta_lon_to_east(double delta_lon, double approx_lat, double altitude);
double north_to_delta_lat(double north_distance, double approx_lat, double altitude);
double east_to_delta_lon(double east_distance, double approx_lat, double altitude);

Matrix3 skew(const Vector3& v);
Matrix3 quat_to_dcm(const Vector4& q);
Matrix3 quat_to_dcm(const Vector& q);  ///< convenience for dynamic 4-vectors
Vector4 dcm_to_quat(const Matrix3& dcm);
Matrix3 rpy_to_dcm(const Vector3& rpy);
Vector3 dcm_to_rpy(const Matrix3& dcm);
Vector4 rpy_to_quat(const Vector3& rpy);

/// Apply small-angle tilt errors to a DCM: B(tilt) * dcm with B = I - sin(m)/m [t]x + (1-cos m)/m² [t]x².
Matrix3 correct_dcm_with_tilt(const Matrix3& dcm, const Vector3& tilt);
/// Iteratively re-orthonormalise a DCM (Savage one-step correction, iterated).
Matrix3 ortho_dcm(const Matrix3& dcm);

/// d/d(tilt) of ortho_dcm(C (I + [tilt]x)) in direction dtilt (3×3 generator), see NavToolkit.
Matrix3 d_ortho_dcm_wrt_tilt(const Matrix3& C_nav_to_platform, const Vector3& tilts, const Matrix3& dtilt);
/// Jacobian of dcm_to_rpy(abᵀ) w.r.t. the parameters that produce dx, dy, dz = d(ab)/d(param).
Matrix3 d_dcm_to_rpy(const Matrix3& ab, const Matrix3& dx, const Matrix3& dy, const Matrix3& dz);
Matrix3 d_dcm_to_rpy(const Matrix3& a, const Matrix3& dadx, const Matrix3& dady, const Matrix3& dadz,
                     const Matrix3& b, const Matrix3& dbdx, const Matrix3& dbdy, const Matrix3& dbdz);
Matrix3 d_rpy_to_dcm_wrt_r(const Vector3& rpy);
Matrix3 d_rpy_to_dcm_wrt_p(const Vector3& rpy);
Matrix3 d_rpy_to_dcm_wrt_y(const Vector3& rpy);

/// Schwartz (GRS80) gravity magnitude at ellipsoidal altitude and latitude; returns [0, 0, g].
Vector3 calculate_gravity_schwartz(double alt, double lat);

/// Earth-related quantities at a position/velocity (port of navtk::filtering::EarthModel).
struct EarthModel {
  EarthModel(const Vector3& pos_llh, const Vector3& vel_ned);
  double lat, alt, v_n, v_e;
  double sin_l, cos_l, tan_l, sec_l, sin_2l;
  double r_n, r_e, r_zero;
  double lat_factor, lon_factor;  ///< metres per radian of latitude / longitude at this point
  Vector3 omega_en_n, omega_ie_n, omega_in_n;
  Vector3 g_n;
};

// --- additions for the inertial / alignment port (NavToolkit navigation.hpp, gravity.cpp, math.cpp)

/// Roll, pitch, yaw (rad) from a quaternion [w, x, y, z] (C_platform_to_nav convention).
Vector3 quat_to_rpy(const Vector4& q);
/// Normalised quaternion.
Vector4 quat_norm(const Vector4& q);
/// Sixth-order rotation-vector -> DCM (NavToolkit `rot_vec_to_dcm`).
Matrix3 rot_vec_to_dcm(const Vector3& phi);
/// Rodrigues rotation about `axis` (normalised internally) by `angle` (rad).
Matrix3 axis_angle_to_dcm(const Vector3& axis, double angle);
/// Titterton & Weston 1967-model gravity, NED, with R0 = sqrt(Rn Re).
Vector3 calculate_gravity_titterton(double alt, double lat, double R0);
/// Wrap an angle into (-pi, pi].
double wrap_to_pi(double angle);

}  // namespace pntos::cobra::nav
