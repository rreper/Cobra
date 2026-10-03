// Eigen port of NavToolkit's standard (NED, ellipsoidal) strapdown mechanization and the
// `Inertial` wrapper that applies IMU error models (navtk/inertial/{mechanization_standard,
// inertial_functions, Inertial}.{hpp,cpp}). Aiding-altitude and wander-azimuth variants are not
// ported (Cobra does not use them).
#pragma once

#include <pntos/api/types.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <tuple>

namespace pntos::cobra::inertial {

using api::Matrix3;
using api::Timestamp;
using api::Vector3;

enum class GravModel { SCHWARTZ, TITTERTON };
enum class DcmIntegrationMethod { FIRST_ORDER, SIXTH_ORDER, EXPONENTIAL };
enum class IntegrationMethod { RECTANGULAR, TRAPEZOIDAL, SIMPSONS_RULE };

struct MechanizationOptions {
  GravModel grav_model = GravModel::SCHWARTZ;
  DcmIntegrationMethod dcm_method = DcmIntegrationMethod::SIXTH_ORDER;
  IntegrationMethod int_method = IntegrationMethod::TRAPEZOIDAL;
};

/// Earth rotation rate used inside the mechanization (NavToolkit hard-codes this value here,
/// slightly different from nav::ROTATION_RATE, "for consistency with the Kotlin version").
inline constexpr double kMechOmega = 7.292115e-5;

/// Position (lat, lon rad; alt m HAE), NED velocity (m/s), C_sensor_to_NED at `time`.
struct StandardPva {
  Timestamp time{0};
  Vector3 llh = Vector3::Zero();
  Vector3 vned = Vector3::Zero();
  Matrix3 C_s_to_n = Matrix3::Identity();
};

/// IMU error parameters at `time`: meas = (1 + sf) * true + bias.
struct ImuErrors {
  Vector3 accel_biases = Vector3::Zero();        ///< m/s²
  Vector3 gyro_biases = Vector3::Zero();         ///< rad/s
  Vector3 accel_scale_factors = Vector3::Zero(); ///< unitless
  Vector3 gyro_scale_factors = Vector3::Zero();  ///< unitless
  Timestamp time{0};
};

/// Specific force in NED (m/s²) from integrated increments over dt with the coning/sculling
/// cross-term 0.5 * dth x dv.
Vector3 calc_force_ned(const Matrix3& C_s_to_n, double dt, const Vector3& dth, const Vector3& dv);
/// Mean specific force between two solutions: (v2 - v1)/dt minus the Coriolis/gravity offset at pva1.
Vector3 calc_force_ned(const StandardPva& pva1, const StandardPva& pva2);
/// Sensor rotation rate w.r.t. the nav frame, in the sensor frame (rad/s).
Vector3 calc_rot_rate(const Matrix3& C_s_to_n0, double r_e, double r_n, double alt0, double cos_l, double dt,
                      const Vector3& dth, double sin_l, double tan_l, const Vector3& v_ned0,
                      double omega = nav::ROTATION_RATE);
Vector3 calc_rot_rate(const StandardPva& pva, double dt, const Vector3& dth);
/// Small-angle mean rotation rate between two solutions (rpy of C_s1_to_s2 over dt).
Vector3 calc_rot_rate(const StandardPva& pva1, const StandardPva& pva2);
/// Coriolis + transport + gravity offset: accel_ned = f_ned + offset (Titterton 3.76-3.78).
Vector3 calc_force_and_acceleration_offset(double r_e, double r_n, double alt0, double cos_l, const Vector3& g,
                                           double sec_l, double sin_l, const Vector3& v_ned0,
                                           double omega = nav::ROTATION_RATE);

/// One mechanization step: returns (llh1, vned1, C_s_to_n1).
std::tuple<Vector3, Vector3, Matrix3> mechanization_standard(const Vector3& dv_s, const Vector3& dth_s, double dt,
                                                             const Vector3& llh0, const Matrix3& C_s_to_n0,
                                                             const Vector3& v_ned0, const Vector3& v_ned_prev,
                                                             const MechanizationOptions& opts = {});

/// Holds the current and previous solution and the IMU error model; mechanizes integrated IMU
/// increments (delta-v in m/s, delta-theta in rad) time-stamped at the end of their interval.
class Inertial {
 public:
  explicit Inertial(StandardPva start = {}, MechanizationOptions opts = {});

  void reset(const StandardPva& new_pva, const std::optional<StandardPva>& old_pva = std::nullopt);
  const StandardPva& solution() const { return pva_; }
  const StandardPva& previous_solution() const { return pva_old_; }

  const ImuErrors& imu_errors() const { return errors_; }
  void set_imu_errors(const ImuErrors& e) { errors_ = e; }

  /// dt is taken from the solution time to `time`; the increments are corrected for biases and
  /// scale factors before mechanizing.
  void mechanize(Timestamp time, const Vector3& delta_v, const Vector3& delta_theta);

 private:
  StandardPva pva_, pva_old_;
  ImuErrors errors_;
  MechanizationOptions opts_;
};

}  // namespace pntos::cobra::inertial
