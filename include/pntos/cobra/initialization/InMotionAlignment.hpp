// Alignment in motion (C++ addition; NavToolkit and Cobra have no counterpart).
//
// The static alignments need a heading from outside (ManualHeadingAlignment) or a gyro that senses the Earth's
// rate (StaticAlignment). This one finds the whole attitude, the heading included, from the receiver's velocity
// and the IMU while the vehicle drives, and the gyro bias with it. It makes no assumption about how the sensor
// is mounted or about the direction of travel. DESIGN.md section 7.24.
//
// For the velocity epochs i = 0..n of a window, with X(t) the body rotation since the window start (gyros, bias
// b removed) and f the specific force:
//
//   x_i = integral of X f dt  (+ the antenna lever arm's part)                 body frame at the window start
//   y_i = v_i - v_0 - integral of (g - 2 w_ie x v) dt                          NED, from the receiver
//   y_i = C0 (x_i + G_i db + Theta_i ba) + c + noise
//
// C0 is the attitude at the window start, db the gyro-bias correction (G_i = dx_i/db), ba the accelerometer
// bias (-Theta_i = integral of X = dx_i/dba) and c absorbs the noise of the first epoch. C0 is the solution of
// Wahba's problem (SVD), which has no linearisation in the heading; the rest is linear least squares with the
// IMU model's initial sigmas as priors; the two alternate. A level accelerometer bias is a tilt as long as the
// vehicle does not turn; leaving it out lets a few milli-g pass for a gyro bias about the vertical.
//
// A receiver's velocity is older than its time stamp by the delay of the receiver's own filter (0.25 s for the
// u-blox of the example recording), and a tenth of a second of that left unaccounted for costs the heading more
// than anything else does (DESIGN.md section 7.29). With `time_offset_prior_sigma` the delay tau is one more unknown:
// the epochs are evaluated at t_i - tau and the linear step has the column -C0 X_i f_i (the specific force at
// the epoch) for its correction; tau is carried from one solution to the next like the gyro bias.
#pragma once

#include <pntos/cobra/initialization/Alignment.hpp>

#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>

namespace pntos::cobra::inertial {

struct InMotionAlignmentOptions {
  enum class VelocitySource {
    AUTO,      ///< the receiver's velocity once a velocity message has arrived, position differences until then
    VELOCITY,  ///< MeasurementVelocity (NED) and the velocity of PVA messages only
    POSITION,  ///< differences of consecutive position fixes only
  };
  double heading_sigma = 3.0 * 0.017453292519943295;  ///< rad; aligned once the heading sigma is below this
  double max_window = 30.0;                           ///< s of data one solution uses
  int min_epochs = 5;                                 ///< velocity epochs after the first, before a solution counts
  Vector3 lever_arm = Vector3::Zero();                ///< m, antenna relative to the IMU, in the IMU (platform) frame
  VelocitySource velocity_source = VelocitySource::AUTO;
  /// m; horizontal sigma of the difference of two consecutive fixes (their errors are mostly common), vertical
  /// twice that. The velocity derived from it has this sigma divided by the fix interval.
  double position_difference_sigma = 0.15;
  double stationary_speed = 0.25;         ///< m/s; below it, with a quiet gyro, the vehicle stands
  double stationary_rate = 0.02;          ///< rad/s; largest rate, and deviation from the mean, of a quiet gyro
  double stationary_time = 3.0;           ///< s of standstill before the gyro mean is taken as the bias
  std::optional<double> start_time;       ///< s; messages before it are ignored (a switch-on inside a recording)
  /// s; above zero the time offset of the velocity (how much older it is than its time stamp) is estimated
  /// along, with this sigma around zero as its prior. Zero: the stamps are taken as they are. At most 0.5.
  double time_offset_prior_sigma = 0.0;
  /// s; with the offset estimated, aligned only once its sigma is below this as well (as heading_sigma for the
  /// heading). An offset that the data has not told yet is half the prior's zero.
  double time_offset_sigma = 0.08;
};

class InMotionAlignment final : public AlignBase {
 public:
  struct Diagnostics {
    int epochs = 0;                  ///< velocity epochs after the first in the window last solved
    double window = 0;               ///< its length, s
    double heading_sigma = std::numeric_limits<double>::infinity();  ///< rad, of the last solution
    double chi2_per_dof = 0;         ///< of the last solution (the covariance is scaled by it above 1)
    double stationary = 0;           ///< s of standstill behind the gyro-bias prior
    Vector3 gyro_bias = Vector3::Zero();  ///< rad/s, current estimate
    Vector3 accel_bias = Vector3::Zero(); ///< m/s^2, of the last solution
    double elapsed = 0;                   ///< s from the first message used to the alignment
    bool from_positions = false;          ///< the velocity came from position differences
    double time_offset = 0;               ///< s; how much older than its stamps the velocity was found to be (0: not estimated)
    double time_offset_sigma = 0;         ///< s, of the last solution
    std::uint64_t solutions = 0;          ///< windows solved
    std::uint64_t restarts = 0;           ///< windows abandoned (IMU gap, change of velocity source)
    std::uint64_t held = 0;               ///< IMU samples that stood for lost ones
    std::uint64_t ignored = 0;            ///< messages not used (out of order, wrong frame, not finite)
  };

  explicit InMotionAlignment(ImuModel model = stim300_model(), InMotionAlignmentOptions options = {});

  AlignmentStatus process(const api::AspnBase& message) override;
  /// 15x15: position (fix covariance), velocity, tilt (full 3x3, NED), accelerometer bias, gyro bias.
  std::pair<bool, api::Matrix> get_computed_covariance() const override { return {computed_.first, covariance_}; }
  /// The gyro and accelerometer biases found (the level accelerometer biases mostly as the model had them).
  std::pair<bool, ImuErrors> get_imu_errors() const override { return {computed_.first, errors_}; }
  MotionNeeded motion_needed() const override { return MotionNeeded::MOTION_NEEDED; }
  bool accepts_velocity() const override { return true; }
  std::string summary() const override;

  const Diagnostics& diagnostics() const { return diag_; }

 private:
  struct Sample {  ///< IMU increments over (t - dt, t]
    std::int64_t t, dt;
    Vector3 dv, dth;
  };
  struct Epoch {  ///< a velocity of the antenna, NED
    std::int64_t t = 0;
    std::int64_t half_span = 0;  ///< the velocity is a mean over t +- half_span (position differences)
    Vector3 v = Vector3::Zero();
    Vector3 var = Vector3::Zero();
    Vector3 accel = Vector3::Zero();  ///< g - 2 w_ie x v at this epoch
    Vector3 corr = Vector3::Zero();   ///< its integral since the first epoch seen
  };
  struct Fix {
    std::int64_t t = 0;
    Vector3 lla = Vector3::Zero();
    Matrix3 cov = Matrix3::Identity();
  };
  struct Solution {
    Matrix3 C = Matrix3::Identity();          ///< body to NED at the last epoch
    Matrix3 P_att = Matrix3::Identity();      ///< its covariance (rotation error in NED)
    Vector3 gyro_bias = Vector3::Zero(), accel_bias = Vector3::Zero();
    Matrix3 P_bias = Matrix3::Identity(), P_accel = Matrix3::Identity();
    Vector3 v_antenna = Vector3::Zero();      ///< fitted antenna velocity at the last epoch
    Vector3 rate = Vector3::Zero();           ///< body rate there, bias removed
    double chi2_per_dof = 0;
    std::int64_t t = 0;                       ///< IMU time C, v_antenna and rate refer to: the last epoch less the offset
    double time_offset = 0, time_offset_var = 0;  ///< s, s^2
    double time_offset_step = 0;              ///< s; what this solution changed it by
  };

  void on_imu(const aspn23_eigen::MeasurementImu& imu);
  void on_velocity(std::int64_t t, const Vector3& v, const Vector3& var);
  void queue(const Epoch& e);
  void on_fix(const Fix& fix);
  void advance();
  void add_epoch(Epoch e);
  bool stationary(const Epoch& a, const Epoch& b, Vector3* dth_sum, Vector3* dv_sum, double* seconds) const;
  void set_bias_prior();
  void trim();
  void restart();
  Vector3 mean_rate(std::int64_t from, std::int64_t to) const;
  Vector3 mean_force(std::int64_t from, std::int64_t to) const;
  std::optional<Solution> solve();
  template <bool kOffset>
  std::optional<Solution> solve_window();
  bool offset_estimated() const { return opt_.time_offset_prior_sigma > 0; }
  void finish(const Solution& s);

  InMotionAlignmentOptions opt_;
  std::optional<std::int64_t> start_ns_;
  std::deque<Sample> imu_;
  std::deque<Epoch> epochs_;   ///< the window
  std::deque<Epoch> pending_;  ///< epochs the IMU has not reached yet
  std::deque<std::int64_t> recent_dt_;
  std::optional<std::int64_t> last_imu_ns_, first_ns_;
  std::optional<Fix> fix_, previous_fix_;
  bool velocity_seen_ = false;
  Vector3 gravity_ = Vector3::Zero(), wie_ = Vector3::Zero();

  Vector3 bias_ = Vector3::Zero();        ///< current gyro-bias estimate
  Vector3 bias_prior_ = Vector3::Zero();
  Vector3 bias_prior_var_ = Vector3::Ones();
  Vector3 static_dth_ = Vector3::Zero(), static_dv_ = Vector3::Zero();
  double static_seconds_ = 0;
  double offset_ = 0;  ///< s, current estimate of the time offset (0 unless estimated)

  api::Matrix covariance_ = api::Matrix::Zero(15, 15);
  ImuErrors errors_;
  Diagnostics diag_;
};

}  // namespace pntos::cobra::inertial
