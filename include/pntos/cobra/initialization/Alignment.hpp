// Eigen port of NavToolkit's static alignment classes (navtk/inertial/{AlignBase, StaticAlignment,
// ManualHeadingAlignment, quaternion_static_alignment}, navtk/filtering/containers/ImuModel).
#pragma once

#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/inertial/Mechanization.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>
#include <aspn23/eigen/MeasurementPosition.hpp>

#include <functional>
#include <vector>

namespace pntos::cobra::inertial {

/// IMU stochastic model (NavToolkit ImuModel). Units as ImuConfig.
struct ImuModel {
  Vector3 accel_random_walk_sigma = Vector3::Zero();
  Vector3 gyro_random_walk_sigma = Vector3::Zero();
  Vector3 accel_bias_sigma = Vector3::Zero();
  Vector3 accel_bias_tau = Vector3::Zero();
  Vector3 gyro_bias_sigma = Vector3::Zero();
  Vector3 gyro_bias_tau = Vector3::Zero();
  Vector3 accel_scale_factor = Vector3::Zero();
  Vector3 gyro_scale_factor = Vector3::Zero();
  Vector3 accel_bias_initial_sigma = Vector3::Zero();
  Vector3 gyro_bias_initial_sigma = Vector3::Zero();

  static ImuModel from_config(const ImuConfig& c);
  ImuConfig to_config(const std::string& group) const;
};
ImuModel hg1700_model();
ImuModel hg9900_model();
ImuModel stim300_model();

/// C_sensor_to_NED from averaged delta-v and delta-theta of a stationary IMU (gyro-compassing).
/// Throws std::runtime_error if the internal consistency checks fail.
Matrix3 quaternion_static_alignment(const Vector3& dv_avg, const Vector3& dth_avg);

/// First-order propagation of a covariance through an RPY-valued function, using NavToolkit's
/// tilt-based numerical Jacobian with perturbation `eps` (absolute, per element).
api::Matrix first_order_rpy_covariance(const api::Vector& x, const api::Matrix& P,
                                       const std::function<Vector3(const api::Vector&)>& f, double eps = 0.01);

enum class AlignmentStatus { ALIGNING_COARSE, ALIGNING_FINE, ALIGNED_GOOD };
enum class MotionNeeded { NO_MOTION, MOTION_NEEDED, ANY_MOTION };

/// Position, velocity, C_nav_to_sensor and time of an alignment (NavToolkit NavSolution/Pose).
struct NavSolution {
  Vector3 pos = Vector3::Zero();
  Vector3 vel = Vector3::Zero();
  Matrix3 rot_mat = Matrix3::Identity();  ///< C_nav_to_sensor (NavToolkit convention)
  Timestamp time{0};
};

class AlignBase {
 public:
  virtual ~AlignBase() = default;
  AlignBase(bool supports_static, bool supports_dynamic, ImuModel model);

  virtual AlignmentStatus process(const api::AspnBase& message) = 0;
  AlignmentStatus check_alignment_status() const { return status_; }
  virtual std::pair<bool, NavSolution> get_computed_alignment() const { return computed_; }
  /// 15×15 Pinson-block covariance; `first` false until aligned.
  virtual std::pair<bool, api::Matrix> get_computed_covariance() const { return {false, api::Matrix::Zero(15, 15)}; }
  virtual std::pair<bool, ImuErrors> get_imu_errors() const { return {false, ImuErrors{}}; }
  virtual MotionNeeded motion_needed() const = 0;

 protected:
  api::Matrix bias_stats_from_model() const;  ///< 6×6 diag(initial bias sigmas²)
  AlignmentStatus status_ = AlignmentStatus::ALIGNING_COARSE;
  std::pair<bool, NavSolution> computed_{false, NavSolution{}};
  std::vector<aspn23_eigen::MeasurementImu> align_buffer_;
  std::vector<aspn23_eigen::MeasurementPosition> gps_buffer_;
  bool supports_static_, supports_dynamic_;
  ImuModel model_;
};

/// Stationary alignment: average `align_time` seconds of IMU, gyro-compass, take the mean position.
class StaticAlignment : public AlignBase {
 public:
  explicit StaticAlignment(ImuModel model = stim300_model(), double align_time = 120.0,
                           const Matrix3& vel_cov = Matrix3::Identity() * 1e-4);
  AlignmentStatus process(const api::AspnBase& message) override;
  std::pair<bool, api::Matrix> get_computed_covariance() const override;
  MotionNeeded motion_needed() const override { return MotionNeeded::NO_MOTION; }

 protected:
  virtual void calc_alignment(Timestamp imu_time);
  std::pair<Vector3, Matrix3> pos_stats() const;
  std::pair<Vector3, Vector3> calc_avg() const;
  double calc_average_delta_time() const;
  double align_time_;
  Matrix3 tilt_cov_ = Matrix3::Zero();
  Matrix3 vel_cov_;

 private:
  bool sufficient_data() const;
};

/// Static levelling with a user-supplied heading; also estimates accel/gyro biases.
class ManualHeadingAlignment final : public StaticAlignment {
 public:
  ManualHeadingAlignment(double heading, double heading_sigma = 0.017453292519943295, ImuModel model = stim300_model(),
                         double align_time = 120.0, const Matrix3& vel_cov = Matrix3::Identity() * 1e-4);
  AlignmentStatus process(const api::AspnBase& message) override;
  std::pair<bool, api::Matrix> get_computed_covariance() const override;
  std::pair<bool, ImuErrors> get_imu_errors() const override;

 private:
  void calc_align();
  double heading_, heading_sigma_;
};

}  // namespace pntos::cobra::inertial
