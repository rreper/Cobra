// pntOS C++ API — inertial mechanization interfaces and InertialPlugin (unstable in upstream).
#pragma once

#include <pntos/api/common.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>

namespace pntos::api {

enum class InertialFrame : int {
  /// Forces N-E-D in m/s²; rotation rates of the sensor w.r.t. inertial, in the sensor frame (rad/s).
  NED = 0,
};

/// Specific forces (meas_accel) and rotation rates (meas_gyro) repurposing an IMU message.
struct InertialForcesRates {
  std::shared_ptr<aspn23_eigen::MeasurementImu> forces_and_rates;
  InertialFrame frame = InertialFrame::NED;
};

/// Biases and scale factors of a 3-axis gyro / 3-axis accelerometer set, sensor frame.
struct StandardInertialErrors {
  Vector3 accel_biases = Vector3::Zero();        ///< m/s²
  Vector3 gyro_biases = Vector3::Zero();         ///< rad/s
  Vector3 accel_scale_factors = Vector3::Zero(); ///< unitless
  Vector3 gyro_scale_factors = Vector3::Zero();  ///< unitless
};

enum class InertialSolutionRangeType : int {
  BEST_KNOWN_SOLUTION = 0,
  NO_UPDATES_WITHIN_RANGE = 1,
};

enum class InertialType : int {
  STANDARD_MECHANIZATION = 0,
  EXTERNAL = 1,
};

/// Common base for inertials (a mechanization or an external INS).
class CommonInertial {
 public:
  virtual ~CommonInertial() = default;

  virtual AspnMessageType request_solution_message_type() const = 0;
  virtual Message request_current_solution() = 0;
  virtual std::optional<Message> request_solution(Timestamp time) = 0;
  virtual std::optional<std::vector<std::optional<Message>>> request_solutions(
      const std::vector<Timestamp>& times, InertialSolutionRangeType solution_type) = 0;

  virtual bool is_time_in_range(Timestamp time) const = 0;
  virtual Timestamp request_earliest_time() const = 0;
  virtual Timestamp request_latest_time() const = 0;

  virtual std::vector<AspnMessageType> request_process_pntos_message_types() const = 0;
  virtual void process_pntos_message(const Message& message) = 0;

  virtual std::optional<InertialForcesRates> request_forces_and_rates(Timestamp time) = 0;
  virtual std::optional<InertialForcesRates> request_average_forces_and_rates(Timestamp time1, Timestamp time2) = 0;
};

using ExternalInertial = CommonInertial;

/// An inertial that mechanizes raw IMU data and supports resets / error corrections.
class StandardInertialMechanization : public CommonInertial {
 public:
  virtual std::optional<std::vector<AspnMessageType>> request_reset_message_types() const = 0;
  virtual void reset_solution(const Message& message) = 0;
  virtual void correct_sensor_errors(Timestamp time, const StandardInertialErrors& errors) = 0;
  virtual std::optional<StandardInertialErrors> request_sensor_errors(Timestamp time) = 0;
};

/// Factory for inertials.
class InertialPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::INERTIAL; }

  virtual bool is_inertial_type_supported(InertialType type) const = 0;
  /// `solution` is the initial PVA to mechanize from. nullptr if unsupported / invalid.
  virtual std::unique_ptr<CommonInertial> new_inertial(InertialType type, const Message& solution,
                                                       const std::optional<std::string>& config_group = std::nullopt) = 0;
};

}  // namespace pntos::api
