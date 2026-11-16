// The standard measurement processors (ports of pntos.cobra.internal.*MeasurementProcessor).
//
// All Pinson-style processors expect, via receive_aux_data, the inertial PVA (geodetic, with
// quaternion) valid at the measurement time; the body-velocity processor also expects an IMU
// message with rotation rates. generate_model returns nullopt (after logging) on any violated
// precondition, exactly like the Python originals.
#pragma once

#include <cstdint>

#include <pntos/api/state_modeling.hpp>
#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/geoid.hpp>

#include <aspn23/eigen/MeasurementAltitude.hpp>
#include <aspn23/eigen/MeasurementDirection3DToPoints.hpp>
#include <aspn23/eigen/MeasurementVelocity.hpp>

namespace pntos::cobra {

/// Common plumbing: label, block labels, mediator, PVA aux handling.
class PinsonProcessorBase : public api::StandardMeasurementProcessor {
 public:
  PinsonProcessorBase(std::string name, std::string label, std::vector<std::string> state_block_labels,
                      api::Mediator* mediator, std::size_t num_required_blocks);

  const std::string& label() const override { return label_; }
  const std::vector<std::string>& state_block_labels() const override { return state_block_labels_; }
  void receive_aux_data(const api::AuxData& aux) override;

  /// Test hook: the stored inertial PVA aux (nullptr if none).
  std::shared_ptr<const utils::PVA> inertial_pva() const { return inertial_pva_; }

 protected:
  void log(api::LoggingLevel level, const std::string& msg) const;
  /// Checks block count, aux presence and aux/measurement time match. Returns false after logging.
  bool check_common(api::Timestamp meas_time) const;

  std::string name_;
  std::string label_;
  std::vector<std::string> state_block_labels_;
  api::Mediator* mediator_;
  std::size_t num_required_blocks_;
  std::shared_ptr<const utils::PVA> inertial_pva_;
};

/// Geodetic position -> Pinson position/tilt states, with a known lever arm.
/// Variants: kind 0 = pinson_position (1 block), 1 = pinson_with_ned_fogm_position (pinson + 3 FOGM),
/// 2 = pinson_with_lever_arm_position (pinson + 3 FOGM sensor error + 3 lever-arm error).
class PinsonPositionMeasurementProcessor final : public PinsonProcessorBase {
 public:
  enum class Kind { Plain, WithNedFogm, WithLeverArm };
  PinsonPositionMeasurementProcessor(Kind kind, std::string label, std::vector<std::string> state_block_labels,
                                     api::Mediator* mediator, const api::Vector3& l_ps_p);
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<PinsonPositionMeasurementProcessor>(*this);
  }
  Kind kind() const { return kind_; }

 private:
  Kind kind_;
  api::Vector3 l_ps_p_;
};

/// NED velocity -> Pinson velocity error states.
class PinsonVelocityMeasurementProcessor final : public PinsonProcessorBase {
 public:
  PinsonVelocityMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels,
                                     api::Mediator* mediator);
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<PinsonVelocityMeasurementProcessor>(*this);
  }
};

/// C++ addition: an angular rate the vehicle is known to have, in the sensor frame and relative to the
/// navigation frame (zero while it stands: ZeroMotionGeneratorConfig), against what the gyro shows -> Pinson
/// gyro-bias states. Aux data: the inertial's rate over the second around the measurement (raw, the Earth's and
/// the transport rate removed: INERTIAL_FORCES_AND_RATES) and the bias it is applying (INERTIAL_SENSOR_ERRORS).
/// With the bias state x = applied bias - true bias: measurement - (rate - applied bias) = x + noise.
class PinsonZeroRateMeasurementProcessor final : public PinsonProcessorBase {
 public:
  PinsonZeroRateMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels, api::Mediator* mediator);
  void receive_aux_data(const api::AuxData& aux) override;
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<PinsonZeroRateMeasurementProcessor>(*this);
  }

 private:
  std::optional<api::Vector3> rate_, bias_;
  std::int64_t rate_ns_ = 0;
};

/// Geodetic PVA (position + velocity part) -> Pinson position/velocity/tilt states.
class PinsonPosVelMeasurementProcessor final : public PinsonProcessorBase {
 public:
  PinsonPosVelMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels,
                                   api::Mediator* mediator, const api::Vector3& l_ps_p);
  void receive_aux_data(const api::AuxData& aux) override;
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<PinsonPosVelMeasurementProcessor>(*this);
  }

 private:
  api::Vector3 l_ps_p_;
};

/// Sensor-frame (1-3 axis) velocity -> Pinson velocity/tilt states, with lever-arm tangential
/// velocity correction using the gyro rates (aux IMU) and gyro bias states.
class PinsonBodyVelocityMeasurementProcessor : public PinsonProcessorBase {
 public:
  PinsonBodyVelocityMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels,
                                         api::Mediator* mediator, const api::Vector3& l_ps_p,
                                         const nav::Vector4& orientation_ps_p);
  void receive_aux_data(const api::AuxData& aux) override;
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<PinsonBodyVelocityMeasurementProcessor>(*this);
  }
  bool has_force_and_rate_aux() const { return force_and_rate_aux_ != nullptr; }

 protected:
  /// For a derived model that reports under its own name.
  PinsonBodyVelocityMeasurementProcessor(std::string name, std::string label, std::vector<std::string> state_block_labels,
                                         api::Mediator* mediator, const api::Vector3& l_ps_p,
                                         const nav::Vector4& orientation_ps_p);
  api::Vector3 l_ps_p_;
  nav::Vector4 orientation_ps_p_;
  std::shared_ptr<const aspn23_eigen::MeasurementImu> force_and_rate_aux_;
};

/// C++ addition: the body-velocity model for an odometer (wheel speed, Doppler radar, ...) and for the motion
/// constraints of a ground vehicle, with an optional second state block of three calibration states:
///
///   [0] scale factor error of the forward axis (measured = (1 + s) * true)
///   [1] pitch misalignment, [2] yaw misalignment (rad): small rotations of the sensor's axes about the nominal
///       sensor y and z axes (LeverArmOrientationMPConfig::orientation is the nominal mounting)
///
/// Without them a mounting error of a tenth of a degree is a constant lateral and vertical velocity error of
/// 2 mm/s per m/s, which the filter can only hide in other states (docs/DESIGN.md 7.23). The message is a
/// SENSOR-frame MeasurementVelocity; axes that are absent (NaN) are not used, so a wheel speed is (x, NaN, NaN)
/// and the constraints "no sideways, no vertical motion" are (NaN, 0, 0) with the covariance that expresses how
/// well they hold. A covariance of 3x3 is cut down to the axes that are present. With one state block label the
/// model is exactly PinsonBodyVelocityMeasurementProcessor's.
class PinsonOdometryMeasurementProcessor final : public PinsonBodyVelocityMeasurementProcessor {
 public:
  PinsonOdometryMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels, api::Mediator* mediator,
                                     const api::Vector3& l_ps_p, const nav::Vector4& orientation_ps_p);
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<PinsonOdometryMeasurementProcessor>(*this);
  }
  static constexpr int kCalibrationStates = 3;
};

/// Altitude (MeasurementAltitude HAE, geodetic position or PVA) -> Pinson down-position error +
/// a 1-state altitude bias block. MSL altitudes need a geoid model, which is not yet ported.
class AltitudeMeasurementProcessor final : public PinsonProcessorBase {
 public:
  /// `geoid` converts MSL altitudes to HAE (C++ addition); without one, MSL measurements are rejected.
  AltitudeMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels,
                               api::Mediator* mediator, const api::Vector3& l_ps_p,
                               std::shared_ptr<const nav::Geoid> geoid = nullptr);
  bool has_geoid() const { return geoid_ != nullptr; }
  void receive_aux_data(const api::AuxData& aux) override;
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<AltitudeMeasurementProcessor>(*this);
  }
  std::optional<api::Timestamp> inertial_solution_time() const { return inertial_time_; }

 private:
  api::Vector3 l_ps_p_;
  std::shared_ptr<const nav::Geoid> geoid_;
  std::optional<api::Timestamp> inertial_time_;
  api::Vector3 inertial_pos_ = api::Vector3::Zero();
  api::Matrix3 C_platform_to_nav_ = api::Matrix3::Identity();
};

/// Geodetic position -> whole-valued [LLH, v, RPY] states (e.g. via PinsonErrorToStandard VSB)
/// plus 3 NED-frame FOGM sensor-error states.
class PositionMeasurementProcessor final : public api::StandardMeasurementProcessor {
 public:
  PositionMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels,
                               api::Mediator* mediator, const api::Vector3& l_ps_p);
  const std::string& label() const override { return label_; }
  const std::vector<std::string>& state_block_labels() const override { return state_block_labels_; }
  void receive_aux_data(const api::AuxData& aux) override;
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<PositionMeasurementProcessor>(*this);
  }

 private:
  std::string label_;
  std::vector<std::string> state_block_labels_;
  api::Mediator* mediator_;
  api::Vector3 l_ps_p_;
};

/// Sine-space direction observations to known points -> Pinson position/tilt states.
class Direction3DToPointsMeasurementProcessor final : public PinsonProcessorBase {
 public:
  Direction3DToPointsMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels,
                                          api::Mediator* mediator, const api::Vector3& l_ps_p,
                                          const nav::Vector4& orientation);
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<Direction3DToPointsMeasurementProcessor>(*this);
  }

 private:
  api::Vector3 l_ps_p_;
  nav::Vector4 orientation_;
};

}  // namespace pntos::cobra
