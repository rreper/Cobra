// The standard measurement processors (ports of pntos.cobra.internal.*MeasurementProcessor).
//
// All Pinson-style processors expect, via receive_aux_data, the inertial PVA (geodetic, with
// quaternion) valid at the measurement time; the body-velocity processor also expects an IMU
// message with rotation rates. generate_model returns nullopt (after logging) on any violated
// precondition, exactly like the Python originals.
#pragma once

#include <pntos/api/state_modeling.hpp>
#include <pntos/cobra/utils/aspn.hpp>

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
class PinsonBodyVelocityMeasurementProcessor final : public PinsonProcessorBase {
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

 private:
  api::Vector3 l_ps_p_;
  nav::Vector4 orientation_ps_p_;
  std::shared_ptr<const aspn23_eigen::MeasurementImu> force_and_rate_aux_;
};

/// Altitude (MeasurementAltitude HAE, geodetic position or PVA) -> Pinson down-position error +
/// a 1-state altitude bias block. MSL altitudes need a geoid model, which is not yet ported.
class AltitudeMeasurementProcessor final : public PinsonProcessorBase {
 public:
  AltitudeMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels,
                               api::Mediator* mediator, const api::Vector3& l_ps_p);
  void receive_aux_data(const api::AuxData& aux) override;
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<AltitudeMeasurementProcessor>(*this);
  }
  std::optional<api::Timestamp> inertial_solution_time() const { return inertial_time_; }

 private:
  api::Vector3 l_ps_p_;
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
