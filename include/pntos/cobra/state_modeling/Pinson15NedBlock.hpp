// 15-state Pinson INS error model in the NED frame (port of pntos.cobra.internal.Pinson15NedBlock).
//
// States: 0-2 NED position error (m), 3-5 NED velocity error (m/s), 6-8 NED tilt error (rad),
// 9-11 accel bias (m/s², sensor frame), 12-14 gyro bias (rad/s, sensor frame). Error states are
// additive (true = estimated + error). Based on Titterton & Weston 2nd ed. p.345 with the
// position states converted to NED metres and Schwartz gravity-gradient terms.
#pragma once

#include <pntos/api/state_modeling.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/utils/aspn.hpp>

namespace pntos::cobra {

class Pinson15NedBlock final : public api::StandardStateBlock {
 public:
  Pinson15NedBlock(std::string label, api::Mediator* mediator, const ImuConfig& imu_model, bool legacy_q_rotation = false);

  const std::string& label() const override { return label_; }
  std::size_t num_states() const override { return 15; }

  /// Expects the inertial PVA (geodetic, with quaternion) and an IMU message carrying specific
  /// forces/rates in the NED frame.
  void receive_aux_data(const api::AuxData& aux) override;

  std::optional<api::StandardDynamicsModel> generate_dynamics(const api::GenXandP& gen_x_and_p, api::Timestamp from,
                                                              api::Timestamp to) override;
  std::unique_ptr<api::StandardStateBlock> clone() const override { return std::make_unique<Pinson15NedBlock>(*this); }

  /// Continuous-time F (15×15) from the current aux data.
  api::Matrix generate_f_pinson15() const;
  /// Continuous-time Q (15×15): random-walk / FOGM driving noise rotated into NED.
  /// NOTE: the Python original mutated its stored Q in place on every call (cumulative rotation);
  /// this port rotates a copy, which is the intended behaviour.
  api::Matrix generate_q_pinson15() const;
  /// Rescale the position columns of Phi for the change in rad->m factors between aux PVAs.
  void scale_phi(api::Matrix& Phi) const;

  // Test hooks
  const ImuConfig& imu_model() const { return imu_model_; }
  ImuConfig& imu_model() { return imu_model_; }
  bool has_pva_aux() const { return new_pva_aux_ != nullptr; }
  bool has_force_aux() const { return force_and_rate_aux_ != nullptr; }

 private:
  std::string label_;
  api::Mediator* mediator_;
  ImuConfig imu_model_;
  api::Matrix pre_Q_;  ///< sensor-frame Q, never mutated after construction
  /// PinsonStateBlockConfig::legacy_q_rotation: rotate the stored Q in place like the Python original.
  bool legacy_inplace_q_ = false;

 public:
  bool legacy_q_rotation() const { return legacy_inplace_q_; }
  std::shared_ptr<const utils::PVA> old_pva_aux_;
  std::shared_ptr<const utils::PVA> new_pva_aux_;
  std::shared_ptr<const aspn23_eigen::MeasurementImu> force_and_rate_aux_;
};

}  // namespace pntos::cobra
