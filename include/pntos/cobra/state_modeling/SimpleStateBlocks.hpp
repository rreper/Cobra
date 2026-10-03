// Simple state blocks: FOGM, constant, clock bias (ports of FogmBlock, ConstantStateBlock,
// ClockBiasStateBlock).
#pragma once

#include <pntos/api/state_modeling.hpp>

namespace pntos::cobra {

/// N independent first-order Gauss-Markov processes: F = diag(-1/tau), Q = diag(2 sigma²/tau).
/// Phi = exp(F dt) (exact for the diagonal F), Qd = ½ (Phi Q Phiᵀ + Q) dt.
class FogmBlock final : public api::StandardStateBlock {
 public:
  /// Throws std::invalid_argument if sigmas/taus are empty, mismatched or any tau <= 0.
  FogmBlock(std::string label, api::Mediator* mediator, const api::Vector& sigmas, const api::Vector& taus);

  const std::string& label() const override { return label_; }
  std::size_t num_states() const override { return static_cast<std::size_t>(F_.rows()); }
  void receive_aux_data(const api::AuxData& aux) override;
  std::optional<api::StandardDynamicsModel> generate_dynamics(const api::GenXandP& gen_x_and_p, api::Timestamp from,
                                                              api::Timestamp to) override;
  std::unique_ptr<api::StandardStateBlock> clone() const override { return std::make_unique<FogmBlock>(*this); }

 private:
  std::string label_;
  api::Mediator* mediator_;
  api::Matrix F_;  // diagonal
  api::Matrix Q_;  // diagonal
};

/// States with no dynamics: Phi = I, Qd = Q dt.
class ConstantStateBlock final : public api::StandardStateBlock {
 public:
  ConstantStateBlock(std::string label, api::Mediator* mediator, std::size_t num_states,
                     const std::optional<api::Matrix>& Q = std::nullopt);

  const std::string& label() const override { return label_; }
  std::size_t num_states() const override { return n_; }
  void receive_aux_data(const api::AuxData&) override {}
  std::optional<api::StandardDynamicsModel> generate_dynamics(const api::GenXandP& gen_x_and_p, api::Timestamp from,
                                                              api::Timestamp to) override;
  std::unique_ptr<api::StandardStateBlock> clone() const override {
    return std::make_unique<ConstantStateBlock>(*this);
  }

 private:
  std::string label_;
  api::Mediator* mediator_;
  std::size_t n_;
  api::Matrix Q_;
};

/// Clock bias (s), drift (s/s) and optional drift rate (s/s²) with a Hwang–Brown Allan-variance Q.
class ClockBiasStateBlock final : public api::StandardStateBlock {
 public:
  ClockBiasStateBlock(std::string label, api::Mediator* mediator, double h_0, double h_neg2,
                      std::optional<double> q3 = std::nullopt);

  const std::string& label() const override { return label_; }
  std::size_t num_states() const override { return q3_ ? 3 : 2; }
  void receive_aux_data(const api::AuxData&) override {}
  std::optional<api::StandardDynamicsModel> generate_dynamics(const api::GenXandP& gen_x_and_p, api::Timestamp from,
                                                              api::Timestamp to) override;
  std::unique_ptr<api::StandardStateBlock> clone() const override {
    return std::make_unique<ClockBiasStateBlock>(*this);
  }

  /// Discrete process noise from Brown & Hwang (10.4.1–10.4.3), 3×3 with zero third row/col.
  static api::Matrix hwang_brown_q(double h_0, double h_neg2, double dt);

  /// Test hook (the Python tests poke the private fields).
  void set_q3(std::optional<double> q3) { q3_ = q3; }

 private:
  std::string label_;
  api::Mediator* mediator_;
  double h_0_;
  double h_neg2_;
  std::optional<double> q3_;
};

}  // namespace pntos::cobra
