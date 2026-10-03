#include <pntos/cobra/state_modeling/SimpleStateBlocks.hpp>

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Matrix;
using api::Vector;

namespace {
double dt_seconds(api::Timestamp from, api::Timestamp to) {
  return static_cast<double>(to.elapsed_nsec - from.elapsed_nsec) * 1e-9;
}
}  // namespace

// ----------------------------------------------------------------------------- FogmBlock

FogmBlock::FogmBlock(std::string label, api::Mediator* mediator, const Vector& sigmas, const Vector& taus)
    : label_(std::move(label)), mediator_(mediator) {
  if (sigmas.size() == 0 || taus.size() == 0) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, "FogmBlock sigmas or taus arguments are empty.");
    throw std::invalid_argument("FogmBlock sigmas or taus arguments are empty.");
  }
  if (sigmas.size() != taus.size()) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, "FogmBlock sigmas and taus have a size mismatch.");
    throw std::invalid_argument("FogmBlock sigmas and taus have a size mismatch.");
  }
  if ((taus.array() <= 0.0).any()) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, "FogmBlock taus arguments must be positive.");
    throw std::invalid_argument("FogmBlock taus arguments must be positive.");
  }
  const Eigen::Index n = taus.size();
  F_ = Matrix::Zero(n, n);
  Q_ = Matrix::Zero(n, n);
  for (Eigen::Index i = 0; i < n; ++i) {
    F_(i, i) = -1.0 / taus(i);
    Q_(i, i) = 2.0 * sigmas(i) * sigmas(i) / taus(i);
  }
}

void FogmBlock::receive_aux_data(const api::AuxData&) {
  if (mediator_) mediator_->log_message(LoggingLevel::WARN, "FogmBlock does not require aux data.");
}

std::optional<api::StandardDynamicsModel> FogmBlock::generate_dynamics(const api::GenXandP&, api::Timestamp from,
                                                                       api::Timestamp to) {
  const double dt = dt_seconds(from, to);
  const Eigen::Index n = F_.rows();
  Matrix Phi = Matrix::Zero(n, n);
  for (Eigen::Index i = 0; i < n; ++i) Phi(i, i) = std::exp(F_(i, i) * dt);  // expm of a diagonal matrix
  Matrix Qd = 0.5 * (Phi * Q_ * Phi.transpose() + Q_) * dt;
  api::StandardDynamicsModel model;
  model.Phi = Phi;
  model.Qd = Qd;
  model.g = [Phi](const Vector& x) { return Vector(Phi * x); };
  return model;
}

// ---------------------------------------------------------------------- ConstantStateBlock

ConstantStateBlock::ConstantStateBlock(std::string label, api::Mediator* mediator, std::size_t num_states,
                                       const std::optional<Matrix>& Q)
    : label_(std::move(label)), mediator_(mediator), n_(num_states) {
  const auto n = static_cast<Eigen::Index>(num_states);
  Q_ = Q ? *Q : Matrix::Zero(n, n);
  if (Q_.rows() != n || Q_.cols() != n) throw std::invalid_argument("ConstantStateBlock Q must be num_states square");
}

std::optional<api::StandardDynamicsModel> ConstantStateBlock::generate_dynamics(const api::GenXandP&,
                                                                                api::Timestamp from,
                                                                                api::Timestamp to) {
  const double dt = dt_seconds(from, to);
  const auto n = static_cast<Eigen::Index>(n_);
  api::StandardDynamicsModel model;
  model.Phi = Matrix::Identity(n, n);
  model.Qd = Q_ * dt;  // first-order discretisation
  model.g = [](const Vector& x) { return x; };
  return model;
}

// ---------------------------------------------------------------------- ClockBiasStateBlock

ClockBiasStateBlock::ClockBiasStateBlock(std::string label, api::Mediator* mediator, double h_0, double h_neg2,
                                         std::optional<double> q3)
    : label_(std::move(label)), mediator_(mediator), h_0_(h_0), h_neg2_(h_neg2), q3_(q3) {}

Matrix ClockBiasStateBlock::hwang_brown_q(double h_0, double h_n2, double dt) {
  const double pi2 = std::numbers::pi * std::numbers::pi;
  Matrix Q = Matrix::Zero(3, 3);
  Q(0, 0) = 0.5 * h_0 * dt + (2.0 / 3.0) * pi2 * h_n2 * dt * dt * dt;
  Q(0, 1) = pi2 * h_n2 * dt * dt;
  Q(1, 0) = Q(0, 1);
  Q(1, 1) = 2.0 * pi2 * h_n2 * dt;
  return Q;
}

std::optional<api::StandardDynamicsModel> ClockBiasStateBlock::generate_dynamics(const api::GenXandP&,
                                                                                 api::Timestamp from,
                                                                                 api::Timestamp to) {
  const double dt = dt_seconds(from, to);
  api::StandardDynamicsModel model;
  Matrix Qd = hwang_brown_q(h_0_, h_neg2_, dt);
  if (!q3_) {
    model.Phi = Matrix(2, 2);
    model.Phi << 1.0, dt, 0.0, 1.0;
    model.Qd = Qd.topLeftCorner(2, 2);
  } else {
    const double q3 = *q3_;
    model.Phi = Matrix(3, 3);
    model.Phi << 1.0, dt, 0.5 * dt * dt, 0.0, 1.0, dt, 0.0, 0.0, 1.0;
    Matrix q_three(3, 3);
    const double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;
    q_three << q3 * dt5 / 20.0, q3 * dt4 / 8.0, q3 * dt3 / 6.0,  //
        q3 * dt4 / 8.0, q3 * dt3 / 3.0, q3 * dt2 / 2.0,          //
        q3 * dt3 / 6.0, q3 * dt2 / 2.0, q3 * dt;
    model.Qd = Qd + q_three;
  }
  const Matrix Phi = model.Phi;
  model.g = [Phi](const Vector& x) { return Vector(Phi * x); };
  return model;
}

}  // namespace pntos::cobra
