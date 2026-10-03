#include <pntos/cobra/EkfFusionStrategyPlugin.hpp>
#include <pntos/cobra/utils/arrays.hpp>

#include <Eigen/Cholesky>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Matrix;
using api::Vector;

EkfFusionStrategy::EkfFusionStrategy(api::Mediator* mediator) : x_(0), P_(0, 0), mediator_(mediator) {}

std::size_t EkfFusionStrategy::add_states(const Vector& initial_estimate, const Matrix& initial_covariance,
                                          const std::optional<Matrix>& cross_covariance) {
  const Eigen::Index n_new = initial_estimate.size();
  const Eigen::Index n_cur = x_.size();
  utils::require_shape(initial_covariance, "initial_covariance", n_new, n_new);

  Matrix C = Matrix::Zero(n_cur, n_new);
  if (cross_covariance) {
    if (n_cur == 0) throw std::invalid_argument("cross_covariance given but there are no existing states");
    utils::require_shape(*cross_covariance, "cross_covariance", n_cur, n_new);
    C = *cross_covariance;
  }

  Vector x_new(n_cur + n_new);
  x_new << x_, initial_estimate;
  Matrix P_new(n_cur + n_new, n_cur + n_new);
  P_new.topLeftCorner(n_cur, n_cur) = P_;
  P_new.topRightCorner(n_cur, n_new) = C;
  P_new.bottomLeftCorner(n_new, n_cur) = C.transpose();
  P_new.bottomRightCorner(n_new, n_new) = initial_covariance;

  x_ = std::move(x_new);
  P_ = std::move(P_new);
  return static_cast<std::size_t>(n_cur);
}

void EkfFusionStrategy::remove_states(std::size_t first_index, std::size_t count) {
  if (count == 0) {
    if (mediator_) mediator_->log_message(LoggingLevel::WARN, "Attempted to remove 0 states, ignoring.");
    return;
  }
  const auto n = static_cast<std::size_t>(x_.size());
  if (first_index + count > n) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::ERROR, "Tried to remove states [" + std::to_string(first_index) + ":" +
                                                      std::to_string(first_index + count) +
                                                      "] but state vector is only length " + std::to_string(n) + ".");
    return;
  }
  const auto i0 = static_cast<Eigen::Index>(first_index);
  const auto c = static_cast<Eigen::Index>(count);
  const auto n_i = static_cast<Eigen::Index>(n);
  const Eigen::Index n_after = n_i - i0 - c;

  Vector x_new(n_i - c);
  x_new << x_.head(i0), x_.tail(n_after);

  Matrix P_new(n_i - c, n_i - c);
  P_new.topLeftCorner(i0, i0) = P_.topLeftCorner(i0, i0);
  P_new.topRightCorner(i0, n_after) = P_.topRightCorner(i0, n_after);
  P_new.bottomLeftCorner(n_after, i0) = P_.bottomLeftCorner(n_after, i0);
  P_new.bottomRightCorner(n_after, n_after) = P_.bottomRightCorner(n_after, n_after);

  x_ = std::move(x_new);
  P_ = std::move(P_new);
}

std::optional<Vector> EkfFusionStrategy::estimate() const {
  if (x_.size() == 0) return std::nullopt;
  return x_;
}

void EkfFusionStrategy::set_estimate_slice(const Vector& new_estimate, std::size_t first_index) {
  const auto n = new_estimate.size();
  if (static_cast<Eigen::Index>(first_index) + n > x_.size()) {
    throw std::invalid_argument("estimate slice exceeds size of current state vector.");
  }
  x_.segment(static_cast<Eigen::Index>(first_index), n) = new_estimate;
}

std::optional<Matrix> EkfFusionStrategy::covariance() const {
  if (x_.size() == 0) return std::nullopt;
  return P_;
}

void EkfFusionStrategy::set_covariance_slice(const Matrix& new_covariance, std::size_t first_row,
                                             const std::optional<std::size_t>& first_col_opt) {
  const std::size_t first_col = first_col_opt.value_or(first_row);
  const auto r0 = static_cast<Eigen::Index>(first_row);
  const auto c0 = static_cast<Eigen::Index>(first_col);
  if (r0 + new_covariance.rows() > P_.rows() || c0 + new_covariance.cols() > P_.cols()) {
    throw std::invalid_argument("Covariance slice spans [" + std::to_string(first_row) + ":" +
                                std::to_string(r0 + new_covariance.rows()) + ", " + std::to_string(first_col) + ":" +
                                std::to_string(c0 + new_covariance.cols()) + "], but full covariance matrix is only " +
                                std::to_string(P_.rows()) + "x" + std::to_string(P_.cols()) + ".");
  }
  P_.block(r0, c0, new_covariance.rows(), new_covariance.cols()) = new_covariance;
}

void EkfFusionStrategy::symmetrize_covariance(double rtol, double atol) {
  if (!utils::is_symmetric(P_, mediator_, rtol, atol)) P_ = 0.5 * (P_ + P_.transpose()).eval();
}

void EkfFusionStrategy::propagate(const api::StandardDynamicsModel& dynamics_model) {
  symmetrize_covariance();
  const Eigen::Index ns = x_.size();
  utils::require_shape(dynamics_model.Phi, "Phi", ns, ns);
  utils::require_shape(dynamics_model.Qd, "Qd", ns, ns);

  Vector x_new = dynamics_model.g(x_);
  utils::require_size(x_new, "x", ns);
  x_ = std::move(x_new);
  P_ = (dynamics_model.Phi * P_ * dynamics_model.Phi.transpose() + dynamics_model.Qd).eval();
}

void EkfFusionStrategy::update(const api::StandardMeasurementModel& measurement_model) {
  const Eigen::Index num_meas = measurement_model.z.size();
  const Eigen::Index ns = x_.size();
  utils::require_shape(measurement_model.H, "H", num_meas, ns);
  utils::require_shape(measurement_model.R, "R", num_meas, num_meas);

  Vector h_x = measurement_model.h(x_);
  utils::require_size(h_x, "h(x)", num_meas);
  const Vector resid = measurement_model.z - h_x;

  const Matrix& H = measurement_model.H;
  const Matrix S = (H * P_ * H.transpose() + measurement_model.R).eval();
  // K = P H^T S^{-1}  ==>  K^T = S^{-1} H P  (S symmetric)
  Matrix K;
  Eigen::LDLT<Matrix> ldlt(S);
  if (ldlt.info() == Eigen::Success) {
    K = ldlt.solve(H * P_).transpose();
  } else {
    K = P_ * H.transpose() * S.inverse();
  }
  x_ += K * resid;
  const Matrix I = Matrix::Identity(ns, ns);
  if (joseph_) {
    const Matrix IKH = I - K * H;
    P_ = (IKH * P_ * IKH.transpose() + K * measurement_model.R * K.transpose()).eval();
  } else {
    P_ = ((I - K * H) * P_).eval();
  }
}

std::unique_ptr<api::StandardFusionStrategy> EkfFusionStrategy::clone() const {
  return std::make_unique<EkfFusionStrategy>(*this);
}

std::unique_ptr<api::StandardFusionStrategy> EkfFusionStrategyPlugin::new_fusion_strategy(api::FusionType type) {
  if (is_fusion_type_supported(type)) {
    auto s = std::make_unique<EkfFusionStrategy>(mediator_);
    s->set_joseph_form(joseph_form_);
    return s;
  }
  if (mediator_)
    mediator_->log_message(LoggingLevel::ERROR,
                           "Fusion strategy type not currently supported. Make sure to call "
                           "FusionStrategyPlugin.is_fusion_type_supported before requesting a new fusion strategy.");
  return nullptr;
}

}  // namespace pntos::cobra
