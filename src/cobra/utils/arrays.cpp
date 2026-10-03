#include <pntos/cobra/utils/arrays.hpp>

namespace pntos::cobra::utils {

std::optional<api::EstimateWithCovariance> validate_manual_ewc(const api::EstimateWithCovariance& ewc,
                                                               std::size_t num_states, api::Mediator& mediator) {
  using api::LoggingLevel;
  const auto n = static_cast<Eigen::Index>(num_states);
  if (ewc.estimate.size() != n) {
    mediator.log_message(LoggingLevel::ERROR, "Expected estimate to have " + std::to_string(num_states) +
                                                  " states but got " + std::to_string(ewc.estimate.size()) + ".");
    return std::nullopt;
  }
  api::Matrix cov = ewc.covariance;
  if (cov.rows() == n && cov.cols() == 1) {
    // 1-D covariance: interpret as variances on the diagonal (Python: eye(n) * cov)
    api::Matrix diag = api::Matrix::Zero(n, n);
    for (Eigen::Index i = 0; i < n; ++i) diag(i, i) = cov(i, 0);
    cov = diag;
  } else if (cov.rows() != n || cov.cols() != n) {
    if (cov.rows() != cov.cols()) {
      mediator.log_message(LoggingLevel::ERROR, "Expected covariance to be a square matrix but got shape (" +
                                                    std::to_string(cov.rows()) + ", " + std::to_string(cov.cols()) +
                                                    ").");
    } else {
      mediator.log_message(LoggingLevel::ERROR, "Expected covariance to correspond to " + std::to_string(num_states) +
                                                    " states but it instead corresponds to " +
                                                    std::to_string(cov.rows()) + " states.");
    }
    return std::nullopt;
  }
  return api::EstimateWithCovariance{ewc.type, ewc.estimate, cov};
}

}  // namespace pntos::cobra::utils
