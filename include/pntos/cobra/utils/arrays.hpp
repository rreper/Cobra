// Matrix validation helpers (port of pntos.cobra.utils.arrays).
//
// Policy: shape errors are programming errors and throw std::invalid_argument (the Python code
// mostly relied on numpy raising ValueError for the same conditions, which its tests assert on).
#pragma once

#include <pntos/api/common.hpp>

#include <stdexcept>
#include <string>

namespace pntos::cobra::utils {

/// Throw std::invalid_argument unless `m` has the given shape (negative = don't care).
inline void require_shape(const api::Matrix& m, const std::string& name, Eigen::Index rows, Eigen::Index cols) {
  if ((rows >= 0 && m.rows() != rows) || (cols >= 0 && m.cols() != cols)) {
    throw std::invalid_argument("Expected shape (" + std::to_string(rows) + ", " + std::to_string(cols) + ") for " +
                                name + ", but got (" + std::to_string(m.rows()) + ", " + std::to_string(m.cols()) +
                                ")");
  }
}

inline void require_size(const api::Vector& v, const std::string& name, Eigen::Index size) {
  if (size >= 0 && v.size() != size) {
    throw std::invalid_argument("Expected " + std::to_string(size) + " elements for " + name + ", but got " +
                                std::to_string(v.size()));
  }
}

/// Validate and log (instead of throwing) — mirrors validate_array(err=False).
inline bool check_shape(const api::Matrix& m, api::Mediator& mediator, const std::string& name, Eigen::Index rows,
                        Eigen::Index cols) {
  if ((rows >= 0 && m.rows() != rows) || (cols >= 0 && m.cols() != cols)) {
    mediator.log_message(api::LoggingLevel::ERROR, "Expected shape (" + std::to_string(rows) + ", " +
                                                       std::to_string(cols) + ") for " + name + ", but got (" +
                                                       std::to_string(m.rows()) + ", " + std::to_string(m.cols()) +
                                                       ")");
    return false;
  }
  return true;
}

/// True if `mat` is square and equal to its transpose within tolerances (numpy allclose semantics:
/// |a - b| <= atol + rtol * |b|). Logs an error and returns false if not square.
inline bool is_symmetric(const api::Matrix& mat, api::Mediator* mediator = nullptr, double rtol = 1e-5,
                         double atol = 1e-8) {
  if (mat.rows() != mat.cols()) {
    if (mediator)
      mediator->log_message(api::LoggingLevel::ERROR, "Expected matrix to be 2D and square but got shape (" +
                                                          std::to_string(mat.rows()) + ", " +
                                                          std::to_string(mat.cols()) + ")");
    return false;
  }
  for (Eigen::Index i = 0; i < mat.rows(); ++i)
    for (Eigen::Index j = 0; j < mat.cols(); ++j)
      if (std::abs(mat(i, j) - mat(j, i)) > atol + rtol * std::abs(mat(j, i))) return false;
  return true;
}

/// Validate a manually-configured EstimateWithCovariance against a block size, converting a
/// 1-D covariance (variances) into a diagonal matrix. Logs and returns nullopt on failure.
std::optional<api::EstimateWithCovariance> validate_manual_ewc(const api::EstimateWithCovariance& ewc,
                                                               std::size_t num_states, api::Mediator& mediator);

}  // namespace pntos::cobra::utils
