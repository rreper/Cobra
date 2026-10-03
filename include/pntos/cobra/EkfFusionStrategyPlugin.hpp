// Extended Kalman Filter fusion strategy (port of pntos.cobra.EkfFusionStrategyPlugin).
#pragma once

#include <pntos/api/fusion_strategy.hpp>

namespace pntos::cobra {

/// Textbook EKF over a growable state vector x (N×1) and covariance P (N×N).
///
/// Differences from the Python original (deliberate, see docs/COBRA_ANALYSIS.md §12):
///  * shape mismatches throw std::invalid_argument instead of logging + numpy ValueError;
///  * the covariance update uses the Joseph form (I-KH)P(I-KH)^T + KRK^T and a Cholesky solve for
///    the gain, which is numerically safer than (I-KH)P with an explicit inverse. The results agree
///    with the simple form to rounding for well-conditioned problems.
class EkfFusionStrategy final : public api::StandardFusionStrategy {
 public:
  explicit EkfFusionStrategy(api::Mediator* mediator);

  std::size_t num_states() const override { return static_cast<std::size_t>(x_.size()); }

  std::size_t add_states(const api::Vector& initial_estimate, const api::Matrix& initial_covariance,
                         const std::optional<api::Matrix>& cross_covariance = std::nullopt) override;
  void remove_states(std::size_t first_index, std::size_t count) override;

  std::optional<api::Vector> estimate() const override;
  void set_estimate_slice(const api::Vector& new_estimate, std::size_t first_index) override;

  std::optional<api::Matrix> covariance() const override;
  void set_covariance_slice(const api::Matrix& new_covariance, std::size_t first_row,
                            const std::optional<std::size_t>& first_col = std::nullopt) override;

  void propagate(const api::StandardDynamicsModel& dynamics_model) override;
  void update(const api::StandardMeasurementModel& measurement_model) override;

  std::unique_ptr<api::StandardFusionStrategy> clone() const override;

  /// Use the simple (I-KH)P covariance update instead of Joseph form (bit-compat mode for goldens).
  void set_joseph_form(bool enable) { joseph_ = enable; }

 private:
  void symmetrize_covariance(double rtol = 1e-5, double atol = 1e-8);

  api::Vector x_;
  api::Matrix P_;
  api::Mediator* mediator_;
  bool joseph_ = true;
};

/// Factory producing EkfFusionStrategy instances.
class EkfFusionStrategyPlugin final : public api::FusionStrategyPlugin {
 public:
  /// `joseph_form`: covariance update form for every strategy created (default Joseph; false gives
  /// Python's (I - K H) P arithmetic).
  explicit EkfFusionStrategyPlugin(std::string identifier, bool joseph_form = true)
      : identifier_(std::move(identifier)), joseph_form_(joseph_form) {}

  void init_plugin(const std::optional<std::string>& plugin_resources_location, api::Mediator* mediator) override {
    mediator_ = mediator;
  }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }

  bool is_fusion_type_supported(api::FusionType type) const override { return type == api::FusionType::STANDARD; }
  std::unique_ptr<api::StandardFusionStrategy> new_fusion_strategy(api::FusionType type) override;

 private:
  std::string identifier_;
  api::Mediator* mediator_ = nullptr;
  bool joseph_form_;
};

}  // namespace pntos::cobra
