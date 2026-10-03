// pntOS C++ API — StandardFusionStrategy (the numerical filter core) and FusionStrategyPlugin.
#pragma once

#include <pntos/api/state_modeling.hpp>

namespace pntos::api {

/// Bayesian inference on a linearised discrete-time Gaussian system (e.g. an EKF).
///
/// Manages an estimate x (N×1) and covariance P (N×N) that grow/shrink with add/remove_states and
/// evolve with propagate/update.
class StandardFusionStrategy {
 public:
  virtual ~StandardFusionStrategy() = default;

  virtual std::size_t num_states() const = 0;

  /// Append states; cross_covariance (n_existing × n_new) defaults to zero. Returns the index of
  /// the first new state.
  virtual std::size_t add_states(const Vector& initial_estimate, const Matrix& initial_covariance,
                                 const std::optional<Matrix>& cross_covariance = std::nullopt) = 0;
  virtual void remove_states(std::size_t first_index, std::size_t count) = 0;

  /// nullopt if no states.
  virtual std::optional<Vector> estimate() const = 0;
  virtual void set_estimate_slice(const Vector& new_estimate, std::size_t first_index) = 0;

  virtual std::optional<Matrix> covariance() const = 0;
  /// Overwrite the block at (first_row, first_col); first_col defaults to first_row.
  virtual void set_covariance_slice(const Matrix& new_covariance, std::size_t first_row,
                                    const std::optional<std::size_t>& first_col = std::nullopt) = 0;

  virtual void propagate(const StandardDynamicsModel& dynamics_model) = 0;
  virtual void update(const StandardMeasurementModel& measurement_model) = 0;

  virtual std::unique_ptr<StandardFusionStrategy> clone() const = 0;
};

/// Factory for fusion strategies.
class FusionStrategyPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::FUSION_STRATEGY; }

  virtual bool is_fusion_type_supported(FusionType type) const = 0;
  /// nullptr if unsupported.
  virtual std::unique_ptr<StandardFusionStrategy> new_fusion_strategy(FusionType type) = 0;
};

}  // namespace pntos::api
