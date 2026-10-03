// pntOS C++ API — StandardFusionEngine and FusionPlugin.
#pragma once

#include <pntos/api/fusion_strategy.hpp>
#include <pntos/api/state_modeling.hpp>

namespace pntos::api {

/// Bookkeeping layer between modular state blocks / measurement processors and a fixed-size
/// StandardFusionStrategy. Unsafe to use until strategy() is non-null.
class StandardFusionEngine {
 public:
  virtual ~StandardFusionEngine() = default;

  virtual Timestamp time() const = 0;
  virtual void set_time(Timestamp time) = 0;

  virtual StandardFusionStrategy* strategy() const = 0;
  virtual void set_strategy(std::unique_ptr<StandardFusionStrategy> strategy) = 0;

  /// Total real states (virtual blocks excluded).
  virtual std::size_t num_states() const = 0;
  virtual std::optional<std::vector<std::string>> state_block_labels() const = 0;

  virtual void add_state_block(std::unique_ptr<StandardStateBlock> block,
                               const EstimateWithCovariance& initial_estimate_covariance,
                               const std::optional<CrossCovariances>& cross_covariances = std::nullopt) = 0;

  /// Estimate/covariance of a real or virtual block (converted for virtual blocks).
  virtual std::optional<Vector> get_state_block_estimate(const std::string& block_label) = 0;
  virtual std::optional<Matrix> get_state_block_covariance(const std::string& block_label) = 0;
  virtual std::optional<Matrix> get_state_block_cross_covariance(const std::string& block_label1,
                                                                 const std::string& block_label2) = 0;

  virtual void set_state_block_estimate(const std::string& block_label, const Vector& estimate) = 0;
  virtual void set_state_block_covariance(const std::string& block_label, const Matrix& covariance) = 0;
  virtual void set_state_block_cross_covariance(const std::string& block_label1,
                                                const std::string& block_label2,
                                                const Matrix& covariance) = 0;
  virtual void remove_state_block(const std::string& block_label) = 0;

  virtual std::optional<std::vector<std::string>> virtual_state_block_target_labels() const = 0;
  virtual bool has_virtual_state_block(const std::string& vsb_target_label) const = 0;
  virtual void add_virtual_state_block(std::unique_ptr<VirtualStateBlock> vsb) = 0;
  virtual void remove_virtual_state_block(const std::string& vsb_target_label) = 0;

  virtual std::optional<std::vector<std::string>> measurement_processor_labels() const = 0;
  virtual void add_measurement_processor(std::unique_ptr<StandardMeasurementProcessor> processor) = 0;
  virtual void remove_measurement_processor(const std::string& processor_label) = 0;

  virtual void propagate(Timestamp time) = 0;
  /// Propagates to the message time, then updates with the named processor.
  virtual void update(const std::string& processor_label, const Message& message) = 0;

  /// Estimate/covariance of `block_labels` at a future time without changing the engine.
  virtual std::optional<EstimateWithCovariance> peek_ahead(Timestamp time,
                                                           const std::vector<std::string>& block_labels) = 0;
  /// Current joint estimate/covariance of `block_labels` (in that order).
  virtual std::optional<EstimateWithCovariance> generate_x_and_p(const std::vector<std::string>& block_labels) = 0;

  virtual void give_state_block_aux_data(const std::string& block_label, const AuxData& aux) = 0;
  virtual void give_measurement_processor_aux_data(const std::string& processor_label, const AuxData& aux) = 0;
  virtual void give_virtual_state_block_aux_data(const std::string& target_label, const AuxData& aux) = 0;

  virtual std::unique_ptr<StandardFusionEngine> clone() const = 0;
};

/// Factory for fusion engines.
class FusionPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::FUSION; }

  virtual bool is_fusion_type_supported(FusionType type) const = 0;
  /// nullptr if unsupported.
  virtual std::unique_ptr<StandardFusionEngine> new_fusion_engine(FusionType type) = 0;
};

}  // namespace pntos::api
