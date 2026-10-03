// pntOS C++ API — standard state model: dynamics/measurement models, state blocks, virtual
// state blocks, measurement processors, state model providers, StateModelingPlugin.
#pragma once

#include <pntos/api/common.hpp>

#include <functional>

namespace pntos::api {

class StandardFusionEngine;  // fusion.hpp

/// x_k = g(x_{k-1}) + w_k,  Phi = dg/dx,  Qd = cov(w_k).
struct StandardDynamicsModel {
  std::function<Vector(const Vector&)> g;
  Matrix Phi;
  Matrix Qd;
};

/// z = h(x) + v,  H = dh/dx,  R = cov(v).
struct StandardMeasurementModel {
  Vector z;
  std::function<Vector(const Vector&)> h;
  Matrix H;
  Matrix R;
};

/// Lazily returns the joint estimate/covariance of the given block labels (nullopt if any label is
/// unknown). Blocks are assembled in the order given.
using GenXandP = std::function<std::optional<EstimateWithCovariance>(const std::vector<std::string>&)>;

using AuxData = std::vector<std::optional<Message>>;

/// A set of states and their dynamics.
class StandardStateBlock {
 public:
  virtual ~StandardStateBlock() = default;

  virtual const std::string& label() const = 0;
  virtual std::size_t num_states() const = 0;

  virtual void receive_aux_data(const AuxData& aux) = 0;

  /// nullopt if time_from > time_to or required aux data is missing.
  virtual std::optional<StandardDynamicsModel> generate_dynamics(const GenXandP& gen_x_and_p,
                                                                 Timestamp time_from,
                                                                 Timestamp time_to) = 0;

  /// Deep copy (used by StandardFusionEngine::peek_ahead). Replaces Python's __deepcopy__.
  virtual std::unique_ptr<StandardStateBlock> clone() const = 0;
};

/// Converts states from one representation (`source`) to another (`target`).
class VirtualStateBlock {
 public:
  virtual ~VirtualStateBlock() = default;

  virtual const std::string& source() const = 0;
  virtual const std::string& target() const = 0;

  virtual void receive_aux_data(const AuxData& aux) = 0;

  virtual EstimateWithCovariance convert(const EstimateWithCovariance& ewc, Timestamp time) = 0;
  virtual Vector convert_estimate(const Vector& estimate, Timestamp time) = 0;
  /// M×N Jacobian of the transform at `estimate`.
  virtual Matrix jacobian(const Vector& estimate, Timestamp time) = 0;

  virtual std::unique_ptr<VirtualStateBlock> clone() const = 0;
};

/// Turns raw measurements into StandardMeasurementModels against a list of state blocks.
class StandardMeasurementProcessor {
 public:
  virtual ~StandardMeasurementProcessor() = default;

  virtual const std::string& label() const = 0;
  /// Labels of the blocks this processor models, in order. May be extended by the processor.
  virtual const std::vector<std::string>& state_block_labels() const = 0;

  virtual void receive_aux_data(const AuxData& aux) = 0;

  /// nullopt if the message cannot be processed (unsupported type, missing aux, rejected).
  virtual std::optional<StandardMeasurementModel> generate_model(const Message& message,
                                                                 const GenXandP& gen_x_and_p) = 0;

  virtual std::unique_ptr<StandardMeasurementProcessor> clone() const = 0;
};

/// Factory for state blocks, virtual state blocks and measurement processors (standard model).
class StandardStateModelProvider {
 public:
  virtual ~StandardStateModelProvider() = default;

  virtual const std::vector<std::string>& processor_identifiers() const = 0;
  virtual const std::vector<std::string>& block_identifiers() const = 0;
  virtual const std::vector<std::string>& virtual_block_identifiers() const = 0;

  virtual std::unique_ptr<StandardMeasurementProcessor> new_processor(
      std::size_t processor_index, StandardFusionEngine* engine, const std::string& label,
      const std::vector<std::string>& state_block_labels,
      const std::optional<std::string>& config_group) = 0;

  virtual std::unique_ptr<StandardStateBlock> new_block(std::size_t block_index, StandardFusionEngine* engine,
                                                        const std::string& label,
                                                        const std::optional<std::string>& config_group) = 0;

  virtual std::unique_ptr<VirtualStateBlock> new_virtual_block(std::size_t virtual_block_index,
                                                               const std::string& source_label,
                                                               const std::string& target_label,
                                                               const std::optional<std::string>& config_group) = 0;
};

/// Factory-of-factories: provides state model providers.
class StateModelingPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::STATE_MODELING; }

  virtual bool is_fusion_type_supported(FusionType type) const = 0;
  /// nullptr if `type` is unsupported.
  virtual std::unique_ptr<StandardStateModelProvider> new_state_model_provider(FusionType type) = 0;
};

}  // namespace pntos::api
