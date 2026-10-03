// Port of pntos.cobra.standard_plugins.fusion.StandardFusionPlugin (StandardFusionEngine + plugin).
#pragma once

#include <pntos/api/fusion.hpp>
#include <pntos/cobra/fusion/VirtualStateBlockManager.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace pntos::cobra {

/// Bookkeeping fusion engine: maps labelled state blocks onto a flat StandardFusionStrategy,
/// assembles block-diagonal dynamics and full-width measurement models, and routes virtual
/// state blocks through a VirtualStateBlockManager.
class StandardFusionEngine final : public api::StandardFusionEngine {
 public:
  StandardFusionEngine(api::Mediator* mediator, bool save_x_and_p_after_prop = false,
                       bool save_x_and_p_after_update = false);

  api::Timestamp time() const override { return time_; }
  void set_time(api::Timestamp time) override { time_ = time; }

  api::StandardFusionStrategy* strategy() const override { return strategy_.get(); }
  void set_strategy(std::unique_ptr<api::StandardFusionStrategy> strategy) override {
    strategy_ = std::move(strategy);
  }

  std::size_t num_states() const override { return num_states_; }
  std::optional<std::vector<std::string>> state_block_labels() const override;

  void add_state_block(std::unique_ptr<api::StandardStateBlock> block,
                       const api::EstimateWithCovariance& initial_estimate_covariance,
                       const std::optional<api::CrossCovariances>& cross_covariances = std::nullopt) override;

  std::optional<api::Vector> get_state_block_estimate(const std::string& block_label) override;
  std::optional<api::Matrix> get_state_block_covariance(const std::string& block_label) override;
  std::optional<api::Matrix> get_state_block_cross_covariance(const std::string& block_label1,
                                                              const std::string& block_label2) override;

  void set_state_block_estimate(const std::string& block_label, const api::Vector& estimate) override;
  void set_state_block_covariance(const std::string& block_label, const api::Matrix& covariance) override;
  void set_state_block_cross_covariance(const std::string& block_label1, const std::string& block_label2,
                                        const api::Matrix& covariance) override;
  void remove_state_block(const std::string& block_label) override;

  std::optional<std::vector<std::string>> virtual_state_block_target_labels() const override {
    return vsb_manager_.get_virtual_state_block_labels();
  }
  bool has_virtual_state_block(const std::string& vsb_target_label) const override {
    return vsb_manager_.has_node(vsb_target_label);
  }
  void add_virtual_state_block(std::unique_ptr<api::VirtualStateBlock> vsb) override {
    vsb_manager_.add_virtual_state_block(std::move(vsb));
  }
  void remove_virtual_state_block(const std::string& vsb_target_label) override {
    vsb_manager_.remove_virtual_state_block(vsb_target_label);
  }

  std::optional<std::vector<std::string>> measurement_processor_labels() const override;
  void add_measurement_processor(std::unique_ptr<api::StandardMeasurementProcessor> processor) override;
  void remove_measurement_processor(const std::string& processor_label) override;

  void propagate(api::Timestamp time) override;
  void update(const std::string& processor_label, const api::Message& message) override;

  std::optional<api::EstimateWithCovariance> peek_ahead(api::Timestamp time,
                                                        const std::vector<std::string>& block_labels) override;
  std::optional<api::EstimateWithCovariance> generate_x_and_p(const std::vector<std::string>& block_labels) override;

  void give_state_block_aux_data(const std::string& block_label, const api::AuxData& aux) override;
  void give_measurement_processor_aux_data(const std::string& processor_label, const api::AuxData& aux) override;
  void give_virtual_state_block_aux_data(const std::string& target_label, const api::AuxData& aux) override {
    vsb_manager_.give_virtual_state_block_aux_data(target_label, aux);
  }

  /// Deep copy: strategy, blocks, processors and virtual blocks are cloned.
  std::unique_ptr<api::StandardFusionEngine> clone() const override;

  /// Real label behind `label` (itself for a real block, the root for a virtual one).
  std::optional<std::string> get_real_label(const std::string& label);
  /// Joint covariance of `block_labels` (real or virtual) in the given order.
  std::optional<api::Matrix> build_joint_covariance(const std::vector<std::string>& block_labels, Eigen::Index size);

  /// Test hook.
  const VirtualStateBlockManager& vsb_manager() const { return vsb_manager_; }

 private:
  struct StateBlockInfo {
    std::string label;
    Eigen::Index num_states;
    Eigen::Index start_index;
    Eigen::Index stop_index;
    std::unique_ptr<api::StandardStateBlock> block;
  };

  StateBlockInfo* find_block(const std::string& label);
  const StateBlockInfo* find_block(const std::string& label) const;
  api::StandardMeasurementProcessor* find_processor(const std::string& label);
  void re_index_stateblocks();
  void save_x_and_p_to_registry();
  void log(api::LoggingLevel level, const std::string& msg) const;
  /// Resolves a (possibly virtual) label to its real block, logging as the Python does.
  const StateBlockInfo* resolve(const std::string& label, bool& is_real, const char* what);
  api::GenXandP gen_x_and_p_func();

  api::Mediator* mediator_;
  api::Timestamp time_{0};
  std::vector<StateBlockInfo> sb_;  ///< insertion order == state order
  std::vector<std::unique_ptr<api::StandardMeasurementProcessor>> mp_;
  VirtualStateBlockManager vsb_manager_;
  Eigen::Index num_states_ = 0;
  std::unique_ptr<api::StandardFusionStrategy> strategy_;
  bool save_after_prop_;
  bool save_after_update_;
  std::vector<std::string> saved_state_labels_;
};

class StandardFusionPlugin final : public api::FusionPlugin {
 public:
  explicit StandardFusionPlugin(std::string identifier) : identifier_(std::move(identifier)) {}

  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override { mediator_ = mediator; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }

  bool is_fusion_type_supported(api::FusionType type) const override { return type == api::FusionType::STANDARD; }
  /// Reads FusionEngineConfig from the registry (defaults if the group is absent).
  std::unique_ptr<api::StandardFusionEngine> new_fusion_engine(api::FusionType type) override;

 private:
  std::string identifier_;
  api::Mediator* mediator_ = nullptr;
};

}  // namespace pntos::cobra
