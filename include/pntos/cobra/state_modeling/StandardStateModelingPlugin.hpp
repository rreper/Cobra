// Standard state model provider / plugin (port of pntos.cobra.StandardStateModelingPlugin).
#pragma once

#include <pntos/api/state_modeling.hpp>

namespace pntos::cobra {

/// Provides: 9 measurement processors, 4 state blocks, 2 virtual state blocks (see configs.hpp for
/// the identifiers; the index order matches the Python implementation).
class StandardStateModelProvider final : public api::StandardStateModelProvider {
 public:
  explicit StandardStateModelProvider(api::Mediator* mediator);

  const std::vector<std::string>& processor_identifiers() const override { return processor_ids_; }
  const std::vector<std::string>& block_identifiers() const override { return block_ids_; }
  const std::vector<std::string>& virtual_block_identifiers() const override { return vsb_ids_; }

  std::unique_ptr<api::StandardMeasurementProcessor> new_processor(
      std::size_t processor_index, api::StandardFusionEngine* engine, const std::string& label,
      const std::vector<std::string>& state_block_labels, const std::optional<std::string>& config_group) override;

  std::unique_ptr<api::StandardStateBlock> new_block(std::size_t block_index, api::StandardFusionEngine* engine,
                                                     const std::string& label,
                                                     const std::optional<std::string>& config_group) override;

  std::unique_ptr<api::VirtualStateBlock> new_virtual_block(std::size_t virtual_block_index,
                                                            const std::string& source_label,
                                                            const std::string& target_label,
                                                            const std::optional<std::string>& config_group) override;

 private:
  api::Mediator* mediator_;
  std::vector<std::string> processor_ids_;
  std::vector<std::string> block_ids_;
  std::vector<std::string> vsb_ids_;
};

class StandardStateModelingPlugin final : public api::StateModelingPlugin {
 public:
  explicit StandardStateModelingPlugin(std::string identifier) : identifier_(std::move(identifier)) {}

  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override { mediator_ = mediator; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }

  bool is_fusion_type_supported(api::FusionType type) const override { return type == api::FusionType::STANDARD; }
  std::unique_ptr<api::StandardStateModelProvider> new_state_model_provider(api::FusionType type) override;

 private:
  std::string identifier_;
  api::Mediator* mediator_ = nullptr;
};

}  // namespace pntos::cobra
