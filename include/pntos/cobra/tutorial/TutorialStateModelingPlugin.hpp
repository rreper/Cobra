// Port of pntos.cobra.tutorial_plugins.state_modeling.* — the simplified state model used by the
// tutorial apps: a tutorial Pinson15 block (no gravity-gradient terms, no Phi rescaling), the FOGM
// block, a plain NED velocity processor and a position processor with a NED FOGM sensor-error block.
#pragma once

#include <pntos/api/state_modeling.hpp>
#include <pntos/cobra/utils/aspn.hpp>

namespace pntos::cobra {

/// Maps a NED MeasurementVelocity onto the 15 Pinson states (H = [0 I 0 0 0]). Unlike the standard
/// PinsonVelocityMeasurementProcessor it does not check the aux PVA time against the measurement.
class TutorialPinsonVelocityMeasurementProcessor final : public api::StandardMeasurementProcessor {
 public:
  TutorialPinsonVelocityMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels,
                                             api::Mediator* mediator);
  const std::string& label() const override { return label_; }
  const std::vector<std::string>& state_block_labels() const override { return state_block_labels_; }
  void receive_aux_data(const api::AuxData& aux) override;
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<TutorialPinsonVelocityMeasurementProcessor>(*this);
  }

 private:
  std::string label_;
  std::vector<std::string> state_block_labels_;
  api::Mediator* mediator_;
  std::shared_ptr<const utils::PVA> inertial_pva_;
};

/// Maps a geodetic MeasurementPosition onto [Pinson15, 3-state NED FOGM position sensor error]:
/// z = NED(pos - inertial pos), H = [I 0 C l_ps_p 0 0 | -I].
class TutorialPinsonWithNedFogmPositionMeasurementProcessor final : public api::StandardMeasurementProcessor {
 public:
  TutorialPinsonWithNedFogmPositionMeasurementProcessor(std::string label, std::vector<std::string> state_block_labels,
                                                        api::Mediator* mediator, const api::Vector3& l_ps_p);
  const std::string& label() const override { return label_; }
  const std::vector<std::string>& state_block_labels() const override { return state_block_labels_; }
  void receive_aux_data(const api::AuxData& aux) override;
  std::optional<api::StandardMeasurementModel> generate_model(const api::Message& message,
                                                              const api::GenXandP& gen_x_and_p) override;
  std::unique_ptr<api::StandardMeasurementProcessor> clone() const override {
    return std::make_unique<TutorialPinsonWithNedFogmPositionMeasurementProcessor>(*this);
  }

 private:
  std::string label_;
  std::vector<std::string> state_block_labels_;
  api::Mediator* mediator_;
  std::shared_ptr<const utils::PVA> inertial_pva_;
  api::Vector3 l_ps_p_;
};

/// Provider: processors ["pinson_velocity", "pinson_with_ned_fogm_position"], blocks ["pinson15", "fogm"],
/// no virtual blocks. "pinson15" reads an ImuConfig from the config group, "fogm" a FogmConfig, and the
/// position processor a MountingConfig (lever arm).
class TutorialPosInsStateModelProvider final : public api::StandardStateModelProvider {
 public:
  TutorialPosInsStateModelProvider(api::Mediator* mediator, bool legacy_q_rotation);

  const std::vector<std::string>& processor_identifiers() const override { return processor_ids_; }
  const std::vector<std::string>& block_identifiers() const override { return block_ids_; }
  const std::vector<std::string>& virtual_block_identifiers() const override { return vsb_ids_; }

  std::unique_ptr<api::StandardMeasurementProcessor> new_processor(
      std::size_t processor_index, api::StandardFusionEngine* engine, const std::string& label,
      const std::vector<std::string>& state_block_labels, const std::optional<std::string>& config_group) override;
  std::unique_ptr<api::StandardStateBlock> new_block(std::size_t block_index, api::StandardFusionEngine* engine,
                                                     const std::string& label,
                                                     const std::optional<std::string>& config_group) override;
  std::unique_ptr<api::VirtualStateBlock> new_virtual_block(std::size_t, const std::string&, const std::string&,
                                                            const std::optional<std::string>&) override {
    return nullptr;
  }

 private:
  api::Mediator* mediator_;
  bool legacy_q_rotation_;
  std::vector<std::string> processor_ids_{"pinson_velocity", "pinson_with_ned_fogm_position"};
  std::vector<std::string> block_ids_{"pinson15", "fogm"};
  std::vector<std::string> vsb_ids_;
};

/// StateModelingPlugin producing TutorialPosInsStateModelProvider instances.
/// `legacy_q_rotation` selects the Python-compatible Pinson process-noise rotation (see
/// PinsonStateBlockConfig::legacy_q_rotation); the tutorial reads a bare ImuConfig so the switch lives here.
class TutorialPosInsStateModelingPlugin final : public api::StateModelingPlugin {
 public:
  explicit TutorialPosInsStateModelingPlugin(std::string identifier, bool legacy_q_rotation = true)
      : identifier_(std::move(identifier)), legacy_q_rotation_(legacy_q_rotation) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override { mediator_ = mediator; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }
  bool is_fusion_type_supported(api::FusionType type) const override { return type == api::FusionType::STANDARD; }
  std::unique_ptr<api::StandardStateModelProvider> new_state_model_provider(api::FusionType type) override;

 private:
  std::string identifier_;
  bool legacy_q_rotation_;
  api::Mediator* mediator_ = nullptr;
};

}  // namespace pntos::cobra
