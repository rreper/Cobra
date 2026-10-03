#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/state_modeling/MeasurementProcessors.hpp>
#include <pntos/cobra/state_modeling/Pinson15NedBlock.hpp>
#include <pntos/cobra/state_modeling/SimpleStateBlocks.hpp>
#include <pntos/cobra/state_modeling/StandardStateModelingPlugin.hpp>
#include <pntos/cobra/state_modeling/VirtualStateBlocks.hpp>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Vector3;

StandardStateModelProvider::StandardStateModelProvider(api::Mediator* mediator)
    : mediator_(mediator),
      processor_ids_{mp::kPinsonPosition,     mp::kPinsonVelocity,     mp::kPinsonWithNedFogmPosition,
                     mp::kPinsonAltitude,     mp::kPinsonWithLeverArmPosition, mp::kPinsonBodyVelocity,
                     mp::kPinsonPosVel,       mp::kPosition,           mp::kDirection3DToPoints},
      block_ids_{PinsonStateBlockConfig::kIdentifier, FogmStateBlockConfig::kIdentifier,
                 ClockBiasStateBlockConfig::kIdentifier, ConstantStateBlockConfig::kIdentifier},
      vsb_ids_{PinsonErrorToStandardVSBConfig::kIdentifier, StateExtractorConfig::kIdentifier} {}

namespace {
Vector3 v3(const Vec3& a) { return Vector3(a[0], a[1], a[2]); }
nav::Vector4 v4(const Vec4& a) { return nav::Vector4(a[0], a[1], a[2], a[3]); }
}  // namespace

std::unique_ptr<api::StandardMeasurementProcessor> StandardStateModelProvider::new_processor(
    std::size_t index, api::StandardFusionEngine*, const std::string& label,
    const std::vector<std::string>& state_block_labels, const std::optional<std::string>& config_group) {
  auto err = [&](const std::string& m) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, m);
  };
  if (index >= processor_ids_.size()) {
    err("Invalid processor index of " + std::to_string(index) + ". StandardStateModelProvider provides " +
        std::to_string(processor_ids_.size()) + " processors.");
    return nullptr;
  }
  const std::string& id = processor_ids_[index];
  if (id == mp::kPinsonVelocity) {
    return std::make_unique<PinsonVelocityMeasurementProcessor>(label, state_block_labels, mediator_);
  }
  if (!config_group) {
    err("A config group is required for processor " + id);
    return nullptr;
  }
  if (id == mp::kPinsonBodyVelocity || id == mp::kDirection3DToPoints) {
    auto c = LeverArmOrientationMPConfig::from_registry(*mediator_, *config_group);
    if (!c) {
      err("Could not get " + id + " sensor config from registry.");
      return nullptr;
    }
    if (id == mp::kPinsonBodyVelocity)
      return std::make_unique<PinsonBodyVelocityMeasurementProcessor>(label, state_block_labels, mediator_,
                                                                      v3(c->lever_arm), v4(c->orientation));
    return std::make_unique<Direction3DToPointsMeasurementProcessor>(label, state_block_labels, mediator_,
                                                                     v3(c->lever_arm), v4(c->orientation));
  }
  auto c = LeverArmMPConfig::from_registry(*mediator_, *config_group);
  if (!c) {
    err("Could not get position sensor config from registry.");
    return nullptr;
  }
  using Kind = PinsonPositionMeasurementProcessor::Kind;
  if (id == mp::kPinsonPosition)
    return std::make_unique<PinsonPositionMeasurementProcessor>(Kind::Plain, label, state_block_labels, mediator_,
                                                                v3(c->lever_arm));
  if (id == mp::kPinsonWithNedFogmPosition)
    return std::make_unique<PinsonPositionMeasurementProcessor>(Kind::WithNedFogm, label, state_block_labels,
                                                                mediator_, v3(c->lever_arm));
  if (id == mp::kPinsonWithLeverArmPosition)
    return std::make_unique<PinsonPositionMeasurementProcessor>(Kind::WithLeverArm, label, state_block_labels,
                                                                mediator_, v3(c->lever_arm));
  if (id == mp::kPinsonAltitude)
    return std::make_unique<AltitudeMeasurementProcessor>(label, state_block_labels, mediator_, v3(c->lever_arm));
  if (id == mp::kPinsonPosVel)
    return std::make_unique<PinsonPosVelMeasurementProcessor>(label, state_block_labels, mediator_, v3(c->lever_arm));
  if (id == mp::kPosition)
    return std::make_unique<PositionMeasurementProcessor>(label, state_block_labels, mediator_, v3(c->lever_arm));
  return nullptr;
}

std::unique_ptr<api::StandardStateBlock> StandardStateModelProvider::new_block(
    std::size_t index, api::StandardFusionEngine*, const std::string& label,
    const std::optional<std::string>& config_group) {
  auto err = [&](const std::string& m) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, m);
  };
  if (index >= block_ids_.size()) {
    err("Invalid block index of " + std::to_string(index) + ". StandardStateModelProvider provides " +
        std::to_string(block_ids_.size()) + " state blocks.");
    return nullptr;
  }
  const std::string& id = block_ids_[index];
  if (!config_group) {
    err("A config group is required for state block " + id);
    return nullptr;
  }
  try {
    if (id == PinsonStateBlockConfig::kIdentifier) {
      auto c = PinsonStateBlockConfig::from_registry(*mediator_, *config_group);
      if (!c) {
        err("Could not get IMU config from registry.");
        return nullptr;
      }
      return std::make_unique<Pinson15NedBlock>(label, mediator_, c->imu_model);
    }
    if (id == FogmStateBlockConfig::kIdentifier) {
      auto c = FogmStateBlockConfig::from_registry(*mediator_, *config_group);
      if (!c) {
        err("Could not get fogm config from registry.");
        return nullptr;
      }
      return std::make_unique<FogmBlock>(label, mediator_, api::Vector(to_matrix(c->fogm_model.sigma)),
                                         api::Vector(to_matrix(c->fogm_model.tau)));
    }
    if (id == ClockBiasStateBlockConfig::kIdentifier) {
      auto c = ClockBiasStateBlockConfig::from_registry(*mediator_, *config_group);
      if (!c) {
        err("Could not get config for state block \"" + label + "\" from registry.");
        return nullptr;
      }
      return std::make_unique<ClockBiasStateBlock>(label, mediator_, c->h_0, c->h_neg2, c->q3);
    }
    if (id == ConstantStateBlockConfig::kIdentifier) {
      auto c = ConstantStateBlockConfig::from_registry(*mediator_, *config_group);
      if (!c) {
        err("Could not get config for state block \"" + label + "\" from registry.");
        return nullptr;
      }
      if (!c->estimate_with_covariance) {
        err("Missing initial EstimateWithCovariance for state block \"" + label + "\". Cannot initialize block.");
        return nullptr;
      }
      return std::make_unique<ConstantStateBlock>(
          label, mediator_, static_cast<std::size_t>(c->estimate_with_covariance->estimate.size()), c->Q);
    }
  } catch (const std::exception& e) {
    err(std::string("Failed to construct state block \"") + label + "\": " + e.what());
    return nullptr;
  }
  return nullptr;
}

std::unique_ptr<api::VirtualStateBlock> StandardStateModelProvider::new_virtual_block(
    std::size_t index, const std::string& source_label, const std::string& target_label,
    const std::optional<std::string>& config_group) {
  auto err = [&](const std::string& m) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, m);
  };
  if (index >= vsb_ids_.size()) {
    err("Invalid virtual block index of " + std::to_string(index) + ". StandardStateModelProvider provides " +
        std::to_string(vsb_ids_.size()) + " virtual state blocks.");
    return nullptr;
  }
  const std::string& id = vsb_ids_[index];
  if (id == PinsonErrorToStandardVSBConfig::kIdentifier)
    return std::make_unique<PinsonErrorToStandard>(mediator_, source_label, target_label);
  if (!config_group) {
    err("A config group is required for virtual state block " + id);
    return nullptr;
  }
  auto c = StateExtractorConfig::from_registry(*mediator_, *config_group);
  if (!c) {
    err("Could not get StateExtractorConfig from registry.");
    return nullptr;
  }
  try {
    return std::make_unique<StateExtractor>(mediator_, source_label, target_label, c->incoming_state_size,
                                            c->indices_to_extract);
  } catch (const std::exception&) {
    return nullptr;
  }
}

std::unique_ptr<api::StandardStateModelProvider> StandardStateModelingPlugin::new_state_model_provider(
    api::FusionType type) {
  if (!is_fusion_type_supported(type)) return nullptr;
  return std::make_unique<StandardStateModelProvider>(mediator_);
}

}  // namespace pntos::cobra
