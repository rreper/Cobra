// Virtual state blocks: PinsonErrorToStandard and StateExtractor (ports of
// pntos.cobra.internal.PinsonErrorToStandard / StateExtractor).
#pragma once

#include <pntos/api/state_modeling.hpp>
#include <pntos/cobra/utils/aspn.hpp>

namespace pntos::cobra {

/// Maps Pinson-style error states plus the nominal PVA (aux) to whole-valued
/// [lat, lon, alt, vN, vE, vD, roll, pitch, yaw, ...trailing states].
/// Throws std::runtime_error if no/invalid/stale PVA aux is available (as the Python original).
class PinsonErrorToStandard final : public api::VirtualStateBlock {
 public:
  PinsonErrorToStandard(api::Mediator* mediator, std::string source, std::string target);

  const std::string& source() const override { return source_; }
  const std::string& target() const override { return target_; }
  void receive_aux_data(const api::AuxData& aux) override;
  api::EstimateWithCovariance convert(const api::EstimateWithCovariance& ewc, api::Timestamp time) override;
  api::Vector convert_estimate(const api::Vector& estimate, api::Timestamp time) override;
  api::Matrix jacobian(const api::Vector& estimate, api::Timestamp time) override;
  std::unique_ptr<api::VirtualStateBlock> clone() const override {
    return std::make_unique<PinsonErrorToStandard>(*this);
  }

  /// Test hook.
  std::shared_ptr<const utils::PVA> pva() const { return pva_; }

 private:
  /// Validates aux and returns position/velocity/quaternion or throws.
  struct Nominal {
    api::Vector3 pos, vel;
    nav::Vector4 quat;
  };
  Nominal nominal(api::Timestamp time, const char* what) const;

  api::Mediator* mediator_;
  std::string source_, target_;
  std::shared_ptr<const utils::PVA> pva_;
};

/// Selects a subset of states (constant 0/1 Jacobian).
class StateExtractor final : public api::VirtualStateBlock {
 public:
  /// Throws std::invalid_argument on an invalid configuration.
  StateExtractor(api::Mediator* mediator, std::string source, std::string target, int incoming_state_size,
                 const std::vector<int>& indices);

  const std::string& source() const override { return source_; }
  const std::string& target() const override { return target_; }
  void receive_aux_data(const api::AuxData& aux) override;
  api::EstimateWithCovariance convert(const api::EstimateWithCovariance& ewc, api::Timestamp time) override;
  api::Vector convert_estimate(const api::Vector& estimate, api::Timestamp time) override;
  api::Matrix jacobian(const api::Vector& estimate, api::Timestamp time) override { return jac_; }
  std::unique_ptr<api::VirtualStateBlock> clone() const override { return std::make_unique<StateExtractor>(*this); }

 private:
  api::Mediator* mediator_;
  std::string source_, target_;
  api::Matrix jac_;
};

}  // namespace pntos::cobra
