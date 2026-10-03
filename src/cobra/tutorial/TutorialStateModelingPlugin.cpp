#include <pntos/cobra/tutorial/TutorialStateModelingPlugin.hpp>

#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/state_modeling/Pinson15NedBlock.hpp>
#include <pntos/cobra/state_modeling/SimpleStateBlocks.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <aspn23/eigen/MeasurementPosition.hpp>
#include <aspn23/eigen/MeasurementVelocity.hpp>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Matrix;
using api::Matrix3;
using api::Vector;
using api::Vector3;

// ----------------------------------------------------------------------------- velocity processor

TutorialPinsonVelocityMeasurementProcessor::TutorialPinsonVelocityMeasurementProcessor(
    std::string label, std::vector<std::string> state_block_labels, api::Mediator* mediator)
    : label_(std::move(label)), state_block_labels_(std::move(state_block_labels)), mediator_(mediator) {}

void TutorialPinsonVelocityMeasurementProcessor::receive_aux_data(const api::AuxData& aux) {
  if (aux.empty() || !aux[0]) return;
  if (auto pva = aux[0]->as<utils::PVA>()) inertial_pva_ = pva;
}

std::optional<api::StandardMeasurementModel> TutorialPinsonVelocityMeasurementProcessor::generate_model(
    const api::Message& message, const api::GenXandP&) {
  auto vel = message.as<aspn23_eigen::MeasurementVelocity>();
  if (!vel || !inertial_pva_) return std::nullopt;
  const Vector3 meas(vel->get_x(), vel->get_y(), vel->get_z());
  Matrix H = Matrix::Zero(3, 15);
  H.block<3, 3>(0, 3) = Matrix3::Identity();
  api::StandardMeasurementModel model;
  model.z = meas - utils::velocity(*inertial_pva_);
  model.H = H;
  model.R = vel->get_covariance();
  model.h = [H](const Vector& x) { return Vector(H * x); };
  return model;
}

// ----------------------------------------------------------------------------- position processor

TutorialPinsonWithNedFogmPositionMeasurementProcessor::TutorialPinsonWithNedFogmPositionMeasurementProcessor(
    std::string label, std::vector<std::string> state_block_labels, api::Mediator* mediator, const Vector3& l_ps_p)
    : label_(std::move(label)),
      state_block_labels_(std::move(state_block_labels)),
      mediator_(mediator),
      l_ps_p_(l_ps_p) {}

void TutorialPinsonWithNedFogmPositionMeasurementProcessor::receive_aux_data(const api::AuxData& aux) {
  if (aux.empty() || !aux[0]) return;
  if (auto pva = aux[0]->as<utils::PVA>()) inertial_pva_ = pva;
}

std::optional<api::StandardMeasurementModel> TutorialPinsonWithNedFogmPositionMeasurementProcessor::generate_model(
    const api::Message& message, const api::GenXandP& gen_x_and_p) {
  auto pos = message.as<aspn23_eigen::MeasurementPosition>();
  if (!pos || !inertial_pva_) return std::nullopt;
  auto q = utils::quaternion(*inertial_pva_);
  if (!q) return std::nullopt;
  const Vector3 llh(pos->get_term1(), pos->get_term2(), pos->get_term3());
  const Vector3 inertial_llh = utils::position(*inertial_pva_);
  const Matrix3 C_platform_to_nav = nav::quat_to_dcm(*q);
  Vector3 z = llh - inertial_llh;
  z(0) = nav::delta_lat_to_north(z(0), llh(0), llh(2));
  z(1) = nav::delta_lon_to_east(z(1), llh(0), llh(2));
  z(2) = -z(2);

  auto ewc = gen_x_and_p(state_block_labels_);
  if (!ewc) return std::nullopt;
  const Eigen::Index n = ewc->estimate.size();
  Matrix H = Matrix::Zero(3, n);
  H.block<3, 3>(0, 0) = Matrix3::Identity();
  // Python: H[:, 6:9] = C @ l — a 3-vector broadcast by numpy into every row of the tilt columns
  // (kept as-is: it is the tutorial's linearisation, not the standard lever-arm skew).
  const Vector3 Cl = C_platform_to_nav * l_ps_p_;
  for (int r = 0; r < 3; ++r) H.row(r).segment<3>(6) = Cl.transpose();
  H.block<3, 3>(0, n - 3) = -Matrix3::Identity();
  const Vector3 l = l_ps_p_;
  const Matrix3 C = C_platform_to_nav;
  api::StandardMeasurementModel model;
  model.z = z;
  model.H = H;
  model.R = pos->get_covariance();
  model.h = [C, l](const Vector& x) {
    const Eigen::Index m = x.size();
    const Vector3 res = x.head<3>() + (Matrix3::Identity() - nav::skew(x.segment<3>(6))) * C * l - x.segment<3>(m - 3);
    return Vector(res);
  };
  return model;
}

// ----------------------------------------------------------------------------- provider / plugin

TutorialPosInsStateModelProvider::TutorialPosInsStateModelProvider(api::Mediator* mediator, bool legacy_q_rotation)
    : mediator_(mediator), legacy_q_rotation_(legacy_q_rotation) {}

std::unique_ptr<api::StandardMeasurementProcessor> TutorialPosInsStateModelProvider::new_processor(
    std::size_t processor_index, api::StandardFusionEngine*, const std::string& label,
    const std::vector<std::string>& state_block_labels, const std::optional<std::string>& config_group) {
  switch (processor_index) {
    case 0:
      return std::make_unique<TutorialPinsonVelocityMeasurementProcessor>(label, state_block_labels, mediator_);
    case 1: {
      if (!config_group || !mediator_) return nullptr;
      auto mounting = MountingConfig::from_registry(*mediator_, *config_group);
      if (!mounting) {
        mediator_->log_message(LoggingLevel::ERROR,
                               "TutorialPosInsStateModelProvider: no MountingConfig in group \"" + *config_group + "\".");
        return nullptr;
      }
      return std::make_unique<TutorialPinsonWithNedFogmPositionMeasurementProcessor>(
          label, state_block_labels, mediator_, Vector3(mounting->lever_arm[0], mounting->lever_arm[1], mounting->lever_arm[2]));
    }
    default:
      return nullptr;
  }
}

std::unique_ptr<api::StandardStateBlock> TutorialPosInsStateModelProvider::new_block(
    std::size_t block_index, api::StandardFusionEngine*, const std::string& label,
    const std::optional<std::string>& config_group) {
  if (!config_group || !mediator_) return nullptr;
  switch (block_index) {
    case 0: {
      auto imu = ImuConfig::from_registry(*mediator_, *config_group);
      if (!imu) {
        mediator_->log_message(LoggingLevel::ERROR,
                               "TutorialPosInsStateModelProvider: no ImuConfig in group \"" + *config_group + "\".");
        return nullptr;
      }
      return std::make_unique<Pinson15NedBlock>(label, mediator_, *imu, legacy_q_rotation_, /*tutorial_model=*/true);
    }
    case 1: {
      auto fogm = FogmConfig::from_registry(*mediator_, *config_group);
      if (!fogm) {
        mediator_->log_message(LoggingLevel::ERROR,
                               "TutorialPosInsStateModelProvider: no FogmConfig in group \"" + *config_group + "\".");
        return nullptr;
      }
      Vector sigma = Eigen::Map<const Vector>(fogm->sigma.data(), static_cast<Eigen::Index>(fogm->sigma.size()));
      Vector tau = Eigen::Map<const Vector>(fogm->tau.data(), static_cast<Eigen::Index>(fogm->tau.size()));
      return std::make_unique<FogmBlock>(label, mediator_, sigma, tau);
    }
    default:
      return nullptr;
  }
}

std::unique_ptr<api::StandardStateModelProvider> TutorialPosInsStateModelingPlugin::new_state_model_provider(
    api::FusionType type) {
  if (!is_fusion_type_supported(type)) return nullptr;
  return std::make_unique<TutorialPosInsStateModelProvider>(mediator_, legacy_q_rotation_);
}

}  // namespace pntos::cobra
