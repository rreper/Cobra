// Port of pntos.apps.standard.pos_ins_leverarm: lever-arm error estimated from a deliberately wrong prior.
#include "app_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  auto args = parse_args(argc, argv);
  auto b = base_config(args.input_log, args.output_log, args.legacy_q);
  auto la = std::make_shared<FogmStateBlockConfig>();
  la->group_ = "config/lever_arm_fogm_block";
  la->label = "pos_sensor_lever_arm";
  la->estimate_with_covariance = api::EstimateWithCovariance{api::EstimateWithCovarianceType::EWC_GENERIC,
                                                             api::Vector::Zero(3), api::Matrix::Identity(3, 3) * 100.0};
  la->fogm_model.group_ = "config/pos_sensor_lever_arm";
  la->fogm_model.sigma = {2.0, 2.0, 2.0};
  la->fogm_model.tau = {1e5, 1e5, 1e5};
  b.orch->additional_sb_configs = std::vector<std::shared_ptr<const StateBlockConfig>>{pos_fogm_block(), la};
  auto mp = std::make_shared<LeverArmMPConfig>(mp::PinsonWithLeverArmPositionMPConfig());
  mp->group_ = "config/pos_measurement_processor";
  mp->label = "pos";
  mp->channel = kPosChannel;
  mp->state_block_labels = {"pinson15", "pos_sensor_error", "pos_sensor_lever_arm"};
  mp->lever_arm = {-8.50, 8.38, -8.05};  // deliberately wrong prior
  b.orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{mp};
  return run_standard_app("pos_ins_leverarm", args, b.all());
}
