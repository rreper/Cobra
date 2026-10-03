// Port of pntos.apps.standard.pos_ins_bodyvel: pos_ins plus simulated body-frame velocity, downsampled 10x.
#include "app_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  const std::string body_vel = "/sensor/simulated/velocity";
  auto args = parse_args(argc, argv);
  auto b = base_config(args.input_log, args.output_log, args.legacy_q);
  b.transport->channels_to_process = std::vector<std::string>{kImuChannel, kPosChannel, body_vel};
  auto pos = std::make_shared<LeverArmMPConfig>(mp::PinsonWithNedFogmPositionMPConfig());
  pos->group_ = "config/pos_measurement_processor";
  pos->label = "pos";
  pos->channel = kPosChannel;
  pos->state_block_labels = {"pinson15", "pos_sensor_error"};
  pos->lever_arm = {-0.50, 0.38, -0.05};
  auto bv = std::make_shared<LeverArmOrientationMPConfig>(mp::PinsonBodyVelocityMPConfig());
  bv->group_ = "config/bodyvel_measurement_processor";
  bv->label = "body_vel";
  bv->channel = body_vel;
  bv->state_block_labels = {"pinson15"};
  bv->lever_arm = {0.0, 0.0, 0.0};
  bv->orientation = {1.0, 0.0, 0.0, 0.0};
  b.orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{pos, bv};
  auto ds = std::make_shared<DownsamplerConfig>();
  ds->group_ = "config/downsampler";
  ds->channels = std::vector<std::string>{body_vel};
  ds->downsampling_factors = {10};
  b.orch->preprocessor_configs =
      std::vector<std::shared_ptr<const PreprocessorConfig>>{b.imu_rotator, b.time_adjuster, b.time_bias, ds};
  return run_standard_app("pos_ins_bodyvel", args, b.all());
}
