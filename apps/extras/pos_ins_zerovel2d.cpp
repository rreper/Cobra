// Port of pntos.apps.extras.pos_ins_zerovel2d: pos_ins plus 2-D zero-velocity (non-holonomic)
// measurements generated every 30 s by the extras ZeroVelocity2dGenerator preprocessor and applied
// through a body-velocity measurement processor.
#include "../standard/app_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  const std::string zerovel_channel = "/generated/zero/velocity2d";
  auto args = parse_args(argc, argv);
  auto b = base_config(args.input_log, args.output_log, args.legacy_q);
  auto pos = std::make_shared<LeverArmMPConfig>(mp::PinsonWithNedFogmPositionMPConfig());
  pos->group_ = "config/pos_measurement_processor";
  pos->label = "pos";
  pos->channel = kPosChannel;
  pos->state_block_labels = {"pinson15", "pos_sensor_error"};
  pos->lever_arm = {-0.50, 0.38, -0.05};
  auto zv = std::make_shared<LeverArmOrientationMPConfig>(mp::PinsonBodyVelocityMPConfig());
  zv->group_ = "config/zerovel2d_measurement_processor";
  zv->label = "zerovel";
  zv->channel = zerovel_channel;
  zv->state_block_labels = {"pinson15"};
  zv->lever_arm = {0.0, 0.0, 0.0};
  zv->orientation = {1.0, 0.0, 0.0, 0.0};
  b.orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{pos, zv};
  auto gen = std::make_shared<ZeroVelocity2dGeneratorConfig>();
  gen->group_ = "config/pseudovel_generator";
  gen->channels = std::vector<std::string>{kPosChannel};
  gen->trigger_dt_sec = 30.0;
  gen->lateral_vel_sigma = 0.5;
  gen->vertical_vel_sigma = 1.0;
  gen->output_channel = zerovel_channel;
  b.orch->preprocessor_configs =
      std::vector<std::shared_ptr<const PreprocessorConfig>>{b.imu_rotator, b.time_adjuster, b.time_bias, gen};
  AppSpec spec = app::standard_app_spec("pos_ins_zerovel2d");
  spec.preprocessors = {"standard", "advanced"};
  return run_standard_app("pos_ins_zerovel2d", args, b.all(), spec);
}
