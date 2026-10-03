// Port of pntos.apps.standard.pos_ins: GPS position + IMU, manual-heading static alignment,
// Pinson15 + 3-state NED position-error FOGM.   pos_ins [output.log] [input.log] [--no-joseph]
#include "app_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  auto args = parse_args(argc, argv);
  auto b = base_config(args.input_log, args.output_log, args.legacy_q);
  auto mp = std::make_shared<LeverArmMPConfig>(mp::PinsonWithNedFogmPositionMPConfig());
  mp->group_ = "config/pos_measurement_processor";
  mp->label = "pos";
  mp->channel = kPosChannel;
  mp->state_block_labels = {"pinson15", "pos_sensor_error"};
  mp->lever_arm = {-0.50, 0.38, -0.05};
  b.orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{mp};
  return run_standard_app("pos_ins", args, b.all());
}
