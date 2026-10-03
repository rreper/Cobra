// Port of pntos.apps.standard.posvel_ins: combined position+velocity (PVA message) updates.
#include "app_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  auto args = parse_args(argc, argv);
  auto b = base_config(args.input_log, args.output_log, args.legacy_q);
  b.transport->channels_to_process = std::vector<std::string>{kImuChannel, kPosChannel, kPvaChannel};
  auto mp = std::make_shared<LeverArmMPConfig>(mp::PosVelMPConfig());
  mp->group_ = "config/posvel_measurement_processor";
  mp->label = "posvel";
  mp->channel = kPvaChannel;
  mp->state_block_labels = {"pinson15"};
  mp->lever_arm = {-0.50, 0.38, -0.05};
  b.orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{mp};
  b.time_bias->channels = std::vector<std::string>{kPvaChannel};
  return run_standard_app("posvel_ins", args, b.all());
}
