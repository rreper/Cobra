// Port of pntos.apps.standard.pos_vel_ins: position (plain Pinson model) + NED velocity updates.
#include "app_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  auto args = parse_args(argc, argv);
  auto b = base_config(args.input_log, args.output_log, args.legacy_q);
  b.transport->channels_to_process = std::vector<std::string>{kImuChannel, kPosChannel, kVelChannel};
  auto pos = std::make_shared<LeverArmMPConfig>(mp::PinsonPositionMPConfig());
  pos->group_ = "config/pos_measurement_processor";
  pos->label = "pos";
  pos->channel = kPosChannel;
  pos->state_block_labels = {"pinson15"};
  pos->lever_arm = {-0.50, 0.38, -0.05};
  auto vel = std::make_shared<PlainMPConfig>(mp::PinsonVelocityMPConfig());
  vel->group_ = "config/vel_measurement_processor";
  vel->label = "vel";
  vel->channel = kVelChannel;
  vel->state_block_labels = {"pinson15"};
  b.orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{pos, vel};
  b.time_bias->channels = std::vector<std::string>{kPosChannel, kVelChannel};
  return run_standard_app("pos_vel_ins", args, b.all());
}
