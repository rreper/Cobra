// Port of pntos.apps.standard.pos_ins_vsb: position processed against whole-state PVA through the
// pinson_error_to_standard virtual state block.
#include "app_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  auto args = parse_args(argc, argv);
  auto b = base_config(args.input_log, args.output_log, args.legacy_q);
  api::Vector d(3);
  d << 2.22e-13, 3.73e-13, 9.0;  // lat/lon error in rad², altitude in m²
  b.orch->additional_sb_configs = std::vector<std::shared_ptr<const StateBlockConfig>>{
      pos_fogm_block(api::Matrix(d.asDiagonal()), {2.515e-7, 3.259e-7, 2.0})};
  auto mp = std::make_shared<LeverArmMPConfig>(mp::PositionMPConfig());
  mp->group_ = "config/position";
  mp->label = "pos";
  mp->channel = kPosChannel;
  mp->state_block_labels = {"platform_pva", "pos_sensor_error"};
  mp->lever_arm = {-0.50, 0.38, -0.05};
  b.orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{mp};
  auto pes = std::make_shared<PinsonErrorToStandardVSBConfig>();
  pes->group_ = "config/pes";
  pes->source = "pinson15";
  pes->target = "platform_pva";
  b.orch->vsb_configs = std::vector<std::shared_ptr<const VirtualStateBlockConfig>>{pes};
  return run_standard_app("pos_ins_vsb", args, b.all());
}
