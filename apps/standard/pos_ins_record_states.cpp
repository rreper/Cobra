// Port of pntos.apps.standard.pos_ins_record_states: pos_ins with two differences —
// 1. the filter states are recorded to the `diagnostics` registry group after every propagation and
//    update and written to <output>.hdf5 by DiagnosticLogPlugin (plot with
//    Cobra/postprocessing/plot_results.py pntos_output.log --hdf5-file=pntos_output.hdf5);
// 2. the Pinson error states are only fed back to the inertial once a position error reaches 100 m.
#include "app_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  auto args = parse_args(argc, argv);
  auto b = base_config(args.input_log, args.output_log, args.legacy_q);
  b.fusion->save_x_and_p_after_prop = true;
  b.fusion->save_x_and_p_after_update = true;
  auto mp = std::make_shared<LeverArmMPConfig>(mp::PinsonWithNedFogmPositionMPConfig());
  mp->group_ = "config/pos_measurement_processor";
  mp->label = "pos";
  mp->channel = kPosChannel;
  mp->state_block_labels = {"pinson15", "pos_sensor_error"};
  mp->lever_arm = {-0.50, 0.38, -0.05};
  b.orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{mp};
  FeedbackConfig feedback;
  feedback.group_ = "config/inertial_feedback";
  feedback.pos_error_threshold = 100.0;
  b.orch->feedback_config = feedback;
  AppSpec spec = app::standard_app_spec("pos_ins_record_states");
  spec.diagnostic_log = true;  // file: <output log>.hdf5
  return run_standard_app("pos_ins_record_states", args, b.all(), spec);
}
