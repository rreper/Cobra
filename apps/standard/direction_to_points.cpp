// Port of pntos.apps.standard.direction_to_points: bearing-to-known-feature updates only
// (positions are used for alignment but not as measurements).
#include "app_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  const std::string d2p = "/sensor/simulated/directiontoknownfeature";
  auto args = parse_args(argc, argv);
  auto b = base_config(args.input_log, args.output_log, args.legacy_q);
  b.transport->channels_to_process = std::vector<std::string>{kImuChannel, kPosChannel, d2p};
  auto mp = std::make_shared<LeverArmOrientationMPConfig>(mp::Direction3dToPointsMPConfig());
  mp->group_ = "config/direction3D_to_points_measurement_processor";
  mp->label = "dir3D_to_points";
  mp->channel = d2p;
  mp->state_block_labels = {"pinson15"};
  mp->lever_arm = {0.80, 0.0, 0.05};
  mp->orientation = {0.707106781, 0.0, 0.707106781, 0.0};
  b.orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{mp};
  return run_standard_app("direction_to_points", args, b.all());
}
