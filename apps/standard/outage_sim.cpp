// Port of pntos.apps.standard.outage_sim: pos_ins with a simulated 600 s GNSS outage (1000-1600 s).
// Altitude and velocity processors are configured but their channels are not processed (as in Python).
#include "app_common.hpp"

int main(int argc, char** argv) {
  using namespace pntos::apps;
  auto args = parse_args(argc, argv);
  auto b = base_config(args.input_log, args.output_log, args.legacy_q);
  auto alt_fogm = std::make_shared<FogmStateBlockConfig>();
  alt_fogm->group_ = "config/alt_fogm_block";
  alt_fogm->label = "alt_fogm";
  alt_fogm->estimate_with_covariance = api::EstimateWithCovariance{api::EstimateWithCovarianceType::EWC_GENERIC,
                                                                   api::Vector::Zero(1), api::Matrix::Constant(1, 1, 100.0 * 100.0)};
  alt_fogm->fogm_model.group_ = "config/alt_sensor_error";
  alt_fogm->fogm_model.sigma = {100.0};
  alt_fogm->fogm_model.tau = {3600.0};
  b.orch->additional_sb_configs = std::vector<std::shared_ptr<const StateBlockConfig>>{pos_fogm_block(), alt_fogm};
  auto pos = std::make_shared<LeverArmMPConfig>(mp::PinsonWithNedFogmPositionMPConfig());
  pos->group_ = "config/pos_measurement_processor";
  pos->label = "pos";
  pos->channel = kPosChannel;
  pos->state_block_labels = {"pinson15", "pos_sensor_error"};
  pos->lever_arm = {-0.50, 0.38, -0.05};
  auto alt = std::make_shared<LeverArmMPConfig>(mp::AltitudeMPConfig());
  alt->group_ = "config/alt_measurement_processor";
  alt->label = "alt";
  alt->channel = "/sensor/bmp388/altitude";
  alt->state_block_labels = {"pinson15", "alt_fogm"};
  alt->lever_arm = {0.0, 0.0, 1.0};
  auto vel = std::make_shared<PlainMPConfig>(mp::PinsonVelocityMPConfig());
  vel->group_ = "config/vel_measurement_processor";
  vel->label = "vel";
  vel->channel = kVelChannel;
  vel->state_block_labels = {"pinson15"};
  b.orch->mp_configs = std::vector<std::shared_ptr<const MeasurementProcessorConfig>>{pos, alt, vel};
  b.time_bias->channels = std::vector<std::string>{kPosChannel, kVelChannel};
  auto baro = std::make_shared<BarometerToAltitudeConfig>();
  baro->group_ = "config/pressure_to_alt";
  baro->channels = std::vector<std::string>{"/sensor/bmp388/baro_pressure"};
  baro->alt_sigma = 30.0;
  auto outage = std::make_shared<OutageConfig>();
  outage->group_ = "config/pos_outage";
  outage->channels = std::vector<std::string>{kPosChannel};
  outage->start_time = 1000.0;
  outage->end_time = 1600.0;
  b.orch->preprocessor_configs =
      std::vector<std::shared_ptr<const PreprocessorConfig>>{b.imu_rotator, b.time_adjuster, b.time_bias, baro, outage};
  b.orch->max_prop_interval = 1.0;
  return run_standard_app("outage_sim", args, b.all());
}
