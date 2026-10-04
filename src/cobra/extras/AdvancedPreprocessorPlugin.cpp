#include <pntos/cobra/extras/AdvancedPreprocessorPlugin.hpp>

#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/logging.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>
#include <aspn23/eigen/MeasurementPosition.hpp>
#include <aspn23/eigen/MeasurementPositionVelocityAttitude.hpp>
#include <aspn23/eigen/TypeHeader.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Message;

namespace {
aspn23_eigen::MeasurementVelocity make_template(double lat_sigma, double vert_sigma) {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cov(2, 2);
  cov << lat_sigma * lat_sigma, 0.0, 0.0, vert_sigma * vert_sigma;
  return aspn23_eigen::MeasurementVelocity(aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_VELOCITY, 0, 0, 0, 0),
                                           aspn23_eigen::TypeTimestamp(std::int64_t{0}),
                                           ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_SENSOR,
                                           std::numeric_limits<double>::quiet_NaN(),  // x: not measured (Python None)
                                           0.0, 0.0, cov, ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE,
                                           Eigen::Matrix<double, Eigen::Dynamic, 1>(0), std::vector<aspn23_eigen::TypeIntegrity>{});
}
}  // namespace

ZeroVelocity2dGenerator::ZeroVelocity2dGenerator(api::Mediator* mediator, std::optional<std::vector<std::string>> channels,
                                                 double trigger_dt_sec, double lateral_vel_sigma,
                                                 double vertical_vel_sigma, std::string output_channel)
    : mediator_(mediator),
      channels_(std::move(channels)),
      trigger_dt_ns_(static_cast<std::int64_t>(trigger_dt_sec * 1e9)),
      output_channel_(std::move(output_channel)),
      template_(make_template(lateral_vel_sigma, vertical_vel_sigma)) {}

std::optional<std::vector<Message>> ZeroVelocity2dGenerator::process_pntos_message(const Message& message) {
  if (channels_ && !channels_->empty() &&
      std::find(channels_->begin(), channels_->end(), message.source_identifier) == channels_->end())
    return std::vector<Message>{message};
  auto tov = message.wrapped_message ? utils::time_of_validity(*message.wrapped_message) : std::nullopt;
  if (!tov) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::WARN, "ZeroVelocity2dGenerator received a message from channel " +
                                                     message.source_identifier + " with no time of validity. Ignoring.");
    return std::vector<Message>{message};
  }
  const std::int64_t now = tov->elapsed_nsec;
  if (last_ns_ && (now - *last_ns_) < trigger_dt_ns_) return std::vector<Message>{message};
  auto vel = std::make_shared<aspn23_eigen::MeasurementVelocity>(template_);
  vel->set_time_of_validity(aspn23_eigen::TypeTimestamp(now));
  last_ns_ = now;
  return std::vector<Message>{message, Message(vel, output_channel_)};
}

// ----------------------------------------------------------------------------- SensorDegradation

SensorDegradationPreprocessor::SensorDegradationPreprocessor(const SensorDegradationConfig& config, api::Mediator* mediator)
    : cfg_(config), mediator_(mediator), rng_(static_cast<std::uint64_t>(config.seed)) {}

double SensorDegradationPreprocessor::gauss() { return n01_(rng_); }

api::Vector3 SensorDegradationPreprocessor::ramp_offset(double rel_s) const {
  api::Vector3 off = api::Vector3::Zero();
  for (const auto& r : cfg_.position_ramps) {
    const double start = r[0], duration = r[4];
    if (rel_s < start) continue;
    const double elapsed = duration > 0 ? std::min(rel_s - start, duration) : rel_s - start;
    off += api::Vector3(r[1], r[2], r[3]) * elapsed;
  }
  return off;
}

std::optional<std::vector<Message>> SensorDegradationPreprocessor::process_pntos_message(const Message& message) {
  if (!message.wrapped_message) return std::vector<Message>{message};
  if (auto imu = message.as<aspn23_eigen::MeasurementImu>()) {
    const bool integrated = imu->get_imu_type() == ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED;
    const double dt = cfg_.imu_expected_dt > 0 ? cfg_.imu_expected_dt : 0.01;
    const double noise_scale = integrated ? std::sqrt(dt) : 1.0 / std::sqrt(dt);  // density -> per-sample sigma
    const double bias_scale = integrated ? dt : 1.0;
    auto out = std::make_shared<aspn23_eigen::MeasurementImu>(*imu);
    Eigen::Matrix<double, Eigen::Dynamic, 1> a = imu->get_meas_accel(), g = imu->get_meas_gyro();
    for (int i = 0; i < 3 && i < a.size(); ++i) {
      const auto k = static_cast<std::size_t>(i);
      a(i) += cfg_.accel_bias[k] * bias_scale + cfg_.accel_noise_density[k] * noise_scale * gauss();
      g(i) += cfg_.gyro_bias[k] * bias_scale + cfg_.gyro_noise_density[k] * noise_scale * gauss();
    }
    out->set_meas_accel(a);
    out->set_meas_gyro(g);
    return std::vector<Message>{Message(out, message.source_identifier)};
  }
  if (auto pos = message.as<aspn23_eigen::MeasurementPosition>()) {
    if (pos->get_reference_frame() != ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC) return std::vector<Message>{message};
    const std::int64_t t = pos->get_time_of_validity().get_elapsed_nsec();
    if (!first_position_ns_) first_position_ns_ = t;
    const double rel = static_cast<double>(t - *first_position_ns_) * 1e-9;
    double dn = cfg_.position_noise_sigma_ned[0] * gauss(), de = cfg_.position_noise_sigma_ned[1] * gauss(),
           dd = cfg_.position_noise_sigma_ned[2] * gauss();
    if (!cfg_.position_ramps.empty()) {
      const api::Vector3 ramp = ramp_offset(rel);
      dn += ramp(0);
      de += ramp(1);
      dd += ramp(2);
    }
    while (next_jump_ < cfg_.position_jumps.size() && rel >= cfg_.position_jumps[next_jump_][0]) {
      const auto& jmp = cfg_.position_jumps[next_jump_++];
      dn += jmp[1];
      de += jmp[2];
      dd += jmp[3];
      if (mediator_)
        mediator_->log_message(LoggingLevel::INFO, "SensorDegradation: injected a position jump of " + std::to_string(jmp[1]) + " / " +
                                                       std::to_string(jmp[2]) + " / " + std::to_string(jmp[3]) + " m NED at t=" +
                                                       std::to_string(rel) + " s.");
    }
    auto out = std::make_shared<aspn23_eigen::MeasurementPosition>(*pos);
    const double lat = pos->get_term1(), alt = pos->get_term3();
    out->set_term1(lat + nav::north_to_delta_lat(dn, lat, alt));
    out->set_term2(pos->get_term2() + nav::east_to_delta_lon(de, lat, alt));
    out->set_term3(alt - dd);
    if (cfg_.position_covariance_scale != 1.0) out->set_covariance(pos->get_covariance() * cfg_.position_covariance_scale);
    return std::vector<Message>{Message(out, message.source_identifier)};
  }
  if (auto pva = message.as<utils::PVA>(); pva && !cfg_.derived_position_channel.empty() &&
                                            pva->get_reference_frame() == ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC) {
    const std::int64_t t = pva->get_time_of_validity().get_elapsed_nsec();
    const std::int64_t period = cfg_.derived_position_rate_hz > 0 ? static_cast<std::int64_t>(1e9 / cfg_.derived_position_rate_hz) : 0;
    if (last_derived_ns_ && t - *last_derived_ns_ < period - 500'000) return std::vector<Message>{message};
    last_derived_ns_ = t;
    const auto& sg = cfg_.derived_position_sigma_ned;
    const double lat = pva->get_p1(), alt = pva->get_p3();
    const double dn = sg[0] * gauss(), de = sg[1] * gauss(), dd = sg[2] * gauss();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cov = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(3, 3);
    cov(0, 0) = sg[0] * sg[0];
    cov(1, 1) = sg[1] * sg[1];
    cov(2, 2) = sg[2] * sg[2];
    auto pos = std::make_shared<aspn23_eigen::MeasurementPosition>(
        aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0), aspn23_eigen::TypeTimestamp(t),
        ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC, lat + nav::north_to_delta_lat(dn, lat, alt),
        pva->get_p2() + nav::east_to_delta_lon(de, lat, alt), alt - dd, cov, ASPN23_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
        Eigen::Matrix<double, Eigen::Dynamic, 1>(0), std::vector<aspn23_eigen::TypeIntegrity>{});
    ++derived_count_;
    return std::vector<Message>{message, Message(pos, cfg_.derived_position_channel)};
  }
  if (auto vel = message.as<aspn23_eigen::MeasurementVelocity>()) {
    auto out = std::make_shared<aspn23_eigen::MeasurementVelocity>(*vel);
    if (utils::present(vel->get_x())) out->set_x(vel->get_x() + cfg_.velocity_noise_sigma[0] * gauss());
    if (utils::present(vel->get_y())) out->set_y(vel->get_y() + cfg_.velocity_noise_sigma[1] * gauss());
    if (utils::present(vel->get_z())) out->set_z(vel->get_z() + cfg_.velocity_noise_sigma[2] * gauss());
    if (cfg_.velocity_covariance_scale != 1.0) out->set_covariance(vel->get_covariance() * cfg_.velocity_covariance_scale);
    return std::vector<Message>{Message(out, message.source_identifier)};
  }
  return std::vector<Message>{message};
}

void AdvancedPreprocessorPlugin::init_plugin(const std::optional<std::string>&, api::Mediator* mediator) {
  mediator_ = mediator;
  if (!mediator_) utils::print_message(LoggingLevel::ERROR, identifier_, "mediator cannot be null");
}

std::unique_ptr<api::Preprocessor> AdvancedPreprocessorPlugin::new_preprocessor(
    std::size_t index, const std::optional<std::string>& config_group) {
  if (!mediator_) {
    utils::print_message(LoggingLevel::ERROR, identifier_,
                         "mediator is null. init_plugin must be called with a valid mediator before new_preprocessor.");
    return nullptr;
  }
  if (index == 1) {
    if (!config_group) {
      mediator_->log_message(LoggingLevel::ERROR, "config_group is a required parameter for preprocessor \"" + ids_[1] +
                                                      "\" and cannot be None.");
      return nullptr;
    }
    auto cfg = SensorDegradationConfig::from_registry(*mediator_, *config_group);
    if (!cfg) {
      mediator_->log_message(LoggingLevel::ERROR, "Failed to populate SensorDegradationConfig for preprocessor " + ids_[1] + ".");
      return nullptr;
    }
    return std::make_unique<SensorDegradationPreprocessor>(*cfg, mediator_);
  }
  if (index != 0) {
    mediator_->log_message(LoggingLevel::ERROR, "Invalid preprocessor index of " + std::to_string(index) +
                                                    ". PreprocessorPlugin provides " + std::to_string(ids_.size()) +
                                                    " preprocessors.");
    return nullptr;
  }
  if (!config_group) {
    mediator_->log_message(LoggingLevel::ERROR, "config_group is a required parameter for preprocessor \"" + ids_[0] +
                                                    "\" and cannot be None.");
    return nullptr;
  }
  auto cfg = ZeroVelocity2dGeneratorConfig::from_registry(*mediator_, *config_group);
  if (!cfg) {
    mediator_->log_message(LoggingLevel::ERROR,
                           "Failed to populate ZeroVelocity2dGeneratorConfig for preprocessor " + ids_[0] + ".");
    return nullptr;
  }
  return std::make_unique<ZeroVelocity2dGenerator>(mediator_, cfg->channels, cfg->trigger_dt_sec, cfg->lateral_vel_sigma,
                                                   cfg->vertical_vel_sigma, cfg->output_channel);
}

}  // namespace pntos::cobra
