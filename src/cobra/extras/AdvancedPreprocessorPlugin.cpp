#include <pntos/cobra/extras/AdvancedPreprocessorPlugin.hpp>

#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/logging.hpp>

#include <aspn23/eigen/TypeHeader.hpp>

#include <algorithm>
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
