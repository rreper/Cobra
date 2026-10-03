#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/inertial/StandardInertialPlugin.hpp>
#include <pntos/cobra/utils/logging.hpp>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Message;
using api::Timestamp;

namespace {
inertial::BufferedImu make_buffered(const std::string& group, api::Mediator* mediator, const Message& solution) {
  auto pva = solution.as<utils::PVA>();
  if (!pva) {
    if (mediator) mediator->log_message(LoggingLevel::ERROR, "Message must be of type MeasurementPositionVelocityAttitude.");
    throw std::invalid_argument("StandardInertial requires a PVA solution");
  }
  std::optional<InertialConfig> cfg = mediator ? InertialConfig::from_registry(*mediator, group) : std::nullopt;
  if (!cfg) {
    if (mediator) mediator->log_message(LoggingLevel::ERROR, "Unable to retrieve config from registry.");
    throw std::invalid_argument("StandardInertial: missing InertialConfig at " + group);
  }
  return inertial::BufferedImu(*pva, cfg->expected_dt, cfg->inertial_buffer_length);
}
}  // namespace

StandardInertial::StandardInertial(const std::string& config_group, api::Mediator* mediator, const Message& solution)
    : mediator_(mediator), inertial_(make_buffered(config_group, mediator, solution)) {}

Message StandardInertial::request_current_solution() {
  return Message(inertial_.calc_pva(request_latest_time()), kIdentifier);
}

std::optional<Message> StandardInertial::request_solution(Timestamp time) {
  auto p = inertial_.calc_pva(time);
  if (!p) return std::nullopt;
  return Message(p, kIdentifier);
}

std::optional<std::vector<std::optional<Message>>> StandardInertial::request_solutions(
    const std::vector<Timestamp>& times, api::InertialSolutionRangeType type) {
  if (times.empty()) return std::nullopt;
  std::vector<std::optional<Message>> out;
  for (auto t : times) {
    std::shared_ptr<utils::PVA> p;
    if (type == api::InertialSolutionRangeType::BEST_KNOWN_SOLUTION)
      p = inertial_.calc_pva(t);
    else if (type == api::InertialSolutionRangeType::NO_UPDATES_WITHIN_RANGE)
      p = inertial_.calc_pva_no_reset_since(t, times[0]);
    else
      return std::nullopt;
    out.push_back(p ? std::optional<Message>(Message(p, kIdentifier)) : std::nullopt);
  }
  return out;
}

void StandardInertial::process_pntos_message(const Message& message) {
  auto imu = message.as<aspn23_eigen::MeasurementImu>();
  if (!imu) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::WARN,
                             "Invalid message type received; ignoring it. See request_process_pntos_message_types() "
                             "for valid message types.");
    return;
  }
  inertial_.add(imu);
}

std::optional<api::InertialForcesRates> StandardInertial::request_forces_and_rates(Timestamp time) {
  auto fr = inertial_.calc_force_and_rate(time);
  if (!fr) return std::nullopt;
  return api::InertialForcesRates{fr, api::InertialFrame::NED};
}

std::optional<api::InertialForcesRates> StandardInertial::request_average_forces_and_rates(Timestamp t1,
                                                                                           Timestamp t2) {
  auto fr = inertial_.calc_force_and_rate(t1, t2);
  if (!fr) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::WARN, "Requested average force and rate spanning time [" +
                                                     std::to_string(t1.seconds()) + ", " + std::to_string(t2.seconds()) +
                                                     "], but inertial only spans [" +
                                                     std::to_string(request_earliest_time().seconds()) + ", " +
                                                     std::to_string(request_latest_time().seconds()) + "]");
    return std::nullopt;
  }
  return api::InertialForcesRates{fr, api::InertialFrame::NED};
}

void StandardInertial::reset_solution(const Message& message) {
  auto pva = message.as<utils::PVA>();
  if (!pva) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, "Message must be of type MeasurementPositionVelocityAttitude.");
    return;
  }
  inertial_.reset(pva.get(), nullptr);
}

void StandardInertial::correct_sensor_errors(Timestamp time, const api::StandardInertialErrors& e) {
  inertial::ImuErrors errs{e.accel_biases, e.gyro_biases, e.accel_scale_factors, e.gyro_scale_factors, time};
  inertial_.reset(nullptr, &errs);
}

std::optional<api::StandardInertialErrors> StandardInertial::request_sensor_errors(Timestamp time) {
  auto e = inertial_.imu_errors(time);
  return api::StandardInertialErrors{e.accel_biases, e.gyro_biases, e.accel_scale_factors, e.gyro_scale_factors};
}

// ----------------------------------------------------------------------------- plugin

void StandardInertialPlugin::init_plugin(const std::optional<std::string>&, api::Mediator* mediator) {
  mediator_ = mediator;
  if (!mediator) utils::print_message(LoggingLevel::ERROR, identifier_, "mediator cannot be None.");
}

std::unique_ptr<api::CommonInertial> StandardInertialPlugin::new_inertial(api::InertialType type,
                                                                          const Message& solution,
                                                                          const std::optional<std::string>& group) {
  if (!mediator_) {
    utils::print_message(LoggingLevel::ERROR, identifier_,
                         "mediator is None. StandardInertialPlugin.init_plugin must be called and passed a valid "
                         "mediator before new_inertial.");
    return nullptr;
  }
  if (!group) {
    mediator_->log_message(LoggingLevel::ERROR, "config_group is a required parameter for this plugin and cannot be None.");
    return nullptr;
  }
  if (!is_inertial_type_supported(type)) {
    mediator_->log_message(LoggingLevel::ERROR, "Unsupported type requested.");
    return nullptr;
  }
  try {
    return std::make_unique<StandardInertial>(*group, mediator_, solution);
  } catch (const std::exception&) {
    return nullptr;
  }
}

}  // namespace pntos::cobra
