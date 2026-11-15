#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/inertial/StandardInertialPlugin.hpp>
#include <pntos/cobra/utils/logging.hpp>

#include <cstdio>

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
  try {
    return inertial::BufferedImu(*pva, cfg->expected_dt, cfg->inertial_buffer_length, {}, {}, cfg->robustness);
  } catch (const std::invalid_argument& e) {
    if (mediator)
      mediator->log_message(LoggingLevel::ERROR, std::string("InertialConfig at ") + group + ": " + e.what());
    throw;
  }
}

bool power_of_ten(std::uint64_t n) {
  while (n >= 10 && n % 10 == 0) n /= 10;
  return n == 1;
}
std::string seconds(Timestamp t) {
  char b[40];
  std::snprintf(b, sizeof b, "%.6f", t.seconds());
  return b;
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
  const auto status = inertial_.add_sample(imu);
  // NOT_NEWER has always been dropped silently; the other outcomes only exist with InertialConfig::robustness.
  if (status != inertial::BufferedImu::AddStatus::ACCEPTED && status != inertial::BufferedImu::AddStatus::NOT_NEWER)
    report(status, Timestamp{imu->get_time_of_validity().get_elapsed_nsec()});
}

void StandardInertial::report(inertial::BufferedImu::AddStatus status, Timestamp time) {
  using S = inertial::BufferedImu::AddStatus;
  std::uint64_t& n = reported_.at(static_cast<std::size_t>(status));
  ++n;
  if (!mediator_ || !power_of_ten(n)) return;
  char detail[160] = "";
  switch (status) {
    case S::GAP_FILLED:
      std::snprintf(detail, sizeof detail,
                    "arrived late and its rate was held across the gap (longest interval so far %.3f s, nominal "
                    "%.4f s)",
                    inertial_.stats().longest_gap, inertial_.estimated_dt());
      break;
    case S::SAMPLED_DROPPED:
      std::snprintf(detail, sizeof detail, "is SAMPLED and was dropped (InertialConfig sampled_imu = \"drop\")");
      break;
    case S::NON_FINITE:
      std::snprintf(detail, sizeof detail, "holds a non-finite measurement and was rejected");
      break;
    case S::GAP_TOO_LONG:
      std::snprintf(detail, sizeof detail,
                    "is more than max_imu_gap (%.3f s) ahead of the latest solution at %.6f s and was rejected",
                    inertial_.robustness().max_gap, request_latest_time().seconds());
      break;
    default: return;
  }
  mediator_->log_message(LoggingLevel::WARN, "IMU sample at " + seconds(time) + " s " + detail + "; occurrence " +
                                                 std::to_string(n) + ".");
}

void StandardInertial::report_refused(const char* what, Timestamp time, std::uint64_t& count) {
  ++count;
  // Only with reject_non_finite: without it a refused reset (time outside the buffer) stays as silent as it was.
  if (!mediator_ || !inertial_.robustness().reject_non_finite || !power_of_ten(count)) return;
  const std::string span = seconds(request_earliest_time()) + " - " + seconds(request_latest_time());
  mediator_->log_message(LoggingLevel::WARN, std::string("Inertial ") + what + " at " + seconds(time) +
                                                 " s refused (not finite, or outside the buffered span " + span +
                                                 " s); occurrence " + std::to_string(count) + ".");
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
  if (!inertial_.reset(pva.get(), nullptr))
    report_refused("solution reset", Timestamp{pva->get_time_of_validity().get_elapsed_nsec()}, refused_resets_);
}

void StandardInertial::correct_sensor_errors(Timestamp time, const api::StandardInertialErrors& e) {
  inertial::ImuErrors errs{e.accel_biases, e.gyro_biases, e.accel_scale_factors, e.gyro_scale_factors, time};
  if (!inertial_.reset(nullptr, &errs)) report_refused("sensor error correction", time, refused_corrections_);
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
