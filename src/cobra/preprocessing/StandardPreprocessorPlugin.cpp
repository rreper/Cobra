#include <pntos/cobra/preprocessing/StandardPreprocessorPlugin.hpp>
#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/logging.hpp>

#include <aspn23/eigen/MeasurementAltitude.hpp>
#include <aspn23/eigen/MeasurementBarometer.hpp>
#include <aspn23/eigen/MeasurementImu.hpp>

#include <cmath>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Message;

namespace {
void warn(api::Mediator* m, const std::string& text) {
  if (m) m->log_message(LoggingLevel::WARN, text);
}
std::vector<Message> pass(const Message& m) { return {m}; }
}  // namespace

// ----------------------------------------------------------------------------- Downsampler

DownsamplerPreprocessor::DownsamplerPreprocessor(const DownsamplerConfig& c, api::Mediator* mediator) {
  if (!c.channels) {
    warn(mediator, "Unsupported type for downsampling channels. Downsampling will be disabled.");
    return;
  }
  if (c.channels->size() != c.downsampling_factors.size()) {
    warn(mediator, "Channels to downsample has " + std::to_string(c.channels->size()) +
                       " elements, but downsampling factors has " + std::to_string(c.downsampling_factors.size()) +
                       ". Downsampling will be disabled.");
    return;
  }
  for (std::size_t i = 0; i < c.channels->size(); ++i) {
    const auto& ch = (*c.channels)[i];
    const auto f = c.downsampling_factors[i];
    if (f < 0) {
      warn(mediator, "Downsampling factor of " + std::to_string(f) + " for channel \"" + ch +
                         "\" cannot be negative. Channel will not be downsampled.");
      continue;
    }
    factors_[ch] = f;
    counters_[ch] = -1;  // first message always passes
  }
}

std::optional<std::vector<Message>> DownsamplerPreprocessor::process_pntos_message(const Message& message) {
  auto it = factors_.find(message.source_identifier);
  if (it == factors_.end()) return pass(message);
  const std::int64_t factor = it->second;
  auto& count = counters_[message.source_identifier];
  // Python: (count + 1) % factor; a factor of 0 would raise ZeroDivisionError there, pass everything here.
  count = factor > 0 ? (count + 1) % factor : 0;
  if (count != 0) return std::nullopt;
  return pass(message);
}

// ----------------------------------------------------------------------------- ImuRotation

ImuRotationPreprocessor::ImuRotationPreprocessor(api::Mediator* mediator, const api::Matrix3& C)
    : mediator_(mediator), C_(C) {}

std::optional<std::vector<Message>> ImuRotationPreprocessor::process_pntos_message(const Message& message) {
  auto imu = message.as<aspn23_eigen::MeasurementImu>();
  if (!imu) {
    warn(mediator_, "ImuRotationPreprocessor expected IMU message, but got another type. Cannot rotate.");
    return pass(message);
  }
  auto out = std::make_shared<aspn23_eigen::MeasurementImu>(*imu);
  Eigen::Matrix<double, Eigen::Dynamic, 1> a = imu->get_meas_accel(), g = imu->get_meas_gyro();
  if (a.size() == 3) out->set_meas_accel(Eigen::Matrix<double, Eigen::Dynamic, 1>(C_ * api::Vector3(a)));
  if (g.size() == 3) out->set_meas_gyro(Eigen::Matrix<double, Eigen::Dynamic, 1>(C_ * api::Vector3(g)));
  return std::vector<Message>{Message(out, message.source_identifier)};
}

// ----------------------------------------------------------------------------- TimeAdjuster

TimeAdjusterPreprocessor::TimeAdjusterPreprocessor(std::int64_t expected_dt_nsec, api::Mediator* mediator)
    : mediator_(mediator), expected_dt_nsec_(expected_dt_nsec) {}

std::optional<std::vector<Message>> TimeAdjusterPreprocessor::process_pntos_message(const Message& message) {
  auto tov = message.wrapped_message ? utils::time_of_validity(*message.wrapped_message) : std::nullopt;
  if (!tov) {
    warn(mediator_, "TimeAdjusterPreprocessor received a message from channel " + message.source_identifier +
                        " with no time of validity. Ignoring message.");
    return pass(message);
  }
  const std::int64_t cur = tov->elapsed_nsec;
  if (!last_nsec_) {
    last_nsec_ = cur;
    return pass(message);
  }
  const bool valid = std::llabs((cur - *last_nsec_) - expected_dt_nsec_) < tolerance_nsec_;
  if (valid) {
    last_nsec_ = cur;
    return pass(message);
  }
  const std::int64_t synthetic = *last_nsec_ + expected_dt_nsec_;
  last_nsec_ = synthetic;
  auto out = utils::with_time_of_validity(*message.wrapped_message, api::Timestamp{synthetic});
  return std::vector<Message>{Message(out, message.source_identifier)};
}

// ----------------------------------------------------------------------------- BarometerToAltitude

BarometerToAltitudePreprocessor::BarometerToAltitudePreprocessor(api::Mediator* mediator, std::optional<double> alt_sigma)
    : mediator_(mediator), alt_sigma_(alt_sigma) {}

double BarometerToAltitudePreprocessor::pressure_to_alt(double pressure, double deg_k, double ref_pressure,
                                                        double ref_alt) {
  const double alt = -(deg_k / 0.0065) * (std::pow(pressure / ref_pressure, 8314.32 * 0.0065 / (9.80665 * 28.9644)) - 1.0);
  return alt + ref_alt;
}

std::optional<std::vector<Message>> BarometerToAltitudePreprocessor::process_pntos_message(const Message& message) {
  auto baro = message.as<aspn23_eigen::MeasurementBarometer>();
  if (!baro) {
    warn(mediator_, "BarometerToAltitudePreprocessor expected barometer message, but got another type. Cannot convert.");
    return pass(message);
  }
  const double altitude = pressure_to_alt(baro->get_pressure(), deg_k_);
  const double variance = alt_sigma_ ? *alt_sigma_ * *alt_sigma_
                                     : baro->get_variance() * std::pow(altitude / baro->get_pressure(), 2);
  auto alt = std::make_shared<aspn23_eigen::MeasurementAltitude>(
      aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_ALTITUDE, baro->get_vendor_id(), baro->get_device_id(),
                               baro->get_context_id(), baro->get_sequence_id()),
      baro->get_time_of_validity(), ASPN23_MEASUREMENT_ALTITUDE_REFERENCE_MSL, altitude, variance,
      ASPN23_MEASUREMENT_ALTITUDE_ERROR_MODEL_NONE, baro->get_error_model_params(), baro->get_integrity());
  std::string channel = message.source_identifier;
  const std::string from = "baro_pressure", to = "altitude";
  for (std::size_t pos = channel.find(from); pos != std::string::npos; pos = channel.find(from, pos + to.size()))
    channel.replace(pos, from.size(), to);
  return std::vector<Message>{Message(alt, channel)};
}

// ----------------------------------------------------------------------------- TimeBias

TimeBiasPreprocessor::TimeBiasPreprocessor(std::int64_t bias, api::Mediator* mediator) : mediator_(mediator), bias_(bias) {}

std::optional<std::vector<Message>> TimeBiasPreprocessor::process_pntos_message(const Message& message) {
  auto tov = message.wrapped_message ? utils::time_of_validity(*message.wrapped_message) : std::nullopt;
  if (!tov) {
    warn(mediator_, "TimeBiasPreprocessor received a message from channel " + message.source_identifier +
                        " with no time of validity. Ignoring message.");
    return pass(message);
  }
  auto out = utils::with_time_of_validity(*message.wrapped_message, api::Timestamp{tov->elapsed_nsec - bias_});
  return std::vector<Message>{Message(out, message.source_identifier)};
}

// ----------------------------------------------------------------------------- Outage

OutagePreprocessor::OutagePreprocessor(double start_time, double end_time, api::Mediator* mediator)
    : mediator_(mediator), start_(start_time), end_(end_time) {}

std::optional<std::vector<Message>> OutagePreprocessor::process_pntos_message(const Message& message) {
  auto tov = message.wrapped_message ? utils::time_of_validity(*message.wrapped_message) : std::nullopt;
  if (!tov) {
    warn(mediator_, "OutagePreprocessor received a message from channel " + message.source_identifier +
                        " with no time of validity. Ignoring message.");
    return pass(message);
  }
  if (!first_ns_) first_ns_ = tov->elapsed_nsec;
  const double rel = (tov->elapsed_nsec - *first_ns_) * 1e-9;
  const bool in_outage = start_ <= rel && rel < end_;
  if (in_outage && !active_) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::INFO, "Beginning outage from time " + std::to_string(start_) + "s to " +
                                                     std::to_string(end_) + "s (cur_time=" + std::to_string(rel) + "s).");
    active_ = true;
  } else if (!in_outage && active_) {
    if (mediator_) mediator_->log_message(LoggingLevel::INFO, "Ending outage at time " + std::to_string(rel) + "s.");
    active_ = false;
  }
  if (active_) return std::nullopt;
  return pass(message);
}

// ----------------------------------------------------------------------------- plugin

StandardPreprocessorPlugin::StandardPreprocessorPlugin(std::string identifier)
    : identifier_(std::move(identifier)),
      ids_{DownsamplerConfig::kIdentifier, ImuRotatorConfig::kIdentifier,  TimeAdjusterConfig::kIdentifier,
           BarometerToAltitudeConfig::kIdentifier, TimeBiasConfig::kIdentifier, OutageConfig::kIdentifier} {}

void StandardPreprocessorPlugin::init_plugin(const std::optional<std::string>&, api::Mediator* mediator) {
  if (!mediator) utils::print_message(LoggingLevel::ERROR, identifier_, "mediator cannot be None");
  mediator_ = mediator;
}

std::unique_ptr<api::Preprocessor> StandardPreprocessorPlugin::new_preprocessor(
    std::size_t index, const std::optional<std::string>& group) {
  if (!mediator_) {
    utils::print_message(LoggingLevel::ERROR, identifier_,
                         "mediator is None. PreprocessorPlugin.init_plugin must be called and passed a valid mediator "
                         "before new_preprocessor.");
    return nullptr;
  }
  if (index >= ids_.size()) {
    mediator_->log_message(LoggingLevel::ERROR, "Invalid preprocessor index of " + std::to_string(index) +
                                                    ". StandardPreprocessorPlugin provides " +
                                                    std::to_string(ids_.size()) + " preprocessors.");
    return nullptr;
  }
  const std::string& id = ids_[index];
  if (!group) {
    mediator_->log_message(LoggingLevel::ERROR,
                           "config_group is a required parameter for preprocessor \"" + id + "\" and cannot be None.");
    return nullptr;
  }
  auto fail = [&](const char* cfg) {
    mediator_->log_message(LoggingLevel::ERROR, std::string("Failed to populate ") + cfg + " for preprocessor " + id + ".");
    return nullptr;
  };
  switch (index) {
    case 0: {
      auto c = DownsamplerConfig::from_registry(*mediator_, *group);
      if (!c) return fail("DownsamplerConfig");
      return std::make_unique<DownsamplerPreprocessor>(*c, mediator_);
    }
    case 1: {
      auto c = ImuRotatorConfig::from_registry(*mediator_, *group);
      if (!c) return fail("ImuRotatorConfig");
      return std::make_unique<ImuRotationPreprocessor>(mediator_, api::Matrix3(to_matrix(c->C_imu_to_platform)));
    }
    case 2: {
      auto c = TimeAdjusterConfig::from_registry(*mediator_, *group);
      if (!c) return fail("TimeAdjusterConfig");
      return std::make_unique<TimeAdjusterPreprocessor>(c->expected_dt_nsec, mediator_);
    }
    case 3: {
      auto c = BarometerToAltitudeConfig::from_registry(*mediator_, *group);
      if (!c) return fail("BarometerToAltitudeConfig");
      return std::make_unique<BarometerToAltitudePreprocessor>(mediator_, c->alt_sigma);
    }
    case 4: {
      auto c = TimeBiasConfig::from_registry(*mediator_, *group);
      if (!c) return fail("TimeBiasConfig");
      return std::make_unique<TimeBiasPreprocessor>(c->time_bias, mediator_);
    }
    default: {
      auto c = OutageConfig::from_registry(*mediator_, *group);
      if (!c) return fail("OutageConfig");
      return std::make_unique<OutagePreprocessor>(c->start_time, c->end_time, mediator_);
    }
  }
}

}  // namespace pntos::cobra
