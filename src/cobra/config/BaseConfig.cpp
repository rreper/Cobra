#include <pntos/cobra/config/BaseConfig.hpp>

namespace pntos::cobra {

using api::LoggingLevel;
using api::RegistryValue;

// ----------------------------------------------------------------------------- ConfigWriter

ConfigWriter::ConfigWriter(api::Mediator& mediator, const std::string& group)
    : mediator_(mediator), group_(group), kv_(mediator.registry().batch_start(group)) {}

ConfigWriter::~ConfigWriter() {
  if (kv_) kv_->batch_end();
}

void ConfigWriter::write(const std::string& key, RegistryValue v) {
  if (kv_->has_key(key)) {
    auto old = kv_->get(key);
    bool same = old && old->index() == v.index();
    if (same) {
      if (v.index() == 5) same = std::get<api::Matrix>(*old).rows() == std::get<api::Matrix>(v).rows() &&
                                 std::get<api::Matrix>(*old).cols() == std::get<api::Matrix>(v).cols() &&
                                 std::get<api::Matrix>(*old) == std::get<api::Matrix>(v);
      else if (v.index() == 6) same = false;
      else same = *old == v;
    }
    if (!same) mediator_.log_message(LoggingLevel::WARN, "Overwriting registry key '" + key + "'.");
  }
  kv_->set(key, std::move(v));
}

void ConfigWriter::scalar(const std::string& key, double v) { write(key, v); }
void ConfigWriter::scalar(const std::string& key, std::int64_t v) { write(key, v); }
void ConfigWriter::scalar(const std::string& key, bool v) { write(key, v); }
void ConfigWriter::scalar(const std::string& key, const std::string& v) { write(key, v); }
void ConfigWriter::matrix(const std::string& key, const api::Matrix& m) { write(key, m); }
void ConfigWriter::strings(const std::string& key, const api::StringArray& v) { write(key, v); }

void ConfigWriter::ewc(const api::EstimateWithCovariance& e) {
  write("_estimate", api::Matrix(e.estimate));
  write("_covariance", e.covariance);
  write("_ewc_type", static_cast<std::int64_t>(e.type));
}

void ConfigWriter::nested(const std::string& key, const BaseConfig& nested) {
  kv_->batch_end();
  nested.to_registry(mediator_);
  kv_->batch_restart();
  write("_" + key + "_groups", nested.group());
}

void ConfigWriter::nested(const std::string& key, const std::vector<std::shared_ptr<const BaseConfig>>& nested) {
  api::StringArray groups;
  for (const auto& n : nested) {
    groups.push_back(n->group());
    kv_->batch_end();
    n->to_registry(mediator_);
    kv_->batch_restart();
  }
  write("_" + key + "_groups", groups);
}

// ----------------------------------------------------------------------------- ConfigReader

ConfigReader::ConfigReader(api::Mediator& mediator, const std::string& group) : mediator_(mediator), group_(group) {
  if (!mediator.registry().has_group(group)) {
    mediator.log_message(LoggingLevel::ERROR, "config_from_registry: group " + group + " does not exist in the registry.");
    ok_ = false;
    return;
  }
  kv_ = mediator.registry().batch_start(group);
  live_ = true;
}

ConfigReader::~ConfigReader() {
  if (kv_ && live_) kv_->batch_end();
}

void ConfigReader::suspend() {
  if (kv_ && live_) {
    kv_->batch_end();
    live_ = false;
  }
}

void ConfigReader::resume() {
  if (kv_ && !live_) {
    kv_->batch_restart();
    live_ = true;
  }
}

void ConfigReader::fail(const std::string& key) {
  mediator_.log_message(LoggingLevel::WARN, "Could not retrieve " + key + " from store");
  ok_ = false;
}

std::optional<api::EstimateWithCovariance> ConfigReader::ewc() {
  if (!kv_ || !kv_->has_key("_ewc_type")) return std::nullopt;
  auto type = kv_->get_value<std::int64_t>("_ewc_type");
  auto est = kv_->get_value<api::Vector>("_estimate");
  auto cov = kv_->get_value<api::Matrix>("_covariance");
  if (!type || !est || !cov) {
    fail("estimate_with_covariance");
    return std::nullopt;
  }
  return api::EstimateWithCovariance{static_cast<api::EstimateWithCovarianceType>(*type), *est, *cov};
}

api::EstimateWithCovariance ConfigReader::require_ewc() {
  auto e = ewc();
  if (!e) {
    fail("estimate_with_covariance");
    return {};
  }
  return *e;
}

std::optional<std::string> ConfigReader::nested_group(const std::string& key) {
  return optional<std::string>("_" + key + "_groups");
}

std::optional<api::StringArray> ConfigReader::nested_groups(const std::string& key) {
  return optional<api::StringArray>("_" + key + "_groups");
}

}  // namespace pntos::cobra
