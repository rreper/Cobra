// Config convention (port of pntos.cobra.config.BaseConfig / config_to_registry / config_from_registry).
//
// A config is a struct with a registry `group`. Its fields are stored flat in that group with the
// same key layout as the Python implementation, so configs written by either side are readable by
// the other:
//   * scalars (int/float/str/bool) under their field name; enums as their integer value
//   * numeric sequences / matrices as a double matrix under the field name
//   * string sequences as a string array
//   * EstimateWithCovariance as `_estimate`, `_covariance`, `_ewc_type`
//   * nested config: written to its own group, pointer key `_<field>_groups` (string, or string
//     array for a sequence of configs)
//   * optional fields that are unset are simply absent
//
// Python used dataclass reflection; here every config implements write()/read() explicitly with the
// helpers in ConfigIO.
#pragma once

#include <pntos/api/common.hpp>

#include <string>

namespace pntos::cobra {

class BaseConfig {
 public:
  virtual ~BaseConfig() = default;
  virtual const std::string& group() const = 0;
  /// Store this config into the registry reachable through `mediator`.
  virtual void to_registry(api::Mediator& mediator) const = 0;
};

/// Helpers for implementing to_registry / from_registry.
class ConfigWriter {
 public:
  ConfigWriter(api::Mediator& mediator, const std::string& group);
  ~ConfigWriter();
  ConfigWriter(const ConfigWriter&) = delete;
  ConfigWriter& operator=(const ConfigWriter&) = delete;

  void scalar(const std::string& key, double v);
  void scalar(const std::string& key, std::int64_t v);
  void scalar(const std::string& key, int v) { scalar(key, static_cast<std::int64_t>(v)); }
  void scalar(const std::string& key, bool v);
  void scalar(const std::string& key, const std::string& v);
  void scalar(const std::string& key, const char* v) { scalar(key, std::string(v)); }
  void matrix(const std::string& key, const api::Matrix& m);
  void vector(const std::string& key, const api::Vector& v) { matrix(key, api::Matrix(v)); }
  void strings(const std::string& key, const api::StringArray& v);
  void ewc(const api::EstimateWithCovariance& e);
  /// Writes `nested` to its own group and records the pointer key.
  void nested(const std::string& key, const BaseConfig& nested);
  void nested(const std::string& key, const std::vector<std::shared_ptr<const BaseConfig>>& nested);
  template <class T>
  void optional(const std::string& key, const std::optional<T>& v) {
    if (v) scalar(key, *v);
  }

 private:
  void write(const std::string& key, api::RegistryValue v);
  api::Mediator& mediator_;
  std::string group_;
  std::shared_ptr<api::KeyValueStore> kv_;
};

class ConfigReader {
 public:
  /// Logs and sets ok()=false if the group does not exist.
  ConfigReader(api::Mediator& mediator, const std::string& group);
  ~ConfigReader();
  ConfigReader(const ConfigReader&) = delete;
  ConfigReader& operator=(const ConfigReader&) = delete;

  bool ok() const { return ok_; }
  const std::string& group() const { return group_; }
  api::Mediator& mediator() const { return mediator_; }

  /// Required field: logs a WARN and marks the read failed if missing/unconvertible.
  template <class T>
  T require(const std::string& key, const T& fallback = T{}) {
    auto v = optional<T>(key);
    if (!v) {
      fail(key);
      return fallback;
    }
    return *v;
  }
  /// Optional field: nullopt if missing.
  template <class T>
  std::optional<T> optional(const std::string& key) {
    if (!kv_ || !kv_->has_key(key)) return std::nullopt;
    auto v = kv_->template get_value<T>(key);
    if (!v) {
      fail(key);
      return std::nullopt;
    }
    return v;
  }
  std::optional<api::EstimateWithCovariance> ewc();
  api::EstimateWithCovariance require_ewc();
  /// Group name(s) recorded for a nested config field, if present.
  std::optional<std::string> nested_group(const std::string& key);
  std::optional<api::StringArray> nested_groups(const std::string& key);
  /// Suspend/resume this batch so a nested group can be read (registry batches are per group).
  void suspend();
  void resume();

  void fail(const std::string& key);

 private:
  api::Mediator& mediator_;
  std::string group_;
  std::shared_ptr<api::KeyValueStore> kv_;
  bool ok_ = true;
  bool live_ = false;
};

}  // namespace pntos::cobra
