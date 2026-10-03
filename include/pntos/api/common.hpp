// pntOS C++ API — KeyValueStore, Registry, Mediator, CommonPlugin.
//
// Mirrors pntos.api.plugins.common (Python) and pntOS-C plugins/common.h. Where the Python API
// relies on dynamic typing (generic get_value, callback identity), this header uses templates,
// std::variant and opaque notification tokens.
#pragma once

#include <pntos/api/types.hpp>

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace pntos::api {

class KeyValueStore;

/// Token identifying a notification registration (replaces Python's callback-identity semantics).
struct NotifyToken {
  std::uint64_t id = 0;
  friend bool operator==(NotifyToken a, NotifyToken b) { return a.id == b.id; }
};

/// Callback invoked on batch_end for modified keys: (group, modified_keys, store).
/// Must not set values in the store or call the Mediator (see concurrency rules).
using KeyNotifyCallback =
    std::function<void(const std::string& group, const std::vector<std::string>& modified_keys,
                       KeyValueStore& store)>;

/// Convert a RegistryValue to T if a conversion exists (same conversion table as Cobra's
/// StandardKeyValueStore). Returns nullopt if not convertible.
template <class T>
std::optional<T> registry_value_as(const RegistryValue& v);

/// A key-value store implemented with a string key; the unit of a registry "group".
///
/// All getters/setters are only valid between Registry::batch_start()/batch_restart() and
/// batch_end(). See Registry.
class KeyValueStore {
 public:
  virtual ~KeyValueStore() = default;

  /// Keys currently in the store, nullopt if none.
  virtual std::optional<std::vector<std::string>> keys() const = 0;
  virtual bool has_key(const std::string& key) const = 0;
  /// Type of the stored value, KEY_DNE if absent.
  virtual RegistryValueType get_type(const std::string& key) const = 0;

  /// Raw stored value, nullopt if absent.
  virtual std::optional<RegistryValue> get(const std::string& key) const = 0;
  /// Typed access with conversion (int→double, number→string, string array→matrix, ...).
  template <class T>
  std::optional<T> get_value(const std::string& key) const {
    auto v = get(key);
    if (!v) return std::nullopt;
    return registry_value_as<T>(*v);
  }
  /// Bytes for the given key per data_format(); nullopt if absent or unconvertible.
  virtual std::optional<std::vector<unsigned char>> get_raw(const std::optional<std::string>& key) const = 0;

  virtual void set(const std::string& key, RegistryValue value) = 0;
  virtual void set_raw(const std::optional<std::string>& key, const std::vector<unsigned char>& bytes) = 0;
  virtual bool remove_key(const std::string& key) = 0;
  virtual void clear() = 0;

  virtual std::vector<RegistryValue> values() const = 0;
  virtual std::vector<std::pair<std::string, RegistryValue>> items() const = 0;
  virtual std::size_t size() const = 0;

  /// Ends a batch: flushes permanent keys, runs notification callbacks for modified keys.
  virtual void batch_end() = 0;
  /// Re-acquires the store after batch_end().
  virtual void batch_restart() = 0;

  /// Register a callback for `key` (or every key when nullopt). Returns a token, or nullopt if
  /// the store cannot notify.
  virtual std::optional<NotifyToken> request_notify(const std::optional<std::string>& key,
                                                    KeyNotifyCallback callback) = 0;
  virtual bool remove_notify(NotifyToken token) = 0;

  /// Tag subsequent sets as permanently stored; returns the resulting permanent flag.
  virtual bool set_permanent(bool permanent) = 0;

  virtual KeyValueStoreDataFormat data_format() const = 0;
};

/// RAII helper: calls batch_end() when destroyed (the C++ analogue of `with registry.batch_start()`).
class Batch {
 public:
  explicit Batch(std::shared_ptr<KeyValueStore> kv) : kv_(std::move(kv)) {}
  Batch(const Batch&) = delete;
  Batch& operator=(const Batch&) = delete;
  Batch(Batch&& o) noexcept : kv_(std::move(o.kv_)) { o.kv_ = nullptr; }
  ~Batch() {
    if (kv_) kv_->batch_end();
  }
  KeyValueStore& operator*() const { return *kv_; }
  KeyValueStore* operator->() const { return kv_.get(); }
  std::shared_ptr<KeyValueStore> store() const { return kv_; }

 private:
  std::shared_ptr<KeyValueStore> kv_;
};

/// A registry of key/value data organised by (string) groups.
class Registry {
 public:
  virtual ~Registry() = default;

  /// Begin a batch on `group` (creating it if needed). The returned store stays owned by the
  /// registry; the caller must call batch_end() (or wrap it in a Batch).
  virtual std::shared_ptr<KeyValueStore> batch_start(const std::string& group) = 0;
  /// Convenience: batch_start wrapped in RAII.
  Batch batch(const std::string& group) { return Batch(batch_start(group)); }

  virtual std::optional<std::vector<std::string>> group_array() const = 0;
  virtual bool has_group(const std::string& group) const = 0;
  virtual bool request_notify_new_group(std::function<void(const std::string& group)> callback) = 0;
};

/// The set of callbacks handed to a plugin on initialisation. All functions are thread-safe for
/// plugins to call; plugins must not call functions they are themselves responsible for.
class Mediator {
 public:
  virtual ~Mediator() = default;

  virtual std::vector<std::string> filter_description_list() const = 0;

  /// Returns one optional Message per requested time, or nullopt if filter_description is invalid.
  virtual std::optional<std::vector<std::optional<Message>>> request_solutions(
      const std::vector<Timestamp>& solution_times,
      const std::optional<std::string>& filter_description = std::nullopt) = 0;

  virtual void process_pntos_message(const Message& message) = 0;

  virtual void broadcast_aspn_message(const Message& message,
                                      const std::optional<std::string>& transport = std::nullopt,
                                      const std::optional<std::string>& destination_identifier =
                                          std::nullopt) = 0;

  virtual void log_message(LoggingLevel level, const std::string& message) = 0;

  virtual Registry& registry() = 0;
};

/// Common definitions that all plugins must provide.
class CommonPlugin {
 public:
  virtual ~CommonPlugin() = default;

  /// Called once before any other method. `mediator` is null only for controller plugins and
  /// must outlive the plugin.
  virtual void init_plugin(const std::optional<std::string>& plugin_resources_location,
                           Mediator* mediator) = 0;
  virtual void shutdown_plugin() = 0;

  virtual const std::string& identifier() const = 0;
  /// The abstract plugin kind (replaces Python isinstance checks against the API base classes).
  virtual PluginType plugin_type() const = 0;
};

// ---------------------------------------------------------------------------
// registry_value_as conversions (mirrors StandardKeyValueStore.type_conversion)
// ---------------------------------------------------------------------------
namespace detail {
inline std::optional<Matrix> string_array_to_matrix(const StringArray& a) {
  Matrix m(static_cast<Eigen::Index>(a.size()), 1);
  for (std::size_t i = 0; i < a.size(); ++i) {
    try {
      std::size_t pos = 0;
      double d = std::stod(a[i], &pos);
      if (pos != a[i].size()) return std::nullopt;
      m(static_cast<Eigen::Index>(i), 0) = d;
    } catch (...) {
      return std::nullopt;
    }
  }
  return m;
}
inline std::string matrix_to_string(const Matrix& m) {
  std::string s = "[";
  for (Eigen::Index i = 0; i < m.size(); ++i) {
    if (i) s += ", ";
    s += std::to_string(m(i));
  }
  return s + "]";
}
}  // namespace detail

template <>
inline std::optional<std::string> registry_value_as<std::string>(const RegistryValue& v) {
  switch (v.index()) {
    case 0: return std::get<std::string>(v);
    case 1: {
      const auto& a = std::get<StringArray>(v);
      return a.size() == 1 ? std::optional<std::string>(a[0]) : std::nullopt;
    }
    case 2: return std::to_string(std::get<std::int64_t>(v));
    case 3: return std::get<bool>(v) ? "True" : "False";
    case 4: return std::to_string(std::get<double>(v));
    case 5: return detail::matrix_to_string(std::get<Matrix>(v));
    default: return std::nullopt;
  }
}
template <>
inline std::optional<StringArray> registry_value_as<StringArray>(const RegistryValue& v) {
  switch (v.index()) {
    case 0: return StringArray{std::get<std::string>(v)};
    case 1: return std::get<StringArray>(v);
    case 2: return StringArray{std::to_string(std::get<std::int64_t>(v))};
    case 3: return StringArray{std::get<bool>(v) ? "True" : "False"};
    case 4: return StringArray{std::to_string(std::get<double>(v))};
    case 5: {
      const auto& m = std::get<Matrix>(v);
      StringArray out;
      out.reserve(static_cast<std::size_t>(m.size()));
      for (Eigen::Index i = 0; i < m.size(); ++i) out.push_back(std::to_string(m(i)));
      return out;
    }
    default: return std::nullopt;
  }
}
template <>
inline std::optional<std::int64_t> registry_value_as<std::int64_t>(const RegistryValue& v) {
  switch (v.index()) {
    case 0: {
      try {
        std::size_t pos = 0;
        auto r = std::stoll(std::get<std::string>(v), &pos);
        if (pos != std::get<std::string>(v).size()) return std::nullopt;
        return r;
      } catch (...) {
        return std::nullopt;
      }
    }
    case 2: return std::get<std::int64_t>(v);
    case 3: return std::get<bool>(v) ? 1 : 0;
    default: return std::nullopt;  // float→int is unsupported in Cobra too
  }
}
template <>
inline std::optional<int> registry_value_as<int>(const RegistryValue& v) {
  auto r = registry_value_as<std::int64_t>(v);
  if (!r) return std::nullopt;
  return static_cast<int>(*r);
}
template <>
inline std::optional<bool> registry_value_as<bool>(const RegistryValue& v) {
  switch (v.index()) {
    case 0: {
      const auto& s = std::get<std::string>(v);
      if (s == "True" || s == "true" || s == "1") return true;
      if (s == "False" || s == "false" || s == "0" || s.empty()) return false;
      return !s.empty();
    }
    case 2: return std::get<std::int64_t>(v) != 0;
    case 3: return std::get<bool>(v);
    default: return std::nullopt;
  }
}
template <>
inline std::optional<double> registry_value_as<double>(const RegistryValue& v) {
  switch (v.index()) {
    case 0: {
      try {
        std::size_t pos = 0;
        auto r = std::stod(std::get<std::string>(v), &pos);
        if (pos != std::get<std::string>(v).size()) return std::nullopt;
        return r;
      } catch (...) {
        return std::nullopt;
      }
    }
    case 2: return static_cast<double>(std::get<std::int64_t>(v));
    case 3: return std::get<bool>(v) ? 1.0 : 0.0;
    case 4: return std::get<double>(v);
    default: return std::nullopt;
  }
}
template <>
inline std::optional<Matrix> registry_value_as<Matrix>(const RegistryValue& v) {
  switch (v.index()) {
    case 1: return detail::string_array_to_matrix(std::get<StringArray>(v));
    case 2: return Matrix::Constant(1, 1, static_cast<double>(std::get<std::int64_t>(v)));
    case 3: return Matrix::Constant(1, 1, std::get<bool>(v) ? 1.0 : 0.0);
    case 4: return Matrix::Constant(1, 1, std::get<double>(v));
    case 5: return std::get<Matrix>(v);
    default: return std::nullopt;
  }
}
template <>
inline std::optional<Vector> registry_value_as<Vector>(const RegistryValue& v) {
  auto m = registry_value_as<Matrix>(v);
  if (!m) return std::nullopt;
  return Vector(Eigen::Map<const Vector>(m->data(), m->size()));
}
template <>
inline std::optional<Message> registry_value_as<Message>(const RegistryValue& v) {
  if (v.index() == 6) return std::get<Message>(v);
  return std::nullopt;
}

}  // namespace pntos::api
