// In-memory group/key/value registry (port of pntos.cobra.StandardRegistryPlugin).
#pragma once

#include <pntos/api/registry.hpp>
#include <pntos/cobra/config/BaseConfig.hpp>

#include <filesystem>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <unordered_map>
#include <vector>

namespace pntos::cobra {

using LogFunc = std::function<void(api::LoggingLevel, const std::string&)>;

/// One registry group. Insertion-ordered keys (Python dict semantics). Misuse outside a batch is
/// logged as an ERROR (not thrown), as in Cobra.
///
/// Permanency: keys tagged with set_permanent(true) are written on batch_end to
/// `<dir>/<group>.registry` as a simple typed text format and reloaded on construction.
/// Messages are not persisted (Cobra pickled them; there is no portable equivalent).
class StandardKeyValueStore final : public api::KeyValueStore {
 public:
  StandardKeyValueStore(std::string group, LogFunc log, const std::optional<std::string>& permanency_dir = std::nullopt);

  std::optional<std::vector<std::string>> keys() const override;
  bool has_key(const std::string& key) const override;
  api::RegistryValueType get_type(const std::string& key) const override;
  std::optional<api::RegistryValue> get(const std::string& key) const override;
  std::optional<std::vector<unsigned char>> get_raw(const std::optional<std::string>& key) const override;
  void set(const std::string& key, api::RegistryValue value) override;
  void set_raw(const std::optional<std::string>& key, const std::vector<unsigned char>& bytes) override;
  bool remove_key(const std::string& key) override;
  void clear() override;
  std::vector<api::RegistryValue> values() const override;
  std::vector<std::pair<std::string, api::RegistryValue>> items() const override;
  std::size_t size() const override { return order_.size(); }
  void batch_end() override;
  void batch_restart() override;
  std::optional<api::NotifyToken> request_notify(const std::optional<std::string>& key,
                                                 api::KeyNotifyCallback callback) override;
  bool remove_notify(api::NotifyToken token) override;
  bool set_permanent(bool permanent) override;
  api::KeyValueStoreDataFormat data_format() const override { return api::KeyValueStoreDataFormat::UNSPECIFIED; }

  /// Used by StandardRegistry (Cobra accessed the private flag directly).
  bool batch_live() const { return batch_live_; }
  void mark_batch_live() { batch_live_ = true; }
  const std::string& group() const { return group_; }

 private:
  void check_batch_operation() const;
  void load_permanent();
  void save_permanent();

  std::string group_;
  LogFunc log_;
  std::unordered_map<std::string, api::RegistryValue> store_;
  std::vector<std::string> order_;  // insertion order of keys
  struct Callback {
    std::optional<std::string> key;
    api::KeyNotifyCallback fn;
  };
  std::map<std::uint64_t, Callback> callbacks_;
  std::uint64_t next_token_ = 1;
  std::set<std::string> permanent_keys_;
  bool set_permanent_ = false;
  std::vector<std::string> modified_keys_;  // ordered, unique
  bool batch_live_ = false;
  std::optional<std::filesystem::path> permanency_file_;
};

class StandardRegistry final : public api::Registry {
 public:
  explicit StandardRegistry(LogFunc log, const std::optional<std::string>& permanency_dir = std::nullopt);

  std::shared_ptr<api::KeyValueStore> batch_start(const std::string& group) override;
  std::optional<std::vector<std::string>> group_array() const override;
  bool has_group(const std::string& group) const override;
  bool request_notify_new_group(std::function<void(const std::string&)> callback) override;

  /// batch_end any live stores (used on shutdown).
  void end_all_batches();

 private:
  LogFunc log_;
  std::optional<std::string> permanency_dir_;
  std::map<std::string, std::shared_ptr<StandardKeyValueStore>> groups_;
  std::vector<std::string> group_order_;
  std::vector<std::function<void(const std::string&)>> callbacks_;
  mutable std::mutex mutex_;
};

/// Creates StandardRegistry instances pre-populated with the configs given at construction.
class StandardRegistryPlugin final : public api::RegistryPlugin {
 public:
  explicit StandardRegistryPlugin(std::string identifier, std::vector<std::shared_ptr<const BaseConfig>> config = {});

  void init_plugin(const std::optional<std::string>& plugin_resources_location, api::Mediator* mediator) override;
  void shutdown_plugin() override;
  const std::string& identifier() const override { return identifier_; }

  std::shared_ptr<api::Registry> new_registry(const std::optional<std::string>& initial_config = std::nullopt) override;

  api::Mediator* mediator() const { return mediator_; }

 private:
  void log(api::LoggingLevel level, const std::string& message) const;

  std::string identifier_;
  std::vector<std::shared_ptr<const BaseConfig>> config_;
  std::vector<std::shared_ptr<StandardRegistry>> registries_;
  std::optional<std::string> plugin_resources_location_;
  api::Mediator* mediator_ = nullptr;
};

}  // namespace pntos::cobra
