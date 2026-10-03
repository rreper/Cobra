// pntOS C++ API — RegistryPlugin and LoggingPlugin.
#pragma once

#include <pntos/api/common.hpp>

namespace pntos::api {

/// Factory for Registry instances.
class RegistryPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::REGISTRY; }

  /// Create a registry from an implementation-specific initial config (nullopt allowed).
  /// Stores for different groups must be usable concurrently.
  virtual std::shared_ptr<Registry> new_registry(const std::optional<std::string>& initial_config = std::nullopt) = 0;
};

/// Sink for log messages.
class LoggingPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::LOGGING; }

  virtual void log(PluginType source_plugin_type, const std::string& source_plugin_identifier,
                   LoggingLevel level, const std::string& message) = 0;
};

/// A plugin with no API beyond CommonPlugin.
class UtilityPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::UTILITY; }
};

/// Developer UI hook.
class UiPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::UI; }

  virtual bool requires_main_thread() const = 0;
  /// Only called (from the main thread) when requires_main_thread() is true.
  virtual void run_main_thread() = 0;
};

}  // namespace pntos::api
