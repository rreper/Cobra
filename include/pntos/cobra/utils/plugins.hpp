// Plugin sorting / validation helpers (port of pntos.cobra.utils.plugins, minus the catalog UI).
#pragma once

#include <pntos/api/api.hpp>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace pntos::cobra {

/// Plugins bucketed by abstract type (names match the Python dataclass fields).
struct SortedPlugins {
  std::vector<std::shared_ptr<api::ControllerPlugin>> controller_plugins;
  std::vector<std::shared_ptr<api::FusionPlugin>> fusion_plugins;
  std::vector<std::shared_ptr<api::FusionStrategyPlugin>> fusion_strategy_plugins;
  std::vector<std::shared_ptr<api::InertialPlugin>> inertial_plugins;
  std::vector<std::shared_ptr<api::InitializationPlugin>> initialization_plugins;
  std::vector<std::shared_ptr<api::LoggingPlugin>> logging_plugins;
  std::vector<std::shared_ptr<api::OrchestrationPlugin>> orchestration_plugins;
  std::vector<std::shared_ptr<api::PlatformIntegrationPlugin>> platform_integration_plugins;
  std::vector<std::shared_ptr<api::PreprocessorPlugin>> preprocessor_plugins;
  std::vector<std::shared_ptr<api::RegistryPlugin>> registry_plugins;
  std::vector<std::shared_ptr<api::StateModelingPlugin>> state_modeling_plugins;
  std::vector<std::shared_ptr<api::TransportPlugin>> transport_plugins;
  std::vector<std::shared_ptr<api::UiPlugin>> ui_plugins;
  std::vector<std::shared_ptr<api::UtilityPlugin>> utility_plugins;

  /// Count for a Python-style field name ("fusion_plugins", ...); -1 if unknown.
  int count(const std::string& field) const;
};

SortedPlugins sort_plugins(const api::PluginList& plugins);

using LogFn = std::function<void(api::LoggingLevel, const std::string&)>;
/// (min, max) inclusive expected counts per field name. Logs an ERROR and returns false on the
/// first violation (or if `expected` is empty / names an unknown field).
bool validate_plugins(const SortedPlugins& sorted, const LogFn& log,
                      const std::vector<std::pair<std::string, std::pair<int, int>>>& expected);

}  // namespace pntos::cobra
