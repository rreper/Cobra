#include <pntos/cobra/utils/plugins.hpp>

namespace pntos::cobra {

using api::LoggingLevel;
using api::PluginType;

namespace {
template <class T>
void push(std::vector<std::shared_ptr<T>>& out, const std::shared_ptr<api::CommonPlugin>& p) {
  if (auto t = std::dynamic_pointer_cast<T>(p)) out.push_back(std::move(t));
}
}  // namespace

SortedPlugins sort_plugins(const api::PluginList& plugins) {
  SortedPlugins s;
  for (const auto& p : plugins) {
    if (!p) continue;
    switch (p->plugin_type()) {
      case PluginType::CONTROLLER: push(s.controller_plugins, p); break;
      case PluginType::FUSION: push(s.fusion_plugins, p); break;
      case PluginType::FUSION_STRATEGY: push(s.fusion_strategy_plugins, p); break;
      case PluginType::INERTIAL: push(s.inertial_plugins, p); break;
      case PluginType::INITIALIZATION: push(s.initialization_plugins, p); break;
      case PluginType::LOGGING: push(s.logging_plugins, p); break;
      case PluginType::ORCHESTRATION: push(s.orchestration_plugins, p); break;
      case PluginType::PLATFORM_INTEGRATION: push(s.platform_integration_plugins, p); break;
      case PluginType::PREPROCESSOR: push(s.preprocessor_plugins, p); break;
      case PluginType::REGISTRY: push(s.registry_plugins, p); break;
      case PluginType::STATE_MODELING: push(s.state_modeling_plugins, p); break;
      case PluginType::TRANSPORT: push(s.transport_plugins, p); break;
      case PluginType::UI: push(s.ui_plugins, p); break;
      case PluginType::UTILITY: push(s.utility_plugins, p); break;
      default: break;
    }
  }
  return s;
}

int SortedPlugins::count(const std::string& f) const {
  if (f == "controller_plugins") return static_cast<int>(controller_plugins.size());
  if (f == "fusion_plugins") return static_cast<int>(fusion_plugins.size());
  if (f == "fusion_strategy_plugins") return static_cast<int>(fusion_strategy_plugins.size());
  if (f == "inertial_plugins") return static_cast<int>(inertial_plugins.size());
  if (f == "initialization_plugins") return static_cast<int>(initialization_plugins.size());
  if (f == "logging_plugins") return static_cast<int>(logging_plugins.size());
  if (f == "orchestration_plugins") return static_cast<int>(orchestration_plugins.size());
  if (f == "platform_integration_plugins") return static_cast<int>(platform_integration_plugins.size());
  if (f == "preprocessor_plugins") return static_cast<int>(preprocessor_plugins.size());
  if (f == "registry_plugins") return static_cast<int>(registry_plugins.size());
  if (f == "state_modeling_plugins") return static_cast<int>(state_modeling_plugins.size());
  if (f == "transport_plugins") return static_cast<int>(transport_plugins.size());
  if (f == "ui_plugins") return static_cast<int>(ui_plugins.size());
  if (f == "utility_plugins") return static_cast<int>(utility_plugins.size());
  return -1;
}

bool validate_plugins(const SortedPlugins& sorted, const LogFn& log,
                      const std::vector<std::pair<std::string, std::pair<int, int>>>& expected) {
  if (expected.empty()) {
    log(LoggingLevel::ERROR, "No plugins were given criteria to validate. At least one plugin must be validated");
    return false;
  }
  for (const auto& [name, range] : expected) {
    const int n = sorted.count(name);
    if (n < 0) {
      log(LoggingLevel::ERROR, "Unknown argument: " + name);
      return false;
    }
    const auto [lo, hi] = range;
    if (n < lo || n > hi) {
      if (lo == hi)
        log(LoggingLevel::ERROR, "Expected " + std::to_string(lo) + " " + name + " but received " + std::to_string(n));
      else
        log(LoggingLevel::ERROR, "Expected between " + std::to_string(lo) + " to " + std::to_string(hi) + " " + name +
                                     " but received " + std::to_string(n));
      return false;
    }
  }
  return true;
}

}  // namespace pntos::cobra
