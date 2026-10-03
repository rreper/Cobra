// pntOS C++ API — ControllerPlugin and PlatformIntegrationPlugin.
#pragma once

#include <pntos/api/common.hpp>

namespace pntos::api {

using PluginList = std::vector<std::shared_ptr<CommonPlugin>>;
using ResourceLocations = std::optional<std::vector<std::optional<std::string>>>;

/// The conceptual "main": owns the concurrency model, creates the Mediator, wires plugins.
///
/// Rules: must call init_plugin() (with a Mediator) on every plugin it uses before anything else;
/// must not load new plugins; shutdown_plugin() on the controller shuts down all plugins.
class ControllerPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::CONTROLLER; }

  /// Take over from the app. `plugin_resources_locations`, if present, has one entry per plugin.
  /// `initial_config` is an implementation-specific config source (whole config, path, URI).
  virtual void take_control(const PluginList& plugins,
                            const ResourceLocations& plugin_resources_locations = std::nullopt,
                            const std::optional<std::string>& initial_config = std::nullopt) = 0;
};

/// Platform-specific control (unstable API). Receives a subset of the plugins after the
/// controller has initialised them; must not create mediators or route messages.
class PlatformIntegrationPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::PLATFORM_INTEGRATION; }

  virtual void take_control(const PluginList& plugins,
                            const ResourceLocations& plugin_resources_locations = std::nullopt,
                            const std::optional<std::string>& initial_config = std::nullopt) = 0;
};

}  // namespace pntos::api
