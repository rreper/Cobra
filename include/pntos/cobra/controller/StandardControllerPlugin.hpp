// Port of pntos.cobra.standard_plugins.controller.StandardControllerPlugin (single-threaded controller).
#pragma once

#include <pntos/cobra/controller/StandardMediator.hpp>

namespace pntos::cobra {

/// Validates the plugin set, creates one StandardMediator per plugin, initialises registry and
/// logging first, then every other plugin, hands the orchestration its plugins, reads
/// ControllerConfig, starts the transports and blocks until Ctrl+C / the exit event.
///
/// Expected plugins: 1 registry, 1 logging, 1 orchestration, >=1 transport, >=1 fusion; UI plugins
/// optional (at most one may require the main thread).
class StandardControllerPlugin final : public api::ControllerPlugin {
 public:
  explicit StandardControllerPlugin(std::string identifier);

  void init_plugin(const std::optional<std::string>& plugin_resources_location, api::Mediator* mediator) override;
  void shutdown_plugin() override;
  const std::string& identifier() const override { return identifier_; }

  /// Throws std::runtime_error if the plugin set is invalid. Returns after shutdown; see exit_code().
  void take_control(const api::PluginList& plugins, const api::ResourceLocations& plugin_resources_locations = std::nullopt,
                    const std::optional<std::string>& initial_config = std::nullopt) override;

  /// Library mode (C++ addition): everything take_control does before it blocks — validate, create the
  /// mediators, initialise the plugins, hand the orchestration its plugins, read ControllerConfig and start
  /// the transports — then return. Pair with stop(). Returns false (after logging) if the config is unusable.
  bool start(const api::PluginList& plugins, const api::ResourceLocations& plugin_resources_locations = std::nullopt,
             const std::optional<std::string>& initial_config = std::nullopt);
  /// Stops the transports and shuts every plugin down. Idempotent.
  void stop();
  bool running() const { return running_; }

  /// ExitCode::ERROR if any plugin logged an ERROR through its mediator (Python: sys.exit(1)).
  ExitCode exit_code() const { return ctx_->exit_event.exit_code(); }
  /// Request shutdown from another thread (what Ctrl+C does).
  void request_shutdown(ExitCode code = ExitCode::SUCCESS) { ctx_->exit_event.set(code); }
  const std::shared_ptr<MediatorContext>& context() const { return ctx_; }

  /// Installs a SIGINT handler that stops the running controller. Call once from main().
  static void install_sigint_handler();

 private:
  struct Sorted {
    std::shared_ptr<api::RegistryPlugin> registry;
    std::shared_ptr<api::LoggingPlugin> logging;
    std::vector<std::shared_ptr<api::TransportPlugin>> transports;
    std::shared_ptr<api::OrchestrationPlugin> orchestration;
    std::vector<std::shared_ptr<api::UiPlugin>> uis;
    api::PluginList for_orchestration;
  };
  Sorted sort_and_validate(const api::PluginList& plugins);
  void log(api::LoggingLevel level, const std::string& message);
  void main_loop();

  std::string identifier_;
  std::optional<std::string> resources_location_;
  std::shared_ptr<MediatorContext> ctx_;
  api::PluginList plugins_;
  std::shared_ptr<api::LoggingPlugin> logging_plugin_;
  std::shared_ptr<api::RegistryPlugin> registry_plugin_;
  std::vector<std::shared_ptr<api::TransportPlugin>> transport_plugins_;
  std::vector<std::shared_ptr<api::UiPlugin>> ui_plugins_;
  std::vector<std::unique_ptr<StandardMediator>> mediators_;
  bool auto_shutdown_ = true;
  bool running_ = false;
  std::optional<api::NotifyToken> shutdown_token_;
};

}  // namespace pntos::cobra
