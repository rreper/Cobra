#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/controller/StandardControllerPlugin.hpp>
#include <pntos/cobra/utils/logging.hpp>
#include <pntos/cobra/utils/plugins.hpp>

#include <atomic>
#include <csignal>

namespace pntos::cobra {

using api::LoggingLevel;
using api::PluginType;

namespace {
std::atomic<bool> g_sigint{false};
void on_sigint(int) { g_sigint = true; }
}  // namespace

void StandardControllerPlugin::install_sigint_handler() { std::signal(SIGINT, on_sigint); }

StandardControllerPlugin::StandardControllerPlugin(std::string identifier)
    : identifier_(std::move(identifier)), ctx_(std::make_shared<MediatorContext>()) {}

void StandardControllerPlugin::init_plugin(const std::optional<std::string>& plugin_resources_location,
                                           api::Mediator* mediator) {
  if (mediator) log(LoggingLevel::ERROR, "Controller plugin should not be passed a mediator.");
  resources_location_ = plugin_resources_location;
}

void StandardControllerPlugin::log(LoggingLevel level, const std::string& message) {
  if (logging_plugin_)
    logging_plugin_->log(PluginType::CONTROLLER, identifier_, level, message);
  else
    utils::print_message(level, api::to_string(PluginType::CONTROLLER), message);
}

void StandardControllerPlugin::shutdown_plugin() {
  log(LoggingLevel::INFO, "Shutting down all plugins...");
  for (const auto& p : plugins_) {
    if (!p) continue;
    const auto t = p->plugin_type();
    if (t == PluginType::REGISTRY || t == PluginType::LOGGING) continue;
    p->shutdown_plugin();
  }
  if (shutdown_token_ && ctx_->registry) {
    ctx_->registry->batch("controller/flags")->remove_notify(*shutdown_token_);
    shutdown_token_.reset();
  }
  ctx_->ui_interface.reset();
  if (registry_plugin_) registry_plugin_->shutdown_plugin();
  if (logging_plugin_) logging_plugin_->shutdown_plugin();
}

StandardControllerPlugin::Sorted StandardControllerPlugin::sort_and_validate(const api::PluginList& plugins) {
  SortedPlugins sorted = sort_plugins(plugins);
  auto logfn = [this](LoggingLevel l, const std::string& m) { log(l, m); };
  if (!validate_plugins(sorted, logfn,
                        {{"registry_plugins", {1, 1}},
                         {"logging_plugins", {1, 1}},
                         {"orchestration_plugins", {1, 1}},
                         {"transport_plugins", {1, 1000}},
                         {"fusion_plugins", {1, 1000}}}))
    throw std::runtime_error("Not enough plugins to run pntOS.");
  Sorted out;
  out.registry = sorted.registry_plugins[0];
  out.logging = sorted.logging_plugins[0];
  out.transports = sorted.transport_plugins;
  out.orchestration = sorted.orchestration_plugins[0];
  out.uis = sorted.ui_plugins;
  auto add = [&](const auto& v) {
    for (const auto& p : v) out.for_orchestration.push_back(p);
  };
  add(sorted.fusion_plugins);
  add(sorted.fusion_strategy_plugins);
  add(sorted.inertial_plugins);
  add(sorted.initialization_plugins);
  add(sorted.state_modeling_plugins);
  add(sorted.preprocessor_plugins);
  return out;
}

void StandardControllerPlugin::take_control(const api::PluginList& plugins,
                                            const api::ResourceLocations& plugin_resources_locations,
                                            const std::optional<std::string>& initial_config) {
  if (!start(plugins, plugin_resources_locations, initial_config)) return;
  main_loop();
  stop();
}

void StandardControllerPlugin::stop() {
  if (!running_) return;
  running_ = false;
  ctx_->exit_event.set(ctx_->exit_event.exit_code());
  shutdown_plugin();
}

bool StandardControllerPlugin::start(const api::PluginList& plugins, const api::ResourceLocations& plugin_resources_locations,
                                     const std::optional<std::string>& initial_config) {
  plugins_ = plugins;
  Sorted sorted = sort_and_validate(plugins);
  registry_plugin_ = sorted.registry;
  transport_plugins_ = sorted.transports;
  ui_plugins_ = sorted.uis;

  ctx_->controller_plugin = this;
  ctx_->exit_event.clear();

  // One mediator per plugin.
  mediators_.clear();
  for (const auto& p : plugins_) mediators_.push_back(std::make_unique<StandardMediator>(ctx_, p->identifier(), p->plugin_type()));

  api::ResourceLocations locations = plugin_resources_locations;
  if (locations && locations->size() != plugins_.size()) {
    log(LoggingLevel::ERROR, "Length of plugin_resources_location (" + std::to_string(locations->size()) +
                                 ") does not equal the number of plugins (" + std::to_string(plugins_.size()) +
                                 "). Passing None to all plugins instead.");
    locations.reset();
  }
  auto location = [&](std::size_t i) -> std::optional<std::string> {
    return locations ? (*locations)[i] : std::nullopt;
  };
  auto index_of = [&](const api::CommonPlugin* p) {
    for (std::size_t i = 0; i < plugins_.size(); ++i)
      if (plugins_[i].get() == p) return i;
    return std::size_t(0);
  };

  // Registry first, then logging, then everything else.
  const std::size_t reg_i = index_of(registry_plugin_.get());
  registry_plugin_->init_plugin(location(reg_i), mediators_[reg_i].get());
  ctx_->registry = registry_plugin_->new_registry(initial_config);
  ctx_->ui_interface = std::make_unique<UiMediatorInterface>(ctx_->registry);

  const std::size_t log_i = index_of(sorted.logging.get());
  sorted.logging->init_plugin(location(log_i), mediators_[log_i].get());
  logging_plugin_ = sorted.logging;
  ctx_->logging_plugin = sorted.logging;

  for (std::size_t i = 0; i < plugins_.size(); ++i) {
    if (i == reg_i || i == log_i) continue;
    plugins_[i]->init_plugin(location(i), mediators_[i].get());
  }

  ctx_->transport_plugins = transport_plugins_;
  ctx_->orchestration_plugin = sorted.orchestration;
  sorted.orchestration->init_orchestration_plugin(sorted.for_orchestration, *ctx_->stream_config);

  // Controller-specific config.
  StandardMediator temp(ctx_, identifier_, PluginType::CONTROLLER);
  auto config = ControllerConfig::from_registry(temp);
  if (!config) {
    log(LoggingLevel::ERROR,
        "Could not extract ControllerConfig from group \"controller\". Cannot initialize controller plugin.");
    return false;
  }
  shutdown_token_ = ctx_->registry->batch("controller/flags")
                        ->request_notify("ready_to_shutdown", [this](const std::string&, const std::vector<std::string>& keys,
                                                                     api::KeyValueStore& kv) {
                          if (std::find(keys.begin(), keys.end(), "ready_to_shutdown") == keys.end()) return;
                          auto ready = kv.get_value<bool>("ready_to_shutdown");
                          if (!ready || !*ready) return;
                          if (auto_shutdown_)
                            ctx_->exit_event.set(ExitCode::SUCCESS);
                          else
                            log(LoggingLevel::INFO, "Press Ctrl + C at any time to shut down pntOS...");
                        });
  ctx_->buffer_time_nsec = static_cast<std::int64_t>(config->buffer_length_sec * 1e9);
  if (config->publish_interval) ctx_->publish_interval_nsec = static_cast<std::int64_t>(*config->publish_interval * 1e9);
  auto_shutdown_ = config->auto_shutdown;

  log(LoggingLevel::INFO, "Press Ctrl + C at any time to shut down pntOS...");
  running_ = true;
  for (const auto& t : transport_plugins_) t->start_listening();
  return true;
}

void StandardControllerPlugin::main_loop() {

  std::vector<std::shared_ptr<api::UiPlugin>> needing_main;
  for (const auto& ui : ui_plugins_)
    if (ui->requires_main_thread()) needing_main.push_back(ui);
  if (needing_main.size() > 1)
    log(LoggingLevel::ERROR, "Only 1 UiPlugin can require the main thread, but found " +
                                 std::to_string(needing_main.size()) + " needing the main thread. Cannot run pntOS.");
  if (!needing_main.empty()) {
    needing_main[0]->run_main_thread();
    return;
  }
  while (!ctx_->exit_event.wait(std::chrono::milliseconds(100))) {
    if (g_sigint.exchange(false)) {
      log(LoggingLevel::INFO, "Keyboard Interrupt Detected.");
      ctx_->exit_event.set(ExitCode::SUCCESS);
    }
  }
}

}  // namespace pntos::cobra
