#include <pntos/cobra/StandardLoggingPlugin.hpp>
#include <pntos/cobra/utils/logging.hpp>

namespace pntos::cobra {

StandardLoggingPlugin::StandardLoggingPlugin(std::string identifier, bool colorize, api::LoggingLevel global_log_level,
                                             std::string date_time_format)
    : identifier_(std::move(identifier)),
      colorize_(colorize),
      global_log_level_(global_log_level),
      date_time_format_(std::move(date_time_format)) {}

void StandardLoggingPlugin::init_plugin(const std::optional<std::string>&, api::Mediator*) {
  log(api::PluginType::LOGGING, identifier_, api::LoggingLevel::INFO,
      std::string("using hard-coded global logging level ") + api::to_string(global_log_level_));
}

void StandardLoggingPlugin::shutdown_plugin() {
  log(api::PluginType::LOGGING, identifier_, api::LoggingLevel::INFO, " Logging plugin shut down correctly.");
}

void StandardLoggingPlugin::log(api::PluginType source_plugin_type, const std::string&, api::LoggingLevel level,
                                const std::string& message) {
  if (static_cast<int>(global_log_level_) >= static_cast<int>(level)) {
    utils::print_message(level, api::to_string(source_plugin_type), message, colorize_, date_time_format_);
  }
}

}  // namespace pntos::cobra
