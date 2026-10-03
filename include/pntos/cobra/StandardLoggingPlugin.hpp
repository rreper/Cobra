// Console logging plugin (port of pntos.cobra.StandardLoggingPlugin).
#pragma once

#include <pntos/api/registry.hpp>

#include <string>

namespace pntos::cobra {

/// Prints log lines to the console; a message is printed when its level <= global_log_level
/// (ERROR < WARN < INFO < DEBUG).
class StandardLoggingPlugin final : public api::LoggingPlugin {
 public:
  explicit StandardLoggingPlugin(std::string identifier, bool colorize = true,
                                 api::LoggingLevel global_log_level = api::LoggingLevel::INFO,
                                 std::string date_time_format = "%d/%m/%Y %H:%M:%S");

  void init_plugin(const std::optional<std::string>& plugin_resources_location, api::Mediator* mediator) override;
  void shutdown_plugin() override;
  const std::string& identifier() const override { return identifier_; }

  void log(api::PluginType source_plugin_type, const std::string& source_plugin_identifier, api::LoggingLevel level,
           const std::string& message) override;

  api::LoggingLevel global_log_level() const { return global_log_level_; }
  void set_global_log_level(api::LoggingLevel level) { global_log_level_ = level; }

 private:
  std::string identifier_;
  bool colorize_;
  api::LoggingLevel global_log_level_;
  std::string date_time_format_;
};

}  // namespace pntos::cobra
