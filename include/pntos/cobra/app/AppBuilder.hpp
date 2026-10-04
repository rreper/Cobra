// Builds and runs the plugin set an AppSpec describes. The compiled apps and the generic runner
// (`cobra_run config.json`) share this code path, so a dumped config file reproduces its app exactly.
#pragma once

#include <pntos/api/api.hpp>
#include <pntos/cobra/config/JsonConfig.hpp>

#include <functional>
#include <optional>
#include <string>

namespace pntos::cobra::app {

/// Command-line overrides applied on top of a config: input/output logs, the Pinson-Q mode and the
/// EKF update form.
struct RunOptions {
  std::optional<std::string> output_log;
  std::optional<std::string> input_log;
  std::optional<bool> legacy_q_rotation;
  std::optional<bool> joseph_form;
  std::optional<std::string> dump_config;    ///< write the effective config here and exit
  std::optional<std::string> dump_registry;  ///< write the registry contents here after init
  std::optional<bool> record_input;          ///< --no-record-input: output log holds only broadcast messages
  bool via_push = false;                     ///< --via-push: replay the LCM log through cobra::Filter instead of the log transport
  bool progress = true;                      ///< print a percentage on stderr
};

/// Parses `[output.log] [input.log] [--no-joseph] [--legacy-q|--corrected-q] [--dump-config f]
/// [--dump-registry f] [--no-record-input] [--quiet]`; positional arguments after a config file (see cobra_run).
RunOptions parse_run_options(int argc, char** argv, int first_positional = 1);

/// The config with the overrides applied (through the JSON form, so every config class is covered).
AppConfig apply_overrides(const AppConfig& config, const RunOptions& options);

/// Instantiates the plugins of `config.app` around the registry built from `config.configs`.
/// Throws std::runtime_error on an unknown plugin name.
api::PluginList build_plugins(const AppConfig& config, const std::function<void(std::uint64_t, std::uint64_t)>& progress = {});

/// Builds, runs the controller to completion and returns the process exit code (0 = success).
int run_app(const AppConfig& config, const RunOptions& options);

/// The LCM log runner on top of the push API: reads the log named by the LcmLogTransportConfig, pushes
/// every selected event through a cobra::Filter and writes the published solutions (and, if configured,
/// the input events) to the output log. Produces the same solutions as the log transport.
int run_app_via_push(const AppConfig& config, const RunOptions& options);

/// The AppSpec every standard app uses (lcm_log, manual-heading alignment, standard everything).
AppSpec standard_app_spec(const std::string& name);

/// Registration of plugins that live outside this library (C++ addition). A registered orchestration
/// name becomes selectable as `AppSpec.orchestration`; registered extra plugins are appended to the plugin
/// list when their name appears in `AppSpec.extra_plugins`. Factories receive the AppConfig being built.
using OrchestrationFactory = std::function<std::shared_ptr<api::OrchestrationPlugin>(const AppConfig&)>;
using ExtraPluginFactory = std::function<std::shared_ptr<api::CommonPlugin>(const AppConfig&)>;
void register_orchestration(const std::string& name, OrchestrationFactory factory);
void register_extra_plugin(const std::string& name, ExtraPluginFactory factory);
std::vector<std::string> registered_orchestrations();
std::vector<std::string> registered_extra_plugins();

}  // namespace pntos::cobra::app
