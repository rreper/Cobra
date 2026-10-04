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
  bool progress = true;                      ///< print a percentage on stderr
};

/// Parses `[output.log] [input.log] [--no-joseph] [--legacy-q|--corrected-q] [--dump-config f]
/// [--dump-registry f] [--quiet]`; positional arguments after a config file (see cobra_run).
RunOptions parse_run_options(int argc, char** argv, int first_positional = 1);

/// The config with the overrides applied (through the JSON form, so every config class is covered).
AppConfig apply_overrides(const AppConfig& config, const RunOptions& options);

/// Instantiates the plugins of `config.app` around the registry built from `config.configs`.
/// Throws std::runtime_error on an unknown plugin name.
api::PluginList build_plugins(const AppConfig& config, const std::function<void(std::uint64_t, std::uint64_t)>& progress = {});

/// Builds, runs the controller to completion and returns the process exit code (0 = success).
int run_app(const AppConfig& config, const RunOptions& options);

/// The AppSpec every standard app uses (lcm_log, manual-heading alignment, standard everything).
AppSpec standard_app_spec(const std::string& name);

}  // namespace pntos::cobra::app
