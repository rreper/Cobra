// JSON config files: the Python app's config list as data.
//
// A file is one object with an "app" section (which plugins to run) and a "configs" list. Every entry
// in "configs" (and every nested config) is an object whose "type" is the Python dataclass name
// (ImuConfig, StandardOrchestrationConfig, PinsonWithNedFogmPositionMPConfig, ...) and whose fields
// use the Python field names. Nested configs with an implied type (imu_model, fogm_model,
// inertial_config, feedback_config, stream_config, pinson_sb_config) may omit "type". An ImuConfig
// may be given as {"preset": "<name>", "group": "..."} (presets/Presets.hpp), optionally with fields
// that override the preset. Every config also serialises back to JSON, so a compiled app can write
// the file that reproduces it (`--dump-config`).
#pragma once

#include <pntos/cobra/config/configs.hpp>

#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace pntos::cobra {

/// Which plugins an app runs (the plugin list of a Python app, as data).
struct AppSpec {
  std::string name = "cobra";
  std::string transport = "lcm_log";                       ///< lcm_log
  std::string initialization = "manual_heading_align";     ///< manual_heading_align | static_align | manual | pva_message
  std::string state_modeling = "standard";                 ///< standard | tutorial
  std::string orchestration = "standard";                  ///< standard | tutorial_pos | tutorial_pos_vel
  std::vector<std::string> preprocessors{"standard"};      ///< standard, advanced
  bool diagnostic_log = false;                             ///< DiagnosticLogPlugin (HDF5 of the diagnostics group)
  std::string diagnostic_log_file;                         ///< empty: <output log>.hdf5
  bool ui_log_plotting = false;                            ///< UiLogPlottingPlugin at shutdown
  std::string logging_level = "INFO";                      ///< ERROR | WARN | INFO | DEBUG
  bool joseph_form = true;                                 ///< EKF update form
  bool legacy_q_rotation = false;                          ///< Pinson-Q handling for plugins without a PinsonStateBlockConfig
  std::vector<std::string> extra_plugins;                  ///< names registered with app::register_extra_plugin
};

struct AppConfig {
  AppSpec app;
  std::vector<std::shared_ptr<const BaseConfig>> configs;
};

namespace jsoncfg {

using json = nlohmann::json;

/// Throws std::runtime_error with the offending field on any error.
std::shared_ptr<BaseConfig> config_from_json(const json& j, const std::string& implied_type = "");
json config_to_json(const BaseConfig& c);
/// The Python class name of a config object (what config_to_json writes as "type").
std::string type_name(const BaseConfig& c);

AppSpec app_spec_from_json(const json& j);
json app_spec_to_json(const AppSpec& s);

AppConfig app_config_from_json(const json& j);
json app_config_to_json(const AppConfig& c);
AppConfig load_app_config(const std::string& path);
void save_app_config(const AppConfig& c, const std::string& path);

/// Every group / key / value of a registry as JSON (Message values as their type name), for
/// comparing a compiled app's registry with the one a config file produces.
json dump_registry(api::Registry& registry);

std::string message_type_name(api::AspnMessageType t);
api::AspnMessageType message_type_from_name(const std::string& name);

}  // namespace jsoncfg
}  // namespace pntos::cobra
