#include <pntos/cobra/app/AppBuilder.hpp>

#include <pntos/cobra/app/Filter.hpp>
#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/transport/LcmLog.hpp>

#include <pntos/cobra/EkfFusionStrategyPlugin.hpp>
#include <pntos/cobra/StandardLoggingPlugin.hpp>
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/controller/StandardControllerPlugin.hpp>
#include <pntos/cobra/diagnostics/DiagnosticLogPlugin.hpp>
#include <pntos/cobra/extras/AdvancedPreprocessorPlugin.hpp>
#include <pntos/cobra/fusion/StandardFusionPlugin.hpp>
#include <pntos/cobra/inertial/StandardInertialPlugin.hpp>
#include <pntos/cobra/initialization/InitializationPlugins.hpp>
#include <pntos/cobra/orchestration/StandardOrchestrationPlugin.hpp>
#include <pntos/cobra/preprocessing/StandardPreprocessorPlugin.hpp>
#include <pntos/cobra/presets/Presets.hpp>
#include <pntos/cobra/state_modeling/StandardStateModelingPlugin.hpp>
#include <pntos/cobra/transport/CsvTransportPlugin.hpp>
#include <pntos/cobra/transport/LcmLogTransportPlugin.hpp>
#include <pntos/cobra/transport/LcmUdpTransportPlugin.hpp>
#include <pntos/cobra/tutorial/TutorialPlugins.hpp>

#include <algorithm>
#include <set>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace pntos::cobra::app {

using jsoncfg::json;

RunOptions parse_run_options(int argc, char** argv, int first_positional) {
  RunOptions o;
  std::vector<std::string> pos;
  for (int i = first_positional; i < argc; ++i) {
    const std::string s = argv[i];
    auto value = [&](const char* flag) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string(flag) + " needs a value");
      return argv[++i];
    };
    if (s == "--no-joseph") o.joseph_form = false;
    else if (s == "--joseph") o.joseph_form = true;
    else if (s == "--legacy-q") o.legacy_q_rotation = true;
    else if (s == "--corrected-q") o.legacy_q_rotation = false;
    else if (s == "--dump-config") o.dump_config = value("--dump-config");
    else if (s == "--dump-registry") o.dump_registry = value("--dump-registry");
    else if (s == "--quiet") o.progress = false;
    else if (s == "--no-record-input") o.record_input = false;
    else if (s == "--via-push") o.via_push = true;
    else if (!s.empty() && s[0] == '-') throw std::runtime_error("unknown option " + s);
    else pos.push_back(s);
  }
  if (!pos.empty()) o.output_log = pos[0];
  if (pos.size() > 1) o.input_log = pos[1];
  return o;
}

namespace {
std::string hdf5_path_for(const std::string& output_log) {
  const auto dot = output_log.rfind('.');
  const auto slash = output_log.rfind('/');
  const bool has_ext = dot != std::string::npos && (slash == std::string::npos || dot > slash);
  return (has_ext ? output_log.substr(0, dot) : output_log) + ".hdf5";
}

api::LoggingLevel level_from(const std::string& s) {
  if (s == "ERROR") return api::LoggingLevel::ERROR;
  if (s == "WARN") return api::LoggingLevel::WARN;
  if (s == "INFO") return api::LoggingLevel::INFO;
  if (s == "DEBUG") return api::LoggingLevel::DEBUG;
  throw std::runtime_error("unknown logging_level \"" + s + "\"");
}

template <class F>
void for_each_config(json& root, F&& f) {
  for (auto& c : root["configs"]) f(c);
}

// Every ImuConfig given as a preset follows the Pinson-Q mode: the Cobra-tuned VN-100 model exists in
// a legacy and a corrected variant (presets::imu_preset_for_mode).
void swap_imu_presets(json& node, bool legacy) {
  if (node.is_object()) {
    if (node.contains("preset") && node.value("type", "ImuConfig") == "ImuConfig")
      node["preset"] = presets::imu_preset_for_mode(node["preset"].get<std::string>(), legacy);
    for (auto& [k, v] : node.items()) swap_imu_presets(v, legacy);
  } else if (node.is_array()) {
    for (auto& v : node) swap_imu_presets(v, legacy);
  }
}
}  // namespace

AppConfig apply_overrides(const AppConfig& config, const RunOptions& options) {
  json root = jsoncfg::app_config_to_json(config);
  std::string output_log;
  for_each_config(root, [&](json& c) {
    const std::string type = c.value("type", "");
    if (type == "LcmLogTransportConfig") {
      if (options.input_log) c["input_file"] = *options.input_log;
      if (options.output_log) c["output_file"] = *options.output_log;
      if (options.record_input) c["record_input_channels"] = *options.record_input;
      output_log = c.value("output_file", "");
    }
    if (type == "LcmTransportConfig" && options.output_log) {
      c["output_file"] = *options.output_log;
      output_log = *options.output_log;
    }
    if (type == "CsvTransportConfig") {
      if (options.output_log) c["output_file"] = *options.output_log;
      if (options.input_log) c["imu_file"] = *options.input_log;  // single-file CSV input
      output_log = c.value("output_file", "");
    }
  });
  for_each_config(root, [&](json& c) {
    const std::string type = c.value("type", "");
    if (type == "StandardOrchestrationConfig" && options.legacy_q_rotation)
      c["pinson_sb_config"]["legacy_q_rotation"] = *options.legacy_q_rotation;
    if (type == "UiLogPlottingConfig" && options.output_log) c["logfile"] = *options.output_log;
  });
  if (options.legacy_q_rotation) {
    root["app"]["legacy_q_rotation"] = *options.legacy_q_rotation;
    swap_imu_presets(root["configs"], *options.legacy_q_rotation);
  }
  if (options.joseph_form) root["app"]["joseph_form"] = *options.joseph_form;
  return jsoncfg::app_config_from_json(root);
}

api::PluginList build_plugins(const AppConfig& config, const std::function<void(std::uint64_t, std::uint64_t)>& progress) {
  const AppSpec& s = config.app;
  api::PluginList plugins;

  if (s.transport == "lcm_log") {
    auto t = std::make_shared<LcmLogTransportPlugin>("Cobra LCM Log Transport Plugin");
    if (progress) t->set_progress_callback(progress);
    plugins.push_back(t);
  } else if (s.transport == "push") {
    plugins.push_back(std::make_shared<PushTransportPlugin>("Cobra Push Transport Plugin"));
  } else if (s.transport == "lcm_udp") {
    plugins.push_back(std::make_shared<LcmUdpTransportPlugin>("Cobra LCM Transport Plugin"));
  } else if (s.transport == "csv") {
    plugins.push_back(std::make_shared<CsvTransportPlugin>("Cobra CSV Transport Plugin"));
  } else {
    throw std::runtime_error("unknown transport \"" + s.transport + "\" (lcm_log | lcm_udp | csv | push)");
  }

  plugins.push_back(std::make_shared<EkfFusionStrategyPlugin>("Cobra EKF Fusion Strategy Plugin", s.joseph_form));
  plugins.push_back(std::make_shared<StandardFusionPlugin>("Cobra Standard Fusion Plugin"));

  if (s.state_modeling == "standard")
    plugins.push_back(std::make_shared<StandardStateModelingPlugin>("Cobra Standard State Modeling Plugin"));
  else if (s.state_modeling == "tutorial")
    plugins.push_back(std::make_shared<TutorialPosInsStateModelingPlugin>("Cobra Tutorial State Modeling Plugin", s.legacy_q_rotation));
  else
    throw std::runtime_error("unknown state_modeling \"" + s.state_modeling + "\" (standard | tutorial)");

  plugins.push_back(std::make_shared<StandardInertialPlugin>("Cobra Standard Inertial Plugin"));

  if (s.initialization == "manual_heading_align")
    plugins.push_back(std::make_shared<ManualHeadingAlignInitializationPlugin>("Cobra Manual Heading Static Align Initialization Plugin"));
  else if (s.initialization == "static_align")
    plugins.push_back(std::make_shared<StaticAlignInitializationPlugin>("Cobra Static Align Initialization Plugin"));
  else if (s.initialization == "manual")
    plugins.push_back(std::make_shared<TutorialInitializationPlugin>("Cobra Manual Initialization Plugin"));
  else if (s.initialization == "pva_message")
    plugins.push_back(std::make_shared<PvaMessageInitializationPlugin>("Cobra PVA Message Initialization Plugin"));
  else
    throw std::runtime_error("unknown initialization \"" + s.initialization +
                             "\" (manual_heading_align | static_align | manual | pva_message)");

  plugins.push_back(std::make_shared<StandardLoggingPlugin>("Cobra Standard Logging Plugin", true, level_from(s.logging_level)));
  plugins.push_back(std::make_shared<StandardRegistryPlugin>("Cobra Standard Registry Plugin", config.configs));

  for (const auto& p : s.preprocessors) {
    if (p == "standard") plugins.push_back(std::make_shared<StandardPreprocessorPlugin>("Cobra Standard Preprocessor Plugin"));
    else if (p == "advanced") plugins.push_back(std::make_shared<AdvancedPreprocessorPlugin>("Cobra Extras Preprocessor Plugin"));
    else throw std::runtime_error("unknown preprocessor plugin \"" + p + "\" (standard | advanced)");
  }
  if (s.ui_log_plotting) plugins.push_back(std::make_shared<UiLogPlottingPlugin>("Cobra UI Logfile Plotting Plugin"));

  if (s.orchestration == "standard")
    plugins.push_back(std::make_shared<StandardOrchestrationPlugin>("Cobra Standard Orchestration Plugin"));
  else if (s.orchestration == "tutorial_pos")
    plugins.push_back(std::make_shared<TutorialPosOrchestrationPlugin>("Cobra Tutorial Orchestration Plugin"));
  else if (s.orchestration == "tutorial_pos_vel")
    plugins.push_back(std::make_shared<TutorialPosVelOrchestrationPlugin>("Cobra Tutorial Orchestration Plugin"));
  else
    throw std::runtime_error("unknown orchestration \"" + s.orchestration + "\" (standard | tutorial_pos | tutorial_pos_vel)");

  if (s.diagnostic_log) {
    std::string file = s.diagnostic_log_file;
    if (file.empty()) {  // next to the output log, like the Python's OUTPUT.hdf5 next to its output
      for (const auto& c : config.configs)
        if (auto* t = dynamic_cast<const LcmLogTransportConfig*>(c.get()); t && t->output_file) file = hdf5_path_for(*t->output_file);
      if (file.empty()) file = DiagnosticLogPlugin::kDefaultOutputFile;
    }
    plugins.push_back(std::make_shared<DiagnosticLogPlugin>("Cobra HDF5 Diagnostic Log Plugin", file));
  }
  return plugins;
}

int run_app(const AppConfig& input, const RunOptions& options) {
  AppConfig config = apply_overrides(input, options);
  if (options.dump_config) {
    jsoncfg::save_app_config(config, *options.dump_config);
    std::cerr << "wrote " << *options.dump_config << "\n";
    return 0;
  }
  if (options.via_push) return run_app_via_push(config, options);
  const std::string name = config.app.name;
  std::function<void(std::uint64_t, std::uint64_t)> progress;
  if (options.progress)
    progress = [name](std::uint64_t done, std::uint64_t total) {
      std::cerr << "\r[" << name << "] " << (100 * done / std::max<std::uint64_t>(total, 1)) << "%" << std::flush;
    };
  api::PluginList plugins = build_plugins(config, progress);

  std::shared_ptr<StandardRegistryPlugin> registry;
  for (const auto& p : plugins)
    if (auto r = std::dynamic_pointer_cast<StandardRegistryPlugin>(p)) registry = r;

  StandardControllerPlugin::install_sigint_handler();
  StandardControllerPlugin controller("Cobra Standard Controller Plugin");
  controller.init_plugin(std::nullopt, nullptr);
  if (options.dump_registry && registry) {
    // The registry is populated in init_plugin; a second instance of the same configs gives the same
    // contents, so dump from a scratch registry before the run starts.
    class NullMediator final : public api::Mediator {
     public:
      std::vector<std::string> filter_description_list() const override { return {}; }
      std::optional<std::vector<std::optional<api::Message>>> request_solutions(const std::vector<api::Timestamp>&,
                                                                                 const std::optional<std::string>&) override {
        return std::nullopt;
      }
      void process_pntos_message(const api::Message&) override {}
      void broadcast_aspn_message(const api::Message&, const std::optional<std::string>&, const std::optional<std::string>&) override {}
      void log_message(api::LoggingLevel, const std::string&) override {}
      api::Registry& registry() override { throw std::logic_error("no registry"); }
    } med;
    StandardRegistryPlugin scratch("scratch", config.configs);
    scratch.init_plugin(std::nullopt, &med);
    auto reg = scratch.new_registry();
    std::ofstream out(*options.dump_registry);
    out << jsoncfg::dump_registry(*reg).dump(2) << '\n';
  }
  controller.take_control(plugins);
  if (options.progress) std::cerr << "\n";
  return controller.exit_code() == ExitCode::SUCCESS ? 0 : 1;
}

AppSpec standard_app_spec(const std::string& name) {
  AppSpec s;
  s.name = name;
  return s;
}

}  // namespace pntos::cobra::app

namespace pntos::cobra::app {

int run_app_via_push(const AppConfig& input, const RunOptions& options) {
  AppConfig config = apply_overrides(input, options);
  const LcmLogTransportConfig* tc = nullptr;
  for (const auto& c : config.configs)
    if (auto* t = dynamic_cast<const LcmLogTransportConfig*>(c.get())) tc = t;
  if (!tc || !tc->input_file) throw std::runtime_error("run_app_via_push needs an LcmLogTransportConfig with an input_file");
  lcm::LcmLogReader reader(*tc->input_file);
  std::unique_ptr<lcm::LcmLogWriter> writer;
  if (tc->output_file) writer = std::make_unique<lcm::LcmLogWriter>(*tc->output_file);
  std::optional<std::set<std::string>> channels;
  if (tc->channels_to_process) channels = std::set<std::string>(tc->channels_to_process->begin(), tc->channels_to_process->end());

  RunOptions quiet = options;
  Filter filter(config, quiet);  // the Filter replaces the transport with the push transport
  if (writer)
    filter.set_solution_callback([&](const api::Message& sol) {
      if (auto bytes = lcm::encode(*sol.wrapped_message)) writer->write(lcm::now_us(), sol.source_identifier, *bytes);
    });
  const std::string name = config.app.name;
  const std::uint64_t total = reader.size();
  std::uint64_t next_report = total / 100;
  std::size_t pushed = 0;
  while (auto ev = reader.next()) {
    if (writer && tc->record_input_channels) writer->write(lcm::now_us(), ev->channel, ev->data);
    if (channels && !channels->count(ev->channel)) continue;
    std::shared_ptr<api::AspnBase> msg;
    try {
      msg = lcm::decode(ev->data);
    } catch (const std::exception&) {
      msg = nullptr;
    }
    if (!msg) continue;
    filter.push(api::Message(msg, ev->channel));
    ++pushed;
    if (options.progress && reader.tell() >= next_report) {
      std::cerr << "\r[" << name << " via push] " << (100 * reader.tell() / std::max<std::uint64_t>(total, 1)) << "%" << std::flush;
      next_report += std::max<std::uint64_t>(total / 100, 1);
    }
  }
  const int code = filter.stop();
  if (writer) writer->close();
  if (options.progress) std::cerr << "\n";
  return code;
}

}  // namespace pntos::cobra::app
