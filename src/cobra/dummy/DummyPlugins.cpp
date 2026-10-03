#include <pntos/cobra/dummy/DummyPlugins.hpp>
#include <pntos/cobra/utils/logging.hpp>

#include <aspn23/eigen/MeasurementPosition.hpp>

#include <cmath>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Message;
using api::PluginType;

// ----------------------------------------------------------------------------- DummyMediator

namespace {
template <class T>
std::shared_ptr<T> first_of(const api::PluginList& plugins, PluginType type) {
  for (const auto& p : plugins)
    if (p && p->plugin_type() == type)
      if (auto t = std::dynamic_pointer_cast<T>(p)) return t;
  return nullptr;
}
}  // namespace

std::vector<std::string> DummyMediator::filter_description_list() const {
  if (auto o = first_of<api::OrchestrationPlugin>(plugins_, PluginType::ORCHESTRATION)) return o->filter_description_list();
  return {};
}

std::optional<std::vector<std::optional<Message>>> DummyMediator::request_solutions(
    const std::vector<api::Timestamp>& times, const std::optional<std::string>& filter_description) {
  if (auto o = first_of<api::OrchestrationPlugin>(plugins_, PluginType::ORCHESTRATION))
    return o->request_solutions(times, filter_description);
  return std::nullopt;
}

void DummyMediator::process_pntos_message(const Message& message) {
  for (const auto& p : plugins_) {
    if (!p || p->plugin_type() != PluginType::ORCHESTRATION) continue;
    auto o = std::dynamic_pointer_cast<api::OrchestrationPlugin>(p);
    if (!o) continue;
    o->process_pntos_message(message, false);
    auto solutions = o->request_solutions({api::Timestamp{0}});
    if (solutions && !solutions->empty() && (*solutions)[0])
      broadcast_aspn_message(*(*solutions)[0], std::nullopt, message.source_identifier + "_echo");
  }
}

void DummyMediator::broadcast_aspn_message(const Message& message, const std::optional<std::string>&,
                                           const std::optional<std::string>& destination_identifier) {
  const std::string dest = destination_identifier.value_or("somewhere");
  for (const auto& p : plugins_)
    if (p && p->plugin_type() == PluginType::TRANSPORT)
      if (auto t = std::dynamic_pointer_cast<api::TransportPlugin>(p)) t->broadcast_message(message, dest);
}

void DummyMediator::log_message(LoggingLevel level, const std::string& message) {
  bool logged = false;
  for (const auto& p : plugins_) {
    if (!p || p->plugin_type() != PluginType::LOGGING) continue;
    if (auto l = std::dynamic_pointer_cast<api::LoggingPlugin>(p)) {
      l->log(PluginType::CONTROLLER, "unknown_dummy", level, message);
      logged = true;
    }
  }
  if (!logged) utils::print_message(level, "unknown_dummy", message);
}

api::Registry& DummyMediator::registry() {
  if (!registry_) throw std::logic_error("DummyMediator has no registry");
  return *registry_;
}

// ----------------------------------------------------------------------------- DummyOrchestrationPlugin

void DummyOrchestrationPlugin::init_orchestration_plugin(const std::optional<api::PluginList>& plugins,
                                                         api::MessageStreamConfig& stream_config) {
  // Keep every plugin except ourselves: the dummy controller passes the full list, and holding a
  // shared_ptr to ourselves would be a reference cycle (Python's GC hides this; C++ would leak).
  plugins_.clear();
  if (plugins)
    for (const auto& p : *plugins)
      if (p.get() != this) plugins_.push_back(p);
  stream_config.immediate_stream_all(true);
}

void DummyOrchestrationPlugin::process_pntos_message(const Message& message, bool) {
  if (mediator_)
    mediator_->log_message(LoggingLevel::INFO, "Orchestration processing message from " + message.source_identifier);
  last_message_ = message;
}

std::optional<std::vector<std::optional<Message>>> DummyOrchestrationPlugin::request_solutions(
    const std::vector<api::Timestamp>& times, const std::optional<std::string>& filter_description) {
  if (filter_description && *filter_description != "LAST_MESSAGE") return std::nullopt;
  return std::vector<std::optional<Message>>(times.size(), last_message_);
}

// ----------------------------------------------------------------------------- DummyTransportPlugin

DummyTransportPlugin::~DummyTransportPlugin() {
  listening_ = false;
  if (thread_.joinable()) thread_.join();
}

void DummyTransportPlugin::log(const std::string& message, LoggingLevel level) {
  if (!mediator_) throw std::logic_error("DummyTransportPlugin used before init_plugin");
  mediator_->log_message(level, message);
}

void DummyTransportPlugin::init_plugin(const std::optional<std::string>&, api::Mediator* mediator) {
  mediator_ = mediator;
  log("Initialized DummyTransport");
}

void DummyTransportPlugin::shutdown_plugin() {
  log("Shutting down DummyTransport");
  stop_listening();
  if (thread_.joinable()) thread_.join();
}

void DummyTransportPlugin::start_listening() {
  listening_ = true;
  log("DummyTransport listening");
  if (!thread_.joinable()) thread_ = std::thread([this] { run(); });
}

void DummyTransportPlugin::stop_listening() {
  log("DummyTransport stopping");
  listening_ = false;
}

void DummyTransportPlugin::broadcast_message(const Message&, const std::optional<std::string>& channel_name) {
  log("DummyTransport broadcasting on " + channel_name.value_or("(none)"));
}

void DummyTransportPlugin::run() {
  const std::string chan = "channel_foo";
  const double nan = std::nan("");
  auto msg = std::make_shared<aspn23_eigen::MeasurementPosition>(
      aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0), aspn23_eigen::TypeTimestamp(std::int64_t{0}),
      ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC, nan, nan, nan,
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(0, 0),
      ASPN23_MEASUREMENT_POSITION_ERROR_MODEL_NONE, Eigen::Matrix<double, Eigen::Dynamic, 1>(0),
      std::vector<aspn23_eigen::TypeIntegrity>{});
  while (listening_) {
    if (mediator_) {
      log("DummyTransport publishing to " + chan);
      mediator_->process_pntos_message(Message(msg, chan));
      ++sent_;
    }
    std::this_thread::sleep_for(period_);
  }
}

// ----------------------------------------------------------------------------- DummyControllerPlugin

void DummyControllerPlugin::shutdown_plugin() {
  for (const auto& p : plugins_)
    if (p) p->shutdown_plugin();
}

void DummyControllerPlugin::take_control(const api::PluginList& plugins, const api::ResourceLocations&,
                                         const std::optional<std::string>&) {
  plugins_ = plugins;
  mediators_.clear();
  for (const auto& p : plugins_) {
    mediators_.push_back(std::make_unique<DummyMediator>(plugins_));
    p->init_plugin(std::nullopt, mediators_.back().get());
  }
  DummyMessageStreamConfig stream_config;
  for (const auto& p : plugins_)
    if (p->plugin_type() == PluginType::ORCHESTRATION)
      if (auto o = std::dynamic_pointer_cast<api::OrchestrationPlugin>(p))
        o->init_orchestration_plugin(plugins_, stream_config);
  for (const auto& p : plugins_)
    if (p->plugin_type() == PluginType::TRANSPORT)
      if (auto t = std::dynamic_pointer_cast<api::TransportPlugin>(p)) t->start_listening();
  std::this_thread::sleep_for(run_for_);
  shutdown_plugin();
}

}  // namespace pntos::cobra
