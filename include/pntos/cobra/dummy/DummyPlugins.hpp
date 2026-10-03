// Ports of pntos.cobra.dummy_plugins: minimal plugins for wiring tests and the "minimal" app.
#pragma once

#include <pntos/api/api.hpp>

#include <atomic>
#include <memory>
#include <thread>

namespace pntos::cobra {

/// Mediator that forwards to the first matching plugin in a flat list.
class DummyMediator final : public api::Mediator {
 public:
  explicit DummyMediator(api::PluginList plugins = {}) : plugins_(std::move(plugins)) {}
  api::PluginList& plugins() { return plugins_; }

  std::vector<std::string> filter_description_list() const override;
  std::optional<std::vector<std::optional<api::Message>>> request_solutions(
      const std::vector<api::Timestamp>& solution_times,
      const std::optional<std::string>& filter_description = std::nullopt) override;
  /// Forwards to every orchestration plugin, then echoes its solution over all transports.
  void process_pntos_message(const api::Message& message) override;
  void broadcast_aspn_message(const api::Message& message, const std::optional<std::string>& transport = std::nullopt,
                              const std::optional<std::string>& destination_identifier = std::nullopt) override;
  void log_message(api::LoggingLevel level, const std::string& message) override;
  api::Registry& registry() override;
  void set_registry(std::shared_ptr<api::Registry> r) { registry_ = std::move(r); }

 private:
  api::PluginList plugins_;
  std::shared_ptr<api::Registry> registry_;
};

/// Does nothing.
class DummyMessageStreamConfig final : public api::MessageStreamConfig {
 public:
  void sequenced_stream_add(api::AspnMessageType, const std::optional<std::string>& = std::nullopt) override {}
  void sequenced_stream_remove(api::AspnMessageType, const std::optional<std::string>& = std::nullopt) override {}
  void sequenced_stream_all(bool) override {}
  void immediate_stream_add(api::AspnMessageType, const std::optional<std::string>& = std::nullopt) override {}
  void immediate_stream_remove(api::AspnMessageType, const std::optional<std::string>& = std::nullopt) override {}
  void immediate_stream_all(bool) override {}
};

/// Echoes the last message it received as the "LAST_MESSAGE" solution.
class DummyOrchestrationPlugin final : public api::OrchestrationPlugin {
 public:
  explicit DummyOrchestrationPlugin(std::string identifier) : identifier_(std::move(identifier)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override { mediator_ = mediator; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }
  void init_orchestration_plugin(const std::optional<api::PluginList>& plugins,
                                 api::MessageStreamConfig& stream_config) override;
  void process_pntos_message(const api::Message& message, bool sequenced) override;
  std::vector<std::string> filter_description_list() const override { return {"LAST_MESSAGE"}; }
  std::optional<std::vector<std::optional<api::Message>>> request_solutions(
      const std::vector<api::Timestamp>& solution_times,
      const std::optional<std::string>& filter_description = std::nullopt) override;
  api::Mediator* mediator() const { return mediator_; }

 private:
  std::string identifier_;
  api::Mediator* mediator_ = nullptr;
  api::PluginList plugins_;
  std::optional<api::Message> last_message_;
};

/// Produces a fake MeasurementPosition every `period` on its own thread; broadcasts only log.
class DummyTransportPlugin final : public api::TransportPlugin {
 public:
  explicit DummyTransportPlugin(std::string identifier, std::chrono::milliseconds period = std::chrono::milliseconds(100))
      : identifier_(std::move(identifier)), period_(period) {}
  ~DummyTransportPlugin() override;
  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override;
  void shutdown_plugin() override;
  const std::string& identifier() const override { return identifier_; }
  void start_listening() override;
  void stop_listening() override;
  void broadcast_message(const api::Message& message, const std::optional<std::string>& channel_name) override;
  std::size_t messages_sent() const { return sent_.load(); }

 private:
  void log(const std::string& message, api::LoggingLevel level = api::LoggingLevel::INFO);
  void run();
  std::string identifier_;
  std::chrono::milliseconds period_;
  api::Mediator* mediator_ = nullptr;
  std::thread thread_;
  std::atomic<bool> listening_{false};
  std::atomic<std::size_t> sent_{0};
};

/// Initialises every plugin with a DummyMediator, starts transports, runs for `run_for`, shuts down.
class DummyControllerPlugin final : public api::ControllerPlugin {
 public:
  explicit DummyControllerPlugin(std::string identifier, std::chrono::milliseconds run_for = std::chrono::milliseconds(500))
      : identifier_(std::move(identifier)), run_for_(run_for) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator*) override {}
  void shutdown_plugin() override;
  const std::string& identifier() const override { return identifier_; }
  void take_control(const api::PluginList& plugins, const api::ResourceLocations& = std::nullopt,
                    const std::optional<std::string>& = std::nullopt) override;

 private:
  std::string identifier_;
  std::chrono::milliseconds run_for_;
  api::PluginList plugins_;
  std::vector<std::unique_ptr<DummyMediator>> mediators_;
};

}  // namespace pntos::cobra
