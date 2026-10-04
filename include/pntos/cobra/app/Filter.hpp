// Library push API (roadmap Phase 3): the whole Cobra stack in-process, fed by push().
//
//   cobra::Filter f(jsoncfg::load_app_config("configs/pos_ins.json"));
//   f.push(imu_message); f.push(position_message); ...
//   for (const auto& sol : f.take_solutions()) ...        // what the transport would have broadcast
//   auto best = f.solution(api::Timestamp{t});            // on demand
//   f.stop();
//
// The config's transport is replaced by a PushTransportPlugin; everything else (controller, mediator,
// orchestration, registry, logging) runs exactly as in the apps, on the caller's thread.
#pragma once

#include <pntos/api/api.hpp>
#include <pntos/cobra/app/AppBuilder.hpp>
#include <pntos/cobra/config/JsonConfig.hpp>
#include <pntos/cobra/controller/StandardControllerPlugin.hpp>

#include <functional>
#include <mutex>
#include <vector>

namespace pntos::cobra {

/// A transport that receives messages from push() and hands broadcasts to a sink.
class PushTransportPlugin final : public api::TransportPlugin {
 public:
  using Sink = std::function<void(const api::Message& message, const std::string& channel)>;
  explicit PushTransportPlugin(std::string identifier) : identifier_(std::move(identifier)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override { mediator_ = mediator; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }
  void start_listening() override { listening_ = true; }
  void stop_listening() override { listening_ = false; }
  void broadcast_message(const api::Message& message, const std::optional<std::string>& channel_name) override;

  /// Delivers one message to the mediator (immediate or buffered per the stream config). Returns false
  /// if the transport is not listening.
  bool push(const api::Message& message);
  void set_sink(Sink sink) { sink_ = std::move(sink); }
  api::Mediator* mediator() const { return mediator_; }
  bool listening() const { return listening_; }

 private:
  std::string identifier_;
  api::Mediator* mediator_ = nullptr;
  Sink sink_;
  bool listening_ = false;
};

class Filter {
 public:
  /// Builds the plugin set of `config` with a push transport in place of the configured one and starts
  /// the controller. Throws std::runtime_error if the config cannot be built or started.
  explicit Filter(AppConfig config, const app::RunOptions& options = {});
  ~Filter();
  Filter(const Filter&) = delete;
  Filter& operator=(const Filter&) = delete;

  void push(const api::Message& message);
  void push(std::shared_ptr<api::AspnBase> message, const std::string& channel) { push(api::Message(std::move(message), channel)); }

  /// Solutions published by the mediator (once per publish_interval of message time) since the last call.
  std::vector<api::Message> take_solutions();
  /// Called synchronously from push() for every published solution (in addition to the queue).
  void set_solution_callback(std::function<void(const api::Message&)> callback);
  /// BEST solution at `time` on demand (nullopt before alignment or outside the inertial buffer).
  std::optional<api::Message> solution(api::Timestamp time);

  api::Registry& registry();
  const AppConfig& config() const { return config_; }
  bool error_logged() const { return controller_->exit_code() != ExitCode::SUCCESS; }
  /// Shuts the stack down; idempotent. Returns the process exit code (0 unless a plugin logged an ERROR).
  int stop();

 private:
  AppConfig config_;
  api::PluginList plugins_;
  std::shared_ptr<PushTransportPlugin> transport_;
  std::unique_ptr<StandardControllerPlugin> controller_;
  std::mutex mutex_;
  std::vector<api::Message> solutions_;
  std::function<void(const api::Message&)> callback_;
  bool stopped_ = false;
};

}  // namespace pntos::cobra
