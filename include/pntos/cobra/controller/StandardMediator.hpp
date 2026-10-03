// Port of pntos.cobra.standard_plugins.controller.StandardMediator.
//
// Python keeps the shared state (logging/transport/orchestration plugins, registry, message buffer,
// exit event) in class attributes. Here it lives in a MediatorContext that the controller owns and
// every per-plugin StandardMediator shares.
#pragma once

#include <pntos/api/api.hpp>
#include <pntos/cobra/controller/StandardMessageStreamConfig.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace pntos::cobra {

enum class ExitCode : int { SUCCESS = 0, ERROR = 1 };

/// A latch with an exit code (Python's ExitEvent).
class ExitEvent {
 public:
  void set(ExitCode code = ExitCode::SUCCESS);
  void clear();
  bool is_set() const;
  ExitCode exit_code() const;
  /// Blocks until set. Returns false if `timeout` elapsed first.
  bool wait(std::optional<std::chrono::milliseconds> timeout = std::nullopt) const;

 private:
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  bool set_ = false;
  ExitCode code_ = ExitCode::SUCCESS;
};

/// Per-source-channel gate backed by the `ui/channel/<source>` registry group: the UI can set
/// `enabled_mediator=false` to drop a source. Message counts / last TOV are published at most
/// every `update_interval` (rate/jitter/bandwidth statistics of the Python ChannelView are not
/// ported yet).
class UiMediatorInterface {
 public:
  explicit UiMediatorInterface(std::shared_ptr<api::Registry> registry, double update_interval_sec = 0.5);
  ~UiMediatorInterface();
  /// False if the UI asked the mediator to block this source.
  bool new_mediator_message(const api::Message& message);

 private:
  struct Channel {
    std::atomic<bool> enabled{true};
    std::int64_t count = 0;
    std::optional<std::int64_t> last_tov;
    std::string type;
    std::chrono::steady_clock::time_point last_update{};
    std::optional<api::NotifyToken> token;
  };
  Channel& ensure(const std::string& source, const api::Message& message);
  void publish(const std::string& source, Channel& ch);

  std::shared_ptr<api::Registry> registry_;
  std::chrono::milliseconds interval_;
  std::mutex mutex_;
  std::map<std::string, std::unique_ptr<Channel>> channels_;
};

/// State shared by all StandardMediators of one controller.
struct MediatorContext {
  std::shared_ptr<api::LoggingPlugin> logging_plugin;
  std::vector<std::shared_ptr<api::TransportPlugin>> transport_plugins;
  std::shared_ptr<api::OrchestrationPlugin> orchestration_plugin;
  api::ControllerPlugin* controller_plugin = nullptr;  ///< non-null => ERROR logs set the exit event
  std::shared_ptr<StandardMessageStreamConfig> stream_config = std::make_shared<StandardMessageStreamConfig>();
  std::shared_ptr<api::Registry> registry;
  std::unique_ptr<UiMediatorInterface> ui_interface;
  ExitEvent exit_event;

  std::int64_t buffer_time_nsec = 0;
  std::optional<std::int64_t> publish_interval_nsec;

  std::recursive_mutex mutex;              ///< serialises process_pntos_message across transports
  std::vector<api::Message> messages;      ///< sequenced buffer, sorted by time of validity
  std::optional<api::Timestamp> last_solution_time;
};

class StandardMediator final : public api::Mediator {
 public:
  StandardMediator(std::shared_ptr<MediatorContext> ctx, std::string attached_plugin_identifier,
                   api::PluginType attached_plugin_type);

  std::vector<std::string> filter_description_list() const override;
  std::optional<std::vector<std::optional<api::Message>>> request_solutions(
      const std::vector<api::Timestamp>& solution_times,
      const std::optional<std::string>& filter_description = std::nullopt) override;
  /// Immediate messages go straight to the orchestration; sequenced ones are buffered and released
  /// (in time order) once they are older than the buffer length relative to the newest message.
  void process_pntos_message(const api::Message& message) override;
  void broadcast_aspn_message(const api::Message& message, const std::optional<std::string>& transport = std::nullopt,
                              const std::optional<std::string>& destination_identifier = std::nullopt) override;
  void log_message(api::LoggingLevel level, const std::string& message) override;
  api::Registry& registry() override;

  const std::shared_ptr<MediatorContext>& context() const { return ctx_; }

 private:
  void log_as(api::LoggingLevel level, const std::string& message, api::PluginType type);

  std::shared_ptr<MediatorContext> ctx_;
  std::string identifier_;
  api::PluginType type_;
};

}  // namespace pntos::cobra
