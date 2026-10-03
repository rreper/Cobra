// pntOS C++ API — MessageStreamConfig and OrchestrationPlugin.
#pragma once

#include <pntos/api/common.hpp>
#include <pntos/api/controller.hpp>

namespace pntos::api {

/// Lets the orchestration plugin tell the controller which message types/sources must be
/// delivered in time-sorted order (buffered) and which immediately.
class MessageStreamConfig {
 public:
  virtual ~MessageStreamConfig() = default;

  virtual void sequenced_stream_add(AspnMessageType type,
                                    const std::optional<std::string>& source_identifier = std::nullopt) = 0;
  virtual void sequenced_stream_remove(AspnMessageType type,
                                       const std::optional<std::string>& source_identifier = std::nullopt) = 0;
  virtual void sequenced_stream_all(bool enable) = 0;

  virtual void immediate_stream_add(AspnMessageType type,
                                    const std::optional<std::string>& source_identifier = std::nullopt) = 0;
  virtual void immediate_stream_remove(AspnMessageType type,
                                       const std::optional<std::string>& source_identifier = std::nullopt) = 0;
  virtual void immediate_stream_all(bool enable) = 0;
};

/// The heart of the filter: consumes (buffered, sorted) messages and produces solutions.
class OrchestrationPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::ORCHESTRATION; }

  /// Called after init_plugin() and before any other method. `plugins` are the plugins the
  /// orchestration may use (fusion, strategy, inertial, initialization, state modeling, preprocessor).
  virtual void init_orchestration_plugin(const std::optional<PluginList>& plugins,
                                         MessageStreamConfig& stream_config) = 0;

  /// Deliver a message. `sequenced` is true if the message was delayed by buffering.
  virtual void process_pntos_message(const Message& message, bool sequenced) = 0;

  /// UPPER_SNAKE_CASE descriptions containing BEST / DEAD_RECKONING and the ASPN type + _ESTIMATE.
  virtual std::vector<std::string> filter_description_list() const = 0;

  /// One optional solution per time; nullopt if the description is invalid.
  virtual std::optional<std::vector<std::optional<Message>>> request_solutions(
      const std::vector<Timestamp>& solution_times,
      const std::optional<std::string>& filter_description = std::nullopt) = 0;
};

}  // namespace pntos::api
