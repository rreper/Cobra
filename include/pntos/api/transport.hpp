// pntOS C++ API — TransportPlugin.
#pragma once

#include <pntos/api/common.hpp>

namespace pntos::api {

/// Bridges a wire / log / bus to pntOS Messages (bi-directional).
///
/// start_listening() must not block the caller's thread (Cobra's controllers assume the transport
/// manages its own thread). Inbound data is delivered with Mediator::process_pntos_message().
class TransportPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::TRANSPORT; }

  virtual void start_listening() = 0;
  virtual void stop_listening() = 0;

  /// Send a message back out. If channel_name is nullopt the transport decides the route.
  virtual void broadcast_message(const Message& message,
                                 const std::optional<std::string>& channel_name = std::nullopt) = 0;
};

}  // namespace pntos::api
