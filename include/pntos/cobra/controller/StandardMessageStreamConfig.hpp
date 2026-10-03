// Port of pntos.cobra.standard_plugins.controller.StandardMessageStreamConfig.
#pragma once

#include <pntos/api/orchestration.hpp>
#include <pntos/cobra/config/configs.hpp>

#include <map>
#include <mutex>

namespace pntos::cobra {

/// Maps (message type, optional source) to a buffer mode; everything else uses the default mode
/// (IMMEDIATE until one of the *_stream_all calls changes it).
class StandardMessageStreamConfig final : public api::MessageStreamConfig {
 public:
  void sequenced_stream_add(api::AspnMessageType type, const std::optional<std::string>& source = std::nullopt) override;
  void sequenced_stream_remove(api::AspnMessageType type,
                               const std::optional<std::string>& source = std::nullopt) override;
  /// Clears all overrides and makes SEQUENCED the default. `enable` is ignored, as in Cobra (TODO #66).
  void sequenced_stream_all(bool enable) override;
  void immediate_stream_add(api::AspnMessageType type, const std::optional<std::string>& source = std::nullopt) override;
  void immediate_stream_remove(api::AspnMessageType type,
                               const std::optional<std::string>& source = std::nullopt) override;
  /// Clears all overrides and makes IMMEDIATE the default. `enable` is ignored, as in Cobra (TODO #66).
  void immediate_stream_all(bool enable) override;

  /// Exact (type, source) match first, then (type, any), then the default.
  bool is_sequenced(api::AspnMessageType type, const std::optional<std::string>& source = std::nullopt) const;

 private:
  using Key = std::pair<int, std::optional<std::string>>;
  mutable std::mutex mutex_;
  std::map<Key, BufferMode> modes_;
  BufferMode default_mode_ = BufferMode::IMMEDIATE;
};

}  // namespace pntos::cobra
