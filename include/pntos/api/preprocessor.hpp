// pntOS C++ API — Preprocessor and PreprocessorPlugin (unstable upstream).
#pragma once

#include <pntos/api/common.hpp>

namespace pntos::api {

/// Transforms a message into zero or more messages before it reaches the filter.
class Preprocessor {
 public:
  virtual ~Preprocessor() = default;

  /// nullopt (or empty) drops the message; usually returns a single modified message. A
  /// preprocessor may accumulate and later return several messages.
  virtual std::optional<std::vector<Message>> process_pntos_message(const Message& message) = 0;
};

/// Factory for preprocessors; `preprocessor_identifiers()[i]` names the kind created by index i.
class PreprocessorPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::PREPROCESSOR; }

  virtual const std::vector<std::string>& preprocessor_identifiers() const = 0;
  /// nullptr if the index or config group is invalid.
  virtual std::unique_ptr<Preprocessor> new_preprocessor(std::size_t preprocessor_index,
                                                         const std::optional<std::string>& config_group = std::nullopt) = 0;
};

}  // namespace pntos::api
