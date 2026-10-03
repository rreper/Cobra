// Port of pntos.cobra.standard_plugins.DiagnosticLogPlugin.
//
// Records every value written to the `diagnostics` registry group (one list per key, appended on
// each change notification) and writes the lists to an HDF5 file at shutdown. StandardFusionEngine
// fills that group with `state_labels`, `time`, `estimate` and `sigma` when
// FusionEngineConfig::save_x_and_p_after_prop / _after_update are set.
#pragma once

#include <pntos/api/api.hpp>

#include <map>
#include <string>
#include <vector>

namespace pntos::cobra {

class DiagnosticLogPlugin final : public api::UtilityPlugin {
 public:
  static constexpr const char* kGroupToWatch = "diagnostics";
  static constexpr const char* kDefaultOutputFile = "./OUTPUT.hdf5";

  explicit DiagnosticLogPlugin(std::string identifier, std::string output_file = kDefaultOutputFile)
      : identifier_(std::move(identifier)), output_file_(std::move(output_file)) {}

  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override;
  void shutdown_plugin() override;
  const std::string& identifier() const override { return identifier_; }

  const std::map<std::string, std::vector<api::RegistryValue>>& store() const { return store_; }
  const std::string& output_file() const { return output_file_; }

 private:
  std::string identifier_;
  std::string output_file_;
  api::Mediator* mediator_ = nullptr;
  std::map<std::string, std::vector<api::RegistryValue>> store_;
};

}  // namespace pntos::cobra
