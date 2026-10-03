// Port of pntos.cobra.tutorial_plugins.UiLogPlottingPlugin.
//
// The Python plugin opens matplotlib windows at shutdown. This port has no plotting library, so it
// does the analysis part instead: it reads the solution and truth PVA channels back from the output
// LCM log, interpolates truth onto the solution epochs, logs the per-axis RMS position / velocity /
// attitude errors and writes them per epoch to `<log dir>/<log stem>/pva_errors.csv` (the same
// directory the Python plugin saves its figures to). Plot that CSV, or run tools/compare_to_truth.py.
#pragma once

#include <pntos/api/api.hpp>
#include <pntos/cobra/config/configs.hpp>

#include <string>

namespace pntos::cobra {

class UiLogPlottingPlugin final : public api::UiPlugin {
 public:
  explicit UiLogPlottingPlugin(std::string identifier) : identifier_(std::move(identifier)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override;
  void shutdown_plugin() override;
  const std::string& identifier() const override { return identifier_; }
  bool requires_main_thread() const override { return false; }
  void run_main_thread() override {}

  struct Summary {
    std::size_t epochs = 0;
    double rms_pos[3] = {0, 0, 0};  ///< m, NED
    double rms_vel[3] = {0, 0, 0};  ///< m/s, NED
    double rms_rpy[3] = {0, 0, 0};  ///< deg
    std::string csv_path;
  };
  /// The analysis step, separated for tests. Returns nullopt (after logging) when the log cannot be
  /// read or has no overlapping solution/truth data.
  std::optional<Summary> summarise() const;

 private:
  void log(api::LoggingLevel level, const std::string& message) const;
  std::string identifier_;
  api::Mediator* mediator_ = nullptr;
  std::optional<UiLogPlottingConfig> config_;
};

}  // namespace pntos::cobra
