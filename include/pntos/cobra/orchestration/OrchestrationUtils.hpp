// Port of pntos.cobra.utils.orchestration_utils.
#pragma once

#include <pntos/api/api.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include <functional>

namespace pntos::cobra::orch {

using LogFn = std::function<void(api::LoggingLevel, const std::string&)>;

/// Creates the inertial initialization strategy from `alignment_config_group`; null + ERROR log if
/// the plugin does not provide one.
std::unique_ptr<api::InertialInitializationStrategy> set_up_initializer(api::InitializationPlugin& plugin,
                                                                        const std::string& alignment_config_group,
                                                                        const LogFn& log);

inline bool initialization_ready(const api::InertialInitializationStrategy& initializer) {
  return initializer.request_current_status() == api::InitializationStatus::INITIALIZED_GOOD;
}

/// True if the message is not older than the filter (or if the filter has no initial solution yet).
bool has_valid_time(bool have_init_solution, const api::StandardFusionEngine& engine, const api::Message& message,
                    const LogFn& log);

std::optional<api::Message> get_dead_reckoning_solution(api::StandardInertialMechanization& inertial,
                                                        api::Timestamp time, const std::string& imu_sol_channel,
                                                        const LogFn& log);

/// Inertial solution at `time` corrected by the (peeked-ahead) Pinson error states, with the
/// 9×9 PVA covariance from the filter.
std::optional<api::Message> get_best_solution(api::StandardFusionEngine& engine,
                                              api::StandardInertialMechanization& inertial, api::Timestamp time,
                                              const std::string& sb_label, const std::string& best_sol_channel,
                                              const LogFn& log);

struct InertialSetup {
  std::unique_ptr<api::StandardInertialMechanization> inertial;
  api::InitialInertialSolution init_solution;
};
/// Gets the initial solution from the initializer and creates the inertial from it.
std::optional<InertialSetup> set_up_inertial_mechanization(api::InertialInitializationStrategy& initializer,
                                                           api::InertialPlugin& inertial_plugin,
                                                           const std::string& inertial_group, const LogFn& log);

/// Applies the 15-state Pinson error estimate x to a PVA (position, velocity, tilt). Covariance is
/// copied unchanged. Throws std::invalid_argument on a PVA without position/velocity/quaternion.
std::shared_ptr<utils::PVA> apply_error_states(const utils::PVA& pva, const api::Vector& x);

/// Values that are only valid at the current filter time (Python Cache + CacheEntry classes):
/// the inertial solution at filter time, the Pinson estimate/covariance, and the corrected filter
/// solution derived from both. Each is recomputed lazily when the filter time has moved or after
/// an explicit clear().
class SolutionCache {
 public:
  SolutionCache(api::StandardFusionEngine& engine, api::StandardInertialMechanization& inertial,
                std::string imu_sol_channel, std::string best_sol_channel, std::string pinson_label, LogFn log);

  std::optional<api::Message> inertial_solution();
  std::optional<api::EstimateWithCovariance> pinson();
  std::optional<api::Message> filter_solution();

  void clear_inertial_solution() { inertial_time_.reset(); }
  void clear_pinson() { pinson_time_.reset(); }
  void clear_filter_solution() { filter_time_.reset(); }

 private:
  bool valid(const std::optional<api::Timestamp>& t) const;
  api::StandardFusionEngine& engine_;
  api::StandardInertialMechanization& inertial_;
  std::string imu_sol_channel_, best_sol_channel_, pinson_label_;
  LogFn log_;
  std::optional<api::Message> inertial_value_;
  std::optional<api::Timestamp> inertial_time_;
  std::optional<api::EstimateWithCovariance> pinson_value_;
  std::optional<api::Timestamp> pinson_time_;
  std::optional<api::Message> filter_value_;
  std::optional<api::Timestamp> filter_time_;
};

}  // namespace pntos::cobra::orch
