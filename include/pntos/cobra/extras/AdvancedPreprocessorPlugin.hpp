// Port of pntos.cobra.extras.plugins.preprocessor.* (ZeroVelocity2dGenerator + AdvancedPreprocessorPlugin).
#pragma once

#include <pntos/api/preprocessor.hpp>
#include <pntos/cobra/config/configs.hpp>

#include <aspn23/eigen/MeasurementVelocity.hpp>

#include <random>

namespace pntos::cobra {

/// Emits a 2-D zero-velocity measurement (sensor frame, x = not present, y = z = 0) alongside the
/// triggering message. A ground vehicle neither slides sideways nor leaves the ground, so the
/// lateral and vertical body velocities are zero to within the configured sigmas. Triggers on every
/// message of the configured channels (all channels if none) once `trigger_dt` has elapsed since
/// the previous generated measurement; the first trigger always fires.
class ZeroVelocity2dGenerator final : public api::Preprocessor {
 public:
  ZeroVelocity2dGenerator(api::Mediator* mediator, std::optional<std::vector<std::string>> channels, double trigger_dt_sec,
                          double lateral_vel_sigma, double vertical_vel_sigma, std::string output_channel);
  std::optional<std::vector<api::Message>> process_pntos_message(const api::Message& message) override;

  const std::string& output_channel() const { return output_channel_; }
  std::optional<std::int64_t> last_measurement_time_ns() const { return last_ns_; }

 private:
  api::Mediator* mediator_;
  std::optional<std::vector<std::string>> channels_;
  std::int64_t trigger_dt_ns_;
  std::string output_channel_;
  aspn23_eigen::MeasurementVelocity template_;
  std::optional<std::int64_t> last_ns_;
};

/// Emulates a worse sensor set on the fly (SensorDegradationConfig): IMU noise and bias, GNSS position /
/// velocity noise, covariance scaling and one-shot position jumps. Deterministic for a given seed.
class SensorDegradationPreprocessor final : public api::Preprocessor {
 public:
  SensorDegradationPreprocessor(const SensorDegradationConfig& config, api::Mediator* mediator);
  std::optional<std::vector<api::Message>> process_pntos_message(const api::Message& message) override;
  std::size_t jumps_applied() const { return next_jump_; }

 private:
  double gauss();
  SensorDegradationConfig cfg_;
  api::Mediator* mediator_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> n01_{0.0, 1.0};  ///< per instance: a Box-Muller cache must not leak between seeds
  std::optional<std::int64_t> first_position_ns_;
  std::size_t next_jump_ = 0;
};

/// Preprocessor plugin providing ["zero_velocity2d_generator", "sensor_degradation"].
class AdvancedPreprocessorPlugin final : public api::PreprocessorPlugin {
 public:
  explicit AdvancedPreprocessorPlugin(std::string identifier) : identifier_(std::move(identifier)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override;
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }
  const std::vector<std::string>& preprocessor_identifiers() const override { return ids_; }
  std::unique_ptr<api::Preprocessor> new_preprocessor(std::size_t index,
                                                      const std::optional<std::string>& config_group) override;

 private:
  std::string identifier_;
  api::Mediator* mediator_ = nullptr;
  std::vector<std::string> ids_{ZeroVelocity2dGeneratorConfig::kIdentifier, SensorDegradationConfig::kIdentifier};
};

}  // namespace pntos::cobra
