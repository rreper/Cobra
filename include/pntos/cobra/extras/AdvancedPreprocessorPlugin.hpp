// Port of pntos.cobra.extras.plugins.preprocessor.* (ZeroVelocity2dGenerator + AdvancedPreprocessorPlugin).
#pragma once

#include <pntos/api/preprocessor.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/inertial/Stationarity.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>
#include <aspn23/eigen/MeasurementVelocity.hpp>

#include <deque>
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

/// Says that the vehicle stands (C++ addition, ZeroMotionGeneratorConfig; DESIGN.md §7.25). Every IMU message
/// of its channels goes through a StationarityDetector; while the last `window` seconds were quiet, a zero
/// velocity (NED) and a zero angular rate (sensor frame) follow the IMU message every `trigger_dt_sec`, stamped
/// at the middle of the quiet window, so that the second the filter averages the gyro over lies inside it.
///
/// With `confirmation_channels` a receiver's velocity is a second witness (§7.28): a car that creeps at a steady
/// speed is as quiet to the IMU as one that stands. While such a velocity keeps arriving, the messages are made
/// for the receiver's epochs instead: one that shows less than `confirmation_speed`, with half a window of quiet
/// IMU on either side, gives a zero velocity with `confirmed_velocity_sigma` (north and east; down keeps its own
/// `confirmed_vertical_velocity_sigma`) and a zero rate, stamped at that epoch; one that shows more gives nothing
/// and is counted. The velocity may be seconds old when it arrives (the
/// controller's buffer) or ahead of the IMU's verdict (no buffer); in the second case the epoch waits until the
/// IMU has passed it by half a window. After `confirmation_timeout` without a usable velocity the IMU decides
/// alone again.
class ZeroMotionGenerator final : public api::Preprocessor {
 public:
  struct Confirmation {
    std::uint64_t confirmed = 0;     ///< messages for a receiver epoch that showed a standstill
    std::uint64_t alone = 0;         ///< messages on the IMU's word alone
    std::uint64_t contradicted = 0;  ///< receiver epochs that showed motion in the middle of a quiet stretch of the IMU
    double fastest = 0;              ///< m/s, the fastest of those
    std::uint64_t not_heard = 0;     ///< messages on a confirmation channel that could not be used
  };

  ZeroMotionGenerator(const ZeroMotionGeneratorConfig& config, api::Mediator* mediator);
  std::optional<std::vector<api::Message>> process_pntos_message(const api::Message& message) override;

  const inertial::StationarityDetector& detector() const { return detector_; }
  std::uint64_t generated() const { return generated_; }
  const Confirmation& confirmation() const { return confirmation_; }

 private:
  struct Epoch {  ///< a velocity of the receiver
    std::int64_t t;
    double speed;
    std::string channel;
  };
  struct Stretch {  ///< a time the detector called quiet without interruption
    std::int64_t begin, end;
  };
  void on_imu(const aspn23_eigen::MeasurementImu& imu, std::vector<api::Message>& out);
  void on_witness(const api::Message& message, std::vector<api::Message>& out);
  void settle(std::vector<api::Message>& out);
  void decide(const Epoch& epoch, std::vector<api::Message>& out);
  bool quiet_around(std::int64_t t) const;
  void say(std::int64_t stamp, double velocity_sigma, double vertical_sigma, std::vector<api::Message>& out);

  ZeroMotionGeneratorConfig cfg_;
  api::Mediator* mediator_;
  inertial::StationarityDetector detector_;
  std::optional<std::int64_t> last_imu_ns_, last_ns_;
  std::uint64_t generated_ = 0;

  std::int64_t half_window_ns_ = 0;
  std::deque<Stretch> quiet_;                  ///< oldest first, the last minute
  std::deque<Epoch> waiting_;                  ///< epochs the IMU has not passed by half a window
  std::optional<std::int64_t> heard_ns_;       ///< the IMU's time when the last usable velocity arrived
  bool alone_ = true;                          ///< the IMU decides alone (no witness configured, or none heard lately)
  Confirmation confirmation_;
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
  std::optional<std::int64_t> last_derived_ns_;
  std::size_t derived_count_ = 0;

 public:
  std::size_t derived_messages() const { return derived_count_; }
  /// Offset (NED, m) the configured ramps add at `rel_s` seconds after the first position message.
  api::Vector3 ramp_offset(double rel_s) const;
};

/// Preprocessor plugin providing ["zero_velocity2d_generator", "sensor_degradation", "zero_motion_generator"].
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
  std::vector<std::string> ids_{ZeroVelocity2dGeneratorConfig::kIdentifier, SensorDegradationConfig::kIdentifier,
                                ZeroMotionGeneratorConfig::kIdentifier};  // the third is a C++ addition
};

}  // namespace pntos::cobra
