// Cobra config structs (ports of pntos.cobra.config.*). Same registry key layout as Python.
#pragma once

#include <pntos/cobra/config/BaseConfig.hpp>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace pntos::cobra {

using Vec3 = std::array<double, 3>;
using Vec4 = std::array<double, 4>;
using Mat3 = std::array<std::array<double, 3>, 3>;

api::Vector to_vector(const Vec3& v);
api::Vector to_vector(const Vec4& v);
api::Matrix to_matrix(const Mat3& m);
api::Matrix to_matrix(const std::vector<double>& v);

// ----------------------------------------------------------------------------- leaf configs

/// IMU error model: FOGM biases + random walk, per axis.
struct ImuConfig final : BaseConfig {
  std::string group_;
  Vec3 accel_bias_sigma{};         ///< m/s² (steady-state bias sigma)
  Vec3 accel_bias_tau{};           ///< s
  Vec3 accel_random_walk_sigma{};  ///< m/s^(3/2)
  Vec3 gyro_bias_sigma{};          ///< rad/s
  Vec3 gyro_bias_tau{};            ///< s
  Vec3 gyro_random_walk_sigma{};   ///< rad/s^(1/2)
  Vec3 accel_bias_initial_sigma{0, 0, 0};
  Vec3 gyro_bias_initial_sigma{0, 0, 0};

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<ImuConfig> from_registry(api::Mediator& m, const std::string& group);
};

struct FogmConfig final : BaseConfig {
  std::string group_;
  std::vector<double> sigma;
  std::vector<double> tau;
  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<FogmConfig> from_registry(api::Mediator& m, const std::string& group);
};

struct MountingConfig final : BaseConfig {
  std::string group_;
  Vec3 lever_arm{};
  Vec4 orientation{1, 0, 0, 0};  ///< quaternion A->B
  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<MountingConfig> from_registry(api::Mediator& m, const std::string& group);
};

// ----------------------------------------------------------------------------- state blocks

/// Base fields every state block config has. Subclasses fix `identifier` and add fields.
struct StateBlockConfig : BaseConfig {
  std::string group_;
  std::string identifier;
  std::string label;
  std::optional<api::EstimateWithCovariance> estimate_with_covariance;
  std::optional<std::vector<std::string>> aux_channels;

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  /// Reads only the base fields (what the orchestration needs to dispatch).
  static std::optional<StateBlockConfig> from_registry(api::Mediator& m, const std::string& group);

 protected:
  void write_base(ConfigWriter& w) const;
  bool read_base(ConfigReader& r);
};

struct PinsonStateBlockConfig final : StateBlockConfig {
  static constexpr const char* kIdentifier = "pinson15";
  ImuConfig imu_model;
  PinsonStateBlockConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<PinsonStateBlockConfig> from_registry(api::Mediator& m, const std::string& group);
};

struct FogmStateBlockConfig final : StateBlockConfig {
  static constexpr const char* kIdentifier = "fogm";
  FogmConfig fogm_model;
  FogmStateBlockConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<FogmStateBlockConfig> from_registry(api::Mediator& m, const std::string& group);
};

struct ClockBiasStateBlockConfig final : StateBlockConfig {
  static constexpr const char* kIdentifier = "clock_bias";
  double h_0 = 0;
  double h_neg2 = 0;
  std::optional<double> q3;
  ClockBiasStateBlockConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<ClockBiasStateBlockConfig> from_registry(api::Mediator& m, const std::string& group);
};

struct ConstantStateBlockConfig final : StateBlockConfig {
  static constexpr const char* kIdentifier = "constant";
  std::optional<api::Matrix> Q;
  ConstantStateBlockConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<ConstantStateBlockConfig> from_registry(api::Mediator& m, const std::string& group);
};

// ----------------------------------------------------------------------- measurement processors

/// Reserved aux channel names.
inline constexpr const char* kAuxInertialPva = "INERTIAL_PVA";
inline constexpr const char* kAuxInertialForcesAndRates = "INERTIAL_FORCES_AND_RATES";

struct MeasurementProcessorConfig : BaseConfig {
  std::string group_;
  std::string identifier;
  std::string label;
  std::vector<std::string> state_block_labels;
  std::string channel;
  std::optional<std::vector<std::string>> aux_channels;

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<MeasurementProcessorConfig> from_registry(api::Mediator& m, const std::string& group);

 protected:
  void write_base(ConfigWriter& w) const;
  bool read_base(ConfigReader& r);
};

/// Config with a lever arm only (pinson_position, pinson_with_ned_fogm_position,
/// pinson_with_lever_arm_position, pinson_posvel, pinson_altitude, position).
struct LeverArmMPConfig final : MeasurementProcessorConfig {
  Vec3 lever_arm{};
  LeverArmMPConfig(std::string identifier_, std::optional<std::vector<std::string>> aux);
  void to_registry(api::Mediator& m) const override;
  static std::optional<LeverArmMPConfig> from_registry(api::Mediator& m, const std::string& group);
};

/// Config with a lever arm and a sensor orientation (pinson_body_velocity, direction3D_to_points).
struct LeverArmOrientationMPConfig final : MeasurementProcessorConfig {
  Vec3 lever_arm{};
  Vec4 orientation{1, 0, 0, 0};  ///< quaternion, C_platform_to_sensor
  LeverArmOrientationMPConfig(std::string identifier_, std::optional<std::vector<std::string>> aux);
  void to_registry(api::Mediator& m) const override;
  static std::optional<LeverArmOrientationMPConfig> from_registry(api::Mediator& m, const std::string& group);
};

/// Config with no extra fields (pinson_velocity).
struct PlainMPConfig final : MeasurementProcessorConfig {
  PlainMPConfig(std::string identifier_, std::optional<std::vector<std::string>> aux);
};

namespace mp {
inline constexpr const char* kPinsonPosition = "pinson_position";
inline constexpr const char* kPinsonVelocity = "pinson_velocity";
inline constexpr const char* kPinsonWithNedFogmPosition = "pinson_with_ned_fogm_position";
inline constexpr const char* kPinsonAltitude = "pinson_altitude";
inline constexpr const char* kPinsonWithLeverArmPosition = "pinson_with_lever_arm_position";
inline constexpr const char* kPinsonBodyVelocity = "pinson_body_velocity";
inline constexpr const char* kPinsonPosVel = "pinson_posvel";
inline constexpr const char* kPosition = "position";
inline constexpr const char* kDirection3DToPoints = "direction3D_to_points";

// Factory helpers mirroring the Python *MPConfig dataclasses (identifier + fixed aux_channels).
LeverArmMPConfig PinsonPositionMPConfig();
PlainMPConfig PinsonVelocityMPConfig();
LeverArmMPConfig PinsonWithNedFogmPositionMPConfig();
LeverArmMPConfig AltitudeMPConfig();
LeverArmMPConfig PinsonWithLeverArmPositionMPConfig();
LeverArmOrientationMPConfig PinsonBodyVelocityMPConfig();
LeverArmMPConfig PosVelMPConfig();
LeverArmMPConfig PositionMPConfig();
LeverArmOrientationMPConfig Direction3dToPointsMPConfig();
}  // namespace mp

// ------------------------------------------------------------------------- virtual state blocks

struct VirtualStateBlockConfig : BaseConfig {
  std::string group_;
  std::string identifier;
  std::string source;
  std::string target;
  std::optional<std::vector<std::string>> aux_channels;

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<VirtualStateBlockConfig> from_registry(api::Mediator& m, const std::string& group);

 protected:
  void write_base(ConfigWriter& w) const;
  bool read_base(ConfigReader& r);
};

struct PinsonErrorToStandardVSBConfig final : VirtualStateBlockConfig {
  static constexpr const char* kIdentifier = "pinson_error_to_standard";
  PinsonErrorToStandardVSBConfig() {
    identifier = kIdentifier;
    aux_channels = std::vector<std::string>{kAuxInertialPva};
  }
};

struct StateExtractorConfig final : VirtualStateBlockConfig {
  static constexpr const char* kIdentifier = "state_extractor";
  int incoming_state_size = 0;
  std::vector<int> indices_to_extract;
  StateExtractorConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<StateExtractorConfig> from_registry(api::Mediator& m, const std::string& group);
};

// ----------------------------------------------------------------------------- fusion engine

/// Configuration for StandardFusionEngine (group "config/fusion_engine").
struct FusionEngineConfig final : BaseConfig {
  static constexpr const char* kGroup = "config/fusion_engine";
  std::string group_ = kGroup;
  /// Record state_labels/time/estimate/sigma to the `diagnostics` group after every propagate.
  bool save_x_and_p_after_prop = false;
  /// Record state_labels/time/estimate/sigma to the `diagnostics` group after every update.
  bool save_x_and_p_after_update = false;

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<FusionEngineConfig> from_registry(api::Mediator& m, const std::string& group = kGroup);
};

// ----------------------------------------------------------------------------- controller / streams

/// Config for StandardControllerPlugin (group "config/controller").
struct ControllerConfig final : BaseConfig {
  static constexpr const char* kGroup = "config/controller";
  static constexpr double kDefaultBufferLengthSec = 2.0;
  std::string group_ = kGroup;
  /// Sequenced-streamed messages are held this long (s) before reaching the orchestration plugin.
  double buffer_length_sec = kDefaultBufferLengthSec;
  /// Minimum time (s) between solution requests; nullopt disables solution publishing.
  std::optional<double> publish_interval = 1.0;
  /// Shut down automatically once `controller/flags: ready_to_shutdown` is set.
  bool auto_shutdown = true;

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<ControllerConfig> from_registry(api::Mediator& m, const std::string& group = kGroup);
};

enum class BufferMode : int { IMMEDIATE = 0, SEQUENCED = 1 };

/// One message type (optionally restricted to a source) whose buffer mode is overridden.
struct Stream final : BaseConfig {
  std::string group_;
  api::AspnMessageType message_type = ASPN_UNDEFINED;
  std::optional<std::string> source_identifier;

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<Stream> from_registry(api::Mediator& m, const std::string& group);
};

/// Orchestration message-stream config: a default buffer mode plus overrides.
struct StreamConfig final : BaseConfig {
  std::string group_;
  BufferMode default_buffer_mode = BufferMode::SEQUENCED;
  std::optional<std::vector<Stream>> override_streams;

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<StreamConfig> from_registry(api::Mediator& m, const std::string& group);
};

/// IMU immediate-streamed, everything else sequenced (Cobra's DEFAULT_STREAM_CONFIG).
StreamConfig default_stream_config();

}  // namespace pntos::cobra
