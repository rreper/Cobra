// Cobra config structs (ports of pntos.cobra.config.*). Same registry key layout as Python.
#pragma once

#include <pntos/cobra/config/BaseConfig.hpp>

#include <array>
#include <map>
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
  /// Reproduce the Python original's process-noise handling, which re-rotates its stored
  /// sensor-frame Q into NED on every propagation (COBRA_ANALYSIS §12 #1). False (default) rotates a
  /// fresh copy each time, i.e. the configured random-walk sigmas are applied as written. The Python
  /// integration-test limits were tuned with the legacy behaviour; see DESIGN.md §9.5.
  bool legacy_q_rotation = false;
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
  /// C++ addition (not in Cobra): chi-square innovation gate. A measurement whose normalised innovation
  /// squared exceeds the chi-square quantile at this probability (for its number of rows) is rejected and
  /// counted in registry group `fusion/gating`. Absent: FusionEngineConfig::innovation_gate_probability.
  std::optional<double> innovation_gate_probability;
  /// C++ addition: EGM96 grid for processors that accept MSL altitudes (pinson_altitude). Absent: the
  /// PNTOS_GEOID_FILE environment variable, then data/egm96_15min.bin; without any, MSL is rejected.
  std::optional<std::string> geoid_file;

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
  /// C++ addition: default innovation gate probability for every measurement processor (0 = no gate,
  /// Cobra behaviour). See MeasurementProcessorConfig::innovation_gate_probability.
  double innovation_gate_probability = 0.0;

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

// ----------------------------------------------------------------------------- inertial / feedback

/// Inertial mechanization and buffering (Python InertialConfig).
struct InertialConfig final : BaseConfig {
  std::string group_;
  double expected_dt = 0.01;             ///< s between inertial messages
  double inertial_buffer_length = 10.0;  ///< s
  std::vector<std::string> channels;
  Mat3 C_imu_to_platform{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<InertialConfig> from_registry(api::Mediator& m, const std::string& group);
};

/// When to apply inertial resets (Python FeedbackConfig). Zero thresholds => after every update.
struct FeedbackConfig final : BaseConfig {
  std::string group_;
  double time_threshold = 0.0;       ///< s between resets
  double pos_error_threshold = 0.0;  ///< m of any position error state

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<FeedbackConfig> from_registry(api::Mediator& m, const std::string& group);
};

// ----------------------------------------------------------------------------- preprocessors

/// Base fields of every preprocessor config. Subclasses fix `identifier`.
struct PreprocessorConfig : BaseConfig {
  std::string group_;
  std::string identifier;
  std::optional<std::vector<std::string>> channels;
  bool regex = false;

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<PreprocessorConfig> from_registry(api::Mediator& m, const std::string& group);

 protected:
  void write_base(ConfigWriter& w) const;
  bool read_base(ConfigReader& r);
};

struct BarometerToAltitudeConfig final : PreprocessorConfig {
  static constexpr const char* kIdentifier = "baro_converter";
  std::optional<double> alt_sigma;
  BarometerToAltitudeConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<BarometerToAltitudeConfig> from_registry(api::Mediator& m, const std::string& group);
};

struct DownsamplerConfig final : PreprocessorConfig {
  static constexpr const char* kIdentifier = "downsampler";
  std::vector<std::int64_t> downsampling_factors;
  DownsamplerConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<DownsamplerConfig> from_registry(api::Mediator& m, const std::string& group);
};

struct ImuRotatorConfig final : PreprocessorConfig {
  static constexpr const char* kIdentifier = "imu_rotator";
  Mat3 C_imu_to_platform{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  ImuRotatorConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<ImuRotatorConfig> from_registry(api::Mediator& m, const std::string& group);
};

struct TimeAdjusterConfig final : PreprocessorConfig {
  static constexpr const char* kIdentifier = "time_adjuster";
  std::int64_t expected_dt_nsec = 0;
  TimeAdjusterConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<TimeAdjusterConfig> from_registry(api::Mediator& m, const std::string& group);
};

struct TimeBiasConfig final : PreprocessorConfig {
  static constexpr const char* kIdentifier = "time_bias";
  std::int64_t time_bias = 0;  ///< ns subtracted from each timestamp
  TimeBiasConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<TimeBiasConfig> from_registry(api::Mediator& m, const std::string& group);
};

/// Extras: generates 2-D (lateral/vertical) zero-velocity measurements for ground vehicles
/// (Python pntos.cobra.extras.config.ZeroVelocity2dGeneratorConfig).
struct ZeroVelocity2dGeneratorConfig final : PreprocessorConfig {
  static constexpr const char* kIdentifier = "zero_velocity2d_generator";
  double trigger_dt_sec = 0.0;    ///< minimum time between generated measurements (0 = every trigger)
  double lateral_vel_sigma = 0;   ///< m/s
  double vertical_vel_sigma = 0;  ///< m/s
  std::string output_channel;
  ZeroVelocity2dGeneratorConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<ZeroVelocity2dGeneratorConfig> from_registry(api::Mediator& m, const std::string& group);
};

/// C++ addition (roadmap Phase 2): degrades IMU / position / velocity messages to emulate worse sensors.
/// IMU: white noise with the given density (per sqrt(Hz)) and a constant bias are added (scaled by the
/// sample interval for integrated messages). Position: NED noise, a covariance scale and one-shot jumps
/// (outliers not reflected in the covariance) at times relative to the first position message. Velocity:
/// noise on the present axes and a covariance scale. Deterministic for a given seed.
struct SensorDegradationConfig final : PreprocessorConfig {
  static constexpr const char* kIdentifier = "sensor_degradation";
  std::int64_t seed = 1;
  double imu_expected_dt = 0.01;              ///< s, for integrated IMU messages
  Vec3 accel_noise_density{0, 0, 0};          ///< m/s^2/sqrt(Hz)
  Vec3 gyro_noise_density{0, 0, 0};           ///< rad/s/sqrt(Hz)
  Vec3 accel_bias{0, 0, 0};                   ///< m/s^2
  Vec3 gyro_bias{0, 0, 0};                    ///< rad/s
  Vec3 position_noise_sigma_ned{0, 0, 0};     ///< m
  double position_covariance_scale = 1.0;
  Vec3 velocity_noise_sigma{0, 0, 0};         ///< m/s, per message axis
  double velocity_covariance_scale = 1.0;
  std::vector<std::array<double, 4>> position_jumps;  ///< {time_s, north_m, east_m, down_m}
  /// Slow pulls: from `start_s` (relative to the first position message) the offset grows at the given NED
  /// rates for `duration_s` (0 = until the end), then stays at its final value. {start_s, n_mps, e_mps, d_mps, duration_s}
  std::vector<std::array<double, 5>> position_ramps;
  /// Synthesises a second position source from a PVA stream (e.g. the truth channel) seen on `channels`:
  /// one MeasurementPosition on `derived_position_channel` per 1/rate seconds, truth plus NED noise with
  /// `derived_position_sigma_ned`, covariance diag(sigma^2). Empty channel = off.
  std::string derived_position_channel;
  Vec3 derived_position_sigma_ned{5.0, 5.0, 8.0};
  double derived_position_rate_hz = 1.0;
  SensorDegradationConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<SensorDegradationConfig> from_registry(api::Mediator& m, const std::string& group);
};

struct OutageConfig final : PreprocessorConfig {
  static constexpr const char* kIdentifier = "outage";
  double start_time = 0;  ///< s after the first message on the channel
  double end_time = 0;
  OutageConfig() { identifier = kIdentifier; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<OutageConfig> from_registry(api::Mediator& m, const std::string& group);
};

// ----------------------------------------------------------------------------- alignment

/// Manual (fixed) initial solution (Python ManualAlignmentConfig).
struct ManualAlignmentConfig final : BaseConfig {
  std::string group_;
  Vec3 initial_pos{};  ///< rad, rad, m HAE
  Vec3 initial_vel{};  ///< m/s NED
  Vec3 initial_rpy{};  ///< rad
  Vec3 initial_accel_bias{};
  Vec3 initial_gyro_bias{};
  Vec3 initial_accel_scale_factor{};
  Vec3 initial_gyro_scale_factor{};
  double initial_time = 0;  ///< s
  Vec3 initial_pos_var{};
  Vec3 initial_vel_var{};
  Vec3 initial_tilt_var{};
  Vec3 initial_accel_bias_var{};
  Vec3 initial_gyro_bias_var{};
  Vec3 initial_accel_scale_factor_var{};
  Vec3 initial_gyro_scale_factor_var{};

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<ManualAlignmentConfig> from_registry(api::Mediator& m, const std::string& group);
};

/// Static (stationary) alignment (Python StaticAlignmentConfig).
struct StaticAlignmentConfig final : BaseConfig {
  std::string group_;
  double static_time = 0;  ///< s of IMU data before aligning
  ImuConfig imu_model;
  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<StaticAlignmentConfig> from_registry(api::Mediator& m, const std::string& group);
};

/// Static alignment with a user-supplied heading (Python ManualHeadingAlignmentConfig).
struct ManualHeadingAlignmentConfig final : BaseConfig {
  std::string group_;
  double static_time = 0;
  ImuConfig imu_model;
  double heading = 0;        ///< rad
  double heading_sigma = 0;  ///< rad
  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<ManualHeadingAlignmentConfig> from_registry(api::Mediator& m, const std::string& group);
};

// ----------------------------------------------------------------------------- orchestration

/// Configuration of StandardOrchestrationPlugin (group "config/orchestration").
///
/// Nested configs are stored by group pointer, so on the read side only the *base* fields of the
/// state block / processor / VSB / preprocessor configs are reconstructed; the providers read their
/// own full configs from the group when instantiating.
struct StandardOrchestrationConfig final : BaseConfig {
  static constexpr const char* kGroup = "config/orchestration";
  std::string group_ = kGroup;

  std::string best_sol_channel;
  std::string imu_sol_channel;
  std::vector<std::string> alignment_channels;
  PinsonStateBlockConfig pinson_sb_config;
  std::optional<std::vector<std::shared_ptr<const StateBlockConfig>>> additional_sb_configs;
  std::optional<std::vector<std::shared_ptr<const VirtualStateBlockConfig>>> vsb_configs;
  std::optional<std::vector<std::shared_ptr<const MeasurementProcessorConfig>>> mp_configs;
  InertialConfig inertial_config;
  std::optional<FeedbackConfig> feedback_config;
  /// Write side: the alignment config to store. Read side: only `alignment_config_group` is filled.
  std::shared_ptr<const BaseConfig> alignment_config;
  std::string alignment_config_group;
  std::optional<std::vector<std::shared_ptr<const PreprocessorConfig>>> preprocessor_configs;
  double max_prop_interval = 2.0;  ///< s
  bool publish_before_update = false;
  bool publish_after_update = false;
  double max_filter_lag = ControllerConfig::kDefaultBufferLengthSec;  ///< s
  StreamConfig stream_config = default_stream_config();

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<StandardOrchestrationConfig> from_registry(api::Mediator& m,
                                                                  const std::string& group = kGroup);
};

/// Channels used by the tutorial orchestration plugins (Python TutorialOrchestrationConfig,
/// group "config/orchestration").
struct TutorialOrchestrationConfig final : BaseConfig {
  static constexpr const char* kGroup = "config/orchestration";
  std::string group_ = kGroup;
  std::string position_channel;
  std::string velocity_channel = "unused";
  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<TutorialOrchestrationConfig> from_registry(api::Mediator& m, const std::string& group = kGroup);
};

/// Log file and channels the UI log plotting plugin summarises at shutdown (Python UiLogPlottingConfig,
/// group "config/ui_logfile_plotting").
struct UiLogPlottingConfig final : BaseConfig {
  static constexpr const char* kGroup = "config/ui_logfile_plotting";
  std::string group_ = kGroup;
  std::string logfile;
  std::string solution_channel;
  std::string truth_channel;
  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<UiLogPlottingConfig> from_registry(api::Mediator& m, const std::string& group = kGroup);
};

/// Initial solution taken from the first PVA on a channel (Python PvaMessageInitializationConfig).
struct PvaMessageInitializationConfig final : BaseConfig {
  std::string group_;
  std::string initial_pva_channel;
  Vec3 initial_accel_bias_sigma{};
  Vec3 initial_gyro_bias_sigma{};
  std::optional<std::array<double, 9>> initial_pva_sigma;  ///< overrides the message covariance
  std::optional<double> start_time;                        ///< s; PVAs before this are ignored

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<PvaMessageInitializationConfig> from_registry(api::Mediator& m, const std::string& group);
};

// ----------------------------------------------------------------------------- transports

/// LcmLogTransportPlugin (group "config/lcm_log_transport").
struct LcmLogTransportConfig final : BaseConfig {
  static constexpr const char* kGroup = "config/lcm_log_transport";
  std::string group_ = kGroup;
  std::optional<std::string> input_file;   ///< LCM log to replay
  std::optional<std::string> output_file;  ///< LCM log to record to (overwritten; must differ from input)
  std::optional<std::vector<std::string>> channels_to_process;  ///< nullopt = all
  bool record_input_channels = true;       ///< copy input events into the output log

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<LcmLogTransportConfig> from_registry(api::Mediator& m, const std::string& group = kGroup);
};

/// LcmUdpTransportPlugin (network LCM over UDP multicast, transport/LcmUdpTransportPlugin.hpp).
/// `url` and `subscribe_to` are Cobra's; `idle_timeout_sec` and `output_file` are C++ additions.
struct LcmTransportConfig final : BaseConfig {
  static constexpr const char* kGroup = "config/lcm_transport";
  std::string group_ = kGroup;
  std::string url = "udpm://239.255.76.67:7667?ttl=0";
  std::string subscribe_to = "^((?!pntos).)*$";
  double idle_timeout_sec = 0;             ///< > 0: request shutdown after this long without a message
  std::optional<std::string> output_file;  ///< LCM log recording what was received and broadcast

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<LcmTransportConfig> from_registry(api::Mediator& m, const std::string& group = kGroup);
};

/// C++ addition: a generic config for out-of-tree plugins (registered through app::register_orchestration /
/// register_extra_plugin). Its values are written as-is into registry group `group_`; JSON form
/// `{"type": "RegistryConfig", "group": "config/x", "values": {...}}` with strings, bools, integers, numbers,
/// string arrays, numeric arrays (column vectors) and arrays of numeric arrays (matrices).
struct RegistryConfig final : BaseConfig {
  std::string group_;
  std::map<std::string, api::RegistryValue> values;

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  /// Every key of the group as stored; nullopt if the group does not exist.
  static std::optional<RegistryConfig> from_registry(api::Mediator& m, const std::string& group);
};

}  // namespace pntos::cobra
