#include <pntos/cobra/config/configs.hpp>

namespace pntos::cobra {

using api::Matrix;
using api::Vector;

api::Vector to_vector(const Vec3& v) { return Vector(Eigen::Vector3d(v[0], v[1], v[2])); }
api::Vector to_vector(const Vec4& v) { return Vector(Eigen::Vector4d(v[0], v[1], v[2], v[3])); }
api::Matrix to_matrix(const Mat3& m) {
  Matrix out(3, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) out(i, j) = m[i][j];
  return out;
}
api::Matrix to_matrix(const std::vector<double>& v) {
  Matrix out(static_cast<Eigen::Index>(v.size()), 1);
  for (std::size_t i = 0; i < v.size(); ++i) out(static_cast<Eigen::Index>(i), 0) = v[i];
  return out;
}

namespace {
template <std::size_t N>
std::optional<std::array<double, N>> read_arr(ConfigReader& r, const std::string& key) {
  auto v = r.optional<Vector>(key);
  if (!v || v->size() != static_cast<Eigen::Index>(N)) {
    if (v) r.fail(key);
    return std::nullopt;
  }
  std::array<double, N> out{};
  for (std::size_t i = 0; i < N; ++i) out[i] = (*v)(static_cast<Eigen::Index>(i));
  return out;
}
template <std::size_t N>
std::array<double, N> req_arr(ConfigReader& r, const std::string& key) {
  auto a = read_arr<N>(r, key);
  if (!a) {
    r.fail(key);
    return {};
  }
  return *a;
}
std::vector<double> req_vec(ConfigReader& r, const std::string& key) {
  auto v = r.require<Vector>(key);
  return std::vector<double>(v.data(), v.data() + v.size());
}
std::vector<std::string> req_strings(ConfigReader& r, const std::string& key) {
  return r.require<api::StringArray>(key);
}
}  // namespace

// ----------------------------------------------------------------------------- ImuConfig

void ImuConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.vector("accel_bias_sigma", to_vector(accel_bias_sigma));
  w.vector("accel_bias_tau", to_vector(accel_bias_tau));
  w.vector("accel_random_walk_sigma", to_vector(accel_random_walk_sigma));
  w.vector("gyro_bias_sigma", to_vector(gyro_bias_sigma));
  w.vector("gyro_bias_tau", to_vector(gyro_bias_tau));
  w.vector("gyro_random_walk_sigma", to_vector(gyro_random_walk_sigma));
  w.vector("accel_bias_initial_sigma", to_vector(accel_bias_initial_sigma));
  w.vector("gyro_bias_initial_sigma", to_vector(gyro_bias_initial_sigma));
}

std::optional<ImuConfig> ImuConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  ImuConfig c;
  c.group_ = group;
  c.accel_bias_sigma = req_arr<3>(r, "accel_bias_sigma");
  c.accel_bias_tau = req_arr<3>(r, "accel_bias_tau");
  c.accel_random_walk_sigma = req_arr<3>(r, "accel_random_walk_sigma");
  c.gyro_bias_sigma = req_arr<3>(r, "gyro_bias_sigma");
  c.gyro_bias_tau = req_arr<3>(r, "gyro_bias_tau");
  c.gyro_random_walk_sigma = req_arr<3>(r, "gyro_random_walk_sigma");
  c.accel_bias_initial_sigma = read_arr<3>(r, "accel_bias_initial_sigma").value_or(Vec3{0, 0, 0});
  c.gyro_bias_initial_sigma = read_arr<3>(r, "gyro_bias_initial_sigma").value_or(Vec3{0, 0, 0});
  if (!r.ok()) return std::nullopt;
  return c;
}

// ----------------------------------------------------------------------------- FogmConfig

void FogmConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.matrix("sigma", to_matrix(sigma));
  w.matrix("tau", to_matrix(tau));
}

std::optional<FogmConfig> FogmConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  FogmConfig c;
  c.group_ = group;
  c.sigma = req_vec(r, "sigma");
  c.tau = req_vec(r, "tau");
  if (!r.ok()) return std::nullopt;
  return c;
}

// ----------------------------------------------------------------------------- MountingConfig

void MountingConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.vector("lever_arm", to_vector(lever_arm));
  w.vector("orientation", to_vector(orientation));
}

std::optional<MountingConfig> MountingConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  MountingConfig c;
  c.group_ = group;
  c.lever_arm = req_arr<3>(r, "lever_arm");
  c.orientation = req_arr<4>(r, "orientation");
  if (!r.ok()) return std::nullopt;
  return c;
}

// ----------------------------------------------------------------------------- StateBlockConfig

void StateBlockConfig::write_base(ConfigWriter& w) const {
  w.scalar("identifier", identifier);
  w.scalar("label", label);
  if (estimate_with_covariance) w.ewc(*estimate_with_covariance);
  if (aux_channels) w.strings("aux_channels", *aux_channels);
}

bool StateBlockConfig::read_base(ConfigReader& r) {
  group_ = r.group();
  identifier = r.require<std::string>("identifier");
  label = r.require<std::string>("label");
  estimate_with_covariance = r.ewc();
  aux_channels = r.optional<api::StringArray>("aux_channels");
  return r.ok();
}

void StateBlockConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  write_base(w);
}

std::optional<StateBlockConfig> StateBlockConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  StateBlockConfig c;
  if (!c.read_base(r)) return std::nullopt;
  return c;
}

void PinsonStateBlockConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  write_base(w);
  w.nested("imu_model", imu_model);
  if (legacy_q_rotation) w.scalar("legacy_q_rotation", true);  // absent in Python-written registries
}

std::optional<PinsonStateBlockConfig> PinsonStateBlockConfig::from_registry(api::Mediator& m,
                                                                            const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  PinsonStateBlockConfig c;
  if (!c.read_base(r)) return std::nullopt;
  auto g = r.nested_group("imu_model");
  if (!g) {
    r.fail("imu_model");
    return std::nullopt;
  }
  r.suspend();
  auto imu = ImuConfig::from_registry(m, *g);
  r.resume();
  if (!imu) return std::nullopt;
  c.imu_model = *imu;
  c.legacy_q_rotation = r.optional<bool>("legacy_q_rotation").value_or(false);
  return c;
}

void FogmStateBlockConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  write_base(w);
  w.nested("fogm_model", fogm_model);
}

std::optional<FogmStateBlockConfig> FogmStateBlockConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  FogmStateBlockConfig c;
  if (!c.read_base(r)) return std::nullopt;
  auto g = r.nested_group("fogm_model");
  if (!g) {
    r.fail("fogm_model");
    return std::nullopt;
  }
  r.suspend();
  auto f = FogmConfig::from_registry(m, *g);
  r.resume();
  if (!f) return std::nullopt;
  c.fogm_model = *f;
  return c;
}

void ClockBiasStateBlockConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  write_base(w);
  w.scalar("h_0", h_0);
  w.scalar("h_neg2", h_neg2);
  w.optional("q3", q3);
}

std::optional<ClockBiasStateBlockConfig> ClockBiasStateBlockConfig::from_registry(api::Mediator& m,
                                                                                  const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  ClockBiasStateBlockConfig c;
  if (!c.read_base(r)) return std::nullopt;
  c.h_0 = r.require<double>("h_0");
  c.h_neg2 = r.require<double>("h_neg2");
  c.q3 = r.optional<double>("q3");
  if (!r.ok()) return std::nullopt;
  return c;
}

void ConstantStateBlockConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  write_base(w);
  if (Q) w.matrix("Q", *Q);
}

std::optional<ConstantStateBlockConfig> ConstantStateBlockConfig::from_registry(api::Mediator& m,
                                                                                const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  ConstantStateBlockConfig c;
  if (!c.read_base(r)) return std::nullopt;
  c.Q = r.optional<Matrix>("Q");
  if (!r.ok()) return std::nullopt;
  return c;
}

// -------------------------------------------------------------------- MeasurementProcessorConfig

void MeasurementProcessorConfig::write_base(ConfigWriter& w) const {
  w.scalar("identifier", identifier);
  w.scalar("label", label);
  w.strings("state_block_labels", state_block_labels);
  w.scalar("channel", channel);
  if (aux_channels) w.strings("aux_channels", *aux_channels);
  w.optional("innovation_gate_probability", innovation_gate_probability);
  w.optional("geoid_file", geoid_file);
}

bool MeasurementProcessorConfig::read_base(ConfigReader& r) {
  group_ = r.group();
  identifier = r.require<std::string>("identifier");
  label = r.require<std::string>("label");
  state_block_labels = req_strings(r, "state_block_labels");
  channel = r.require<std::string>("channel");
  aux_channels = r.optional<api::StringArray>("aux_channels");
  innovation_gate_probability = r.optional<double>("innovation_gate_probability");
  geoid_file = r.optional<std::string>("geoid_file");
  return r.ok();
}

void MeasurementProcessorConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  write_base(w);
}

std::optional<MeasurementProcessorConfig> MeasurementProcessorConfig::from_registry(api::Mediator& m,
                                                                                    const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  MeasurementProcessorConfig c;
  if (!c.read_base(r)) return std::nullopt;
  return c;
}

LeverArmMPConfig::LeverArmMPConfig(std::string identifier_, std::optional<std::vector<std::string>> aux) {
  identifier = std::move(identifier_);
  aux_channels = std::move(aux);
}

void LeverArmMPConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  write_base(w);
  w.vector("lever_arm", to_vector(lever_arm));
}

std::optional<LeverArmMPConfig> LeverArmMPConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  LeverArmMPConfig c("", std::nullopt);
  if (!c.read_base(r)) return std::nullopt;
  c.lever_arm = req_arr<3>(r, "lever_arm");
  if (!r.ok()) return std::nullopt;
  return c;
}

LeverArmOrientationMPConfig::LeverArmOrientationMPConfig(std::string identifier_,
                                                         std::optional<std::vector<std::string>> aux) {
  identifier = std::move(identifier_);
  aux_channels = std::move(aux);
}

void LeverArmOrientationMPConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  write_base(w);
  w.vector("lever_arm", to_vector(lever_arm));
  w.vector("orientation", to_vector(orientation));
}

std::optional<LeverArmOrientationMPConfig> LeverArmOrientationMPConfig::from_registry(api::Mediator& m,
                                                                                      const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  LeverArmOrientationMPConfig c("", std::nullopt);
  if (!c.read_base(r)) return std::nullopt;
  c.lever_arm = req_arr<3>(r, "lever_arm");
  c.orientation = req_arr<4>(r, "orientation");
  if (!r.ok()) return std::nullopt;
  return c;
}

PlainMPConfig::PlainMPConfig(std::string identifier_, std::optional<std::vector<std::string>> aux) {
  identifier = std::move(identifier_);
  aux_channels = std::move(aux);
}

namespace mp {
namespace {
std::vector<std::string> pva_aux() { return {kAuxInertialPva}; }
}  // namespace
LeverArmMPConfig PinsonPositionMPConfig() { return LeverArmMPConfig(kPinsonPosition, pva_aux()); }
PlainMPConfig PinsonVelocityMPConfig() { return PlainMPConfig(kPinsonVelocity, pva_aux()); }
LeverArmMPConfig PinsonWithNedFogmPositionMPConfig() { return LeverArmMPConfig(kPinsonWithNedFogmPosition, pva_aux()); }
LeverArmMPConfig AltitudeMPConfig() { return LeverArmMPConfig(kPinsonAltitude, pva_aux()); }
LeverArmMPConfig PinsonWithLeverArmPositionMPConfig() {
  return LeverArmMPConfig(kPinsonWithLeverArmPosition, pva_aux());
}
LeverArmOrientationMPConfig PinsonBodyVelocityMPConfig() {
  return LeverArmOrientationMPConfig(kPinsonBodyVelocity,
                                     std::vector<std::string>{kAuxInertialPva, kAuxInertialForcesAndRates});
}
LeverArmMPConfig PosVelMPConfig() { return LeverArmMPConfig(kPinsonPosVel, pva_aux()); }
LeverArmMPConfig PositionMPConfig() { return LeverArmMPConfig(kPosition, std::nullopt); }
LeverArmOrientationMPConfig Direction3dToPointsMPConfig() {
  return LeverArmOrientationMPConfig(kDirection3DToPoints, pva_aux());
}
}  // namespace mp

// ---------------------------------------------------------------------- VirtualStateBlockConfig

void VirtualStateBlockConfig::write_base(ConfigWriter& w) const {
  w.scalar("identifier", identifier);
  w.scalar("source", source);
  w.scalar("target", target);
  if (aux_channels) w.strings("aux_channels", *aux_channels);
}

bool VirtualStateBlockConfig::read_base(ConfigReader& r) {
  group_ = r.group();
  identifier = r.require<std::string>("identifier");
  source = r.require<std::string>("source");
  target = r.require<std::string>("target");
  aux_channels = r.optional<api::StringArray>("aux_channels");
  return r.ok();
}

void VirtualStateBlockConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  write_base(w);
}

std::optional<VirtualStateBlockConfig> VirtualStateBlockConfig::from_registry(api::Mediator& m,
                                                                              const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  VirtualStateBlockConfig c;
  if (!c.read_base(r)) return std::nullopt;
  return c;
}

void StateExtractorConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  write_base(w);
  w.scalar("incoming_state_size", static_cast<std::int64_t>(incoming_state_size));
  std::vector<double> idx(indices_to_extract.begin(), indices_to_extract.end());
  w.matrix("indices_to_extract", to_matrix(idx));
}

std::optional<StateExtractorConfig> StateExtractorConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  StateExtractorConfig c;
  if (!c.read_base(r)) return std::nullopt;
  c.incoming_state_size = static_cast<int>(r.require<std::int64_t>("incoming_state_size"));
  auto idx = r.require<Vector>("indices_to_extract");
  c.indices_to_extract.clear();
  for (Eigen::Index i = 0; i < idx.size(); ++i) c.indices_to_extract.push_back(static_cast<int>(idx(i)));
  if (!r.ok()) return std::nullopt;
  return c;
}

// ----------------------------------------------------------------------------- FusionEngineConfig

void FusionEngineConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("save_x_and_p_after_prop", save_x_and_p_after_prop);
  w.scalar("save_x_and_p_after_update", save_x_and_p_after_update);
  if (innovation_gate_probability > 0) w.scalar("innovation_gate_probability", innovation_gate_probability);
}

std::optional<FusionEngineConfig> FusionEngineConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  FusionEngineConfig c;
  c.group_ = group;
  c.save_x_and_p_after_prop = r.optional<bool>("save_x_and_p_after_prop").value_or(false);
  c.save_x_and_p_after_update = r.optional<bool>("save_x_and_p_after_update").value_or(false);
  c.innovation_gate_probability = r.optional<double>("innovation_gate_probability").value_or(0.0);
  if (!r.ok()) return std::nullopt;
  return c;
}

// ----------------------------------------------------------------------------- ControllerConfig

void ControllerConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("buffer_length_sec", buffer_length_sec);
  w.optional("publish_interval", publish_interval);
  w.scalar("auto_shutdown", auto_shutdown);
}

std::optional<ControllerConfig> ControllerConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  ControllerConfig c;
  c.group_ = group;
  c.buffer_length_sec = r.optional<double>("buffer_length_sec").value_or(kDefaultBufferLengthSec);
  c.publish_interval = r.optional<double>("publish_interval");
  c.auto_shutdown = r.optional<bool>("auto_shutdown").value_or(true);
  if (!r.ok()) return std::nullopt;
  return c;
}

// ----------------------------------------------------------------------------- Stream / StreamConfig

void Stream::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("message_type", static_cast<std::int64_t>(message_type));
  w.optional("source_identifier", source_identifier);
}

std::optional<Stream> Stream::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  Stream s;
  s.group_ = group;
  s.message_type = static_cast<api::AspnMessageType>(r.require<std::int64_t>("message_type"));
  s.source_identifier = r.optional<std::string>("source_identifier");
  if (!r.ok()) return std::nullopt;
  return s;
}

void StreamConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("default_buffer_mode", static_cast<std::int64_t>(default_buffer_mode));
  if (override_streams) {
    std::vector<std::shared_ptr<const BaseConfig>> nested;
    for (const auto& s : *override_streams) nested.push_back(std::make_shared<Stream>(s));
    w.nested("override_streams", nested);
  }
}

std::optional<StreamConfig> StreamConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  StreamConfig c;
  c.group_ = group;
  c.default_buffer_mode = static_cast<BufferMode>(
      r.optional<std::int64_t>("default_buffer_mode").value_or(static_cast<std::int64_t>(BufferMode::SEQUENCED)));
  auto groups = r.nested_groups("override_streams");
  if (!r.ok()) return std::nullopt;
  if (groups) {
    r.suspend();
    std::vector<Stream> streams;
    for (const auto& g : *groups) {
      auto s = Stream::from_registry(m, g);
      if (!s) {
        r.resume();
        return std::nullopt;
      }
      streams.push_back(*s);
    }
    r.resume();
    c.override_streams = std::move(streams);
  }
  return c;
}

StreamConfig default_stream_config() {
  Stream imu;
  imu.group_ = "config/imu_stream";
  imu.message_type = ASPN_MEASUREMENT_IMU;
  StreamConfig c;
  c.group_ = "config/stream_config";
  c.override_streams = std::vector<Stream>{imu};
  return c;
}

// ----------------------------------------------------------------------------- InertialConfig

namespace {
Mat3 to_mat3(const api::Matrix& m) {
  if (m.rows() != 3 || m.cols() != 3) throw std::invalid_argument("expected a 3x3 matrix");
  Mat3 out;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) out[i][j] = m(i, j);
  return out;
}
template <class T>
std::optional<std::vector<std::shared_ptr<const T>>> read_nested_list(
    ConfigReader& r, api::Mediator& m, const std::string& key,
    std::optional<T> (*reader)(api::Mediator&, const std::string&)) {
  auto groups = r.nested_groups(key);
  if (!groups) return std::nullopt;
  std::vector<std::shared_ptr<const T>> out;
  r.suspend();
  for (const auto& g : *groups) {
    auto c = reader(m, g);
    if (!c) {
      r.resume();
      r.fail(key);
      return std::nullopt;
    }
    out.push_back(std::make_shared<const T>(*c));
  }
  r.resume();
  return out;
}
template <class T>
std::vector<std::shared_ptr<const BaseConfig>> as_base(const std::vector<std::shared_ptr<const T>>& v) {
  return std::vector<std::shared_ptr<const BaseConfig>>(v.begin(), v.end());
}
}  // namespace

void InertialConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("expected_dt", expected_dt);
  w.scalar("inertial_buffer_length", inertial_buffer_length);
  w.strings("channels", channels);
  w.matrix("C_imu_to_platform", to_matrix(C_imu_to_platform));
}

std::optional<InertialConfig> InertialConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  InertialConfig c;
  c.group_ = group;
  c.expected_dt = r.require<double>("expected_dt");
  c.inertial_buffer_length = r.require<double>("inertial_buffer_length");
  c.channels = r.require<api::StringArray>("channels");
  auto C = r.require<api::Matrix>("C_imu_to_platform");
  if (!r.ok()) return std::nullopt;
  try {
    c.C_imu_to_platform = to_mat3(C);
  } catch (const std::exception&) {
    r.fail("C_imu_to_platform");
    return std::nullopt;
  }
  return c;
}

// ----------------------------------------------------------------------------- FeedbackConfig

void FeedbackConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("time_threshold", time_threshold);
  w.scalar("pos_error_threshold", pos_error_threshold);
}

std::optional<FeedbackConfig> FeedbackConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  FeedbackConfig c;
  c.group_ = group;
  c.time_threshold = r.optional<double>("time_threshold").value_or(0.0);
  c.pos_error_threshold = r.optional<double>("pos_error_threshold").value_or(0.0);
  if (!r.ok()) return std::nullopt;
  return c;
}

// ----------------------------------------------------------------------------- preprocessors

void PreprocessorConfig::write_base(ConfigWriter& w) const {
  w.scalar("identifier", identifier);
  if (channels) w.strings("channels", *channels);
  w.scalar("regex", regex);
}

bool PreprocessorConfig::read_base(ConfigReader& r) {
  group_ = r.group();
  identifier = r.require<std::string>("identifier");
  channels = r.optional<api::StringArray>("channels");
  regex = r.optional<bool>("regex").value_or(false);
  return r.ok();
}

void PreprocessorConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  write_base(w);
}

std::optional<PreprocessorConfig> PreprocessorConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  PreprocessorConfig c;
  if (!c.read_base(r)) return std::nullopt;
  return c;
}

#define PNTOS_PP_IMPL(Type, WRITE, READ)                                                            \
  void Type::to_registry(api::Mediator& m) const {                                                  \
    ConfigWriter w(m, group_);                                                                      \
    write_base(w);                                                                                  \
    WRITE                                                                                           \
  }                                                                                                 \
  std::optional<Type> Type::from_registry(api::Mediator& m, const std::string& group) {             \
    ConfigReader r(m, group);                                                                       \
    if (!r.ok()) return std::nullopt;                                                               \
    Type c;                                                                                         \
    if (!c.read_base(r)) return std::nullopt;                                                       \
    READ                                                                                            \
    if (!r.ok()) return std::nullopt;                                                               \
    return c;                                                                                       \
  }

PNTOS_PP_IMPL(BarometerToAltitudeConfig, { w.optional("alt_sigma", alt_sigma); },
              { c.alt_sigma = r.optional<double>("alt_sigma"); })
PNTOS_PP_IMPL(
    DownsamplerConfig,
    {
      std::vector<double> f(downsampling_factors.begin(), downsampling_factors.end());
      w.matrix("downsampling_factors", to_matrix(f));
    },
    {
      auto f = r.require<api::Matrix>("downsampling_factors");
      for (Eigen::Index i = 0; i < f.size(); ++i) c.downsampling_factors.push_back(static_cast<std::int64_t>(f(i)));
    })
PNTOS_PP_IMPL(ImuRotatorConfig, { w.matrix("C_imu_to_platform", to_matrix(C_imu_to_platform)); },
              {
                auto C = r.require<api::Matrix>("C_imu_to_platform");
                if (r.ok()) {
                  try {
                    c.C_imu_to_platform = to_mat3(C);
                  } catch (const std::exception&) {
                    r.fail("C_imu_to_platform");
                  }
                }
              })
PNTOS_PP_IMPL(TimeAdjusterConfig, { w.scalar("expected_dt_nsec", expected_dt_nsec); },
              { c.expected_dt_nsec = r.require<std::int64_t>("expected_dt_nsec"); })
PNTOS_PP_IMPL(TimeBiasConfig, { w.scalar("time_bias", time_bias); },
              { c.time_bias = r.require<std::int64_t>("time_bias"); })
PNTOS_PP_IMPL(
    OutageConfig,
    {
      w.scalar("start_time", start_time);
      w.scalar("end_time", end_time);
    },
    {
      c.start_time = r.require<double>("start_time");
      c.end_time = r.require<double>("end_time");
    })
PNTOS_PP_IMPL(
    ZeroVelocity2dGeneratorConfig,
    {
      w.scalar("trigger_dt_sec", trigger_dt_sec);
      w.scalar("lateral_vel_sigma", lateral_vel_sigma);
      w.scalar("vertical_vel_sigma", vertical_vel_sigma);
      w.scalar("output_channel", output_channel);
    },
    {
      c.trigger_dt_sec = r.optional<double>("trigger_dt_sec").value_or(0.0);
      c.lateral_vel_sigma = r.require<double>("lateral_vel_sigma");
      c.vertical_vel_sigma = r.require<double>("vertical_vel_sigma");
      c.output_channel = r.require<std::string>("output_channel");
    })
PNTOS_PP_IMPL(
    SensorDegradationConfig,
    {
      w.scalar("seed", seed);
      w.scalar("imu_expected_dt", imu_expected_dt);
      w.vector("accel_noise_density", to_vector(accel_noise_density));
      w.vector("gyro_noise_density", to_vector(gyro_noise_density));
      w.vector("accel_bias", to_vector(accel_bias));
      w.vector("gyro_bias", to_vector(gyro_bias));
      w.vector("position_noise_sigma_ned", to_vector(position_noise_sigma_ned));
      w.scalar("position_covariance_scale", position_covariance_scale);
      w.vector("velocity_noise_sigma", to_vector(velocity_noise_sigma));
      w.scalar("velocity_covariance_scale", velocity_covariance_scale);
      api::Matrix jumps(static_cast<Eigen::Index>(position_jumps.size()), 4);
      for (std::size_t i = 0; i < position_jumps.size(); ++i)
        for (int k = 0; k < 4; ++k) jumps(static_cast<Eigen::Index>(i), k) = position_jumps[i][static_cast<std::size_t>(k)];
      if (!position_jumps.empty()) w.matrix("position_jumps", jumps);
    },
    {
      c.seed = r.optional<std::int64_t>("seed").value_or(1);
      c.imu_expected_dt = r.optional<double>("imu_expected_dt").value_or(0.01);
      c.accel_noise_density = read_arr<3>(r, "accel_noise_density").value_or(Vec3{0, 0, 0});
      c.gyro_noise_density = read_arr<3>(r, "gyro_noise_density").value_or(Vec3{0, 0, 0});
      c.accel_bias = read_arr<3>(r, "accel_bias").value_or(Vec3{0, 0, 0});
      c.gyro_bias = read_arr<3>(r, "gyro_bias").value_or(Vec3{0, 0, 0});
      c.position_noise_sigma_ned = read_arr<3>(r, "position_noise_sigma_ned").value_or(Vec3{0, 0, 0});
      c.position_covariance_scale = r.optional<double>("position_covariance_scale").value_or(1.0);
      c.velocity_noise_sigma = read_arr<3>(r, "velocity_noise_sigma").value_or(Vec3{0, 0, 0});
      c.velocity_covariance_scale = r.optional<double>("velocity_covariance_scale").value_or(1.0);
      if (auto m = r.optional<api::Matrix>("position_jumps"); m && m->cols() == 4)
        for (Eigen::Index i = 0; i < m->rows(); ++i)
          c.position_jumps.push_back({(*m)(i, 0), (*m)(i, 1), (*m)(i, 2), (*m)(i, 3)});
    })
#undef PNTOS_PP_IMPL

// ----------------------------------------------------------------------------- alignment configs

void ManualAlignmentConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.vector("initial_pos", to_vector(initial_pos));
  w.vector("initial_vel", to_vector(initial_vel));
  w.vector("initial_rpy", to_vector(initial_rpy));
  w.vector("initial_accel_bias", to_vector(initial_accel_bias));
  w.vector("initial_gyro_bias", to_vector(initial_gyro_bias));
  w.vector("initial_accel_scale_factor", to_vector(initial_accel_scale_factor));
  w.vector("initial_gyro_scale_factor", to_vector(initial_gyro_scale_factor));
  w.scalar("initial_time", initial_time);
  w.vector("initial_pos_var", to_vector(initial_pos_var));
  w.vector("initial_vel_var", to_vector(initial_vel_var));
  w.vector("initial_tilt_var", to_vector(initial_tilt_var));
  w.vector("initial_accel_bias_var", to_vector(initial_accel_bias_var));
  w.vector("initial_gyro_bias_var", to_vector(initial_gyro_bias_var));
  w.vector("initial_accel_scale_factor_var", to_vector(initial_accel_scale_factor_var));
  w.vector("initial_gyro_scale_factor_var", to_vector(initial_gyro_scale_factor_var));
}

std::optional<ManualAlignmentConfig> ManualAlignmentConfig::from_registry(api::Mediator& m,
                                                                          const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  ManualAlignmentConfig c;
  c.group_ = group;
  c.initial_pos = req_arr<3>(r, "initial_pos");
  c.initial_vel = req_arr<3>(r, "initial_vel");
  c.initial_rpy = req_arr<3>(r, "initial_rpy");
  c.initial_accel_bias = req_arr<3>(r, "initial_accel_bias");
  c.initial_gyro_bias = req_arr<3>(r, "initial_gyro_bias");
  c.initial_accel_scale_factor = req_arr<3>(r, "initial_accel_scale_factor");
  c.initial_gyro_scale_factor = req_arr<3>(r, "initial_gyro_scale_factor");
  c.initial_time = r.require<double>("initial_time");
  c.initial_pos_var = req_arr<3>(r, "initial_pos_var");
  c.initial_vel_var = req_arr<3>(r, "initial_vel_var");
  c.initial_tilt_var = req_arr<3>(r, "initial_tilt_var");
  c.initial_accel_bias_var = req_arr<3>(r, "initial_accel_bias_var");
  c.initial_gyro_bias_var = req_arr<3>(r, "initial_gyro_bias_var");
  c.initial_accel_scale_factor_var = req_arr<3>(r, "initial_accel_scale_factor_var");
  c.initial_gyro_scale_factor_var = req_arr<3>(r, "initial_gyro_scale_factor_var");
  if (!r.ok()) return std::nullopt;
  return c;
}

void StaticAlignmentConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("static_time", static_time);
  w.nested("imu_model", imu_model);
}

std::optional<StaticAlignmentConfig> StaticAlignmentConfig::from_registry(api::Mediator& m,
                                                                          const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  StaticAlignmentConfig c;
  c.group_ = group;
  c.static_time = r.require<double>("static_time");
  auto g = r.nested_group("imu_model");
  if (!g) {
    r.fail("imu_model");
    return std::nullopt;
  }
  r.suspend();
  auto imu = ImuConfig::from_registry(m, *g);
  r.resume();
  if (!imu || !r.ok()) return std::nullopt;
  c.imu_model = *imu;
  return c;
}

void ManualHeadingAlignmentConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("static_time", static_time);
  w.nested("imu_model", imu_model);
  w.scalar("heading", heading);
  w.scalar("heading_sigma", heading_sigma);
}

std::optional<ManualHeadingAlignmentConfig> ManualHeadingAlignmentConfig::from_registry(api::Mediator& m,
                                                                                        const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  ManualHeadingAlignmentConfig c;
  c.group_ = group;
  c.static_time = r.require<double>("static_time");
  c.heading = r.require<double>("heading");
  c.heading_sigma = r.require<double>("heading_sigma");
  auto g = r.nested_group("imu_model");
  if (!g) {
    r.fail("imu_model");
    return std::nullopt;
  }
  r.suspend();
  auto imu = ImuConfig::from_registry(m, *g);
  r.resume();
  if (!imu || !r.ok()) return std::nullopt;
  c.imu_model = *imu;
  return c;
}

// ----------------------------------------------------------------------------- StandardOrchestrationConfig

void StandardOrchestrationConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("best_sol_channel", best_sol_channel);
  w.scalar("imu_sol_channel", imu_sol_channel);
  w.strings("alignment_channels", alignment_channels);
  w.nested("pinson_sb_config", pinson_sb_config);
  if (additional_sb_configs) w.nested("additional_sb_configs", as_base(*additional_sb_configs));
  if (vsb_configs) w.nested("vsb_configs", as_base(*vsb_configs));
  if (mp_configs) w.nested("mp_configs", as_base(*mp_configs));
  w.nested("inertial_config", inertial_config);
  if (feedback_config) w.nested("feedback_config", *feedback_config);
  if (alignment_config) w.nested("alignment_config", *alignment_config);
  if (preprocessor_configs) w.nested("preprocessor_configs", as_base(*preprocessor_configs));
  w.scalar("max_prop_interval", max_prop_interval);
  w.scalar("publish_before_update", publish_before_update);
  w.scalar("publish_after_update", publish_after_update);
  w.scalar("max_filter_lag", max_filter_lag);
  w.nested("stream_config", stream_config);
}

std::optional<StandardOrchestrationConfig> StandardOrchestrationConfig::from_registry(api::Mediator& m,
                                                                                      const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  StandardOrchestrationConfig c;
  c.group_ = group;
  c.best_sol_channel = r.require<std::string>("best_sol_channel");
  c.imu_sol_channel = r.require<std::string>("imu_sol_channel");
  c.alignment_channels = r.require<api::StringArray>("alignment_channels");
  c.max_prop_interval = r.optional<double>("max_prop_interval").value_or(2.0);
  c.publish_before_update = r.optional<bool>("publish_before_update").value_or(false);
  c.publish_after_update = r.optional<bool>("publish_after_update").value_or(false);
  c.max_filter_lag = r.optional<double>("max_filter_lag").value_or(ControllerConfig::kDefaultBufferLengthSec);
  if (!r.ok()) return std::nullopt;

  auto single = [&](const char* key, bool required) -> std::optional<std::string> {
    auto g = r.nested_group(key);
    if (!g && required) r.fail(key);
    return g;
  };
  auto pinson_g = single("pinson_sb_config", true);
  auto inertial_g = single("inertial_config", true);
  auto align_g = single("alignment_config", true);
  auto feedback_g = single("feedback_config", false);
  auto stream_g = single("stream_config", false);
  if (!r.ok()) return std::nullopt;
  c.alignment_config_group = *align_g;

  r.suspend();
  auto pinson = PinsonStateBlockConfig::from_registry(m, *pinson_g);
  auto inertial = InertialConfig::from_registry(m, *inertial_g);
  std::optional<FeedbackConfig> feedback;
  if (feedback_g) feedback = FeedbackConfig::from_registry(m, *feedback_g);
  std::optional<StreamConfig> stream;
  if (stream_g) stream = StreamConfig::from_registry(m, *stream_g);
  r.resume();
  if (!pinson || !inertial || (feedback_g && !feedback) || (stream_g && !stream)) {
    r.fail("nested configs");
    return std::nullopt;
  }
  c.pinson_sb_config = *pinson;
  c.inertial_config = *inertial;
  c.feedback_config = feedback;
  if (stream) c.stream_config = *stream;

  c.additional_sb_configs = read_nested_list<StateBlockConfig>(r, m, "additional_sb_configs",
                                                               &StateBlockConfig::from_registry);
  c.vsb_configs = read_nested_list<VirtualStateBlockConfig>(r, m, "vsb_configs",
                                                            &VirtualStateBlockConfig::from_registry);
  c.mp_configs = read_nested_list<MeasurementProcessorConfig>(r, m, "mp_configs",
                                                              &MeasurementProcessorConfig::from_registry);
  c.preprocessor_configs = read_nested_list<PreprocessorConfig>(r, m, "preprocessor_configs",
                                                                &PreprocessorConfig::from_registry);
  if (!r.ok()) return std::nullopt;
  return c;
}

// ----------------------------------------------------------------------------- PvaMessageInitializationConfig

void PvaMessageInitializationConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("initial_pva_channel", initial_pva_channel);
  w.vector("initial_accel_bias_sigma", to_vector(initial_accel_bias_sigma));
  w.vector("initial_gyro_bias_sigma", to_vector(initial_gyro_bias_sigma));
  if (initial_pva_sigma) {
    std::vector<double> v(initial_pva_sigma->begin(), initial_pva_sigma->end());
    w.matrix("initial_pva_sigma", to_matrix(v));
  }
  w.optional("start_time", start_time);
}

std::optional<PvaMessageInitializationConfig> PvaMessageInitializationConfig::from_registry(api::Mediator& m,
                                                                                            const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  PvaMessageInitializationConfig c;
  c.group_ = group;
  c.initial_pva_channel = r.require<std::string>("initial_pva_channel");
  c.initial_accel_bias_sigma = req_arr<3>(r, "initial_accel_bias_sigma");
  c.initial_gyro_bias_sigma = req_arr<3>(r, "initial_gyro_bias_sigma");
  c.initial_pva_sigma = read_arr<9>(r, "initial_pva_sigma");
  c.start_time = r.optional<double>("start_time");
  if (!r.ok()) return std::nullopt;
  return c;
}

// ----------------------------------------------------------------------------- transport configs

void LcmLogTransportConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.optional("input_file", input_file);
  w.optional("output_file", output_file);
  if (channels_to_process) w.strings("channels_to_process", *channels_to_process);
  w.scalar("record_input_channels", record_input_channels);
}

std::optional<LcmLogTransportConfig> LcmLogTransportConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  LcmLogTransportConfig c;
  c.group_ = group;
  c.input_file = r.optional<std::string>("input_file");
  c.output_file = r.optional<std::string>("output_file");
  c.channels_to_process = r.optional<api::StringArray>("channels_to_process");
  c.record_input_channels = r.optional<bool>("record_input_channels").value_or(true);
  if (!r.ok()) return std::nullopt;
  return c;
}

void LcmTransportConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("url", url);
  w.scalar("subscribe_to", subscribe_to);
}

std::optional<LcmTransportConfig> LcmTransportConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  LcmTransportConfig c;
  c.group_ = group;
  c.url = r.optional<std::string>("url").value_or("tcpq://");
  c.subscribe_to = r.optional<std::string>("subscribe_to").value_or("^((?!pntos).)*$");
  if (!r.ok()) return std::nullopt;
  return c;
}

}  // namespace pntos::cobra

// ----------------------------------------------------------------------------- tutorial / UI configs

namespace pntos::cobra {

void TutorialOrchestrationConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("position_channel", position_channel);
  w.scalar("velocity_channel", velocity_channel);
}

std::optional<TutorialOrchestrationConfig> TutorialOrchestrationConfig::from_registry(api::Mediator& m,
                                                                                      const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  TutorialOrchestrationConfig c;
  c.group_ = group;
  c.position_channel = r.require<std::string>("position_channel");
  c.velocity_channel = r.optional<std::string>("velocity_channel").value_or("unused");
  if (!r.ok()) return std::nullopt;
  return c;
}

void UiLogPlottingConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("logfile", logfile);
  w.scalar("solution_channel", solution_channel);
  w.scalar("truth_channel", truth_channel);
}

std::optional<UiLogPlottingConfig> UiLogPlottingConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  UiLogPlottingConfig c;
  c.group_ = group;
  c.logfile = r.require<std::string>("logfile");
  c.solution_channel = r.require<std::string>("solution_channel");
  c.truth_channel = r.require<std::string>("truth_channel");
  if (!r.ok()) return std::nullopt;
  return c;
}

}  // namespace pntos::cobra
