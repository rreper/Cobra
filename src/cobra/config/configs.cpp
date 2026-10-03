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
}

bool MeasurementProcessorConfig::read_base(ConfigReader& r) {
  group_ = r.group();
  identifier = r.require<std::string>("identifier");
  label = r.require<std::string>("label");
  state_block_labels = req_strings(r, "state_block_labels");
  channel = r.require<std::string>("channel");
  aux_channels = r.optional<api::StringArray>("aux_channels");
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
}

std::optional<FusionEngineConfig> FusionEngineConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  FusionEngineConfig c;
  c.group_ = group;
  c.save_x_and_p_after_prop = r.optional<bool>("save_x_and_p_after_prop").value_or(false);
  c.save_x_and_p_after_update = r.optional<bool>("save_x_and_p_after_update").value_or(false);
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

}  // namespace pntos::cobra
