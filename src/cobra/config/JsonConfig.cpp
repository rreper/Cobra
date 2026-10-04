#include <pntos/cobra/config/JsonConfig.hpp>

#include <pntos/cobra/presets/Presets.hpp>

#include <fstream>
#include <map>
#include <stdexcept>

namespace pntos::cobra::jsoncfg {

namespace {

[[noreturn]] void fail(const std::string& what) { throw std::runtime_error("config JSON: " + what); }

const json& need(const json& j, const char* key) {
  if (!j.contains(key)) fail(std::string("missing field \"") + key + "\" in " + j.value("type", "object"));
  return j.at(key);
}

template <class T>
T get_or(const json& j, const char* key, const T& fallback) {
  if (!j.contains(key) || j.at(key).is_null()) return fallback;
  return j.at(key).get<T>();
}

std::string group_of(const json& j) { return get_or<std::string>(j, "group", ""); }

Vec3 vec3(const json& j, const char* key) {
  const json& a = need(j, key);
  if (!a.is_array() || a.size() != 3) fail(std::string("\"") + key + "\" must be an array of 3 numbers");
  return {a[0].get<double>(), a[1].get<double>(), a[2].get<double>()};
}
Vec4 vec4(const json& j, const char* key) {
  const json& a = need(j, key);
  if (!a.is_array() || a.size() != 4) fail(std::string("\"") + key + "\" must be an array of 4 numbers");
  return {a[0].get<double>(), a[1].get<double>(), a[2].get<double>(), a[3].get<double>()};
}
Mat3 mat3(const json& j, const char* key) {
  const json& a = need(j, key);
  if (!a.is_array() || a.size() != 3) fail(std::string("\"") + key + "\" must be a 3x3 array");
  Mat3 m{};
  for (std::size_t r = 0; r < 3; ++r) {
    if (!a[r].is_array() || a[r].size() != 3) fail(std::string("\"") + key + "\" must be a 3x3 array");
    for (std::size_t c = 0; c < 3; ++c) m[r][c] = a[r][c].get<double>();
  }
  return m;
}
json j(const Vec3& v) { return json::array({v[0], v[1], v[2]}); }
json j(const Vec4& v) { return json::array({v[0], v[1], v[2], v[3]}); }
json j(const Mat3& m) { return json::array({j(m[0]), j(m[1]), j(m[2])}); }

api::Matrix matrix(const json& a, const char* key) {
  if (!a.is_array()) fail(std::string("\"") + key + "\" must be an array");
  if (a.empty()) return api::Matrix(0, 0);
  if (!a[0].is_array()) {  // flat vector -> column
    api::Vector v(static_cast<Eigen::Index>(a.size()));
    for (std::size_t i = 0; i < a.size(); ++i) v(static_cast<Eigen::Index>(i)) = a[i].get<double>();
    return api::Matrix(v);
  }
  api::Matrix m(static_cast<Eigen::Index>(a.size()), static_cast<Eigen::Index>(a[0].size()));
  for (std::size_t r = 0; r < a.size(); ++r) {
    if (a[r].size() != a[0].size()) fail(std::string("\"") + key + "\" rows differ in length");
    for (std::size_t c = 0; c < a[r].size(); ++c) m(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = a[r][c].get<double>();
  }
  return m;
}
json j(const api::Matrix& m) {
  json out = json::array();
  for (Eigen::Index r = 0; r < m.rows(); ++r) {
    json row = json::array();
    for (Eigen::Index c = 0; c < m.cols(); ++c) row.push_back(m(r, c));
    out.push_back(row);
  }
  return out;
}
json jvec(const api::Vector& v) {
  json out = json::array();
  for (Eigen::Index i = 0; i < v.size(); ++i) out.push_back(v(i));
  return out;
}

std::vector<double> doubles(const json& j, const char* key) { return need(j, key).get<std::vector<double>>(); }
std::optional<std::vector<std::string>> opt_strings(const json& j, const char* key) {
  if (!j.contains(key) || j.at(key).is_null()) return std::nullopt;
  return j.at(key).get<std::vector<std::string>>();
}

api::EstimateWithCovariance ewc_from(const json& e) {
  api::EstimateWithCovariance out;
  const std::string t = get_or<std::string>(e, "type", "EWC_GENERIC");
  if (t == "EWC_GENERIC") out.type = api::EstimateWithCovarianceType::EWC_GENERIC;
  else if (t == "EWC_ATTITUDE_QUAT") out.type = api::EstimateWithCovarianceType::EWC_ATTITUDE_QUAT;
  else fail("unknown estimate_with_covariance type " + t);
  out.estimate = api::Vector(matrix(need(e, "estimate"), "estimate"));
  if (e.contains("covariance_diag")) {
    const api::Vector d = api::Vector(matrix(e.at("covariance_diag"), "covariance_diag"));
    out.covariance = api::Matrix(d.asDiagonal());
  } else {
    out.covariance = matrix(need(e, "covariance"), "covariance");
  }
  return out;
}
json ewc_to(const api::EstimateWithCovariance& e) {
  json out;
  out["type"] = e.type == api::EstimateWithCovarianceType::EWC_GENERIC ? "EWC_GENERIC" : "EWC_ATTITUDE_QUAT";
  out["estimate"] = jvec(e.estimate);
  if (e.covariance.isDiagonal(0.0))
    out["covariance_diag"] = jvec(e.covariance.diagonal());
  else
    out["covariance"] = j(e.covariance);
  return out;
}

const std::map<std::string, api::AspnMessageType>& message_types() {
  static const std::map<std::string, api::AspnMessageType> m = {
      {"MEASUREMENT_IMU", ASPN_MEASUREMENT_IMU},
      {"MEASUREMENT_POSITION", ASPN_MEASUREMENT_POSITION},
      {"MEASUREMENT_VELOCITY", ASPN_MEASUREMENT_VELOCITY},
      {"MEASUREMENT_POSITION_VELOCITY_ATTITUDE", ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE},
      {"MEASUREMENT_ALTITUDE", ASPN_MEASUREMENT_ALTITUDE},
      {"MEASUREMENT_BAROMETER", ASPN_MEASUREMENT_BAROMETER},
      {"MEASUREMENT_DIRECTION_3D_TO_POINTS", ASPN_MEASUREMENT_DIRECTION_3D_TO_POINTS},
  };
  return m;
}

// ------------------------------------------------------------------ leaf configs

ImuConfig imu_from(const json& jc) {
  ImuConfig c;
  const std::string group = group_of(jc);
  if (jc.contains("preset")) {
    auto p = presets::imu_preset(jc.at("preset").get<std::string>(), group);
    if (!p) fail("unknown IMU preset \"" + jc.at("preset").get<std::string>() + "\"");
    c = *p;
  } else {
    c.group_ = group;
    c.accel_bias_sigma = vec3(jc, "accel_bias_sigma");
    c.accel_bias_tau = vec3(jc, "accel_bias_tau");
    c.accel_random_walk_sigma = vec3(jc, "accel_random_walk_sigma");
    c.gyro_bias_sigma = vec3(jc, "gyro_bias_sigma");
    c.gyro_bias_tau = vec3(jc, "gyro_bias_tau");
    c.gyro_random_walk_sigma = vec3(jc, "gyro_random_walk_sigma");
  }
  // overrides (also on top of a preset)
  auto over = [&](const char* key, Vec3& field) {
    if (jc.contains(key) && jc.contains("preset")) field = vec3(jc, key);
  };
  over("accel_bias_sigma", c.accel_bias_sigma);
  over("accel_bias_tau", c.accel_bias_tau);
  over("accel_random_walk_sigma", c.accel_random_walk_sigma);
  over("gyro_bias_sigma", c.gyro_bias_sigma);
  over("gyro_bias_tau", c.gyro_bias_tau);
  over("gyro_random_walk_sigma", c.gyro_random_walk_sigma);
  if (jc.contains("accel_bias_initial_sigma")) c.accel_bias_initial_sigma = vec3(jc, "accel_bias_initial_sigma");
  if (jc.contains("gyro_bias_initial_sigma")) c.gyro_bias_initial_sigma = vec3(jc, "gyro_bias_initial_sigma");
  return c;
}
json imu_to(const ImuConfig& c) {
  json o;
  o["type"] = "ImuConfig";
  o["group"] = c.group_;
  if (const std::string preset = presets::matching_imu_preset(c); !preset.empty()) {
    // Readable and swappable between Pinson-Q modes (app::apply_overrides); the initial sigmas are
    // written when they differ from the preset's.
    o["preset"] = preset;
    const ImuConfig p = *presets::imu_preset(preset, c.group_);
    if (c.accel_bias_initial_sigma != p.accel_bias_initial_sigma) o["accel_bias_initial_sigma"] = j(c.accel_bias_initial_sigma);
    if (c.gyro_bias_initial_sigma != p.gyro_bias_initial_sigma) o["gyro_bias_initial_sigma"] = j(c.gyro_bias_initial_sigma);
    return o;
  }
  o["accel_bias_sigma"] = j(c.accel_bias_sigma);
  o["accel_bias_tau"] = j(c.accel_bias_tau);
  o["accel_random_walk_sigma"] = j(c.accel_random_walk_sigma);
  o["gyro_bias_sigma"] = j(c.gyro_bias_sigma);
  o["gyro_bias_tau"] = j(c.gyro_bias_tau);
  o["gyro_random_walk_sigma"] = j(c.gyro_random_walk_sigma);
  o["accel_bias_initial_sigma"] = j(c.accel_bias_initial_sigma);
  o["gyro_bias_initial_sigma"] = j(c.gyro_bias_initial_sigma);
  return o;
}

FogmConfig fogm_from(const json& jc) {
  FogmConfig c;
  c.group_ = group_of(jc);
  c.sigma = doubles(jc, "sigma");
  c.tau = doubles(jc, "tau");
  return c;
}
json fogm_to(const FogmConfig& c) {
  return json{{"type", "FogmConfig"}, {"group", c.group_}, {"sigma", c.sigma}, {"tau", c.tau}};
}

InertialConfig inertial_from(const json& jc) {
  InertialConfig c;
  c.group_ = group_of(jc);
  c.expected_dt = get_or(jc, "expected_dt", 0.01);
  c.inertial_buffer_length = get_or(jc, "inertial_buffer_length", 10.0);
  c.channels = need(jc, "channels").get<std::vector<std::string>>();
  if (jc.contains("C_imu_to_platform")) c.C_imu_to_platform = mat3(jc, "C_imu_to_platform");
  return c;
}
json inertial_to(const InertialConfig& c) {
  return json{{"type", "InertialConfig"}, {"group", c.group_}, {"expected_dt", c.expected_dt},
              {"inertial_buffer_length", c.inertial_buffer_length}, {"channels", c.channels},
              {"C_imu_to_platform", j(c.C_imu_to_platform)}};
}

FeedbackConfig feedback_from(const json& jc) {
  FeedbackConfig c;
  c.group_ = group_of(jc);
  c.time_threshold = get_or(jc, "time_threshold", 0.0);
  c.pos_error_threshold = get_or(jc, "pos_error_threshold", 0.0);
  return c;
}
json feedback_to(const FeedbackConfig& c) {
  return json{{"type", "FeedbackConfig"}, {"group", c.group_}, {"time_threshold", c.time_threshold},
              {"pos_error_threshold", c.pos_error_threshold}};
}

Stream stream_from(const json& jc) {
  Stream s;
  s.group_ = group_of(jc);
  const json& mt = need(jc, "message_type");
  s.message_type = mt.is_string() ? message_type_from_name(mt.get<std::string>())
                                  : static_cast<api::AspnMessageType>(mt.get<int>());
  if (jc.contains("source_identifier") && !jc.at("source_identifier").is_null())
    s.source_identifier = jc.at("source_identifier").get<std::string>();
  return s;
}
json stream_to(const Stream& s) {
  json o{{"type", "Stream"}, {"group", s.group_}, {"message_type", message_type_name(s.message_type)}};
  if (s.source_identifier) o["source_identifier"] = *s.source_identifier;
  return o;
}

StreamConfig stream_config_from(const json& jc) {
  StreamConfig c = default_stream_config();
  if (jc.contains("group")) c.group_ = group_of(jc);
  const std::string mode = get_or<std::string>(jc, "default_buffer_mode", "SEQUENCED");
  if (mode == "SEQUENCED") c.default_buffer_mode = BufferMode::SEQUENCED;
  else if (mode == "IMMEDIATE") c.default_buffer_mode = BufferMode::IMMEDIATE;
  else fail("default_buffer_mode must be SEQUENCED or IMMEDIATE");
  if (jc.contains("override_streams")) {
    std::vector<Stream> streams;
    for (const auto& s : jc.at("override_streams")) streams.push_back(stream_from(s));
    c.override_streams = streams;
  }
  return c;
}
json stream_config_to(const StreamConfig& c) {
  json o{{"type", "StreamConfig"}, {"group", c.group_},
         {"default_buffer_mode", c.default_buffer_mode == BufferMode::SEQUENCED ? "SEQUENCED" : "IMMEDIATE"}};
  if (c.override_streams) {
    json arr = json::array();
    for (const auto& s : *c.override_streams) arr.push_back(stream_to(s));
    o["override_streams"] = arr;
  }
  return o;
}

// ------------------------------------------------------------------ state blocks

void sb_base_from(const json& jc, StateBlockConfig& c) {
  c.group_ = group_of(jc);
  c.label = need(jc, "label").get<std::string>();
  if (jc.contains("estimate_with_covariance") && !jc.at("estimate_with_covariance").is_null())
    c.estimate_with_covariance = ewc_from(jc.at("estimate_with_covariance"));
  c.aux_channels = opt_strings(jc, "aux_channels");
}
void sb_base_to(const StateBlockConfig& c, json& o) {
  o["group"] = c.group_;
  o["label"] = c.label;
  if (c.estimate_with_covariance) o["estimate_with_covariance"] = ewc_to(*c.estimate_with_covariance);
  if (c.aux_channels) o["aux_channels"] = *c.aux_channels;
}

// ------------------------------------------------------------------ measurement processors

struct MpKind {
  const char* type;
  const char* identifier;
  enum { LeverArm, LeverArmOrientation, Plain } shape;
};
const std::vector<MpKind>& mp_kinds() {
  static const std::vector<MpKind> k = {
      {"PinsonPositionMPConfig", mp::kPinsonPosition, MpKind::LeverArm},
      {"PinsonVelocityMPConfig", mp::kPinsonVelocity, MpKind::Plain},
      {"PinsonWithNedFogmPositionMPConfig", mp::kPinsonWithNedFogmPosition, MpKind::LeverArm},
      {"AltitudeMPConfig", mp::kPinsonAltitude, MpKind::LeverArm},
      {"PinsonWithLeverArmPositionMPConfig", mp::kPinsonWithLeverArmPosition, MpKind::LeverArm},
      {"PinsonBodyVelocityMPConfig", mp::kPinsonBodyVelocity, MpKind::LeverArmOrientation},
      {"PosVelMPConfig", mp::kPinsonPosVel, MpKind::LeverArm},
      {"PositionMPConfig", mp::kPosition, MpKind::LeverArm},
      {"Direction3dToPointsMPConfig", mp::kDirection3DToPoints, MpKind::LeverArmOrientation},
  };
  return k;
}
template <class T>
T mp_factory(const char* identifier);
template <>
LeverArmMPConfig mp_factory<LeverArmMPConfig>(const char* id) {
  const std::string s = id;
  if (s == mp::kPinsonPosition) return mp::PinsonPositionMPConfig();
  if (s == mp::kPinsonWithNedFogmPosition) return mp::PinsonWithNedFogmPositionMPConfig();
  if (s == mp::kPinsonAltitude) return mp::AltitudeMPConfig();
  if (s == mp::kPinsonWithLeverArmPosition) return mp::PinsonWithLeverArmPositionMPConfig();
  if (s == mp::kPinsonPosVel) return mp::PosVelMPConfig();
  return mp::PositionMPConfig();
}
template <>
LeverArmOrientationMPConfig mp_factory<LeverArmOrientationMPConfig>(const char* id) {
  return std::string(id) == mp::kPinsonBodyVelocity ? mp::PinsonBodyVelocityMPConfig() : mp::Direction3dToPointsMPConfig();
}

void mp_base_from(const json& jc, MeasurementProcessorConfig& c) {
  c.group_ = group_of(jc);
  c.label = need(jc, "label").get<std::string>();
  c.channel = need(jc, "channel").get<std::string>();
  c.state_block_labels = need(jc, "state_block_labels").get<std::vector<std::string>>();
  if (jc.contains("aux_channels")) c.aux_channels = opt_strings(jc, "aux_channels");
  if (jc.contains("innovation_gate_probability") && !jc.at("innovation_gate_probability").is_null())
    c.innovation_gate_probability = jc.at("innovation_gate_probability").get<double>();
  if (jc.contains("geoid_file") && !jc.at("geoid_file").is_null()) c.geoid_file = jc.at("geoid_file").get<std::string>();
}
void mp_base_to(const MeasurementProcessorConfig& c, json& o) {
  o["group"] = c.group_;
  o["label"] = c.label;
  o["channel"] = c.channel;
  o["state_block_labels"] = c.state_block_labels;
  if (c.aux_channels) o["aux_channels"] = *c.aux_channels;
  if (c.innovation_gate_probability) o["innovation_gate_probability"] = *c.innovation_gate_probability;
  if (c.geoid_file) o["geoid_file"] = *c.geoid_file;
}

std::shared_ptr<BaseConfig> mp_from(const MpKind& k, const json& jc) {
  switch (k.shape) {
    case MpKind::LeverArm: {
      auto c = std::make_shared<LeverArmMPConfig>(mp_factory<LeverArmMPConfig>(k.identifier));
      mp_base_from(jc, *c);
      c->lever_arm = vec3(jc, "lever_arm");
      return c;
    }
    case MpKind::LeverArmOrientation: {
      auto c = std::make_shared<LeverArmOrientationMPConfig>(mp_factory<LeverArmOrientationMPConfig>(k.identifier));
      mp_base_from(jc, *c);
      c->lever_arm = vec3(jc, "lever_arm");
      if (jc.contains("orientation")) c->orientation = vec4(jc, "orientation");
      return c;
    }
    default: {
      auto c = std::make_shared<PlainMPConfig>(mp::PinsonVelocityMPConfig());
      mp_base_from(jc, *c);
      return c;
    }
  }
}

// ------------------------------------------------------------------ preprocessors

void pp_base_from(const json& jc, PreprocessorConfig& c) {
  c.group_ = group_of(jc);
  c.channels = opt_strings(jc, "channels");
  c.regex = get_or(jc, "regex", false);
}
void pp_base_to(const PreprocessorConfig& c, json& o) {
  o["group"] = c.group_;
  if (c.channels) o["channels"] = *c.channels;
  if (c.regex) o["regex"] = true;
}

// ------------------------------------------------------------------ alignment

ManualAlignmentConfig manual_from(const json& jc) {
  ManualAlignmentConfig c;
  c.group_ = group_of(jc);
  c.initial_pos = vec3(jc, "initial_pos");
  c.initial_vel = vec3(jc, "initial_vel");
  c.initial_rpy = vec3(jc, "initial_rpy");
  c.initial_accel_bias = vec3(jc, "initial_accel_bias");
  c.initial_gyro_bias = vec3(jc, "initial_gyro_bias");
  if (jc.contains("initial_accel_scale_factor")) c.initial_accel_scale_factor = vec3(jc, "initial_accel_scale_factor");
  if (jc.contains("initial_gyro_scale_factor")) c.initial_gyro_scale_factor = vec3(jc, "initial_gyro_scale_factor");
  c.initial_time = need(jc, "initial_time").get<double>();
  c.initial_pos_var = vec3(jc, "initial_pos_var");
  c.initial_vel_var = vec3(jc, "initial_vel_var");
  c.initial_tilt_var = vec3(jc, "initial_tilt_var");
  c.initial_accel_bias_var = vec3(jc, "initial_accel_bias_var");
  c.initial_gyro_bias_var = vec3(jc, "initial_gyro_bias_var");
  if (jc.contains("initial_accel_scale_factor_var")) c.initial_accel_scale_factor_var = vec3(jc, "initial_accel_scale_factor_var");
  if (jc.contains("initial_gyro_scale_factor_var")) c.initial_gyro_scale_factor_var = vec3(jc, "initial_gyro_scale_factor_var");
  return c;
}
json manual_to(const ManualAlignmentConfig& c) {
  json o{{"type", "ManualAlignmentConfig"}, {"group", c.group_}};
  o["initial_pos"] = j(c.initial_pos);
  o["initial_vel"] = j(c.initial_vel);
  o["initial_rpy"] = j(c.initial_rpy);
  o["initial_accel_bias"] = j(c.initial_accel_bias);
  o["initial_gyro_bias"] = j(c.initial_gyro_bias);
  o["initial_accel_scale_factor"] = j(c.initial_accel_scale_factor);
  o["initial_gyro_scale_factor"] = j(c.initial_gyro_scale_factor);
  o["initial_time"] = c.initial_time;
  o["initial_pos_var"] = j(c.initial_pos_var);
  o["initial_vel_var"] = j(c.initial_vel_var);
  o["initial_tilt_var"] = j(c.initial_tilt_var);
  o["initial_accel_bias_var"] = j(c.initial_accel_bias_var);
  o["initial_gyro_bias_var"] = j(c.initial_gyro_bias_var);
  o["initial_accel_scale_factor_var"] = j(c.initial_accel_scale_factor_var);
  o["initial_gyro_scale_factor_var"] = j(c.initial_gyro_scale_factor_var);
  return o;
}

// ------------------------------------------------------------------ orchestration

template <class T>
std::vector<std::shared_ptr<const T>> list_from(const json& jc, const char* key) {
  std::vector<std::shared_ptr<const T>> out;
  for (const auto& e : need(jc, key)) {
    auto c = std::dynamic_pointer_cast<T>(config_from_json(e));
    if (!c) fail(std::string("entry of \"") + key + "\" has the wrong config type (" + e.value("type", "?") + ")");
    out.push_back(c);
  }
  return out;
}
template <class T>
json list_to(const std::vector<std::shared_ptr<const T>>& v) {
  json arr = json::array();
  for (const auto& c : v) arr.push_back(config_to_json(*c));
  return arr;
}

StandardOrchestrationConfig orch_from(const json& jc) {
  StandardOrchestrationConfig c;
  if (jc.contains("group")) c.group_ = group_of(jc);
  c.best_sol_channel = need(jc, "best_sol_channel").get<std::string>();
  c.imu_sol_channel = need(jc, "imu_sol_channel").get<std::string>();
  c.alignment_channels = need(jc, "alignment_channels").get<std::vector<std::string>>();
  {
    auto p = std::dynamic_pointer_cast<PinsonStateBlockConfig>(config_from_json(need(jc, "pinson_sb_config"), "PinsonStateBlockConfig"));
    if (!p) fail("pinson_sb_config must be a PinsonStateBlockConfig");
    c.pinson_sb_config = *p;
  }
  if (jc.contains("additional_sb_configs")) c.additional_sb_configs = list_from<StateBlockConfig>(jc, "additional_sb_configs");
  if (jc.contains("vsb_configs")) c.vsb_configs = list_from<VirtualStateBlockConfig>(jc, "vsb_configs");
  if (jc.contains("mp_configs")) c.mp_configs = list_from<MeasurementProcessorConfig>(jc, "mp_configs");
  c.inertial_config = inertial_from(need(jc, "inertial_config"));
  if (jc.contains("feedback_config") && !jc.at("feedback_config").is_null())
    c.feedback_config = feedback_from(jc.at("feedback_config"));
  c.alignment_config = config_from_json(need(jc, "alignment_config"));
  c.alignment_config_group = c.alignment_config->group();
  if (jc.contains("preprocessor_configs")) c.preprocessor_configs = list_from<PreprocessorConfig>(jc, "preprocessor_configs");
  c.max_prop_interval = get_or(jc, "max_prop_interval", 2.0);
  c.publish_before_update = get_or(jc, "publish_before_update", false);
  c.publish_after_update = get_or(jc, "publish_after_update", false);
  c.max_filter_lag = get_or(jc, "max_filter_lag", ControllerConfig::kDefaultBufferLengthSec);
  if (jc.contains("stream_config")) c.stream_config = stream_config_from(jc.at("stream_config"));
  return c;
}
json orch_to(const StandardOrchestrationConfig& c) {
  json o{{"type", "StandardOrchestrationConfig"}, {"group", c.group_}};
  o["best_sol_channel"] = c.best_sol_channel;
  o["imu_sol_channel"] = c.imu_sol_channel;
  o["alignment_channels"] = c.alignment_channels;
  o["pinson_sb_config"] = config_to_json(c.pinson_sb_config);
  if (c.additional_sb_configs) o["additional_sb_configs"] = list_to(*c.additional_sb_configs);
  if (c.vsb_configs) o["vsb_configs"] = list_to(*c.vsb_configs);
  if (c.mp_configs) o["mp_configs"] = list_to(*c.mp_configs);
  o["inertial_config"] = inertial_to(c.inertial_config);
  if (c.feedback_config) o["feedback_config"] = feedback_to(*c.feedback_config);
  if (c.alignment_config) o["alignment_config"] = config_to_json(*c.alignment_config);
  if (c.preprocessor_configs) o["preprocessor_configs"] = list_to(*c.preprocessor_configs);
  o["max_prop_interval"] = c.max_prop_interval;
  o["publish_before_update"] = c.publish_before_update;
  o["publish_after_update"] = c.publish_after_update;
  o["max_filter_lag"] = c.max_filter_lag;
  o["stream_config"] = stream_config_to(c.stream_config);
  return o;
}

}  // namespace

// ================================================================== public API

std::string message_type_name(api::AspnMessageType t) {
  for (const auto& [name, type] : message_types())
    if (type == t) return name;
  return std::to_string(static_cast<int>(t));
}

api::AspnMessageType message_type_from_name(const std::string& name) {
  auto it = message_types().find(name);
  if (it == message_types().end()) fail("unknown message_type \"" + name + "\"");
  return it->second;
}

std::shared_ptr<BaseConfig> config_from_json(const json& jc, const std::string& implied_type) {
  if (!jc.is_object()) fail("a config must be a JSON object");
  const std::string type = jc.contains("type") ? jc.at("type").get<std::string>() : implied_type;
  if (type.empty()) fail("config object without \"type\"");

  if (type == "ImuConfig") return std::make_shared<ImuConfig>(imu_from(jc));
  if (type == "FogmConfig") return std::make_shared<FogmConfig>(fogm_from(jc));
  if (type == "MountingConfig") {
    auto c = std::make_shared<MountingConfig>();
    c->group_ = group_of(jc);
    c->lever_arm = vec3(jc, "lever_arm");
    if (jc.contains("orientation")) c->orientation = vec4(jc, "orientation");
    return c;
  }
  if (type == "PinsonStateBlockConfig") {
    auto c = std::make_shared<PinsonStateBlockConfig>();
    sb_base_from(jc, *c);
    c->imu_model = imu_from(need(jc, "imu_model"));
    c->legacy_q_rotation = get_or(jc, "legacy_q_rotation", false);
    return c;
  }
  if (type == "FogmStateBlockConfig") {
    auto c = std::make_shared<FogmStateBlockConfig>();
    sb_base_from(jc, *c);
    c->fogm_model = fogm_from(need(jc, "fogm_model"));
    return c;
  }
  if (type == "ClockBiasStateBlockConfig") {
    auto c = std::make_shared<ClockBiasStateBlockConfig>();
    sb_base_from(jc, *c);
    c->h_0 = need(jc, "h_0").get<double>();
    c->h_neg2 = need(jc, "h_neg2").get<double>();
    if (jc.contains("q3") && !jc.at("q3").is_null()) c->q3 = jc.at("q3").get<double>();
    return c;
  }
  if (type == "ConstantStateBlockConfig") {
    auto c = std::make_shared<ConstantStateBlockConfig>();
    sb_base_from(jc, *c);
    if (jc.contains("Q") && !jc.at("Q").is_null()) c->Q = matrix(jc.at("Q"), "Q");
    return c;
  }
  for (const auto& k : mp_kinds())
    if (type == k.type) return mp_from(k, jc);
  if (type == "PinsonErrorToStandardVSBConfig" || type == "StateExtractorConfig") {
    std::shared_ptr<VirtualStateBlockConfig> c;
    if (type == "StateExtractorConfig") {
      auto se = std::make_shared<StateExtractorConfig>();
      se->incoming_state_size = need(jc, "incoming_state_size").get<int>();
      se->indices_to_extract = need(jc, "indices_to_extract").get<std::vector<int>>();
      c = se;
    } else {
      c = std::make_shared<PinsonErrorToStandardVSBConfig>();
    }
    c->group_ = group_of(jc);
    c->source = need(jc, "source").get<std::string>();
    c->target = need(jc, "target").get<std::string>();
    if (jc.contains("aux_channels")) c->aux_channels = opt_strings(jc, "aux_channels");
    return c;
  }
  if (type == "FusionEngineConfig") {
    auto c = std::make_shared<FusionEngineConfig>();
    if (jc.contains("group")) c->group_ = group_of(jc);
    c->save_x_and_p_after_prop = get_or(jc, "save_x_and_p_after_prop", false);
    c->save_x_and_p_after_update = get_or(jc, "save_x_and_p_after_update", false);
    c->innovation_gate_probability = get_or(jc, "innovation_gate_probability", 0.0);
    return c;
  }
  if (type == "ControllerConfig") {
    auto c = std::make_shared<ControllerConfig>();
    if (jc.contains("group")) c->group_ = group_of(jc);
    c->buffer_length_sec = get_or(jc, "buffer_length_sec", ControllerConfig::kDefaultBufferLengthSec);
    if (jc.contains("publish_interval")) {
      if (jc.at("publish_interval").is_null()) c->publish_interval = std::nullopt;
      else c->publish_interval = jc.at("publish_interval").get<double>();
    }
    c->auto_shutdown = get_or(jc, "auto_shutdown", true);
    return c;
  }
  if (type == "Stream") return std::make_shared<Stream>(stream_from(jc));
  if (type == "StreamConfig") return std::make_shared<StreamConfig>(stream_config_from(jc));
  if (type == "InertialConfig") return std::make_shared<InertialConfig>(inertial_from(jc));
  if (type == "FeedbackConfig") return std::make_shared<FeedbackConfig>(feedback_from(jc));
  if (type == "DownsamplerConfig") {
    auto c = std::make_shared<DownsamplerConfig>();
    pp_base_from(jc, *c);
    c->downsampling_factors = need(jc, "downsampling_factors").get<std::vector<std::int64_t>>();
    return c;
  }
  if (type == "ImuRotatorConfig") {
    auto c = std::make_shared<ImuRotatorConfig>();
    pp_base_from(jc, *c);
    c->C_imu_to_platform = mat3(jc, "C_imu_to_platform");
    return c;
  }
  if (type == "TimeAdjusterConfig") {
    auto c = std::make_shared<TimeAdjusterConfig>();
    pp_base_from(jc, *c);
    c->expected_dt_nsec = need(jc, "expected_dt_nsec").get<std::int64_t>();
    return c;
  }
  if (type == "BarometerToAltitudeConfig") {
    auto c = std::make_shared<BarometerToAltitudeConfig>();
    pp_base_from(jc, *c);
    if (jc.contains("alt_sigma") && !jc.at("alt_sigma").is_null()) c->alt_sigma = jc.at("alt_sigma").get<double>();
    return c;
  }
  if (type == "TimeBiasConfig") {
    auto c = std::make_shared<TimeBiasConfig>();
    pp_base_from(jc, *c);
    c->time_bias = need(jc, "time_bias").get<std::int64_t>();
    return c;
  }
  if (type == "OutageConfig") {
    auto c = std::make_shared<OutageConfig>();
    pp_base_from(jc, *c);
    c->start_time = need(jc, "start_time").get<double>();
    c->end_time = need(jc, "end_time").get<double>();
    return c;
  }
  if (type == "ZeroVelocity2dGeneratorConfig") {
    auto c = std::make_shared<ZeroVelocity2dGeneratorConfig>();
    pp_base_from(jc, *c);
    c->trigger_dt_sec = get_or(jc, "trigger_dt_sec", 0.0);
    c->lateral_vel_sigma = need(jc, "lateral_vel_sigma").get<double>();
    c->vertical_vel_sigma = need(jc, "vertical_vel_sigma").get<double>();
    c->output_channel = need(jc, "output_channel").get<std::string>();
    return c;
  }
  if (type == "SensorDegradationConfig") {
    auto c = std::make_shared<SensorDegradationConfig>();
    pp_base_from(jc, *c);
    c->seed = get_or<std::int64_t>(jc, "seed", 1);
    c->imu_expected_dt = get_or(jc, "imu_expected_dt", 0.01);
    if (jc.contains("accel_noise_density")) c->accel_noise_density = vec3(jc, "accel_noise_density");
    if (jc.contains("gyro_noise_density")) c->gyro_noise_density = vec3(jc, "gyro_noise_density");
    if (jc.contains("accel_bias")) c->accel_bias = vec3(jc, "accel_bias");
    if (jc.contains("gyro_bias")) c->gyro_bias = vec3(jc, "gyro_bias");
    if (jc.contains("position_noise_sigma_ned")) c->position_noise_sigma_ned = vec3(jc, "position_noise_sigma_ned");
    c->position_covariance_scale = get_or(jc, "position_covariance_scale", 1.0);
    if (jc.contains("velocity_noise_sigma")) c->velocity_noise_sigma = vec3(jc, "velocity_noise_sigma");
    c->velocity_covariance_scale = get_or(jc, "velocity_covariance_scale", 1.0);
    if (jc.contains("position_jumps"))
      for (const auto& row : jc.at("position_jumps")) {
        if (!row.is_array() || row.size() != 4) fail("position_jumps entries must be [time_s, north, east, down]");
        c->position_jumps.push_back({row[0].get<double>(), row[1].get<double>(), row[2].get<double>(), row[3].get<double>()});
      }
    return c;
  }
  if (type == "ManualAlignmentConfig") return std::make_shared<ManualAlignmentConfig>(manual_from(jc));
  if (type == "StaticAlignmentConfig") {
    auto c = std::make_shared<StaticAlignmentConfig>();
    c->group_ = group_of(jc);
    c->static_time = need(jc, "static_time").get<double>();
    c->imu_model = imu_from(need(jc, "imu_model"));
    return c;
  }
  if (type == "ManualHeadingAlignmentConfig") {
    auto c = std::make_shared<ManualHeadingAlignmentConfig>();
    c->group_ = group_of(jc);
    c->static_time = need(jc, "static_time").get<double>();
    c->imu_model = imu_from(need(jc, "imu_model"));
    c->heading = need(jc, "heading").get<double>();
    c->heading_sigma = need(jc, "heading_sigma").get<double>();
    return c;
  }
  if (type == "PvaMessageInitializationConfig") {
    auto c = std::make_shared<PvaMessageInitializationConfig>();
    c->group_ = group_of(jc);
    c->initial_pva_channel = need(jc, "initial_pva_channel").get<std::string>();
    c->initial_accel_bias_sigma = vec3(jc, "initial_accel_bias_sigma");
    c->initial_gyro_bias_sigma = vec3(jc, "initial_gyro_bias_sigma");
    if (jc.contains("initial_pva_sigma") && !jc.at("initial_pva_sigma").is_null()) {
      auto v = jc.at("initial_pva_sigma").get<std::vector<double>>();
      if (v.size() != 9) fail("initial_pva_sigma must have 9 entries");
      std::array<double, 9> a{};
      std::copy(v.begin(), v.end(), a.begin());
      c->initial_pva_sigma = a;
    }
    if (jc.contains("start_time") && !jc.at("start_time").is_null()) c->start_time = jc.at("start_time").get<double>();
    return c;
  }
  if (type == "StandardOrchestrationConfig") return std::make_shared<StandardOrchestrationConfig>(orch_from(jc));
  if (type == "TutorialOrchestrationConfig") {
    auto c = std::make_shared<TutorialOrchestrationConfig>();
    if (jc.contains("group")) c->group_ = group_of(jc);
    c->position_channel = need(jc, "position_channel").get<std::string>();
    c->velocity_channel = get_or<std::string>(jc, "velocity_channel", "unused");
    return c;
  }
  if (type == "UiLogPlottingConfig") {
    auto c = std::make_shared<UiLogPlottingConfig>();
    if (jc.contains("group")) c->group_ = group_of(jc);
    c->logfile = need(jc, "logfile").get<std::string>();
    c->solution_channel = need(jc, "solution_channel").get<std::string>();
    c->truth_channel = need(jc, "truth_channel").get<std::string>();
    return c;
  }
  if (type == "LcmLogTransportConfig") {
    auto c = std::make_shared<LcmLogTransportConfig>();
    if (jc.contains("group")) c->group_ = group_of(jc);
    if (jc.contains("input_file") && !jc.at("input_file").is_null()) c->input_file = jc.at("input_file").get<std::string>();
    if (jc.contains("output_file") && !jc.at("output_file").is_null()) c->output_file = jc.at("output_file").get<std::string>();
    c->channels_to_process = opt_strings(jc, "channels_to_process");
    c->record_input_channels = get_or(jc, "record_input_channels", true);
    return c;
  }
  if (type == "LcmTransportConfig") {
    auto c = std::make_shared<LcmTransportConfig>();
    if (jc.contains("group")) c->group_ = group_of(jc);
    c->url = get_or<std::string>(jc, "url", c->url);
    c->subscribe_to = get_or<std::string>(jc, "subscribe_to", c->subscribe_to);
    return c;
  }
  fail("unknown config type \"" + type + "\"");
}

std::string type_name(const BaseConfig& c) {
  if (dynamic_cast<const MeasurementProcessorConfig*>(&c)) {
    const auto& m = static_cast<const MeasurementProcessorConfig&>(c);
    for (const auto& k : mp_kinds())
      if (m.identifier == k.identifier) return k.type;
    return "MeasurementProcessorConfig";
  }
#define PNTOS_TN(T) if (dynamic_cast<const T*>(&c)) return #T;
  PNTOS_TN(ImuConfig) PNTOS_TN(FogmConfig) PNTOS_TN(MountingConfig) PNTOS_TN(PinsonStateBlockConfig)
  PNTOS_TN(FogmStateBlockConfig) PNTOS_TN(ClockBiasStateBlockConfig) PNTOS_TN(ConstantStateBlockConfig)
  PNTOS_TN(PinsonErrorToStandardVSBConfig) PNTOS_TN(StateExtractorConfig) PNTOS_TN(FusionEngineConfig)
  PNTOS_TN(ControllerConfig) PNTOS_TN(Stream) PNTOS_TN(StreamConfig) PNTOS_TN(InertialConfig) PNTOS_TN(FeedbackConfig)
  PNTOS_TN(DownsamplerConfig) PNTOS_TN(ImuRotatorConfig) PNTOS_TN(TimeAdjusterConfig) PNTOS_TN(BarometerToAltitudeConfig)
  PNTOS_TN(TimeBiasConfig) PNTOS_TN(OutageConfig) PNTOS_TN(ZeroVelocity2dGeneratorConfig) PNTOS_TN(SensorDegradationConfig)
  PNTOS_TN(ManualAlignmentConfig)
  PNTOS_TN(StaticAlignmentConfig) PNTOS_TN(ManualHeadingAlignmentConfig) PNTOS_TN(PvaMessageInitializationConfig)
  PNTOS_TN(StandardOrchestrationConfig) PNTOS_TN(TutorialOrchestrationConfig) PNTOS_TN(UiLogPlottingConfig)
  PNTOS_TN(LcmLogTransportConfig) PNTOS_TN(LcmTransportConfig)
#undef PNTOS_TN
  return "BaseConfig";
}

json config_to_json(const BaseConfig& c) {
  const std::string type = type_name(c);
  if (auto* p = dynamic_cast<const ImuConfig*>(&c)) return imu_to(*p);
  if (auto* p = dynamic_cast<const FogmConfig*>(&c)) return fogm_to(*p);
  if (auto* p = dynamic_cast<const MountingConfig*>(&c))
    return json{{"type", type}, {"group", p->group_}, {"lever_arm", j(p->lever_arm)}, {"orientation", j(p->orientation)}};
  if (auto* p = dynamic_cast<const PinsonStateBlockConfig*>(&c)) {
    json o{{"type", type}};
    sb_base_to(*p, o);
    o["imu_model"] = imu_to(p->imu_model);
    o["legacy_q_rotation"] = p->legacy_q_rotation;
    return o;
  }
  if (auto* p = dynamic_cast<const FogmStateBlockConfig*>(&c)) {
    json o{{"type", type}};
    sb_base_to(*p, o);
    o["fogm_model"] = fogm_to(p->fogm_model);
    return o;
  }
  if (auto* p = dynamic_cast<const ClockBiasStateBlockConfig*>(&c)) {
    json o{{"type", type}};
    sb_base_to(*p, o);
    o["h_0"] = p->h_0;
    o["h_neg2"] = p->h_neg2;
    if (p->q3) o["q3"] = *p->q3;
    return o;
  }
  if (auto* p = dynamic_cast<const ConstantStateBlockConfig*>(&c)) {
    json o{{"type", type}};
    sb_base_to(*p, o);
    if (p->Q) o["Q"] = j(*p->Q);
    return o;
  }
  if (auto* p = dynamic_cast<const LeverArmMPConfig*>(&c)) {
    json o{{"type", type}};
    mp_base_to(*p, o);
    o["lever_arm"] = j(p->lever_arm);
    return o;
  }
  if (auto* p = dynamic_cast<const LeverArmOrientationMPConfig*>(&c)) {
    json o{{"type", type}};
    mp_base_to(*p, o);
    o["lever_arm"] = j(p->lever_arm);
    o["orientation"] = j(p->orientation);
    return o;
  }
  if (auto* p = dynamic_cast<const MeasurementProcessorConfig*>(&c)) {
    json o{{"type", type}};
    mp_base_to(*p, o);
    return o;
  }
  if (auto* p = dynamic_cast<const StateExtractorConfig*>(&c)) {
    json o{{"type", type}, {"group", p->group_}, {"source", p->source}, {"target", p->target},
           {"incoming_state_size", p->incoming_state_size}, {"indices_to_extract", p->indices_to_extract}};
    if (p->aux_channels) o["aux_channels"] = *p->aux_channels;
    return o;
  }
  if (auto* p = dynamic_cast<const VirtualStateBlockConfig*>(&c)) {
    json o{{"type", type}, {"group", p->group_}, {"source", p->source}, {"target", p->target}};
    if (p->aux_channels) o["aux_channels"] = *p->aux_channels;
    return o;
  }
  if (auto* p = dynamic_cast<const FusionEngineConfig*>(&c))
    return json{{"type", type}, {"group", p->group_}, {"save_x_and_p_after_prop", p->save_x_and_p_after_prop},
                {"save_x_and_p_after_update", p->save_x_and_p_after_update}, {"innovation_gate_probability", p->innovation_gate_probability}};
  if (auto* p = dynamic_cast<const ControllerConfig*>(&c)) {
    json o{{"type", type}, {"group", p->group_}, {"buffer_length_sec", p->buffer_length_sec}, {"auto_shutdown", p->auto_shutdown}};
    o["publish_interval"] = p->publish_interval ? json(*p->publish_interval) : json(nullptr);
    return o;
  }
  if (auto* p = dynamic_cast<const Stream*>(&c)) return stream_to(*p);
  if (auto* p = dynamic_cast<const StreamConfig*>(&c)) return stream_config_to(*p);
  if (auto* p = dynamic_cast<const InertialConfig*>(&c)) return inertial_to(*p);
  if (auto* p = dynamic_cast<const FeedbackConfig*>(&c)) return feedback_to(*p);
  if (auto* p = dynamic_cast<const DownsamplerConfig*>(&c)) {
    json o{{"type", type}};
    pp_base_to(*p, o);
    o["downsampling_factors"] = p->downsampling_factors;
    return o;
  }
  if (auto* p = dynamic_cast<const ImuRotatorConfig*>(&c)) {
    json o{{"type", type}};
    pp_base_to(*p, o);
    o["C_imu_to_platform"] = j(p->C_imu_to_platform);
    return o;
  }
  if (auto* p = dynamic_cast<const TimeAdjusterConfig*>(&c)) {
    json o{{"type", type}};
    pp_base_to(*p, o);
    o["expected_dt_nsec"] = p->expected_dt_nsec;
    return o;
  }
  if (auto* p = dynamic_cast<const BarometerToAltitudeConfig*>(&c)) {
    json o{{"type", type}};
    pp_base_to(*p, o);
    if (p->alt_sigma) o["alt_sigma"] = *p->alt_sigma;
    return o;
  }
  if (auto* p = dynamic_cast<const TimeBiasConfig*>(&c)) {
    json o{{"type", type}};
    pp_base_to(*p, o);
    o["time_bias"] = p->time_bias;
    return o;
  }
  if (auto* p = dynamic_cast<const OutageConfig*>(&c)) {
    json o{{"type", type}};
    pp_base_to(*p, o);
    o["start_time"] = p->start_time;
    o["end_time"] = p->end_time;
    return o;
  }
  if (auto* p = dynamic_cast<const ZeroVelocity2dGeneratorConfig*>(&c)) {
    json o{{"type", type}};
    pp_base_to(*p, o);
    o["trigger_dt_sec"] = p->trigger_dt_sec;
    o["lateral_vel_sigma"] = p->lateral_vel_sigma;
    o["vertical_vel_sigma"] = p->vertical_vel_sigma;
    o["output_channel"] = p->output_channel;
    return o;
  }
  if (auto* p = dynamic_cast<const SensorDegradationConfig*>(&c)) {
    json o{{"type", type}};
    pp_base_to(*p, o);
    o["seed"] = p->seed;
    o["imu_expected_dt"] = p->imu_expected_dt;
    o["accel_noise_density"] = j(p->accel_noise_density);
    o["gyro_noise_density"] = j(p->gyro_noise_density);
    o["accel_bias"] = j(p->accel_bias);
    o["gyro_bias"] = j(p->gyro_bias);
    o["position_noise_sigma_ned"] = j(p->position_noise_sigma_ned);
    o["position_covariance_scale"] = p->position_covariance_scale;
    o["velocity_noise_sigma"] = j(p->velocity_noise_sigma);
    o["velocity_covariance_scale"] = p->velocity_covariance_scale;
    json jumps = json::array();
    for (const auto& r : p->position_jumps) jumps.push_back(json::array({r[0], r[1], r[2], r[3]}));
    o["position_jumps"] = jumps;
    return o;
  }
  if (auto* p = dynamic_cast<const ManualAlignmentConfig*>(&c)) return manual_to(*p);
  if (auto* p = dynamic_cast<const StaticAlignmentConfig*>(&c))
    return json{{"type", type}, {"group", p->group_}, {"static_time", p->static_time}, {"imu_model", imu_to(p->imu_model)}};
  if (auto* p = dynamic_cast<const ManualHeadingAlignmentConfig*>(&c))
    return json{{"type", type}, {"group", p->group_}, {"static_time", p->static_time}, {"imu_model", imu_to(p->imu_model)},
                {"heading", p->heading}, {"heading_sigma", p->heading_sigma}};
  if (auto* p = dynamic_cast<const PvaMessageInitializationConfig*>(&c)) {
    json o{{"type", type}, {"group", p->group_}, {"initial_pva_channel", p->initial_pva_channel},
           {"initial_accel_bias_sigma", j(p->initial_accel_bias_sigma)}, {"initial_gyro_bias_sigma", j(p->initial_gyro_bias_sigma)}};
    if (p->initial_pva_sigma) o["initial_pva_sigma"] = std::vector<double>(p->initial_pva_sigma->begin(), p->initial_pva_sigma->end());
    if (p->start_time) o["start_time"] = *p->start_time;
    return o;
  }
  if (auto* p = dynamic_cast<const StandardOrchestrationConfig*>(&c)) return orch_to(*p);
  if (auto* p = dynamic_cast<const TutorialOrchestrationConfig*>(&c))
    return json{{"type", type}, {"group", p->group_}, {"position_channel", p->position_channel}, {"velocity_channel", p->velocity_channel}};
  if (auto* p = dynamic_cast<const UiLogPlottingConfig*>(&c))
    return json{{"type", type}, {"group", p->group_}, {"logfile", p->logfile}, {"solution_channel", p->solution_channel},
                {"truth_channel", p->truth_channel}};
  if (auto* p = dynamic_cast<const LcmLogTransportConfig*>(&c)) {
    json o{{"type", type}, {"group", p->group_}, {"record_input_channels", p->record_input_channels}};
    if (p->input_file) o["input_file"] = *p->input_file;
    if (p->output_file) o["output_file"] = *p->output_file;
    if (p->channels_to_process) o["channels_to_process"] = *p->channels_to_process;
    return o;
  }
  if (auto* p = dynamic_cast<const LcmTransportConfig*>(&c))
    return json{{"type", type}, {"group", p->group_}, {"url", p->url}, {"subscribe_to", p->subscribe_to}};
  fail("config_to_json: unsupported config class");
}

AppSpec app_spec_from_json(const json& ja) {
  AppSpec s;
  if (ja.is_null()) return s;
  s.name = get_or(ja, "name", s.name);
  s.transport = get_or(ja, "transport", s.transport);
  s.initialization = get_or(ja, "initialization", s.initialization);
  s.state_modeling = get_or(ja, "state_modeling", s.state_modeling);
  s.orchestration = get_or(ja, "orchestration", s.orchestration);
  s.preprocessors = get_or(ja, "preprocessors", s.preprocessors);
  s.diagnostic_log = get_or(ja, "diagnostic_log", s.diagnostic_log);
  s.diagnostic_log_file = get_or(ja, "diagnostic_log_file", s.diagnostic_log_file);
  s.ui_log_plotting = get_or(ja, "ui_log_plotting", s.ui_log_plotting);
  s.logging_level = get_or(ja, "logging_level", s.logging_level);
  s.joseph_form = get_or(ja, "joseph_form", s.joseph_form);
  s.legacy_q_rotation = get_or(ja, "legacy_q_rotation", s.legacy_q_rotation);
  return s;
}

json app_spec_to_json(const AppSpec& s) {
  return json{{"name", s.name},
              {"transport", s.transport},
              {"initialization", s.initialization},
              {"state_modeling", s.state_modeling},
              {"orchestration", s.orchestration},
              {"preprocessors", s.preprocessors},
              {"diagnostic_log", s.diagnostic_log},
              {"diagnostic_log_file", s.diagnostic_log_file},
              {"ui_log_plotting", s.ui_log_plotting},
              {"logging_level", s.logging_level},
              {"joseph_form", s.joseph_form},
              {"legacy_q_rotation", s.legacy_q_rotation}};
}

AppConfig app_config_from_json(const json& jc) {
  if (!jc.is_object()) fail("top level must be an object with \"app\" and \"configs\"");
  AppConfig out;
  out.app = app_spec_from_json(jc.contains("app") ? jc.at("app") : json(nullptr));
  for (const auto& e : need(jc, "configs")) out.configs.push_back(config_from_json(e));
  return out;
}

json app_config_to_json(const AppConfig& c) {
  json arr = json::array();
  for (const auto& cfg : c.configs) arr.push_back(config_to_json(*cfg));
  return json{{"app", app_spec_to_json(c.app)}, {"configs", arr}};
}

AppConfig load_app_config(const std::string& path) {
  std::ifstream in(path);
  if (!in) fail("cannot open " + path);
  json jc;
  try {
    jc = json::parse(in, nullptr, true, /*ignore_comments=*/true);
  } catch (const std::exception& e) {
    fail(path + ": " + e.what());
  }
  return app_config_from_json(jc);
}

void save_app_config(const AppConfig& c, const std::string& path) {
  std::ofstream out(path);
  if (!out) fail("cannot write " + path);
  out << app_config_to_json(c).dump(2) << '\n';
}

json dump_registry(api::Registry& registry) {
  json out = json::object();
  auto groups = registry.group_array();
  if (!groups) return out;
  std::vector<std::string> names = *groups;
  std::sort(names.begin(), names.end());
  for (const auto& g : names) {
    json jg = json::object();
    auto kv = registry.batch(g);
    auto keys = kv->keys();
    if (keys) {
      std::vector<std::string> ks = *keys;
      std::sort(ks.begin(), ks.end());
      for (const auto& k : ks) {
        auto v = kv->get(k);
        if (!v) continue;
        std::visit(
            [&](const auto& val) {
              using T = std::decay_t<decltype(val)>;
              if constexpr (std::is_same_v<T, api::Matrix>) jg[k] = j(val);
              else if constexpr (std::is_same_v<T, api::Message>) jg[k] = "<Message " + val.source_identifier + ">";
              else jg[k] = val;
            },
            *v);
      }
    }
    out[g] = jg;
  }
  return out;
}

}  // namespace pntos::cobra::jsoncfg
