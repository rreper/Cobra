#include <pntos/cobra/transport/LcmConversions.hpp>

#include <aspn23/eigen/MeasurementAltitude.hpp>
#include <aspn23/eigen/MeasurementBarometer.hpp>
#include <aspn23/eigen/MeasurementDirection3DToPoints.hpp>
#include <aspn23/eigen/MeasurementImu.hpp>
#include <aspn23/eigen/MeasurementPosition.hpp>
#include <aspn23/eigen/MeasurementPositionVelocityAttitude.hpp>
#include <aspn23/eigen/MeasurementVelocity.hpp>

#include <aspn23_lcm/measurement_IMU.hpp>
#include <aspn23_lcm/measurement_altitude.hpp>
#include <aspn23_lcm/measurement_barometer.hpp>
#include <aspn23_lcm/measurement_direction_3d_to_points.hpp>
#include <aspn23_lcm/measurement_position.hpp>
#include <aspn23_lcm/measurement_position_velocity_attitude.hpp>
#include <aspn23_lcm/measurement_velocity.hpp>

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace pntos::cobra::lcm {

namespace {
using DynVec = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using RowMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

std::int64_t read_hash(const std::uint8_t* d, std::size_t len) {
  if (len < 8) return 0;
  std::uint64_t v = 0;
  for (int i = 0; i < 8; ++i) v = (v << 8) | d[i];
  return static_cast<std::int64_t>(v);
}

template <class L>
L decode_as(const std::uint8_t* d, std::size_t len) {
  L m;
  if (m.decode(d, 0, static_cast<int>(len)) < 0) throw std::runtime_error(std::string("LCM decode failed for ") + L::getTypeName());
  return m;
}
template <class L>
std::vector<std::uint8_t> encode_as(const L& m) {
  std::vector<std::uint8_t> out(static_cast<std::size_t>(m.getEncodedSize()));
  if (m.encode(out.data(), 0, static_cast<int>(out.size())) < 0) throw std::runtime_error("LCM encode failed");
  return out;
}

aspn23_eigen::TypeHeader header_from(const aspn23_lcm::type_header& h, Aspn23MessageType type) {
  return aspn23_eigen::TypeHeader(type, static_cast<std::uint32_t>(h.vendor_id), static_cast<std::uint32_t>(h.device_id),
                                  static_cast<std::uint32_t>(h.context_id), static_cast<std::uint32_t>(h.sequence_id));
}
aspn23_lcm::type_header header_to(const aspn23_eigen::TypeHeader& h) {
  aspn23_lcm::type_header o{};
  o.vendor_id = h.get_vendor_id();
  o.device_id = h.get_device_id();
  o.context_id = h.get_context_id();
  o.sequence_id = static_cast<std::int32_t>(h.get_sequence_id());
  return o;
}
aspn23_eigen::TypeTimestamp tov_from(const aspn23_lcm::type_timestamp& t) { return aspn23_eigen::TypeTimestamp(t.elapsed_nsec); }
aspn23_lcm::type_timestamp tov_to(const aspn23_eigen::TypeTimestamp& t) {
  aspn23_lcm::type_timestamp o{};
  o.elapsed_nsec = t.get_elapsed_nsec();
  return o;
}
std::vector<aspn23_eigen::TypeIntegrity> integrity_from(const std::vector<aspn23_lcm::type_integrity>& v) {
  std::vector<aspn23_eigen::TypeIntegrity> out;
  for (const auto& i : v)
    out.emplace_back(static_cast<Aspn23TypeIntegrityIntegrityMethod>(i.integrity_method), i.integrity_value);
  return out;
}
std::vector<aspn23_lcm::type_integrity> integrity_to(const std::vector<aspn23_eigen::TypeIntegrity>& v) {
  std::vector<aspn23_lcm::type_integrity> out;
  for (const auto& i : v) {
    aspn23_lcm::type_integrity o{};
    o.integrity_method = static_cast<std::int8_t>(i.get_integrity_method());
    o.integrity_value = i.get_integrity_value();
    out.push_back(o);
  }
  return out;
}
RowMat cov_from(const std::vector<std::vector<double>>& c) {
  const auto n = static_cast<Eigen::Index>(c.size());
  RowMat m(n, n);
  for (Eigen::Index i = 0; i < n; ++i)
    for (Eigen::Index j = 0; j < n; ++j) m(i, j) = c[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
  return m;
}
std::vector<std::vector<double>> cov_to(const RowMat& m) {
  std::vector<std::vector<double>> out(static_cast<std::size_t>(m.rows()), std::vector<double>(static_cast<std::size_t>(m.cols())));
  for (Eigen::Index i = 0; i < m.rows(); ++i)
    for (Eigen::Index j = 0; j < m.cols(); ++j) out[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = m(i, j);
  return out;
}
DynVec vec_from(const std::vector<double>& v) {
  DynVec out(static_cast<Eigen::Index>(v.size()));
  for (std::size_t i = 0; i < v.size(); ++i) out(static_cast<Eigen::Index>(i)) = v[i];
  return out;
}
std::vector<double> vec_to(const DynVec& v) { return std::vector<double>(v.data(), v.data() + v.size()); }

// ---- per type

std::shared_ptr<api::AspnBase> imu_from(const aspn23_lcm::measurement_IMU& m) {
  DynVec a(3), g(3);
  for (int i = 0; i < 3; ++i) {
    a(i) = m.meas_accel[i];
    g(i) = m.meas_gyro[i];
  }
  return std::make_shared<aspn23_eigen::MeasurementImu>(header_from(m.header, ASPN_MEASUREMENT_IMU), tov_from(m.time_of_validity),
                                                        static_cast<Aspn23MeasurementImuImuType>(m.imu_type), a, g,
                                                        integrity_from(m.integrity));
}
std::vector<std::uint8_t> imu_to(const aspn23_eigen::MeasurementImu& e) {
  aspn23_lcm::measurement_IMU m{};
  m.header = header_to(e.get_header());
  m.time_of_validity = tov_to(e.get_time_of_validity());
  m.imu_type = static_cast<std::int8_t>(e.get_imu_type());
  DynVec a = e.get_meas_accel(), g = e.get_meas_gyro();
  for (int i = 0; i < 3; ++i) {
    m.meas_accel[i] = a.size() > i ? a(i) : 0.0;
    m.meas_gyro[i] = g.size() > i ? g(i) : 0.0;
  }
  m.integrity = integrity_to(e.get_integrity());
  m.num_integrity = static_cast<std::int16_t>(m.integrity.size());
  return encode_as(m);
}

std::shared_ptr<api::AspnBase> position_from(const aspn23_lcm::measurement_position& m) {
  return std::make_shared<aspn23_eigen::MeasurementPosition>(
      header_from(m.header, ASPN_MEASUREMENT_POSITION), tov_from(m.time_of_validity),
      static_cast<Aspn23MeasurementPositionReferenceFrame>(m.reference_frame), m.term1, m.term2, m.term3,
      cov_from(m.covariance), static_cast<Aspn23MeasurementPositionErrorModel>(m.error_model),
      vec_from(m.error_model_params), integrity_from(m.integrity));
}
std::vector<std::uint8_t> position_to(const aspn23_eigen::MeasurementPosition& e) {
  aspn23_lcm::measurement_position m{};
  m.header = header_to(e.get_header());
  m.time_of_validity = tov_to(e.get_time_of_validity());
  m.reference_frame = static_cast<std::int8_t>(e.get_reference_frame());
  m.term1 = e.get_term1();
  m.term2 = e.get_term2();
  m.term3 = e.get_term3();
  m.covariance = cov_to(e.get_covariance());
  m.num_meas = static_cast<std::int16_t>(m.covariance.size());
  m.error_model = static_cast<std::int8_t>(e.get_error_model());
  m.error_model_params = vec_to(e.get_error_model_params());
  m.num_error_model_params = static_cast<std::int32_t>(m.error_model_params.size());
  m.integrity = integrity_to(e.get_integrity());
  m.num_integrity = static_cast<std::int16_t>(m.integrity.size());
  return encode_as(m);
}

std::shared_ptr<api::AspnBase> velocity_from(const aspn23_lcm::measurement_velocity& m) {
  return std::make_shared<aspn23_eigen::MeasurementVelocity>(
      header_from(m.header, ASPN_MEASUREMENT_VELOCITY), tov_from(m.time_of_validity),
      static_cast<Aspn23MeasurementVelocityReferenceFrame>(m.reference_frame), m.x, m.y, m.z, cov_from(m.covariance),
      static_cast<Aspn23MeasurementVelocityErrorModel>(m.error_model), vec_from(m.error_model_params),
      integrity_from(m.integrity));
}
std::vector<std::uint8_t> velocity_to(const aspn23_eigen::MeasurementVelocity& e) {
  aspn23_lcm::measurement_velocity m{};
  m.header = header_to(e.get_header());
  m.time_of_validity = tov_to(e.get_time_of_validity());
  m.reference_frame = static_cast<std::int8_t>(e.get_reference_frame());
  m.x = e.get_x();
  m.y = e.get_y();
  m.z = e.get_z();
  m.covariance = cov_to(e.get_covariance());
  m.num_meas = static_cast<std::int16_t>(m.covariance.size());
  m.error_model = static_cast<std::int8_t>(e.get_error_model());
  m.error_model_params = vec_to(e.get_error_model_params());
  m.num_error_model_params = static_cast<std::int32_t>(m.error_model_params.size());
  m.integrity = integrity_to(e.get_integrity());
  m.num_integrity = static_cast<std::int16_t>(m.integrity.size());
  return encode_as(m);
}

std::shared_ptr<api::AspnBase> pva_from(const aspn23_lcm::measurement_position_velocity_attitude& m) {
  DynVec q(4);
  for (int i = 0; i < 4; ++i) q(i) = m.quaternion[i];
  return std::make_shared<aspn23_eigen::MeasurementPositionVelocityAttitude>(
      header_from(m.header, ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE), tov_from(m.time_of_validity),
      static_cast<Aspn23MeasurementPositionVelocityAttitudeReferenceFrame>(m.reference_frame), m.p1, m.p2, m.p3, m.v1,
      m.v2, m.v3, q, cov_from(m.covariance),
      static_cast<Aspn23MeasurementPositionVelocityAttitudeErrorModel>(m.error_model), vec_from(m.error_model_params),
      integrity_from(m.integrity));
}
std::vector<std::uint8_t> pva_to(const aspn23_eigen::MeasurementPositionVelocityAttitude& e) {
  aspn23_lcm::measurement_position_velocity_attitude m{};
  m.header = header_to(e.get_header());
  m.time_of_validity = tov_to(e.get_time_of_validity());
  m.reference_frame = static_cast<std::int8_t>(e.get_reference_frame());
  m.p1 = e.get_p1();
  m.p2 = e.get_p2();
  m.p3 = e.get_p3();
  m.v1 = e.get_v1();
  m.v2 = e.get_v2();
  m.v3 = e.get_v3();
  DynVec q = e.get_quaternion();
  for (int i = 0; i < 4; ++i) m.quaternion[i] = q.size() == 4 ? q(i) : std::nan("");
  m.covariance = cov_to(e.get_covariance());
  m.num_meas = static_cast<std::int16_t>(m.covariance.size());
  m.error_model = static_cast<std::int8_t>(e.get_error_model());
  m.error_model_params = vec_to(e.get_error_model_params());
  m.num_error_model_params = static_cast<std::int32_t>(m.error_model_params.size());
  m.integrity = integrity_to(e.get_integrity());
  m.num_integrity = static_cast<std::int16_t>(m.integrity.size());
  return encode_as(m);
}

std::shared_ptr<api::AspnBase> altitude_from(const aspn23_lcm::measurement_altitude& m) {
  return std::make_shared<aspn23_eigen::MeasurementAltitude>(
      header_from(m.header, ASPN_MEASUREMENT_ALTITUDE), tov_from(m.time_of_validity),
      static_cast<Aspn23MeasurementAltitudeReference>(m.reference), m.altitude, m.variance,
      static_cast<Aspn23MeasurementAltitudeErrorModel>(m.error_model), vec_from(m.error_model_params),
      integrity_from(m.integrity));
}
std::vector<std::uint8_t> altitude_to(const aspn23_eigen::MeasurementAltitude& e) {
  aspn23_lcm::measurement_altitude m{};
  m.header = header_to(e.get_header());
  m.time_of_validity = tov_to(e.get_time_of_validity());
  m.reference = static_cast<std::int8_t>(e.get_reference());
  m.altitude = e.get_altitude();
  m.variance = e.get_variance();
  m.error_model = static_cast<std::int8_t>(e.get_error_model());
  m.error_model_params = vec_to(e.get_error_model_params());
  m.num_error_model_params = static_cast<std::int32_t>(m.error_model_params.size());
  m.integrity = integrity_to(e.get_integrity());
  m.num_integrity = static_cast<std::int16_t>(m.integrity.size());
  return encode_as(m);
}

std::shared_ptr<api::AspnBase> barometer_from(const aspn23_lcm::measurement_barometer& m) {
  return std::make_shared<aspn23_eigen::MeasurementBarometer>(
      header_from(m.header, ASPN_MEASUREMENT_BAROMETER), tov_from(m.time_of_validity), m.pressure, m.variance,
      static_cast<Aspn23MeasurementBarometerErrorModel>(m.error_model), vec_from(m.error_model_params),
      integrity_from(m.integrity));
}
std::vector<std::uint8_t> barometer_to(const aspn23_eigen::MeasurementBarometer& e) {
  aspn23_lcm::measurement_barometer m{};
  m.header = header_to(e.get_header());
  m.time_of_validity = tov_to(e.get_time_of_validity());
  m.pressure = e.get_pressure();
  m.variance = e.get_variance();
  m.error_model = static_cast<std::int8_t>(e.get_error_model());
  m.error_model_params = vec_to(e.get_error_model_params());
  m.num_error_model_params = static_cast<std::int32_t>(m.error_model_params.size());
  m.integrity = integrity_to(e.get_integrity());
  m.num_integrity = static_cast<std::int16_t>(m.integrity.size());
  return encode_as(m);
}

// ---- direction 3D to points (nested types)

aspn23_eigen::TypeRemotePoint remote_point_from(const aspn23_lcm::type_remote_point& r) {
  return aspn23_eigen::TypeRemotePoint(static_cast<std::uint8_t>(r.included_terms), static_cast<std::uint32_t>(r.id),
                                       static_cast<Aspn23TypeRemotePointPositionReferenceFrame>(r.position_reference_frame),
                                       r.position1, r.position2, r.position3, cov_from(r.position_covariance));
}
aspn23_lcm::type_remote_point remote_point_to(const aspn23_eigen::TypeRemotePoint& r) {
  aspn23_lcm::type_remote_point o{};
  o.included_terms = r.get_included_terms();
  o.id = r.get_id();
  o.position_reference_frame = static_cast<std::int8_t>(r.get_position_reference_frame());
  o.position1 = r.get_position1();
  o.position2 = r.get_position2();
  o.position3 = r.get_position3();
  o.position_covariance = cov_to(r.get_position_covariance());
  o.num_position_components = static_cast<std::int16_t>(o.position_covariance.size());
  return o;
}
aspn23_eigen::TypeImageFeature feature_from(const aspn23_lcm::type_image_feature& f) {
  Eigen::Matrix<std::uint8_t, Eigen::Dynamic, 1> d(static_cast<Eigen::Index>(f.descriptor.size()));
  for (std::size_t i = 0; i < f.descriptor.size(); ++i) d(static_cast<Eigen::Index>(i)) = static_cast<std::uint8_t>(f.descriptor[i]);
  return aspn23_eigen::TypeImageFeature(f.response, f.orientation, f.size, static_cast<std::uint16_t>(f.class_id),
                                        static_cast<std::uint16_t>(f.octave), d);
}
aspn23_lcm::type_image_feature feature_to(const aspn23_eigen::TypeImageFeature& f) {
  aspn23_lcm::type_image_feature o{};
  o.response = f.get_response();
  o.orientation = f.get_orientation();
  o.size = f.get_size();
  o.class_id = f.get_class_id();
  o.octave = f.get_octave();
  auto d = f.get_descriptor();
  o.descriptor.assign(d.data(), d.data() + d.size());  // uint8 -> int16
  o.descriptor_size = static_cast<std::int32_t>(o.descriptor.size());
  return o;
}
std::shared_ptr<api::AspnBase> d2p_from(const aspn23_lcm::measurement_direction_3d_to_points& m) {
  std::vector<aspn23_eigen::TypeDirection3DToPoint> obs;
  for (const auto& o : m.obs) {
    DynVec ob(2);
    ob << o.obs[0], o.obs[1];
    RowMat cov(2, 2);
    cov << o.covariance[0][0], o.covariance[0][1], o.covariance[1][0], o.covariance[1][1];
    obs.emplace_back(remote_point_from(o.remote_point), static_cast<Aspn23TypeDirection3DToPointReferenceFrame>(o.reference_frame),
                     ob, cov, o.has_observation_characteristics != 0, feature_from(o.observation_characteristics),
                     static_cast<Aspn23TypeDirection3DToPointErrorModel>(o.error_model), vec_from(o.error_model_params),
                     integrity_from(o.integrity));
  }
  return std::make_shared<aspn23_eigen::MeasurementDirection3DToPoints>(
      header_from(m.header, ASPN_MEASUREMENT_DIRECTION_3D_TO_POINTS), tov_from(m.time_of_validity), obs);
}
std::vector<std::uint8_t> d2p_to(const aspn23_eigen::MeasurementDirection3DToPoints& e) {
  aspn23_lcm::measurement_direction_3d_to_points m{};
  m.header = header_to(e.get_header());
  m.time_of_validity = tov_to(e.get_time_of_validity());
  for (const auto& ob : e.get_obs()) {
    aspn23_lcm::type_direction_3d_to_point o{};
    o.remote_point = remote_point_to(ob.get_remote_point());
    o.reference_frame = static_cast<std::int8_t>(ob.get_reference_frame());
    DynVec v = ob.get_obs();
    o.obs[0] = v.size() > 0 ? v(0) : 0.0;
    o.obs[1] = v.size() > 1 ? v(1) : 0.0;
    RowMat c = ob.get_covariance();
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 2; ++j) o.covariance[i][j] = (c.rows() == 2 && c.cols() == 2) ? c(i, j) : 0.0;
    o.has_observation_characteristics = ob.get_has_observation_characteristics() ? 1 : 0;
    o.observation_characteristics = feature_to(ob.get_observation_characteristics());
    o.error_model = static_cast<std::int8_t>(ob.get_error_model());
    o.error_model_params = vec_to(ob.get_error_model_params());
    o.num_error_model_params = static_cast<std::int32_t>(o.error_model_params.size());
    o.integrity = integrity_to(ob.get_integrity());
    o.num_integrity = static_cast<std::int16_t>(o.integrity.size());
    m.obs.push_back(o);
  }
  m.num_obs = static_cast<std::int32_t>(m.obs.size());
  return encode_as(m);
}
}  // namespace

std::shared_ptr<api::AspnBase> decode(const std::uint8_t* d, std::size_t len) {
  const std::int64_t hash = read_hash(d, len);
  if (hash == aspn23_lcm::measurement_IMU::getHash()) return imu_from(decode_as<aspn23_lcm::measurement_IMU>(d, len));
  if (hash == aspn23_lcm::measurement_position::getHash())
    return position_from(decode_as<aspn23_lcm::measurement_position>(d, len));
  if (hash == aspn23_lcm::measurement_velocity::getHash())
    return velocity_from(decode_as<aspn23_lcm::measurement_velocity>(d, len));
  if (hash == aspn23_lcm::measurement_position_velocity_attitude::getHash())
    return pva_from(decode_as<aspn23_lcm::measurement_position_velocity_attitude>(d, len));
  if (hash == aspn23_lcm::measurement_altitude::getHash())
    return altitude_from(decode_as<aspn23_lcm::measurement_altitude>(d, len));
  if (hash == aspn23_lcm::measurement_barometer::getHash())
    return barometer_from(decode_as<aspn23_lcm::measurement_barometer>(d, len));
  if (hash == aspn23_lcm::measurement_direction_3d_to_points::getHash())
    return d2p_from(decode_as<aspn23_lcm::measurement_direction_3d_to_points>(d, len));
  return nullptr;
}

std::optional<std::string> type_name_for(const std::uint8_t* d, std::size_t len) {
  const std::int64_t hash = read_hash(d, len);
  if (hash == aspn23_lcm::measurement_IMU::getHash()) return "measurement_IMU";
  if (hash == aspn23_lcm::measurement_position::getHash()) return "measurement_position";
  if (hash == aspn23_lcm::measurement_velocity::getHash()) return "measurement_velocity";
  if (hash == aspn23_lcm::measurement_position_velocity_attitude::getHash()) return "measurement_position_velocity_attitude";
  if (hash == aspn23_lcm::measurement_altitude::getHash()) return "measurement_altitude";
  if (hash == aspn23_lcm::measurement_barometer::getHash()) return "measurement_barometer";
  if (hash == aspn23_lcm::measurement_direction_3d_to_points::getHash()) return "measurement_direction_3d_to_points";
  return std::nullopt;
}

std::optional<std::vector<std::uint8_t>> encode(const api::AspnBase& msg) {
  if (auto* p = dynamic_cast<const aspn23_eigen::MeasurementImu*>(&msg)) return imu_to(*p);
  if (auto* p = dynamic_cast<const aspn23_eigen::MeasurementPosition*>(&msg)) return position_to(*p);
  if (auto* p = dynamic_cast<const aspn23_eigen::MeasurementVelocity*>(&msg)) return velocity_to(*p);
  if (auto* p = dynamic_cast<const aspn23_eigen::MeasurementPositionVelocityAttitude*>(&msg)) return pva_to(*p);
  if (auto* p = dynamic_cast<const aspn23_eigen::MeasurementAltitude*>(&msg)) return altitude_to(*p);
  if (auto* p = dynamic_cast<const aspn23_eigen::MeasurementBarometer*>(&msg)) return barometer_to(*p);
  if (auto* p = dynamic_cast<const aspn23_eigen::MeasurementDirection3DToPoints*>(&msg)) return d2p_to(*p);
  return std::nullopt;
}

}  // namespace pntos::cobra::lcm
