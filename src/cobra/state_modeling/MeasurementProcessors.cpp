#include <pntos/cobra/state_modeling/MeasurementProcessors.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <cmath>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Matrix;
using api::Matrix3;
using api::Vector;
using api::Vector3;
using utils::PVA;

namespace {
std::string secs(api::Timestamp t) { return std::to_string(t.seconds()) + "s"; }
std::string secs(const aspn23_eigen::TypeTimestamp& t) { return std::to_string(t.get_elapsed_nsec() * 1e-9) + "s"; }

/// NED delta (m) of llh - inertial_llh evaluated at the measured llh (Cobra convention).
Vector3 ned_delta(const Vector3& llh, const Vector3& inertial_llh) {
  Vector3 z = llh - inertial_llh;
  z(0) = nav::delta_lat_to_north(z(0), llh(0), llh(2));
  z(1) = nav::delta_lon_to_east(z(1), llh(0), llh(2));
  z(2) = -z(2);
  return z;
}
}  // namespace

// ===================================================================== PinsonProcessorBase

PinsonProcessorBase::PinsonProcessorBase(std::string name, std::string label, std::vector<std::string> labels,
                                         api::Mediator* mediator, std::size_t num_required_blocks)
    : name_(std::move(name)),
      label_(std::move(label)),
      state_block_labels_(std::move(labels)),
      mediator_(mediator),
      num_required_blocks_(num_required_blocks) {
  if (num_required_blocks_ > 0 && state_block_labels_.size() != num_required_blocks_) {
    log(LoggingLevel::ERROR, name_ + " requires " + std::to_string(num_required_blocks_) + " state blocks, got " +
                                 std::to_string(state_block_labels_.size()) + ".");
  }
}

void PinsonProcessorBase::log(LoggingLevel level, const std::string& msg) const {
  if (mediator_) mediator_->log_message(level, msg);
}

void PinsonProcessorBase::receive_aux_data(const api::AuxData& aux) {
  if (aux.empty() || !aux[0]) {
    log(LoggingLevel::ERROR,
        name_ + " expected aux data of type MeasurementPositionVelocityAttitude, but received empty list.");
    return;
  }
  if (aux.size() > 1) {
    log(LoggingLevel::WARN, name_ + " expected a single MeasurementPositionVelocityAttitude aux message, but received " +
                                std::to_string(aux.size()) + " aux messages. Ignoring all except the first message.");
  }
  auto pva = aux[0]->as<PVA>();
  if (!pva) {
    log(LoggingLevel::ERROR, name_ + " expected aux data of type MeasurementPositionVelocityAttitude, but got message of type " +
                                 std::to_string(static_cast<int>(aux[0]->message_type())) + ".");
    return;
  }
  if (!utils::quaternion(*pva)) {
    log(LoggingLevel::ERROR, name_ + " received PVA aux data with no quaternion at time " +
                                 secs(pva->get_time_of_validity()) + ".");
    return;
  }
  inertial_pva_ = pva;
}

bool PinsonProcessorBase::check_common(api::Timestamp meas_time) const {
  if (num_required_blocks_ > 0 && state_block_labels_.size() != num_required_blocks_) {
    log(LoggingLevel::ERROR, name_ + " has wrong number of state blocks. Cannot generate model.");
    return false;
  }
  if (!inertial_pva_) {
    log(LoggingLevel::ERROR, name_ + " cannot process message at time " + secs(meas_time) +
                                 " as it has not received inertial PVA aux data.");
    return false;
  }
  const api::Timestamp aux_time(inertial_pva_->get_time_of_validity());
  if (aux_time != meas_time) {
    log(LoggingLevel::ERROR, name_ + " cannot process message at time " + secs(meas_time) +
                                 " as inertial PVA aux data is at a different time (t=" + secs(aux_time) + ").");
    return false;
  }
  return true;
}

// ======================================================= PinsonPositionMeasurementProcessor

namespace {
const char* kind_name(PinsonPositionMeasurementProcessor::Kind k) {
  switch (k) {
    case PinsonPositionMeasurementProcessor::Kind::Plain: return "PinsonPositionMeasurementProcessor";
    case PinsonPositionMeasurementProcessor::Kind::WithNedFogm: return "PinsonWithNedFogmPositionMeasurementProcessor";
    case PinsonPositionMeasurementProcessor::Kind::WithLeverArm:
      return "PinsonWithLeverArmPositionMeasurementProcessor";
  }
  return "PinsonPositionMeasurementProcessor";
}
std::size_t kind_blocks(PinsonPositionMeasurementProcessor::Kind k) {
  switch (k) {
    case PinsonPositionMeasurementProcessor::Kind::Plain: return 1;
    case PinsonPositionMeasurementProcessor::Kind::WithNedFogm: return 2;
    case PinsonPositionMeasurementProcessor::Kind::WithLeverArm: return 3;
  }
  return 1;
}
}  // namespace

PinsonPositionMeasurementProcessor::PinsonPositionMeasurementProcessor(Kind kind, std::string label,
                                                                       std::vector<std::string> labels,
                                                                       api::Mediator* mediator,
                                                                       const Vector3& l_ps_p)
    : PinsonProcessorBase(kind_name(kind), std::move(label), std::move(labels), mediator, kind_blocks(kind)),
      kind_(kind),
      l_ps_p_(l_ps_p) {}

std::optional<api::StandardMeasurementModel> PinsonPositionMeasurementProcessor::generate_model(
    const api::Message& message, const api::GenXandP& gen_x_and_p) {
  auto pos = message.as<aspn23_eigen::MeasurementPosition>();
  if (!pos) {
    log(LoggingLevel::ERROR, name_ + " expected message of type MeasurementPosition, but got message of type " +
                                 std::to_string(static_cast<int>(message.message_type())) + ". Cannot process message.");
    return std::nullopt;
  }
  const api::Timestamp time(pos->get_time_of_validity());
  if (!check_common(time)) return std::nullopt;
  if (pos->get_reference_frame() != ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC) {
    log(LoggingLevel::ERROR, name_ + " expected MeasurementPosition with a GEODETIC reference frame, but got measurement at time " +
                                 secs(time) + " with a different reference frame. Cannot process message.");
    return std::nullopt;
  }
  const Vector3 llh(pos->get_term1(), pos->get_term2(), pos->get_term3());
  const Vector3 inertial_llh = utils::position(*inertial_pva_);
  const Matrix3 C_platform_to_nav = nav::quat_to_dcm(*utils::quaternion(*inertial_pva_));
  const Vector3 z = ned_delta(llh, inertial_llh);

  auto ewc = gen_x_and_p(state_block_labels_);
  if (!ewc) return std::nullopt;
  const Eigen::Index n = ewc->estimate.size();

  Matrix H = Matrix::Zero(3, n);
  H.block<3, 3>(0, 0) = Matrix3::Identity();
  const Vector3 l = l_ps_p_;
  const Kind kind = kind_;
  std::function<Vector(const Vector&)> h;
  switch (kind) {
    case Kind::Plain:
      H.block<3, 3>(0, 6) = nav::skew(C_platform_to_nav * l);
      h = [C_platform_to_nav, l](const Vector& x) {
        Vector3 r = x.head<3>() + (Matrix3::Identity() - nav::skew(x.segment<3>(6))) * C_platform_to_nav * l;
        return Vector(r);
      };
      break;
    case Kind::WithNedFogm:
      H.block<3, 3>(0, 6) = nav::skew(C_platform_to_nav * l);
      H.block(0, n - 3, 3, 3) = -Matrix3::Identity();
      h = [C_platform_to_nav, l](const Vector& x) {
        Vector3 r = x.head<3>() + (Matrix3::Identity() - nav::skew(x.segment<3>(6))) * C_platform_to_nav * l -
                    x.tail<3>();
        return Vector(r);
      };
      break;
    case Kind::WithLeverArm: {
      const Vector3 dl = ewc->estimate.tail<3>();
      H.block<3, 3>(0, 6) = nav::skew(C_platform_to_nav * (l + dl));
      H.block(0, n - 3, 3, 3) = (Matrix3::Identity() - nav::skew(ewc->estimate.segment<3>(6))) * C_platform_to_nav;
      H.block(0, n - 6, 3, 3) = -Matrix3::Identity();
      h = [C_platform_to_nav, l](const Vector& x) {
        const Eigen::Index m = x.size();
        Vector3 r = x.head<3>() +
                    (Matrix3::Identity() - nav::skew(x.segment<3>(6))) * C_platform_to_nav * (l + x.tail<3>()) -
                    x.segment<3>(m - 6);
        return Vector(r);
      };
      break;
    }
  }
  api::StandardMeasurementModel model;
  model.z = z;
  model.h = std::move(h);
  model.H = H;
  model.R = pos->get_covariance();
  return model;
}

// ======================================================= PinsonVelocityMeasurementProcessor

PinsonVelocityMeasurementProcessor::PinsonVelocityMeasurementProcessor(std::string label,
                                                                       std::vector<std::string> labels,
                                                                       api::Mediator* mediator)
    : PinsonProcessorBase("PinsonVelocityMeasurementProcessor", std::move(label), std::move(labels), mediator, 0) {}

std::optional<api::StandardMeasurementModel> PinsonVelocityMeasurementProcessor::generate_model(
    const api::Message& message, const api::GenXandP& gen_x_and_p) {
  auto vel = message.as<aspn23_eigen::MeasurementVelocity>();
  if (!vel) {
    log(LoggingLevel::ERROR, name_ + " expected message of type MeasurementVelocity, but got another type. Cannot process message.");
    return std::nullopt;
  }
  const api::Timestamp time(vel->get_time_of_validity());
  if (!check_common(time)) return std::nullopt;
  if (vel->get_reference_frame() != ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED) {
    log(LoggingLevel::ERROR, name_ + " expected MeasurementVelocity with a NED reference frame at time " + secs(time) +
                                 ". Cannot process message.");
    return std::nullopt;
  }
  const Vector3 meas_vel(vel->get_x(), vel->get_y(), vel->get_z());
  const Vector3 inertial_vel = utils::velocity(*inertial_pva_);
  auto ewc = gen_x_and_p(state_block_labels_);
  if (!ewc) return std::nullopt;
  Matrix H = Matrix::Zero(3, ewc->estimate.size());
  H.block<3, 3>(0, 3) = Matrix3::Identity();
  api::StandardMeasurementModel model;
  model.z = meas_vel - inertial_vel;
  model.h = [H](const Vector& x) { return Vector(H * x); };
  model.H = H;
  model.R = vel->get_covariance();
  return model;
}

// ========================================================= PinsonPosVelMeasurementProcessor

PinsonPosVelMeasurementProcessor::PinsonPosVelMeasurementProcessor(std::string label, std::vector<std::string> labels,
                                                                   api::Mediator* mediator, const Vector3& l_ps_p)
    : PinsonProcessorBase("PinsonPosVelMeasurementProcessor", std::move(label), std::move(labels), mediator, 1),
      l_ps_p_(l_ps_p) {}

void PinsonPosVelMeasurementProcessor::receive_aux_data(const api::AuxData& aux) {
  if (!aux.empty() && aux[0]) {
    if (auto pva = aux[0]->as<PVA>()) {
      if (pva->get_reference_frame() != ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC) {
        log(LoggingLevel::ERROR, name_ + " expected MeasurementPositionVelocityAttitude aux with a GEODETIC reference frame. Cannot process message.");
        return;
      }
    }
  }
  PinsonProcessorBase::receive_aux_data(aux);
}

std::optional<api::StandardMeasurementModel> PinsonPosVelMeasurementProcessor::generate_model(
    const api::Message& message, const api::GenXandP& gen_x_and_p) {
  auto pva = message.as<PVA>();
  if (!pva) {
    log(LoggingLevel::ERROR, name_ + " expected message of type MeasurementPositionVelocityAttitude, but got another type. Cannot process message.");
    return std::nullopt;
  }
  const api::Timestamp time(pva->get_time_of_validity());
  if (!check_common(time)) return std::nullopt;
  if (pva->get_reference_frame() != ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC) {
    log(LoggingLevel::ERROR, name_ + " expected a GEODETIC PVA at time " + secs(time) + ". Cannot process message.");
    return std::nullopt;
  }
  if (!utils::has_position(*pva) || !utils::has_velocity(*pva)) {
    log(LoggingLevel::ERROR, name_ + " cannot process message at time " + secs(time) + " as the measurement is missing fields.");
    return std::nullopt;
  }
  if (!utils::has_position(*inertial_pva_) || !utils::has_velocity(*inertial_pva_)) {
    log(LoggingLevel::ERROR, name_ + " cannot process message at time " + secs(time) + " as the inertial PVA is missing fields.");
    return std::nullopt;
  }
  const Matrix3 C_platform_to_nav = nav::quat_to_dcm(*utils::quaternion(*inertial_pva_));
  Vector z(6);
  z.head<3>() = ned_delta(utils::position(*pva), utils::position(*inertial_pva_));
  z.tail<3>() = utils::velocity(*pva) - utils::velocity(*inertial_pva_);

  auto ewc = gen_x_and_p(state_block_labels_);
  if (!ewc) return std::nullopt;
  Matrix H = Matrix::Zero(6, ewc->estimate.size());
  H.block<3, 3>(0, 0) = Matrix3::Identity();
  H.block<3, 3>(0, 6) = nav::skew(C_platform_to_nav * l_ps_p_);
  H.block<3, 3>(3, 3) = Matrix3::Identity();
  const Vector3 l = l_ps_p_;
  api::StandardMeasurementModel model;
  model.z = z;
  model.h = [C_platform_to_nav, l](const Vector& x) {
    Vector out(6);
    out.head<3>() = x.head<3>() + (Matrix3::Identity() - nav::skew(x.segment<3>(6))) * C_platform_to_nav * l;
    out.tail<3>() = x.segment<3>(3);
    return out;
  };
  model.H = H;
  // Attitude may be present in the PVA measurement, but only the position/velocity covariance is used.
  Matrix R = pva->get_covariance();
  if (utils::quaternion(*pva) && R.rows() > 6) R = R.topLeftCorner(6, 6).eval();
  model.R = R;
  return model;
}

// ==================================================== PinsonBodyVelocityMeasurementProcessor

PinsonBodyVelocityMeasurementProcessor::PinsonBodyVelocityMeasurementProcessor(std::string label,
                                                                               std::vector<std::string> labels,
                                                                               api::Mediator* mediator,
                                                                               const Vector3& l_ps_p,
                                                                               const nav::Vector4& orientation_ps_p)
    : PinsonProcessorBase("PinsonBodyVelocityMeasurementProcessor", std::move(label), std::move(labels), mediator, 0),
      l_ps_p_(l_ps_p),
      orientation_ps_p_(orientation_ps_p) {}

void PinsonBodyVelocityMeasurementProcessor::receive_aux_data(const api::AuxData& aux) {
  if (aux.empty()) {
    log(LoggingLevel::ERROR, name_ + " expected aux data of type MeasurementPositionVelocityAttitude and MeasurementImu, but received empty list.");
    return;
  }
  if (aux.size() < 2) {
    log(LoggingLevel::ERROR, name_ + " expected two aux messages: MeasurementPositionVelocityAttitude and MeasurementImu, but received " +
                                 std::to_string(aux.size()) + " aux messages.");
    return;
  }
  if (aux.size() > 2) {
    log(LoggingLevel::WARN, name_ + " expected two aux messages but received " + std::to_string(aux.size()) +
                                ". Ignoring all except the first two messages.");
  }
  if (!aux[0] || !aux[1]) {
    log(LoggingLevel::ERROR, name_ + " received a NULL message as aux data.");
    return;
  }
  auto pva = aux[0]->as<PVA>();
  if (!pva) {
    log(LoggingLevel::ERROR, name_ + " expected aux[0] data of type MeasurementPositionVelocityAttitude.");
    return;
  }
  if (!utils::quaternion(*pva)) {
    log(LoggingLevel::ERROR, name_ + " received PVA aux data with no quaternion at time " + secs(pva->get_time_of_validity()) + ".");
    return;
  }
  inertial_pva_ = pva;
  auto imu = aux[1]->as<aspn23_eigen::MeasurementImu>();
  if (!imu) {
    log(LoggingLevel::ERROR, name_ + " expected aux[1] data of type MeasurementImu.");
    return;
  }
  force_and_rate_aux_ = imu;
}

std::optional<api::StandardMeasurementModel> PinsonBodyVelocityMeasurementProcessor::generate_model(
    const api::Message& message, const api::GenXandP& gen_x_and_p) {
  auto vel = message.as<aspn23_eigen::MeasurementVelocity>();
  if (!vel) {
    log(LoggingLevel::ERROR, name_ + " expected message of type MeasurementVelocity. Cannot process message.");
    return std::nullopt;
  }
  const api::Timestamp time(vel->get_time_of_validity());
  if (!check_common(time)) return std::nullopt;
  if (vel->get_reference_frame() != ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_SENSOR) {
    log(LoggingLevel::ERROR, name_ + " expected MeasurementVelocity with a SENSOR reference frame at time " + secs(time) +
                                 ". Cannot process message.");
    return std::nullopt;
  }
  if (!force_and_rate_aux_) {
    log(LoggingLevel::ERROR, name_ + " cannot process message at time " + secs(time) + " as it has not received IMU aux data.");
    return std::nullopt;
  }
  auto ewc = gen_x_and_p(state_block_labels_);
  if (!ewc) return std::nullopt;
  const Vector& x = ewc->estimate;
  const Eigen::Index num_states = x.size();
  const Vector3 inertial_vel = utils::velocity(*inertial_pva_);
  const Matrix3 C_platform_to_sensor = nav::quat_to_dcm(orientation_ps_p_);
  const Matrix3 uncorr_C_ned_to_imu = nav::quat_to_dcm(*utils::quaternion(*inertial_pva_)).transpose();
  const Vector3 att_error = x.segment<3>(6);
  const Matrix3 corr_C_ned_to_imu = uncorr_C_ned_to_imu * (Matrix3::Identity() + nav::skew(att_error));
  const Matrix3 C_ned_to_sensor = C_platform_to_sensor * corr_C_ned_to_imu;
  const Matrix3 uncorr_C_ned_to_sensor = C_platform_to_sensor * uncorr_C_ned_to_imu;
  const Vector3 corr_inertial_vel_ned = inertial_vel + x.segment<3>(3);
  const Matrix3 C_ned_to_sensor_der = -uncorr_C_ned_to_sensor * nav::skew(corr_inertial_vel_ned);

  // Mask of present axes (ASPN optional fields are NaN when absent)
  const double raw[3] = {vel->get_x(), vel->get_y(), vel->get_z()};
  std::vector<int> mask;
  for (int i = 0; i < 3; ++i)
    if (utils::present(raw[i])) mask.push_back(i);
  Vector z(static_cast<Eigen::Index>(mask.size()));
  for (std::size_t i = 0; i < mask.size(); ++i) z(static_cast<Eigen::Index>(i)) = raw[mask[i]];

  Matrix H_full = Matrix::Zero(3, num_states);
  H_full.block<3, 3>(0, 3) = C_ned_to_sensor;
  H_full.block<3, 3>(0, 6) = C_ned_to_sensor_der;
  Matrix H(static_cast<Eigen::Index>(mask.size()), num_states);
  for (std::size_t i = 0; i < mask.size(); ++i) H.row(static_cast<Eigen::Index>(i)) = H_full.row(mask[i]);

  const Vector3 rotation_rate = Vector3(force_and_rate_aux_->get_meas_gyro());
  const Vector3 l = l_ps_p_;
  const double p1 = inertial_pva_->get_p1(), p3 = inertial_pva_->get_p3();
  const bool any_z = z.array().any();
  api::StandardMeasurementModel model;
  model.z = z;
  model.H = H;
  model.R = vel->get_covariance();
  model.h = [=](const Vector& xx) {
    Vector3 tan_vel_sensor = Vector3::Zero();
    const double alt = p3 - xx(2);
    const double lat = p1 + nav::north_to_delta_lat(xx(0), p1, alt);
    const Vector3 gyro_bias = num_states >= 15 ? Vector3(xx.segment<3>(12)) : Vector3::Zero();
    if (any_z && l.array().any()) {
      const double rn = nav::meridian_radius(lat), re = nav::transverse_radius(lat);
      const Vector3 v = corr_inertial_vel_ned;
      const Vector3 w_en_n(v(1) / (re + alt), -v(0) / (rn + alt), -v(1) * std::tan(lat) / (re + alt));
      const Vector3 w_ie_n(nav::ROTATION_RATE * std::cos(lat), 0.0, -nav::ROTATION_RATE * std::sin(lat));
      const Vector3 rate = rotation_rate + gyro_bias - corr_C_ned_to_imu * (w_ie_n - w_en_n);
      tan_vel_sensor = C_platform_to_sensor * rate.cross(l);
    }
    const Vector3 sensor_vel = C_ned_to_sensor * corr_inertial_vel_ned + tan_vel_sensor;
    Vector out(static_cast<Eigen::Index>(mask.size()));
    for (std::size_t i = 0; i < mask.size(); ++i) out(static_cast<Eigen::Index>(i)) = sensor_vel(mask[i]);
    return out;
  };
  return model;
}

// ============================================================ AltitudeMeasurementProcessor

AltitudeMeasurementProcessor::AltitudeMeasurementProcessor(std::string label, std::vector<std::string> labels,
                                                           api::Mediator* mediator, const Vector3& l_ps_p,
                                                           std::shared_ptr<const nav::Geoid> geoid)
    : PinsonProcessorBase("AltitudeMeasurementProcessor", std::move(label), std::move(labels), mediator, 0),
      l_ps_p_(l_ps_p),
      geoid_(std::move(geoid)) {}

void AltitudeMeasurementProcessor::receive_aux_data(const api::AuxData& aux) {
  if (aux.empty() || !aux[0]) {
    log(LoggingLevel::ERROR, name_ + " expected aux data of type MeasurementPositionVelocityAttitude, but received empty list.");
    return;
  }
  if (aux.size() > 1) log(LoggingLevel::WARN, name_ + " expected a single PVA aux message. Ignoring all except the first.");
  auto pva = aux[0]->as<PVA>();
  if (!pva) {
    log(LoggingLevel::ERROR, name_ + " expected aux data of type MeasurementPositionVelocityAttitude.");
    return;
  }
  if (!utils::has_position(*pva)) {
    log(LoggingLevel::ERROR, name_ + " received PVA aux data with no position at time " + secs(pva->get_time_of_validity()) + ".");
    return;
  }
  auto q = utils::quaternion(*pva);
  if (!q) {
    log(LoggingLevel::ERROR, name_ + " received PVA aux data with no quaternion at time " + secs(pva->get_time_of_validity()) + ".");
    return;
  }
  if (pva->get_reference_frame() != ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC) {
    log(LoggingLevel::ERROR, name_ + " received PVA aux data with a non-GEODETIC reference frame.");
    return;
  }
  inertial_pva_ = pva;
  inertial_time_ = api::Timestamp(pva->get_time_of_validity());
  inertial_pos_ = utils::position(*pva);
  C_platform_to_nav_ = nav::quat_to_dcm(*q);
}

std::optional<api::StandardMeasurementModel> AltitudeMeasurementProcessor::generate_model(
    const api::Message& message, const api::GenXandP& gen_x_and_p) {
  if (!inertial_time_) {
    log(LoggingLevel::ERROR, name_ + " cannot process message as it has not received inertial PVA aux data.");
    return std::nullopt;
  }
  double alt = 0, variance = 0;
  api::Timestamp time;
  if (auto a = message.as<aspn23_eigen::MeasurementAltitude>()) {
    time = api::Timestamp(a->get_time_of_validity());
    alt = a->get_altitude();
    variance = a->get_variance();
    if (a->get_reference() == ASPN23_MEASUREMENT_ALTITUDE_REFERENCE_MSL) {
      if (!geoid_) {
        log(LoggingLevel::ERROR, name_ + ": MSL altitudes require a geoid model (geoid_file in the processor config or PNTOS_GEOID_FILE). Cannot process message.");
        return std::nullopt;
      }
      if (!inertial_time_) {
        log(LoggingLevel::ERROR, name_ + " cannot convert an MSL altitude without the inertial position.");
        return std::nullopt;
      }
      alt += geoid_->undulation(inertial_pos_(0), inertial_pos_(1));  // MSL -> HAE
    }
  } else if (auto p = message.as<aspn23_eigen::MeasurementPosition>();
             p && p->get_reference_frame() == ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC) {
    time = api::Timestamp(p->get_time_of_validity());
    alt = p->get_term3();
    variance = p->get_covariance()(2, 2);
  } else if (auto v = message.as<PVA>();
             v && v->get_reference_frame() == ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC) {
    time = api::Timestamp(v->get_time_of_validity());
    alt = v->get_p3();
    variance = v->get_covariance()(2, 2);
  } else {
    log(LoggingLevel::ERROR, name_ + " expected MeasurementAltitude, or a GEODETIC MeasurementPosition / MeasurementPositionVelocityAttitude. Cannot process message.");
    return std::nullopt;
  }
  if (!utils::present(alt)) {
    log(LoggingLevel::ERROR, name_ + " got message without a valid altitude at time " + secs(time) + ". Cannot process message.");
    return std::nullopt;
  }
  if (*inertial_time_ != time) {
    log(LoggingLevel::ERROR, name_ + " cannot process message at time " + secs(time) +
                                 " as inertial PVA aux data is at a different time (t=" + secs(*inertial_time_) + ").");
    return std::nullopt;
  }
  auto ewc = gen_x_and_p(state_block_labels_);
  if (!ewc) return std::nullopt;
  const Eigen::Index n = ewc->estimate.size();
  Matrix H = Matrix::Zero(1, n);
  H(0, 2) = -1;
  H(0, n - 1) = 1;
  H.block<1, 3>(0, 6) = nav::skew(C_platform_to_nav_ * l_ps_p_).row(2);
  const Matrix3 C = C_platform_to_nav_;
  const Vector3 l = l_ps_p_;
  api::StandardMeasurementModel model;
  model.z = Vector::Constant(1, alt - inertial_pos_(2));
  model.H = H;
  model.R = Matrix::Constant(1, 1, variance);
  model.h = [C, l](const Vector& x) {
    const Vector3 lever_arm_diff = (Matrix3::Identity() - nav::skew(x.segment<3>(6))) * C * l;
    return Vector::Constant(1, -x(2) + x(x.size() - 1) + lever_arm_diff(2));
  };
  return model;
}

// ============================================================ PositionMeasurementProcessor

PositionMeasurementProcessor::PositionMeasurementProcessor(std::string label, std::vector<std::string> labels,
                                                           api::Mediator* mediator, const Vector3& l_ps_p)
    : label_(std::move(label)), state_block_labels_(std::move(labels)), mediator_(mediator), l_ps_p_(l_ps_p) {
  if (state_block_labels_.size() != 2 && mediator_)
    mediator_->log_message(LoggingLevel::ERROR, "PositionMeasurementProcessor expects two state block labels but received " +
                                                    std::to_string(state_block_labels_.size()) + ".");
}

void PositionMeasurementProcessor::receive_aux_data(const api::AuxData&) {
  if (mediator_) mediator_->log_message(LoggingLevel::DEBUG, "PositionMeasurementProcessor does not require aux data.");
}

std::optional<api::StandardMeasurementModel> PositionMeasurementProcessor::generate_model(
    const api::Message& message, const api::GenXandP& gen_x_and_p) {
  auto pos = message.as<aspn23_eigen::MeasurementPosition>();
  if (!pos) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::ERROR, "PositionMeasurementProcessor expected message of type MeasurementPosition. Cannot process message.");
    return std::nullopt;
  }
  const api::Timestamp time(pos->get_time_of_validity());
  if (pos->get_reference_frame() != ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::ERROR, "PositionMeasurementProcessor expected a GEODETIC MeasurementPosition at time " +
                                                      secs(time) + ". Cannot process message.");
    return std::nullopt;
  }
  const Vector3 z(pos->get_term1(), pos->get_term2(), pos->get_term3());
  if (z.hasNaN()) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::ERROR, "PositionMeasurementProcessor received a MeasurementPosition at time " +
                                                      secs(time) + " with an invalid position. Cannot process message.");
    return std::nullopt;
  }
  const double north_fac = nav::north_to_delta_lat(1, z(0), z(2));
  const double east_fac = nav::east_to_delta_lon(1, z(0), z(2));
  Matrix3 conversion = Matrix3::Zero();
  conversion(0, 0) = north_fac;
  conversion(1, 1) = east_fac;
  conversion(2, 2) = -1.0;
  const Matrix R = conversion * pos->get_covariance() * conversion;

  auto ewc = gen_x_and_p(state_block_labels_);
  if (!ewc) return std::nullopt;
  const Eigen::Index n = ewc->estimate.size();
  Matrix H = Matrix::Identity(3, n);
  const Vector3 rpy = ewc->estimate.segment<3>(6);
  H.col(6) = conversion * nav::d_rpy_to_dcm_wrt_r(rpy) * l_ps_p_;
  H.col(7) = conversion * nav::d_rpy_to_dcm_wrt_p(rpy) * l_ps_p_;
  H.col(8) = conversion * nav::d_rpy_to_dcm_wrt_y(rpy) * l_ps_p_;
  H.block(0, n - 3, 3, 3) = -Matrix3::Identity();

  const Vector3 l = l_ps_p_;
  api::StandardMeasurementModel model;
  model.z = z;
  model.H = H;
  model.R = R;
  model.h = [z, l](const Vector& x) {
    const Vector3 arm_ned = nav::rpy_to_dcm(x.segment<3>(6)) * l;
    const Vector3 arm_llh(nav::north_to_delta_lat(arm_ned(0), z(0), z(2)), nav::east_to_delta_lon(arm_ned(1), z(0), z(2)),
                          -arm_ned(2));
    Vector3 r = x.head<3>() + arm_llh - x.tail<3>();
    return Vector(r);
  };
  return model;
}

// ================================================= Direction3DToPointsMeasurementProcessor

Direction3DToPointsMeasurementProcessor::Direction3DToPointsMeasurementProcessor(std::string label,
                                                                                 std::vector<std::string> labels,
                                                                                 api::Mediator* mediator,
                                                                                 const Vector3& l_ps_p,
                                                                                 const nav::Vector4& orientation)
    : PinsonProcessorBase("Direction3DToPointsMeasurementProcessor", std::move(label), std::move(labels), mediator, 1),
      l_ps_p_(l_ps_p),
      orientation_(orientation) {}

std::optional<api::StandardMeasurementModel> Direction3DToPointsMeasurementProcessor::generate_model(
    const api::Message& message, const api::GenXandP& gen_x_and_p) {
  auto d2p = message.as<aspn23_eigen::MeasurementDirection3DToPoints>();
  if (!d2p) {
    log(LoggingLevel::ERROR, name_ + " expected message of type MeasurementDirection3DToPoints. Cannot process message.");
    return std::nullopt;
  }
  const api::Timestamp time(d2p->get_time_of_validity());
  if (!check_common(time)) return std::nullopt;
  const auto obs = d2p->get_obs();
  if (obs.empty()) {
    log(LoggingLevel::ERROR, name_ + " received a message with no observations at time " + secs(time) + ".");
    return std::nullopt;
  }
  if (obs[0].get_reference_frame() != ASPN23_TYPE_DIRECTION_3D_TO_POINT_REFERENCE_FRAME_SINE_SPACE) {
    log(LoggingLevel::ERROR, name_ + " expected observations in the SINE_SPACE reference frame at time " + secs(time) +
                                 ". Cannot process message.");
    return std::nullopt;
  }
  const Vector3 inertial_llh = utils::position(*inertial_pva_);
  const Matrix3 C_nav_to_platform = nav::quat_to_dcm(*utils::quaternion(*inertial_pva_)).transpose();
  const Matrix3 C_platform_to_sensor = nav::quat_to_dcm(orientation_);
  const Matrix3 C_nav_to_sensor = C_platform_to_sensor * C_nav_to_platform;
  const Vector3 l_s = l_ps_p_;

  auto ewc = gen_x_and_p(state_block_labels_);
  if (!ewc) return std::nullopt;
  const Eigen::Index num_states = ewc->estimate.size();
  const std::size_t N = obs.size();

  // Per-feature geometry
  std::vector<Vector3> delta_pos(N);        // NED, feature - inertial
  std::vector<Eigen::Matrix<double, 2, 3>> temp(N);  // rows 1:3 of (I - u u^T)/|v|
  Vector z(static_cast<Eigen::Index>(2 * N));
  Matrix H = Matrix::Zero(static_cast<Eigen::Index>(2 * N), num_states);
  Matrix R = Matrix::Zero(static_cast<Eigen::Index>(2 * N), static_cast<Eigen::Index>(2 * N));
  for (std::size_t k = 0; k < N; ++k) {
    const auto rp = obs[k].get_remote_point();
    const Vector3 feature_llh(rp.get_position1(), rp.get_position2(), rp.get_position3());
    Vector3 dp = feature_llh - inertial_llh;
    dp(0) = nav::delta_lat_to_north(dp(0), inertial_llh(0), inertial_llh(2));
    dp(1) = nav::delta_lon_to_east(dp(1), inertial_llh(0), inertial_llh(2));
    dp(2) = -dp(2);
    delta_pos[k] = dp;
    const Vector3 dps = C_nav_to_sensor * dp - C_platform_to_sensor * l_s;
    const double norm = dps.norm();
    const Vector3 u = dps / norm;
    const Eigen::Matrix<double, 2, 1> direction_obs = Eigen::Matrix<double, 2, 1>(obs[k].get_obs().head<2>());
    z.segment<2>(static_cast<Eigen::Index>(2 * k)) = direction_obs - u.tail<2>();
    const Matrix3 A = (Matrix3::Identity() - u * u.transpose()) / norm;
    temp[k] = A.bottomRows<2>();
    H.block(static_cast<Eigen::Index>(2 * k), 0, 2, 3) = temp[k] * (-C_nav_to_sensor);
    H.block(static_cast<Eigen::Index>(2 * k), 6, 2, 3) = temp[k] * (C_nav_to_sensor * -nav::skew(dp));
    Matrix Rk = obs[k].get_covariance();
    Matrix feat_cov = rp.get_position_covariance();
    if (feat_cov.size() == 0) feat_cov = Matrix::Zero(2, 2);
    // Add feature uncertainty if available (Python: H_slice @ feat_cov @ H_slice.T with a 2x2 feat_cov)
    const Matrix H_slice = H.block(static_cast<Eigen::Index>(2 * k), 0, 2, 3);
    if (feat_cov.rows() == 3 && feat_cov.cols() == 3) Rk += H_slice * feat_cov * H_slice.transpose();
    else if (feat_cov.rows() == 2 && feat_cov.cols() == 2) Rk += feat_cov;  // degenerate Python path
    R.block(static_cast<Eigen::Index>(2 * k), static_cast<Eigen::Index>(2 * k), 2, 2) = Rk;
  }

  api::StandardMeasurementModel model;
  model.z = z;
  model.H = H;
  model.R = R;
  model.h = [=](const Vector& x) {
    const Vector3 dpos_ned = x.head<3>();
    const Vector3 dtheta_ned = x.segment<3>(6);
    Vector out(static_cast<Eigen::Index>(2 * N));
    for (std::size_t k = 0; k < N; ++k) {
      const Eigen::Matrix<double, 2, 1> v =
          -temp[k] * C_nav_to_sensor * dpos_ned - temp[k] * C_nav_to_sensor * nav::skew(delta_pos[k]) * dtheta_ned;
      out.segment<2>(static_cast<Eigen::Index>(2 * k)) = v;
    }
    return out;
  };
  return model;
}

}  // namespace pntos::cobra
