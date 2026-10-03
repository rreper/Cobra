#include <pntos/cobra/state_modeling/VirtualStateBlocks.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <set>
#include <stdexcept>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Matrix;
using api::Matrix3;
using api::Vector;
using api::Vector3;

namespace {
constexpr int POS_START = 0, VEL_START = 3, ATT_START = 6, ATT_END = 9;
const Matrix3 kDx = (Matrix3() << 0, 0, 0, 0, 0, -1, 0, 1, 0).finished();
const Matrix3 kDy = (Matrix3() << 0, 0, 1, 0, 0, 0, -1, 0, 0).finished();
const Matrix3 kDz = (Matrix3() << 0, -1, 0, 1, 0, 0, 0, 0, 0).finished();
}  // namespace

// ---------------------------------------------------------------------- PinsonErrorToStandard

PinsonErrorToStandard::PinsonErrorToStandard(api::Mediator* mediator, std::string source, std::string target)
    : mediator_(mediator), source_(std::move(source)), target_(std::move(target)) {}

void PinsonErrorToStandard::receive_aux_data(const api::AuxData& aux) {
  for (auto it = aux.rbegin(); it != aux.rend(); ++it) {
    if (!*it) continue;
    if (auto pva = (*it)->as<utils::PVA>()) {
      if (pva->get_reference_frame() == ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC) {
        pva_ = pva;
        break;
      }
    }
  }
}

PinsonErrorToStandard::Nominal PinsonErrorToStandard::nominal(api::Timestamp time, const char* what) const {
  auto fail = [&](const std::string& msg) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, msg);
    throw std::runtime_error(msg);
  };
  if (!pva_) fail(std::string("No PVA aux data has been provided to PinsonErrorToStandard. Cannot ") + what + ".");
  auto q = utils::quaternion(*pva_);
  if (!q) fail(std::string("An invalid quaternion was provided to PinsonErrorToStandard. Cannot ") + what + ".");
  if (api::Timestamp(pva_->get_time_of_validity()) != time)
    fail("Requested time " + std::to_string(time.elapsed_nsec) + " does not match the latest PVA solution at time " +
         std::to_string(pva_->get_time_of_validity().get_elapsed_nsec()) + ". Cannot " + what + ".");
  if (!utils::has_position(*pva_) || !utils::has_velocity(*pva_))
    fail(std::string("Invalid PVA aux data. Cannot ") + what + ".");
  return Nominal{utils::position(*pva_), utils::velocity(*pva_), *q};
}

api::EstimateWithCovariance PinsonErrorToStandard::convert(const api::EstimateWithCovariance& ewc,
                                                           api::Timestamp time) {
  Vector state = convert_estimate(ewc.estimate, time);
  Matrix jac = jacobian(ewc.estimate, time);
  Matrix cov = jac * ewc.covariance * jac.transpose();
  return api::EstimateWithCovariance{api::EstimateWithCovarianceType::EWC_GENERIC, state, cov};
}

Vector PinsonErrorToStandard::convert_estimate(const Vector& estimate, api::Timestamp time) {
  const Nominal n = nominal(time, "convert estimate");
  const double delta_lat = nav::north_to_delta_lat(estimate(POS_START), n.pos(0), n.pos(2));
  const double delta_lon = nav::east_to_delta_lon(estimate(POS_START + 1), n.pos(0), n.pos(2));
  const double delta_alt = -estimate(POS_START + 2);
  Vector out(estimate.size());
  out(0) = delta_lat + n.pos(0);
  out(1) = delta_lon + n.pos(1);
  out(2) = delta_alt + n.pos(2);
  out(3) = n.vel(0) + estimate(VEL_START);
  out(4) = n.vel(1) + estimate(VEL_START + 1);
  out(5) = n.vel(2) + estimate(VEL_START + 2);
  const Matrix3 pva_dcm = nav::quat_to_dcm(n.quat).transpose();
  const Vector3 tilt = estimate.segment<3>(ATT_START);
  const Matrix3 corr = nav::ortho_dcm(pva_dcm * (Matrix3::Identity() + nav::skew(tilt))).transpose();
  out.segment<3>(6) = nav::dcm_to_rpy(corr);
  if (estimate.size() > ATT_END) out.tail(estimate.size() - ATT_END) = estimate.tail(estimate.size() - ATT_END);
  return out;
}

Matrix PinsonErrorToStandard::jacobian(const Vector& estimate, api::Timestamp time) {
  const Nominal n = nominal(time, "create jacobian");
  Matrix3 m2r;
  m2r << 1.0 / nav::delta_lat_to_north(1, n.pos(0), n.pos(2)), 0, 0,  //
      0, 1.0 / nav::delta_lon_to_east(1, n.pos(0), n.pos(2)), 0,       //
      0, 0, -1;
  Matrix jac = Matrix::Identity(estimate.size(), estimate.size());
  jac.block<3, 3>(POS_START, POS_START) = m2r;
  const Matrix3 pva_dcm = nav::quat_to_dcm(n.quat).transpose();
  const Vector3 tilt = estimate.segment<3>(ATT_START);
  const Matrix3 ddx = nav::d_ortho_dcm_wrt_tilt(pva_dcm, tilt, kDx);
  const Matrix3 ddy = nav::d_ortho_dcm_wrt_tilt(pva_dcm, tilt, kDy);
  const Matrix3 ddz = nav::d_ortho_dcm_wrt_tilt(pva_dcm, tilt, kDz);
  const Matrix3 corr_C_ned_to_s = nav::ortho_dcm(pva_dcm * (Matrix3::Identity() + nav::skew(tilt)));
  const Matrix3 z3 = Matrix3::Zero();
  jac.block<3, 3>(ATT_START, ATT_START) =
      nav::d_dcm_to_rpy(Matrix3::Identity(), z3, z3, z3, corr_C_ned_to_s, ddx, ddy, ddz);
  return jac;
}

// ------------------------------------------------------------------------------ StateExtractor

StateExtractor::StateExtractor(api::Mediator* mediator, std::string source, std::string target,
                               int incoming_state_size, const std::vector<int>& indices)
    : mediator_(mediator), source_(std::move(source)), target_(std::move(target)) {
  auto fail = [&](const std::string& msg) {
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, msg);
    throw std::invalid_argument(msg);
  };
  if (incoming_state_size <= 0)
    fail("StateExtractor argument \"incoming_state_size\" must be greater than 0. Received " +
         std::to_string(incoming_state_size));
  if (indices.empty()) fail("Must provide at least 1 index for an element to keep.");
  for (int i : indices)
    if (i < 0 || i >= incoming_state_size)
      fail("Invalid index provided in list of indices. Value " + std::to_string(i) +
           " exceeds the length of the expected state vector.");
  if (std::set<int>(indices.begin(), indices.end()).size() != indices.size()) fail("Repeat indices are not allowed.");
  jac_ = Matrix::Zero(static_cast<Eigen::Index>(indices.size()), incoming_state_size);
  for (std::size_t i = 0; i < indices.size(); ++i) jac_(static_cast<Eigen::Index>(i), indices[i]) = 1.0;
}

void StateExtractor::receive_aux_data(const api::AuxData&) {
  if (mediator_)
    mediator_->log_message(LoggingLevel::WARN, "StateExtractor does not require aux data. This method is unimplemented.");
}

api::EstimateWithCovariance StateExtractor::convert(const api::EstimateWithCovariance& ewc, api::Timestamp time) {
  if (ewc.covariance.rows() != ewc.covariance.cols() || ewc.covariance.rows() != jac_.cols()) {
    const std::string msg = "Expected a square covariance matching the incoming state size. Cannot convert.";
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, msg);
    throw std::runtime_error(msg);
  }
  Vector state = convert_estimate(ewc.estimate, time);
  Matrix cov = (jac_ * ewc.covariance) * jac_.transpose();
  return api::EstimateWithCovariance{ewc.type, state, cov};
}

Vector StateExtractor::convert_estimate(const Vector& estimate, api::Timestamp) {
  if (jac_.cols() != estimate.size()) {
    const std::string msg = "State block to alias does not contain the expected number of states. Expected " +
                            std::to_string(jac_.cols()) + " but received " + std::to_string(estimate.size()) + ".";
    if (mediator_) mediator_->log_message(LoggingLevel::ERROR, msg);
    throw std::runtime_error(msg);
  }
  return jac_ * estimate;
}

}  // namespace pntos::cobra
