#include <pntos/cobra/utils/aspn.hpp>

#include <aspn23/eigen/aspn_eigen.hpp>

namespace pntos::cobra::utils {

#define PNTOS_FOR_EACH_TIMED_TYPE(X) \
  X(MeasurementImu) \
  X(MeasurementPosition) \
  X(MeasurementPositionVelocityAttitude) \
  X(MeasurementVelocity) \
  X(MeasurementAltitude) \
  X(MeasurementBarometer) \
  X(MeasurementDirection3DToPoints) \
  X(MeasurementAccumulatedDistanceTraveled) \
  X(MeasurementAngularVelocity) \
  X(MeasurementAngularVelocity1D) \
  X(MeasurementAttitude2D) \
  X(MeasurementAttitude3D) \
  X(MeasurementDeltaPosition) \
  X(MeasurementDeltaRange) \
  X(MeasurementDeltaRangeToPoint) \
  X(MeasurementDirection2DToPoints) \
  X(MeasurementDirectionOfMotion2D) \
  X(MeasurementDirectionOfMotion3D) \
  X(MeasurementFrequencyDifference) \
  X(MeasurementHeading) \
  X(MeasurementImage) \
  X(MeasurementMagneticField) \
  X(MeasurementMagneticFieldMagnitude) \
  X(MeasurementPositionAttitude) \
  X(MeasurementRangeRateToPoint) \
  X(MeasurementRangeToPoint) \
  X(MeasurementSatnav) \
  X(MeasurementSatnavSubframe) \
  X(MeasurementSatnavWithSvData) \
  X(MeasurementSpecificForce1D) \
  X(MeasurementSpeed) \
  X(MeasurementTdoa1Tx2Rx) \
  X(MeasurementTdoa2Tx1Rx) \
  X(MeasurementTemperature) \
  X(MeasurementTime) \
  X(MeasurementTimeDifference) \
  X(MeasurementTimeFrequencyDifference)

std::optional<api::Timestamp> time_of_validity(const api::AspnBase& msg) {
#define PNTOS_TOV_CASE(T)                                                         \
  if (auto* p = dynamic_cast<const aspn23_eigen::T*>(&msg)) {                     \
    return api::Timestamp(p->get_time_of_validity().get_elapsed_nsec());          \
  }
  PNTOS_FOR_EACH_TIMED_TYPE(PNTOS_TOV_CASE)
#undef PNTOS_TOV_CASE
  return std::nullopt;
}

std::shared_ptr<api::AspnBase> with_time_of_validity(const api::AspnBase& msg, api::Timestamp tov) {
#define PNTOS_SET_TOV_CASE(T)                                                     \
  if (auto* p = dynamic_cast<const aspn23_eigen::T*>(&msg)) {                     \
    auto copy = std::make_shared<aspn23_eigen::T>(*p);                            \
    copy->set_time_of_validity(tov.to_aspn());                                    \
    return copy;                                                                  \
  }
  PNTOS_FOR_EACH_TIMED_TYPE(PNTOS_SET_TOV_CASE)
#undef PNTOS_SET_TOV_CASE
  return nullptr;
}

std::optional<nav::Vector4> quaternion(const PVA& pva) {
  auto q = pva.get_quaternion();
  if (q.size() != 4) return std::nullopt;
  for (int i = 0; i < 4; ++i)
    if (std::isnan(q(i))) return std::nullopt;
  return nav::Vector4(q(0), q(1), q(2), q(3));
}

std::shared_ptr<PVA> make_pva(const aspn23_eigen::TypeHeader& header, api::Timestamp tov, const api::Vector3& llh,
                              const api::Vector3& vel_ned, const nav::Vector4& quat, const api::Matrix& covariance) {
  aspn23_eigen::TypeHeader h(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, header.get_vendor_id(),
                             header.get_device_id(), header.get_context_id(), header.get_sequence_id());
  Eigen::Matrix<double, Eigen::Dynamic, 1> q(4);
  q << quat(0), quat(1), quat(2), quat(3);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cov = covariance;
  return std::make_shared<PVA>(h, tov.to_aspn(), ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
                               llh(0), llh(1), llh(2), vel_ned(0), vel_ned(1), vel_ned(2), q, cov,
                               ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
                               Eigen::Matrix<double, Eigen::Dynamic, 1>(0), std::vector<aspn23_eigen::TypeIntegrity>{});
}

}  // namespace pntos::cobra::utils
