#include <pntos/cobra/utils/aspn.hpp>

#include <aspn23/eigen/aspn_eigen.hpp>

namespace pntos::cobra::utils {

std::optional<api::Timestamp> time_of_validity(const api::AspnBase& msg) {
#define PNTOS_TOV_CASE(T)                                                                 \
  if (auto* p = dynamic_cast<const aspn23_eigen::T*>(&msg)) {                             \
    return api::Timestamp(p->get_time_of_validity().get_elapsed_nsec());                  \
  }
  PNTOS_TOV_CASE(MeasurementImu)
  PNTOS_TOV_CASE(MeasurementPosition)
  PNTOS_TOV_CASE(MeasurementPositionVelocityAttitude)
  PNTOS_TOV_CASE(MeasurementVelocity)
  PNTOS_TOV_CASE(MeasurementAltitude)
  PNTOS_TOV_CASE(MeasurementBarometer)
  PNTOS_TOV_CASE(MeasurementDirection3DToPoints)
  PNTOS_TOV_CASE(MeasurementAccumulatedDistanceTraveled)
  PNTOS_TOV_CASE(MeasurementAngularVelocity)
  PNTOS_TOV_CASE(MeasurementAngularVelocity1D)
  PNTOS_TOV_CASE(MeasurementAttitude2D)
  PNTOS_TOV_CASE(MeasurementAttitude3D)
  PNTOS_TOV_CASE(MeasurementDeltaPosition)
  PNTOS_TOV_CASE(MeasurementDeltaRange)
  PNTOS_TOV_CASE(MeasurementDeltaRangeToPoint)
  PNTOS_TOV_CASE(MeasurementDirection2DToPoints)
  PNTOS_TOV_CASE(MeasurementDirectionOfMotion2D)
  PNTOS_TOV_CASE(MeasurementDirectionOfMotion3D)
  PNTOS_TOV_CASE(MeasurementFrequencyDifference)
  PNTOS_TOV_CASE(MeasurementHeading)
  PNTOS_TOV_CASE(MeasurementImage)
  PNTOS_TOV_CASE(MeasurementMagneticField)
  PNTOS_TOV_CASE(MeasurementMagneticFieldMagnitude)
  PNTOS_TOV_CASE(MeasurementPositionAttitude)
  PNTOS_TOV_CASE(MeasurementRangeRateToPoint)
  PNTOS_TOV_CASE(MeasurementRangeToPoint)
  PNTOS_TOV_CASE(MeasurementSatnav)
  PNTOS_TOV_CASE(MeasurementSatnavSubframe)
  PNTOS_TOV_CASE(MeasurementSatnavWithSvData)
  PNTOS_TOV_CASE(MeasurementSpecificForce1D)
  PNTOS_TOV_CASE(MeasurementSpeed)
  PNTOS_TOV_CASE(MeasurementTdoa1Tx2Rx)
  PNTOS_TOV_CASE(MeasurementTdoa2Tx1Rx)
  PNTOS_TOV_CASE(MeasurementTemperature)
  PNTOS_TOV_CASE(MeasurementTime)
  PNTOS_TOV_CASE(MeasurementTimeDifference)
  PNTOS_TOV_CASE(MeasurementTimeFrequencyDifference)
#undef PNTOS_TOV_CASE
  return std::nullopt;
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
