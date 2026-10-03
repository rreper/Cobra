// Toolchain smoke test: Eigen and the aspn23_eigen message library link and behave.
#include <Eigen/Dense>
#include <aspn23/eigen/MeasurementPositionVelocityAttitude.hpp>
#include <aspn23/eigen/TypeTimestamp.hpp>
#include <gtest/gtest.h>

TEST(Smoke, EigenWorks) {
  Eigen::Matrix3d m = Eigen::Matrix3d::Identity() * 2.0;
  EXPECT_DOUBLE_EQ(m.determinant(), 8.0);
}

TEST(Smoke, AspnEigenPvaRoundTrip) {
  aspn23_eigen::TypeHeader header(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0);
  aspn23_eigen::TypeTimestamp tov(1'500'000'000);
  Eigen::Matrix<double, Eigen::Dynamic, 1> quat(4);
  quat << 1.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cov =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Identity(9, 9);
  aspn23_eigen::MeasurementPositionVelocityAttitude pva(
      header, tov,
      ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_GEODETIC,
      0.6, -1.5, 300.0, 1.0, 2.0, 3.0, quat, cov,
      ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ERROR_MODEL_NONE,
      Eigen::Matrix<double, Eigen::Dynamic, 1>(0), {});
  EXPECT_EQ(pva.get_message_type(), ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE);
  EXPECT_EQ(pva.get_time_of_validity().get_elapsed_nsec(), 1'500'000'000);
  EXPECT_DOUBLE_EQ(pva.get_p1(), 0.6);
  EXPECT_DOUBLE_EQ(pva.get_v3(), 3.0);
  EXPECT_EQ(pva.get_covariance().rows(), 9);
}
