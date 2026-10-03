// Port of pntos-cobra/tests/test_virtual_state_blocks.py (VSB classes; the VirtualStateBlockManager
// tests live with the fusion engine tests).
#include <pntos/cobra/state_modeling/VirtualStateBlocks.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include "test_support.hpp"

#include <numbers>

using namespace pntos;
using namespace pntos::test;
using api::EstimateWithCovariance;
using api::EstimateWithCovarianceType;
using api::Matrix;
using api::Message;
using api::Timestamp;
using api::Vector;
using api::Vector3;
namespace nav = cobra::nav;

namespace {
const std::string kSource = "test_source";

TEST(StateExtractor, Valid) {
  TestMediator med;
  cobra::StateExtractor vsb(&med, kSource, "first_three_outta_five", 5, {0, 1, 2});
  EstimateWithCovariance ewc{EstimateWithCovarianceType::EWC_GENERIC, vec({0, 1, 2, 3, 4}), Matrix::Identity(5, 5)};
  auto out = vsb.convert(ewc, Timestamp(0));
  EXPECT_EQ(out.type, ewc.type);
  EXPECT_EQ(out.estimate.size(), 3);
  EXPECT_ALLCLOSE(out.estimate, vec({0, 1, 2}));
  EXPECT_ALLCLOSE(out.covariance, Matrix::Identity(3, 3));
  EXPECT_ALLCLOSE(vsb.jacobian(ewc.estimate, Timestamp(0)), mat({{1, 0, 0, 0, 0}, {0, 1, 0, 0, 0}, {0, 0, 1, 0, 0}}));
  EXPECT_EQ(vsb.source(), kSource);
  EXPECT_EQ(vsb.target(), "first_three_outta_five");
}

TEST(StateExtractor, Invalid) {
  TestMediator med;
  EXPECT_THROW(cobra::StateExtractor(&med, kSource, "bad_state_size", 0, {1, 2, 3}), std::invalid_argument);
  EXPECT_THROW(cobra::StateExtractor(&med, kSource, "bad_indices_length", 3, {}), std::invalid_argument);
  EXPECT_THROW(cobra::StateExtractor(&med, kSource, "bad_index", 3, {1, 3}), std::invalid_argument);
  EXPECT_THROW(cobra::StateExtractor(&med, kSource, "duplicate_index", 3, {0, 0}), std::invalid_argument);
  cobra::StateExtractor ok(&med, kSource, "bad_est", 3, {0, 1});
  EXPECT_THROW(ok.convert_estimate(vec({1, 2, 3, 4}), Timestamp(0)), std::runtime_error);
  EXPECT_THROW(ok.convert({EstimateWithCovarianceType::EWC_GENERIC, vec({0, 1, 2}), Matrix::Ones(3, 1)}, Timestamp(0)),
               std::runtime_error);
}

Message pva_at(std::int64_t t, double lat, double lon, double alt, double vn, double ve, double vd,
               const nav::Vector4& q = nav::Vector4(1, 0, 0, 0)) {
  return Message(make_pva(t, lat, lon, alt, vn, ve, vd, Vector(q)), "test");
}

TEST(PinsonErrorToStandard, ConvertMatchesHandComputation) {
  TestMediator med;
  cobra::PinsonErrorToStandard pes(&med, kSource, "pinson_direct");
  const double lat = 0.6, lon = -1.5, alt = 300;
  pes.receive_aux_data({pva_at(0, lat, lon, alt, 1, 2, 3)});
  Vector est = vec({10, 20, 30, 0.1, 0.2, 0.3, 1e-3, 2e-3, 3e-3, 7, 8});
  Vector out = pes.convert_estimate(est, Timestamp(0));
  ASSERT_EQ(out.size(), 11);
  EXPECT_NEAR(out(0), lat + nav::north_to_delta_lat(10, lat, alt), 1e-15);
  EXPECT_NEAR(out(1), lon + nav::east_to_delta_lon(20, lat, alt), 1e-15);
  EXPECT_NEAR(out(2), alt - 30, 1e-12);
  EXPECT_NEAR(out(3), 1.1, 1e-12);
  EXPECT_NEAR(out(4), 2.2, 1e-12);
  EXPECT_NEAR(out(5), 3.3, 1e-12);
  // identity attitude corrected by small tilts: rpy ~ tilt (to first order), sign per Cobra convention
  Vector3 rpy = out.segment<3>(6);
  EXPECT_NEAR(rpy.norm(), est.segment<3>(6).norm(), 1e-5);
  EXPECT_DOUBLE_EQ(out(9), 7);
  EXPECT_DOUBLE_EQ(out(10), 8);
}

TEST(PinsonErrorToStandard, JacobianMatchesFiniteDifference) {
  TestMediator med;
  cobra::PinsonErrorToStandard pes(&med, kSource, "pinson_direct");
  for (int i = 1; i < 50; i += 7) {
    Timestamp t(i * 1000);
    nav::Vector4 q = nav::rpy_to_quat(Vector3(0.05 * i, -0.03 * i, 0.4 * i));
    pes.receive_aux_data({pva_at(t.elapsed_nsec, std::fmod(i * std::numbers::pi / 180, std::numbers::pi / 2.5),
                                 std::fmod(i * std::numbers::pi / 180, std::numbers::pi / 2), i, i + 1, i + 2, i + 3,
                                 q)});
    Vector est(9);
    est << i, i + 1, i + 2, i + 3, i + 4, i + 5, i * 1e-4, i * 1.1e-4, i * 1.2e-4;
    Matrix jac = pes.jacobian(est, t);
    Matrix fd(9, 9);
    const double h = 1e-7;
    Vector base = pes.convert_estimate(est, t);
    for (int c = 0; c < 9; ++c) {
      Vector ep = est, em = est;
      ep(c) += h;
      em(c) -= h;
      fd.col(c) = (pes.convert_estimate(ep, t) - pes.convert_estimate(em, t)) / (2 * h);
    }
    // position/velocity rows are exact linear maps; attitude rows compare to central differences
    EXPECT_TRUE(allclose(jac.topRows(6), fd.topRows(6), 1e-6, 1e-9)) << jac << "\n--\n" << fd;
    EXPECT_TRUE(allclose(jac.bottomRows(3), fd.bottomRows(3), 1e-4, 1e-6)) << jac << "\n--\n" << fd;
    // convert() uses the jacobian for the covariance
    Matrix cov = Matrix::Identity(9, 9) * 0.5;
    auto ewc = pes.convert({EstimateWithCovarianceType::EWC_GENERIC, est, cov}, t);
    EXPECT_ALLCLOSE(ewc.covariance, Matrix(jac * cov * jac.transpose()));
    EXPECT_ALLCLOSE(ewc.estimate, base);
  }
}

TEST(PinsonErrorToStandard, Invalid) {
  TestMediator med;
  cobra::PinsonErrorToStandard pes(&med, kSource, "pinson_direct");
  Vector arr = vec({1, 1, 1, 0, 0, 0, 0, 0, 0});
  // no pva
  EXPECT_THROW(pes.convert_estimate(arr, Timestamp(0)), std::runtime_error);
  EXPECT_THROW(pes.jacobian(arr, Timestamp(0)), std::runtime_error);
  // bad time
  pes.receive_aux_data({pva_at(0, 1, 1, 1, 0, 0, 0)});
  EXPECT_THROW(pes.convert_estimate(arr, Timestamp(1)), std::runtime_error);
  EXPECT_THROW(pes.jacobian(arr, Timestamp(1)), std::runtime_error);
  EXPECT_NO_THROW(pes.convert_estimate(arr, Timestamp(0)));
  // bad quaternion
  Vector nanq = Vector::Constant(4, std::nan(""));
  pes.receive_aux_data({Message(make_pva(1, 1, 1, 1, 0, 0, 0, nanq), "test")});
  EXPECT_THROW(pes.convert_estimate(arr, Timestamp(1)), std::runtime_error);
  EXPECT_THROW(pes.jacobian(arr, Timestamp(1)), std::runtime_error);
  // bad position
  pes.receive_aux_data({Message(make_pva(1, std::nan(""), 1, 1, 0, 0, 0, vec({1, 0, 0, 0})), "test")});
  EXPECT_THROW(pes.convert_estimate(arr, Timestamp(1)), std::runtime_error);
  EXPECT_THROW(pes.jacobian(arr, Timestamp(1)), std::runtime_error);
  // non-geodetic PVA aux is ignored
  cobra::PinsonErrorToStandard pes2(&med, kSource, "x");
  auto eci = std::make_shared<cobra::utils::PVA>(*make_pva(0, 1, 1, 1, 0, 0, 0, vec({1, 0, 0, 0})));
  eci->set_reference_frame(ASPN23_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_REFERENCE_FRAME_ECI);
  pes2.receive_aux_data({Message(eci, "test")});
  EXPECT_EQ(pes2.pva(), nullptr);
}

}  // namespace
