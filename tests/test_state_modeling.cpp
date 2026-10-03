// Port of pntos-cobra/tests/test_state_modeling.py
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/state_modeling/MeasurementProcessors.hpp>
#include <pntos/cobra/state_modeling/Pinson15NedBlock.hpp>
#include <pntos/cobra/state_modeling/StandardStateModelingPlugin.hpp>
#include <pntos/cobra/state_modeling/VirtualStateBlocks.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include "test_support.hpp"

#include <aspn23/eigen/MeasurementAltitude.hpp>
#include <aspn23/eigen/MeasurementDirection3DToPoints.hpp>
#include <aspn23/eigen/MeasurementVelocity.hpp>
#include <aspn23/eigen/TypeImageFeature.hpp>
#include <aspn23/eigen/TypeRemotePoint.hpp>

#include <cmath>
#include <numbers>

using namespace pntos;
using namespace pntos::test;
using api::EstimateWithCovariance;
using api::EstimateWithCovarianceType;
using api::Matrix;
using api::Matrix3;
using api::Message;
using api::Timestamp;
using api::Vector;
using api::Vector3;
namespace nav = cobra::nav;
using cobra::utils::PVA;

namespace {

constexpr std::int64_t kSec = 1'000'000'000;
const Vector3 kLeverArm(-2.0, 3.0, 5.0);
const nav::Vector4 kOrientation(1.0, 0.0, 0.0, 0.0);
constexpr double kDeg = std::numbers::pi / 180.0;

/// conftest.gxp: estimate = 0.01 * ones(n), covariance = I
EstimateWithCovariance gxp(int n) {
  return {EstimateWithCovarianceType::EWC_GENERIC, Vector::Constant(n, 0.01), Matrix::Identity(n, n)};
}
api::GenXandP fixed(const EstimateWithCovariance& e) {
  return [e](const std::vector<std::string>&) -> std::optional<EstimateWithCovariance> { return e; };
}
EstimateWithCovariance zeros(int n) {
  return {EstimateWithCovarianceType::EWC_GENERIC, Vector::Zero(n), Matrix::Identity(n, n)};
}

std::shared_ptr<aspn23_eigen::MeasurementVelocity> make_velocity(std::int64_t tov, double x, double y, double z,
                                                                 const Vector3& var,
                                                                 Aspn23MeasurementVelocityReferenceFrame frame) {
  return std::make_shared<aspn23_eigen::MeasurementVelocity>(
      header(ASPN_MEASUREMENT_VELOCITY), aspn23_eigen::TypeTimestamp(tov), frame, x, y, z, rm(Matrix(var.asDiagonal())),
      ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE, DynVector(0), std::vector<aspn23_eigen::TypeIntegrity>{});
}

class StateModelingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto pinson = std::make_shared<cobra::PinsonStateBlockConfig>();
    pinson->group_ = "config/pinson_block";
    pinson->label = "pinson15";
    pinson->imu_model.group_ = "config/pinson_block";
    const double g = 9.81e-6 * 25;
    pinson->imu_model.accel_bias_sigma = {g, g, g};
    pinson->imu_model.accel_bias_tau = {3600, 3600, 3600};
    pinson->imu_model.accel_random_walk_sigma = {1e-12, 1e-12, 1e-12};
    const double gb = 0.003 * kDeg / 3600;
    pinson->imu_model.gyro_bias_sigma = {gb, gb, gb};
    pinson->imu_model.gyro_bias_tau = {3600, 3600, 3600};
    const double grw = 0.002 * kDeg / 60;
    pinson->imu_model.gyro_random_walk_sigma = {grw, grw, grw};

    auto body = std::make_shared<cobra::LeverArmOrientationMPConfig>(cobra::mp::PinsonBodyVelocityMPConfig());
    body->group_ = "config/test";
    body->label = "NA";
    body->channel = "NA";
    body->state_block_labels = {"NA"};
    body->lever_arm = {-2.0, 3.0, 5.0};
    body->orientation = {1.0, 0.0, 0.0, 0.0};

    auto se = std::make_shared<cobra::StateExtractorConfig>();
    se->group_ = "config/extractor";
    se->source = "some_real_block";
    se->target = "extractor";
    se->incoming_state_size = 3;
    se->indices_to_extract = {0, 1};

    registry_plugin = std::make_unique<cobra::StandardRegistryPlugin>(
        "Standard registry", std::vector<std::shared_ptr<const cobra::BaseConfig>>{pinson, body, se});
    registry_plugin->init_plugin(std::nullopt, &med);
    med.set_registry(registry_plugin->new_registry());

    sm_plugin = std::make_unique<cobra::StandardStateModelingPlugin>("pos_ins_state_modeling");
    sm_plugin->init_plugin(std::nullopt, &med);
    provider = sm_plugin->new_state_model_provider(api::FusionType::STANDARD);
    ASSERT_TRUE(provider);

    pva_aux = Message(make_pva(kSec, 39 * kDeg, -84 * kDeg, 1000, 2, 3, 4, vec({1, 0, 0, 0})), "pva_aux");
    zero_pva_aux = Message(make_pva(0, 0, 0, 0, 0, 0, 0, vec({1, 0, 0, 0})), "pva_aux");
    pos_meas = Message(make_position(kSec, 39.00001 * kDeg, -84.00001 * kDeg, 1005, Vector3(25, 25, 100).asDiagonal()),
                       "gps_position");
    force_aux = Message(make_imu(kSec, Vector3(0, 0, -9.8), Vector3::Zero()), "force_and_rate_aux");
  }

  std::unique_ptr<api::StandardMeasurementProcessor> processor(std::size_t idx, const std::string& label,
                                                               std::vector<std::string> labels,
                                                               std::optional<std::string> group = "config/test") {
    return provider->new_processor(idx, nullptr, label, labels, group);
  }

  struct PosProc {
    std::unique_ptr<api::StandardMeasurementProcessor> mp;
    int num_states;
    int num_labels;
    std::size_t index;
  };
  std::vector<PosProc> all_pos_processors() {
    std::vector<PosProc> out;
    out.push_back({processor(0, "position", {"pinson"}), 15, 1, 0});
    out.push_back({processor(2, "position", {"pinson", "fogm"}), 18, 2, 2});
    out.push_back({processor(4, "position", {"pinson", "fogm", "fogm2"}), 21, 3, 4});
    return out;
  }
  static api::GenXandP gen_all_pos() {
    return [](const std::vector<std::string>& labels) -> std::optional<EstimateWithCovariance> {
      return gxp(static_cast<int>(15 + 3 * (labels.size() - 1)));
    };
  }

  TestMediator med;
  std::unique_ptr<cobra::StandardRegistryPlugin> registry_plugin;
  std::unique_ptr<cobra::StandardStateModelingPlugin> sm_plugin;
  std::unique_ptr<api::StandardStateModelProvider> provider;
  Message pva_aux, zero_pva_aux, pos_meas, force_aux;
};

TEST_F(StateModelingTest, InvalidFusionType) {
  EXPECT_EQ(sm_plugin->new_state_model_provider(api::FusionType::SAMPLED), nullptr);
}

TEST_F(StateModelingTest, EnoughLabelsAndInvalidIndex) {
  EXPECT_EQ(processor(provider->processor_identifiers().size(), "l", {"s"}, ""), nullptr);
  EXPECT_NE(provider->new_block(0, nullptr, "label", "config/pinson_block"), nullptr);
  EXPECT_EQ(provider->new_block(provider->block_identifiers().size() + 1, nullptr, "label", "config/pinson_block"),
            nullptr);
  EXPECT_EQ(processor(provider->processor_identifiers().size() + 1, "label", {"x"}), nullptr);
  EXPECT_EQ(provider->new_virtual_block(provider->virtual_block_identifiers().size() + 1, "source", "target",
                                        "config/vsb"),
            nullptr);
  EXPECT_EQ(provider->processor_identifiers().size(), 9u);
  EXPECT_EQ(provider->block_identifiers().size(), 4u);
  EXPECT_EQ(provider->virtual_block_identifiers().size(), 2u);
}

TEST_F(StateModelingTest, WrongNumberBlocks) {
  for (auto& m : all_pos_processors()) {
    auto bad = processor(m.index, "label", {});
    ASSERT_TRUE(bad);
    bad->receive_aux_data({pva_aux});
    EXPECT_FALSE(bad->generate_model(pos_meas, gen_all_pos()).has_value());
    std::vector<std::string> labs;
    for (int i = 0; i < m.num_labels + 1; ++i) labs.push_back(std::string(1, static_cast<char>('a' + i)));
    auto bad2 = processor(m.index, "label", labs);
    bad2->receive_aux_data({pva_aux});
    EXPECT_FALSE(bad2->generate_model(pos_meas, gen_all_pos()).has_value());
  }
}

TEST_F(StateModelingTest, VsbInstantiation) {
  auto pes = provider->new_virtual_block(0, "some_real_block", "to_direct", std::nullopt);
  EXPECT_NE(dynamic_cast<cobra::PinsonErrorToStandard*>(pes.get()), nullptr);
  auto se = provider->new_virtual_block(1, "some_real_block", "extractor", "config/extractor");
  EXPECT_NE(dynamic_cast<cobra::StateExtractor*>(se.get()), nullptr);
  EXPECT_EQ(provider->new_virtual_block(1, "some_real_block", "extractor", "config/bad"), nullptr);
  EXPECT_EQ(provider->new_virtual_block(1, "some_real_block", "extractor", std::nullopt), nullptr);
}

TEST_F(StateModelingTest, BadMeasInputs) {
  Message eci = Message(make_position(kSec, 39.00001 * kDeg, -84.00001 * kDeg, 1005, Vector3(25, 25, 100).asDiagonal(),
                                      ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_ECI),
                        "gps_position");
  Vector nanq = Vector::Constant(4, std::nan(""));
  Message no_quat(make_pva(kSec, 39 * kDeg, -84 * kDeg, 1000, 2, 3, 4, nanq), "pva_aux");
  for (auto& m : all_pos_processors()) {
    m.mp->receive_aux_data({pva_aux});
    EXPECT_TRUE(m.mp->generate_model(pos_meas, gen_all_pos()).has_value());
    EXPECT_FALSE(m.mp->generate_model(eci, gen_all_pos()).has_value());
    EXPECT_TRUE(m.mp->generate_model(pos_meas, gen_all_pos()).has_value());
    m.mp->receive_aux_data({no_quat});  // rejected -> previous aux kept
    EXPECT_TRUE(m.mp->generate_model(pos_meas, gen_all_pos()).has_value());
  }
}

TEST_F(StateModelingTest, InvalidAux) {
  auto blk = provider->new_block(0, nullptr, "pinson15", "config/pinson_block");
  auto* pinson = dynamic_cast<cobra::Pinson15NedBlock*>(blk.get());
  ASSERT_TRUE(pinson);
  Message bad_aux(std::make_shared<aspn23_eigen::TypeHeader>(ASPN_UNDEFINED, 0, 0, 0, 0), "bad_aux");
  pinson->receive_aux_data({bad_aux});
  EXPECT_FALSE(pinson->has_pva_aux());
  EXPECT_FALSE(pinson->has_force_aux());
  EXPECT_FALSE(pinson->generate_dynamics(gen_all_pos(), Timestamp(0), Timestamp(1)).has_value());
  for (auto& m : all_pos_processors()) {
    auto* p = dynamic_cast<cobra::PinsonProcessorBase*>(m.mp.get());
    p->receive_aux_data({bad_aux});
    EXPECT_EQ(p->inertial_pva(), nullptr);
    p->receive_aux_data({});
    EXPECT_EQ(p->inertial_pva(), nullptr);
    EXPECT_FALSE(m.mp->generate_model(pos_meas, gen_all_pos()).has_value());  // no aux
  }
}

TEST_F(StateModelingTest, StaleAuxDataAndInvalidMeasurement) {
  Message stale(make_pva(0, 39 * kDeg, -84 * kDeg, 1000, 2, 3, 4, vec({1, 0, 0, 0})), "pva_aux");
  for (auto& m : all_pos_processors()) {
    m.mp->receive_aux_data({stale});
    EXPECT_FALSE(m.mp->generate_model(pos_meas, gen_all_pos()).has_value());
  }
  Message bad_meas(std::make_shared<aspn23_eigen::TypeHeader>(ASPN_UNDEFINED, 0, 0, 0, 0), "bad_meas");
  for (auto& m : all_pos_processors()) EXPECT_FALSE(m.mp->generate_model(bad_meas, gen_all_pos()).has_value());
}

TEST_F(StateModelingTest, GenerateModelDirectPos) {
  auto direct = processor(7, "position", {"pinson", "fogm"});
  ASSERT_TRUE(direct);
  auto xp = zeros(18);
  auto mm = direct->generate_model(pos_meas, fixed(xp));
  ASSERT_TRUE(mm);
  auto pos = pos_meas.as<aspn23_eigen::MeasurementPosition>();
  Matrix3 conv = Matrix3::Zero();
  conv(0, 0) = nav::north_to_delta_lat(1, pos->get_term1(), pos->get_term3());
  conv(1, 1) = nav::east_to_delta_lon(1, pos->get_term1(), pos->get_term3());
  conv(2, 2) = -1.0;
  Vector3 rpy = Vector3::Zero();
  Matrix exp_H = Matrix::Identity(3, 18);
  exp_H.col(6) = conv * nav::d_rpy_to_dcm_wrt_r(rpy) * kLeverArm;
  exp_H.col(7) = conv * nav::d_rpy_to_dcm_wrt_p(rpy) * kLeverArm;
  exp_H.col(8) = conv * nav::d_rpy_to_dcm_wrt_y(rpy) * kLeverArm;
  exp_H.block(0, 15, 3, 3) = -Matrix3::Identity();
  Matrix exp_R = conv * Matrix(pos->get_covariance()) * conv;
  Vector3 arm_ned = nav::rpy_to_dcm(rpy) * kLeverArm;
  Vector exp_h = vec({nav::north_to_delta_lat(arm_ned(0), pos->get_term1(), pos->get_term3()),
                      nav::east_to_delta_lon(arm_ned(1), pos->get_term1(), pos->get_term3()), -arm_ned(2)});
  EXPECT_ALLCLOSE(mm->H, exp_H);
  EXPECT_ALLCLOSE(mm->R, exp_R);
  EXPECT_ALLCLOSE(mm->h(xp.estimate), exp_h);
  EXPECT_ALLCLOSE(mm->z, vec({pos->get_term1(), pos->get_term2(), pos->get_term3()}));
}

TEST_F(StateModelingTest, GenerateModelPosAll) {
  auto inertial = pva_aux.as<PVA>();
  auto pos = pos_meas.as<aspn23_eigen::MeasurementPosition>();
  for (auto& m : all_pos_processors()) {
    m.mp->receive_aux_data({pva_aux});
    auto xp = gxp(m.num_states);
    auto mm = m.mp->generate_model(pos_meas, gen_all_pos());
    ASSERT_TRUE(mm);
    const int n = m.num_states;
    Matrix exp_H = Matrix::Zero(3, n);
    exp_H.block<3, 3>(0, 0) = Matrix3::Identity();
    Matrix3 C = nav::quat_to_dcm(*cobra::utils::quaternion(*inertial));
    Matrix3 c_cor = (Matrix3::Identity() - nav::skew(xp.estimate.segment<3>(6))) * C;
    exp_H.block<3, 3>(0, 6) = nav::skew(C * kLeverArm);
    Vector3 exp_pred = xp.estimate.head<3>() + c_cor * kLeverArm;
    auto* pp = dynamic_cast<cobra::PinsonPositionMeasurementProcessor*>(m.mp.get());
    ASSERT_TRUE(pp);
    if (pp->kind() == cobra::PinsonPositionMeasurementProcessor::Kind::WithNedFogm) {
      exp_H.block(0, n - 3, 3, 3) = -Matrix3::Identity();
      exp_pred -= xp.estimate.tail<3>();
    }
    if (pp->kind() == cobra::PinsonPositionMeasurementProcessor::Kind::WithLeverArm) {
      exp_H.block(0, n - 6, 3, 3) = -Matrix3::Identity();
      exp_H.block(0, n - 3, 3, 3) = c_cor;
      exp_H.block<3, 3>(0, 6) += nav::skew(C * xp.estimate.tail<3>());
      exp_pred -= xp.estimate.segment<3>(n - 6);
      exp_pred += c_cor * xp.estimate.tail<3>();
    }
    Vector3 meas_llh(pos->get_term1(), pos->get_term2(), pos->get_term3());
    Vector3 inertial_llh = cobra::utils::position(*inertial);
    Vector3 delta = meas_llh - inertial_llh;
    Vector exp_z = vec({delta(0) * nav::delta_lat_to_north(1, meas_llh(0), meas_llh(2)),
                        delta(1) * nav::delta_lon_to_east(1, meas_llh(0), meas_llh(2)), -delta(2)});
    EXPECT_ALLCLOSE(mm->H, exp_H);
    EXPECT_EQ(mm->R, Matrix(pos->get_covariance()));
    EXPECT_ALLCLOSE(mm->h(xp.estimate), Vector(exp_pred));
    EXPECT_ALLCLOSE(mm->z, exp_z);
  }
}

TEST_F(StateModelingTest, GenerateModelVel) {
  auto velocity_mp = processor(1, "velocity", {"pinson"});
  ASSERT_TRUE(velocity_mp);
  Message vel_meas(make_velocity(kSec, 2.2, 3.3, 4.4, Vector3(1, 4, 0.5), ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED),
                   "gps_velocity");
  velocity_mp->receive_aux_data({pva_aux});
  auto xp = zeros(15);
  auto mm = velocity_mp->generate_model(vel_meas, fixed(xp));
  ASSERT_TRUE(mm);
  Matrix exp_H = Matrix::Zero(3, 15);
  exp_H.block<3, 3>(0, 3) = Matrix3::Identity();
  EXPECT_EQ(mm->H, exp_H);
  EXPECT_EQ(mm->R, Matrix(Vector3(1, 4, 0.5).asDiagonal()));
  EXPECT_EQ(mm->h(xp.estimate), Vector::Zero(3));
  EXPECT_ALLCLOSE(mm->z, vec({2.2 - 2, 3.3 - 3, 4.4 - 4}));
  // wrong frame
  Message sensor_vel(make_velocity(kSec, 2.2, 3.3, 4.4, Vector3(1, 4, 0.5), ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_SENSOR),
                     "v");
  EXPECT_FALSE(velocity_mp->generate_model(sensor_vel, fixed(xp)).has_value());
}

TEST_F(StateModelingTest, GenerateModelPosVel) {
  auto posvel_mp = processor(6, "posvel", {"pinson"});
  ASSERT_TRUE(posvel_mp);
  Matrix cov6 = Matrix::Zero(6, 6);
  cov6.diagonal() << 25, 25, 100, 1, 4, 0.5;
  Vector nanq = Vector::Constant(4, std::nan(""));
  Message posvel_meas(make_pva(kSec, 39.00001 * kDeg, -84.00001 * kDeg, 1005, 2.2, 3.3, 4.4, nanq, cov6), "gps_posvel");
  posvel_mp->receive_aux_data({pva_aux});
  Vector est(15);
  est << 0.1, 0.1, 0.1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0;
  EstimateWithCovariance xp{EstimateWithCovarianceType::EWC_GENERIC, est, Matrix::Identity(15, 15)};
  auto mm = posvel_mp->generate_model(posvel_meas, fixed(xp));
  ASSERT_TRUE(mm);
  auto inertial = pva_aux.as<PVA>();
  Matrix3 C = nav::quat_to_dcm(*cobra::utils::quaternion(*inertial));
  Matrix3 c_cor = (Matrix3::Identity() - nav::skew(est.segment<3>(6))) * C;
  Matrix exp_H = Matrix::Zero(6, 15);
  exp_H.block<3, 3>(0, 0) = Matrix3::Identity();
  exp_H.block<3, 3>(0, 6) = nav::skew(C * kLeverArm);
  exp_H.block<3, 3>(3, 3) = Matrix3::Identity();
  Vector3 meas_pos(39.00001 * kDeg, -84.00001 * kDeg, 1005), meas_vel(2.2, 3.3, 4.4);
  Vector3 delta = meas_pos - cobra::utils::position(*inertial);
  delta(0) *= nav::delta_lat_to_north(1, meas_pos(0), meas_pos(2));
  delta(1) *= nav::delta_lon_to_east(1, meas_pos(0), meas_pos(2));
  delta(2) *= -1;
  Vector exp_z(6);
  exp_z << delta, meas_vel - cobra::utils::velocity(*inertial);
  Vector exp_pred(6);
  exp_pred << est.head<3>() + c_cor * kLeverArm, Vector3::Zero();
  EXPECT_ALLCLOSE(mm->H, exp_H);
  EXPECT_EQ(mm->R, cov6);
  EXPECT_ALLCLOSE(mm->h(est), exp_pred);
  EXPECT_ALLCLOSE(mm->z, exp_z);
}

TEST_F(StateModelingTest, GenerateModelBodyVel) {
  auto body_mp = processor(5, "body_velocity", {"pinson"});
  ASSERT_TRUE(body_mp);
  Message bodyvel_meas(make_velocity(kSec, 4.4, 3.3, 2.2, Vector3(0.5, 0.5, 0.5), ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_SENSOR),
                       "body_velocity");
  body_mp->receive_aux_data({pva_aux, force_aux});
  auto* bp = dynamic_cast<cobra::PinsonBodyVelocityMeasurementProcessor*>(body_mp.get());
  ASSERT_TRUE(bp);
  EXPECT_NE(bp->inertial_pva(), nullptr);
  EXPECT_TRUE(bp->has_force_and_rate_aux());
  auto xp = zeros(15);
  auto mm = body_mp->generate_model(bodyvel_meas, fixed(xp));
  ASSERT_TRUE(mm);
  auto inertial = pva_aux.as<PVA>();
  const Vector& x = xp.estimate;
  Vector3 inertial_vel = cobra::utils::velocity(*inertial);
  Matrix3 C_platform_to_sensor = Matrix3::Identity();
  Matrix3 uncorr_C_ned_to_imu = nav::quat_to_dcm(*cobra::utils::quaternion(*inertial)).transpose();
  Matrix3 corr_C_ned_to_imu = uncorr_C_ned_to_imu * (Matrix3::Identity() + nav::skew(x.segment<3>(6)));
  Matrix3 C_ned_to_sensor = C_platform_to_sensor * corr_C_ned_to_imu;
  Matrix3 uncorr_C_ned_to_sensor = C_platform_to_sensor * uncorr_C_ned_to_imu;
  Vector3 corr_vel = inertial_vel + x.segment<3>(3);
  Matrix3 C_der = -uncorr_C_ned_to_sensor * nav::skew(corr_vel);
  Matrix exp_H = Matrix::Zero(3, 15);
  exp_H.block<3, 3>(0, 3) = C_ned_to_sensor;
  exp_H.block<3, 3>(0, 6) = C_der;
  Vector3 rotation_rate = Vector3::Zero();
  double alt = inertial->get_p3() - x(2);
  double lat = inertial->get_p1() + nav::north_to_delta_lat(x(0), inertial->get_p1(), alt);
  double rn = nav::meridian_radius(lat), re = nav::transverse_radius(lat);
  Vector3 w_en_n(corr_vel(1) / (re + alt), -corr_vel(0) / (rn + alt), -corr_vel(1) * std::tan(lat) / (re + alt));
  Vector3 w_ie_n(nav::ROTATION_RATE * std::cos(lat), 0.0, -nav::ROTATION_RATE * std::sin(lat));
  rotation_rate = rotation_rate + x.segment<3>(12) - corr_C_ned_to_imu * (w_ie_n - w_en_n);
  Vector3 tan_vel_sensor = C_platform_to_sensor * rotation_rate.cross(kLeverArm);
  Vector3 exp_pred = C_ned_to_sensor * corr_vel + tan_vel_sensor;
  EXPECT_ALLCLOSE(mm->H, exp_H);
  EXPECT_EQ(mm->R, Matrix(Vector3(0.5, 0.5, 0.5).asDiagonal()));
  EXPECT_ALLCLOSE(mm->h(x), Vector(exp_pred));
  EXPECT_EQ(mm->z, vec({4.4, 3.3, 2.2}));
}

TEST_F(StateModelingTest, GenerateModelAltAndPosAlt) {
  auto altitude_mp = processor(3, "altitude", {"pinson", "fogm"});
  ASSERT_TRUE(altitude_mp);
  auto alt_msg = std::make_shared<aspn23_eigen::MeasurementAltitude>(
      header(ASPN_MEASUREMENT_ALTITUDE), aspn23_eigen::TypeTimestamp(kSec), ASPN23_MEASUREMENT_ALTITUDE_REFERENCE_HAE,
      1005.0, 100.0, ASPN23_MEASUREMENT_ALTITUDE_ERROR_MODEL_NONE, DynVector(0),
      std::vector<aspn23_eigen::TypeIntegrity>{});
  Message alt_meas(alt_msg, "alt");
  altitude_mp->receive_aux_data({pva_aux});
  auto* ap = dynamic_cast<cobra::AltitudeMeasurementProcessor*>(altitude_mp.get());
  ASSERT_TRUE(ap->inertial_solution_time().has_value());
  auto xp = zeros(16);
  auto inertial = pva_aux.as<PVA>();
  Matrix3 C = nav::quat_to_dcm(*cobra::utils::quaternion(*inertial));
  Matrix exp_H = Matrix::Zero(1, 16);
  exp_H(0, 2) = -1;
  exp_H(0, 15) = 1;
  exp_H.block<1, 3>(0, 6) = nav::skew(C * kLeverArm).row(2);

  auto mm = altitude_mp->generate_model(alt_meas, fixed(xp));
  ASSERT_TRUE(mm);
  EXPECT_EQ(mm->H, exp_H);
  EXPECT_EQ(mm->R, Matrix::Constant(1, 1, 100.0));
  EXPECT_ALLCLOSE(mm->h(xp.estimate), vec({kLeverArm(2)}));
  EXPECT_ALLCLOSE(mm->z, vec({1005.0 - inertial->get_p3()}));

  // Geodetic position as an altitude measurement
  auto mm2 = altitude_mp->generate_model(pos_meas, fixed(xp));
  ASSERT_TRUE(mm2);
  EXPECT_EQ(mm2->H, exp_H);
  EXPECT_EQ(mm2->R, Matrix::Constant(1, 1, 100.0));
  EXPECT_ALLCLOSE(mm2->z, vec({1005.0 - inertial->get_p3()}));

  // MSL is not supported without a geoid model
  auto msl = std::make_shared<aspn23_eigen::MeasurementAltitude>(
      header(ASPN_MEASUREMENT_ALTITUDE), aspn23_eigen::TypeTimestamp(kSec), ASPN23_MEASUREMENT_ALTITUDE_REFERENCE_MSL,
      1005.0, 100.0, ASPN23_MEASUREMENT_ALTITUDE_ERROR_MODEL_NONE, DynVector(0),
      std::vector<aspn23_eigen::TypeIntegrity>{});
  EXPECT_FALSE(altitude_mp->generate_model(Message(msl, "alt"), fixed(xp)).has_value());
}

TEST_F(StateModelingTest, GenerateModelDirection3DToPoints) {
  auto mp = processor(8, "direction3D_to_points", {"pinson"});
  ASSERT_TRUE(mp);
  aspn23_eigen::TypeRemotePoint rp(3, 0, ASPN23_TYPE_REMOTE_POINT_POSITION_REFERENCE_FRAME_GEODETIC, 0.693950, -1.468400,
                                   0.0, rm(Matrix::Identity(3, 3)));
  aspn23_eigen::TypeImageFeature feat(1.0, 1.0, 1.0, 1, 1, Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>::Ones(1));
  aspn23_eigen::TypeDirection3DToPoint obs(rp, ASPN23_TYPE_DIRECTION_3D_TO_POINT_REFERENCE_FRAME_SINE_SPACE,
                                           dyn({0.5, 0.4}), rm(Matrix(Eigen::Vector2d(1e-4, 1e-4).asDiagonal())), false,
                                           feat, ASPN23_TYPE_DIRECTION_3D_TO_POINT_ERROR_MODEL_NONE, DynVector(0),
                                           std::vector<aspn23_eigen::TypeIntegrity>{});
  auto msg = std::make_shared<aspn23_eigen::MeasurementDirection3DToPoints>(
      header(ASPN_MEASUREMENT_DIRECTION_3D_TO_POINTS), aspn23_eigen::TypeTimestamp(kSec),
      std::vector<aspn23_eigen::TypeDirection3DToPoint>{obs});
  Message d2p(msg, "direction3D_to_points");
  mp->receive_aux_data({pva_aux});
  auto xp = zeros(16);
  auto mm = mp->generate_model(d2p, fixed(xp));
  ASSERT_TRUE(mm);

  auto inertial = pva_aux.as<PVA>();
  Vector3 inertial_llh = cobra::utils::position(*inertial);
  Matrix3 C_nav_to_platform = nav::quat_to_dcm(*cobra::utils::quaternion(*inertial)).transpose();
  Matrix3 C_platform_to_sensor = nav::quat_to_dcm(kOrientation);
  Matrix3 C_nav_to_sensor = C_platform_to_sensor * C_nav_to_platform;
  Vector3 feature_llh(0.693950, -1.468400, 0.0);
  Vector3 delta_pos_ned(nav::delta_lat_to_north(feature_llh(0) - inertial_llh(0), inertial_llh(0), inertial_llh(2)),
                        nav::delta_lon_to_east(feature_llh(1) - inertial_llh(1), inertial_llh(0), inertial_llh(2)),
                        inertial_llh(2) - feature_llh(2));
  Vector3 dps = C_nav_to_sensor * delta_pos_ned - C_platform_to_sensor * kLeverArm;
  Vector3 u = dps / dps.norm();
  Vector exp_z = vec({0.5 - u(1), 0.4 - u(2)});
  Matrix3 A = (Matrix3::Identity() - u * u.transpose()) / dps.norm();
  Matrix exp_H = Matrix::Zero(2, 16);
  exp_H.block<2, 3>(0, 0) = A.bottomRows<2>() * (-C_nav_to_sensor);
  exp_H.block<2, 3>(0, 6) = A.bottomRows<2>() * (C_nav_to_sensor * -nav::skew(delta_pos_ned));
  EXPECT_ALLCLOSE(mm->H, exp_H);
  // R: observation covariance + feature covariance mapped through the position part of H
  Matrix exp_R = Matrix(Eigen::Vector2d(1e-4, 1e-4).asDiagonal()) +
                 exp_H.block<2, 3>(0, 0) * Matrix3::Identity() * exp_H.block<2, 3>(0, 0).transpose();
  EXPECT_ALLCLOSE(mm->R, exp_R);
  EXPECT_ALLCLOSE(mm->z, exp_z);
  // h(0) == 0 for the linearised model
  EXPECT_ALLCLOSE(mm->h(xp.estimate), Vector::Zero(2));
  // Linear in x: h(x) == H x for the pinson columns
  Vector x = Vector::Zero(16);
  x << 1, 2, 3, 0, 0, 0, 1e-3, 2e-3, -1e-3, 0, 0, 0, 0, 0, 0, 0;
  EXPECT_ALLCLOSE(mm->h(x), Vector(exp_H * x));
}

TEST_F(StateModelingTest, GenerateDynamicsGolden) {
  // Golden Phi/Qd from the Python test (zero PVA at the origin, 1 s step).
  // clang-format off
  Matrix expected_Phi = mat({
    {1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 4.9, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0,1.0,0.0,0.0,1.0,0.00007292115147,-4.90,0.0,0.0,0.0,0.50,0.0,0.0,0.0,0.0},
    {0.00000000814951,0.0,1.00000154384554,0.0,-0.00007292115147,1.0,0.0,0.0,0.0,0.0,0.0,0.50,0.0,0.0,0.0},
    {0.0,0.0,0.0,0.99999922657297,0.0,0.0,0.0,9.80,0.00035731364219,0.99986111111111,0.0,0.0,0.0,-4.90,0.0},
    {1.1885438536036494e-12,0.0,0.00000000022516,0.0,0.9999992211156,0.00014584230293,-9.80,0.0,0.0,0.0,0.99986111111111,0.00007292115147,4.90,0.0,0.0},
    {0.00000001629903,0.0,0.00000308769109,0.00000000814951,-0.00014584230293,1.00000153321056,0.00071462728438,0.0,0.0,0.0,-0.00007292115147,0.99986111111111,0.0,0.0,0.0},
    {0.0,0.0,0.0,0.0,0.00000015678559,1.1432986068972804e-11,0.99999923175059,0.0,0.0,0.0,0.0000000783928,0.0,-0.99986111111111,0.0,0.0},
    {-4.196626355780571e-16,0.0,0.0,-0.00000015784225,0.0,0.0,0.0,0.99999922391423,0.00007292115147,-0.00000007892113,0.0,0.0,0.0,-0.99986111111111,-0.00003646057573},
    {-1.151003864133914e-11,0.0,0.0,0.0,0.0,0.0,0.0,-0.00007292115147,0.99999999734125,0.0,0.0,0.0,0.0,0.00003646057573,-0.99986111111111},
    {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.99972226080247,0.0,0.0,0.0,0.0,0.0},
    {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.99972226080247,0.0,0.0,0.0,0.0},
    {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.99972226080247,0.0,0.0,0.0},
    {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.99972226080247,0.0,0.0},
    {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.99972226080247,0.0},
    {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.99972226080247}});
  Matrix expected_Qd = mat({
    {8.24017197865543e-12,0.0,0.0,1.6479183703404108e-11,0.0,4.074756876753095e-33,0.0,8.292350065673561e-13,-6.046886652110622e-17,8.351507939480248e-12,0.0,0.0,0.0,0.0,0.0},
    {0.0,8.240171978655426e-12,0.0,0.0,1.6479183703404105e-11,-1.2017656579392867e-15,-8.292350174791127e-13,0.0,0.0,0.0,8.351507939480247e-12,0.0,0.0,0.0,0.0},
    {0.0,0.0,4.176914062500499e-12,0.0,6.091707660324459e-16,8.352667871094247e-12,0.0,0.0,0.0,0.0,0.0,8.351507939480248e-12,0.0,0.0,0.0},
    {1.6479183703404108e-11,0.0,0.0,3.2956048653748266e-11,0.0,4.074753725226001e-33,0.0,1.6584703056165926e-12,-6.046887717994475e-17,1.6700696015643975e-11,0.0,0.0,0.0,-2.8784930485841387e-19,0.0},
    {0.0,1.6479183703404108e-11,6.091707660324459e-16,0.0,3.295604872098478e-11,-1.1851897838136817e-15,-1.6584703230294208e-12,0.0,0.0,0.0,1.6700696015643975e-11,1.2180031508653846e-15,2.8784930485841387e-19,0.0,0.0},
    {4.074756876753095e-33,-1.2017656579392869e-15,8.352667871094247e-12,4.0747537252260006e-33,-1.185189783813682e-15,1.6703015731937017e-11,1.2093754462266956e-16,-6.431687948141506e-40,0.0,0.0,-1.2180031508653846e-15,1.6700696015643975e-11,0.0,0.0,0.0},
    {0.0,-8.292350174791127e-13,0.0,0.0,-1.6584703230294206e-12,1.2093754462266956e-16,3.3846359848335194e-13,0.0,0.0,0.0,1.3093961354985395e-18,0.0,-5.873659709965196e-20,0.0,0.0},
    {8.29235006567356e-13,0.0,0.0,1.6584703056165924e-12,0.0,-6.431687948141506e-40,0.0,3.3846359673092043e-13,9.544541964032542e-24,-1.3182208064880816e-18,0.0,0.0,0.0,-5.873659709965196e-20,-2.1418676284950054e-24},
    {-6.046886652110622e-17,0.0,0.0,-6.046887717994475e-17,0.0,0.0,0.0,9.544541964032542e-24,3.384638585077646e-13,0.0,0.0,0.0,0.0,2.1418676284950054e-24,-5.873659709965196e-20},
    {8.351507939480248e-12,0.0,0.0,1.6700696015643975e-11,0.0,0.0,0.0,-1.3182208064880814e-18,0.0,3.340603304673392e-11,0.0,0.0,0.0,0.0,0.0},
    {0.0,8.351507939480247e-12,0.0,0.0,1.6700696015643975e-11,-1.2180031508653845e-15,1.3093961354985395e-18,0.0,0.0,0.0,3.340603304673392e-11,0.0,0.0,0.0,0.0},
    {0.0,0.0,8.351507939480248e-12,0.0,1.2180031508653845e-15,1.6700696015643975e-11,0.0,0.0,0.0,0.0,0.0,3.340603304673392e-11,0.0,0.0,0.0},
    {0.0,0.0,0.0,0.0,2.8784930485841387e-19,0.0,-5.873659709965196e-20,0.0,0.0,0.0,0.0,0.0,1.17489516719882e-19,0.0,0.0},
    {0.0,0.0,0.0,-2.8784930485841387e-19,0.0,0.0,0.0,-5.873659709965196e-20,2.1418676284950054e-24,0.0,0.0,0.0,0.0,1.17489516719882e-19,0.0},
    {0.0,0.0,0.0,0.0,0.0,0.0,0.0,-2.1418676284950054e-24,-5.873659709965196e-20,0.0,0.0,0.0,0.0,0.0,1.17489516719882e-19}});
  // clang-format on
  auto blk = provider->new_block(0, nullptr, "pinson15", "config/pinson_block");
  ASSERT_TRUE(blk);
  blk->receive_aux_data({zero_pva_aux, force_aux});
  Timestamp from(0), to(kSec);
  auto dyn = blk->generate_dynamics(fixed(zeros(15)), from, to);
  ASSERT_TRUE(dyn);
  EXPECT_TRUE(allclose(expected_Phi, dyn->Phi, 1e-5, 1e-20)) << "\n" << dyn->Phi;
  EXPECT_TRUE(allclose(expected_Qd, dyn->Qd, 1e-5, 1e-20)) << "\n" << dyn->Qd;
  // g is linear: g(x) == Phi x
  Vector x = Vector::Random(15);
  EXPECT_ALLCLOSE(dyn->g(x), Vector(dyn->Phi * x));
}

TEST_F(StateModelingTest, QIsNotMutatedAcrossCalls) {
  // Regression test for the in-place rotation bug in the Python original.
  auto blk = provider->new_block(0, nullptr, "pinson15", "config/pinson_block");
  auto* pinson = dynamic_cast<cobra::Pinson15NedBlock*>(blk.get());
  ASSERT_TRUE(pinson);
  nav::Vector4 q = nav::rpy_to_quat(Vector3(0.1, 0.2, 0.7));
  Message pva(make_pva(kSec, 0.6, -1.5, 300, 1, 2, 0, Vector(q)), "a");
  pinson->receive_aux_data({pva, force_aux});
  Matrix q1 = pinson->generate_q_pinson15();
  Matrix q2 = pinson->generate_q_pinson15();
  EXPECT_EQ(q1, q2);
}

TEST_F(StateModelingTest, LegacyQRotationReproducesThePythonMutation) {
  auto cfg = cobra::PinsonStateBlockConfig::from_registry(med, "config/pinson_block");
  ASSERT_TRUE(cfg);
  EXPECT_FALSE(cfg->legacy_q_rotation);  // absent key -> corrected behaviour
  cobra::Pinson15NedBlock legacy("p", &med, cfg->imu_model, /*legacy_q_rotation=*/true);
  nav::Vector4 q = nav::rpy_to_quat(Vector3(0.1, 0.2, 0.7));
  Message pva(make_pva(kSec, 0.6, -1.5, 300, 1, 2, 0, Vector(q)), "a");
  legacy.receive_aux_data({pva, force_aux});
  Matrix q1 = legacy.generate_q_pinson15();
  Matrix q2 = legacy.generate_q_pinson15();
  EXPECT_FALSE(allclose(q1, q2, 1e-12, 0.0)) << "legacy mode must re-rotate the stored Q";
  // round trip of the flag through the registry
  cobra::PinsonStateBlockConfig c2 = *cfg;
  c2.group_ = "config/pinson_legacy";
  c2.imu_model.group_ = "config/pinson_legacy/imu";
  c2.legacy_q_rotation = true;
  c2.to_registry(med);
  auto back = cobra::PinsonStateBlockConfig::from_registry(med, "config/pinson_legacy");
  ASSERT_TRUE(back);
  EXPECT_TRUE(back->legacy_q_rotation);
}

TEST_F(StateModelingTest, CloneBlockAndProcessorsAreIndependent) {
  auto blk = provider->new_block(0, nullptr, "pinson15", "config/pinson_block");
  auto* pinson = dynamic_cast<cobra::Pinson15NedBlock*>(blk.get());
  auto copy = blk->clone();
  auto* pinson2 = dynamic_cast<cobra::Pinson15NedBlock*>(copy.get());
  ASSERT_TRUE(pinson && pinson2);
  EXPECT_EQ(pinson2->imu_model().accel_bias_sigma, pinson->imu_model().accel_bias_sigma);
  pinson2->imu_model().accel_bias_sigma = {1.0, 2.0, 3.0};
  EXPECT_NE(pinson2->imu_model().accel_bias_sigma, pinson->imu_model().accel_bias_sigma);

  for (auto& m : all_pos_processors()) {
    auto* p = dynamic_cast<cobra::PinsonProcessorBase*>(m.mp.get());
    EXPECT_EQ(p->inertial_pva(), nullptr);
    p->receive_aux_data({pva_aux});
    auto c = m.mp->clone();
    auto* pc = dynamic_cast<cobra::PinsonProcessorBase*>(c.get());
    ASSERT_NE(pc->inertial_pva(), nullptr);
    EXPECT_DOUBLE_EQ(pc->inertial_pva()->get_p1(), p->inertial_pva()->get_p1());
    // Aux messages are shared immutable objects; giving the clone a different aux must not affect the original
    Message other(make_pva(kSec, 3, 3, 3, 0, 0, 0, vec({1, 0, 0, 0})), "pva_aux");
    pc->receive_aux_data({other});
    EXPECT_NE(pc->inertial_pva()->get_p1(), p->inertial_pva()->get_p1());
  }
}

}  // namespace
