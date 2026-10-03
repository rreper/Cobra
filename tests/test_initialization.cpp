// Ports of test_manual_initialization_plugin.py and tests/inertial_alignment/* plus the PVA-message strategy.
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/initialization/InitializationPlugins.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include "test_support.hpp"

using namespace pntos;
using namespace pntos::test;
using api::InitializationStatus;
using api::Matrix;
using api::Message;
using api::Timestamp;
using api::Vector;
using api::Vector3;
using api::Matrix3;
namespace nav = cobra::nav;

namespace {
struct Env {
  TestMediator med;
  cobra::StandardRegistryPlugin reg{"registry", {}};
  Env() {
    reg.init_plugin(std::nullopt, &med);
    med.set_registry(reg.new_registry());
  }
};

TEST(ManualInitialization, PortOfPythonTest) {
  Env env;
  cobra::TutorialInitializationPlugin plugin("Cobra simple initialization plugin");
  cobra::ManualAlignmentConfig c;
  c.group_ = "test";
  c.initial_pos = {1, 2, 3};
  c.initial_vel = {4, 5, 6};
  c.initial_rpy = {0.123, 0.456, 0.789};
  c.initial_accel_bias = {10, 11, 12};
  c.initial_gyro_bias = {13, 14, 15};
  c.initial_accel_scale_factor = {16, 17, 18};
  c.initial_gyro_scale_factor = {19, 20, 21};
  c.initial_time = 1.23;
  c.initial_pos_var = {10, 20, 30};
  c.initial_vel_var = {40, 50, 60};
  c.initial_tilt_var = {70, 80, 90};
  c.initial_accel_bias_var = {100, 110, 120};
  c.initial_gyro_bias_var = {130, 140, 150};
  c.initial_accel_scale_factor_var = {160, 170, 180};
  c.initial_gyro_scale_factor_var = {190, 200, 210};
  c.to_registry(env.med);
  EXPECT_EQ(plugin.identifier(), "Cobra simple initialization plugin");
  plugin.init_plugin(std::nullopt, &env.med);
  EXPECT_TRUE(plugin.is_initialization_type_supported(api::InitializationType::INERTIAL));
  EXPECT_FALSE(plugin.is_initialization_type_supported(api::InitializationType::EWC));
  EXPECT_EQ(plugin.new_initialization_strategy(api::InitializationType::INERTIAL, std::nullopt), nullptr);
  EXPECT_EQ(plugin.new_initialization_strategy(api::InitializationType::INERTIAL, "missing"), nullptr);

  auto s = plugin.new_initialization_strategy(api::InitializationType::INERTIAL, "test");
  ASSERT_TRUE(s);
  auto* aligner = dynamic_cast<api::InertialInitializationStrategy*>(s.get());
  ASSERT_TRUE(aligner);
  EXPECT_EQ(aligner->request_current_status(), InitializationStatus::INITIALIZED_GOOD);
  EXPECT_EQ(aligner->request_motion_needed(), api::InitializationMotionNeeded::ANY_MOTION);
  auto sol = aligner->request_solution();
  ASSERT_TRUE(sol.solution);
  EXPECT_EQ(sol.solution->source_identifier, "Cobra simple initialization");
  auto pva = sol.solution->as<cobra::utils::PVA>();
  ASSERT_TRUE(pva);
  EXPECT_EQ(pva->get_time_of_validity().get_elapsed_nsec(), 1'230'000'000);
  EXPECT_EQ(pva->get_p1(), 1);
  EXPECT_EQ(pva->get_p2(), 2);
  EXPECT_EQ(pva->get_p3(), 3);
  EXPECT_EQ(pva->get_v1(), 4);
  EXPECT_EQ(pva->get_v3(), 6);
  EXPECT_ALLCLOSE(Vector(nav::quat_to_rpy(*cobra::utils::quaternion(*pva))), vec({0.123, 0.456, 0.789}));
  Vector d(9);
  d << 10, 20, 30, 40, 50, 60, 70, 80, 90;
  EXPECT_ALLCLOSE(Matrix(pva->get_covariance()), Matrix(d.asDiagonal()));
  ASSERT_TRUE(sol.inertial_error_covariance);
  EXPECT_ALLCLOSE(Vector(sol.inertial_error_covariance->diagonal()), vec({100, 110, 120, 130, 140, 150}));
  ASSERT_TRUE(sol.inertial_errors);
  EXPECT_ALLCLOSE(Vector(sol.inertial_errors->accel_biases), vec({10, 11, 12}));
  EXPECT_ALLCLOSE(Vector(sol.inertial_errors->gyro_biases), vec({13, 14, 15}));
  EXPECT_ALLCLOSE(Vector(sol.inertial_errors->accel_scale_factors), vec({16, 17, 18}));
  EXPECT_ALLCLOSE(Vector(sol.inertial_errors->gyro_scale_factors), vec({19, 20, 21}));
  EXPECT_EQ(sol.status, InitializationStatus::INITIALIZED_GOOD);
}

// ---- tests/inertial_alignment/utils.py
constexpr double kLat = 1, kLon = 2, kAlt = 3;
Message position_at(std::int64_t t) { return Message(make_position(t, kLat, kLon, kAlt, Matrix::Identity(3, 3)), ""); }
Message imu_at(std::int64_t t) {
  const double dt = 1e-2;
  auto m = make_imu(t, Vector3(1e-12, 1e-12, -9.81) * dt, Vector3(1e-12, 1e-4, 1e-12) * dt);
  m->set_imu_type(ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED);
  return Message(m, "");
}

void check_inertial_align_plugin(api::InitializationPlugin& plugin, const std::string& group, double static_time,
                                 const std::string& identifier, bool expect_inertial_errors) {
  EXPECT_EQ(plugin.identifier(), identifier);
  EXPECT_TRUE(plugin.is_initialization_type_supported(api::InitializationType::INERTIAL));
  EXPECT_FALSE(plugin.is_initialization_type_supported(api::InitializationType::EWC));
  auto s = plugin.new_initialization_strategy(api::InitializationType::INERTIAL, group);
  ASSERT_TRUE(s);
  auto* aligner = dynamic_cast<api::InertialInitializationStrategy*>(s.get());
  ASSERT_TRUE(aligner);
  EXPECT_EQ(aligner->request_current_status(), InitializationStatus::INITIALIZING_COARSE);
  EXPECT_EQ(aligner->request_motion_needed(), api::InitializationMotionNeeded::NO_MOTION);
  auto sol = aligner->request_solution();
  EXPECT_FALSE(sol.solution);
  EXPECT_FALSE(sol.inertial_error_covariance);
  EXPECT_FALSE(sol.inertial_errors);
  EXPECT_EQ(sol.status, InitializationStatus::INITIALIZING_COARSE);

  const int pos_dt_cs = 100;
  const int align_cs = static_cast<int>(static_time) * 100;
  for (int ii = 0; ii <= align_cs; ++ii) {
    const std::int64_t t = ii * 10'000'000LL;
    aligner->process_pntos_message(imu_at(t));
    if (ii % pos_dt_cs == 0 && ii != align_cs) aligner->process_pntos_message(position_at(t));
  }
  ASSERT_EQ(aligner->request_current_status(), InitializationStatus::INITIALIZED_GOOD);
  sol = aligner->request_solution();
  ASSERT_TRUE(sol.solution);
  EXPECT_EQ(sol.solution->source_identifier, "Cobra initializer");
  auto pva = sol.solution->as<cobra::utils::PVA>();
  ASSERT_TRUE(pva);
  EXPECT_EQ(pva->get_time_of_validity().get_elapsed_nsec(), align_cs * 10'000'000LL);
  EXPECT_NEAR(pva->get_p1(), kLat, 1e-9);
  EXPECT_NEAR(pva->get_p2(), kLon, 1e-9);
  EXPECT_NEAR(pva->get_p3(), kAlt, 1e-9);
  EXPECT_EQ(pva->get_v1(), 0);
  EXPECT_EQ(pva->get_v2(), 0);
  EXPECT_EQ(pva->get_v3(), 0);
  Vector3 rpy = nav::quat_to_rpy(*cobra::utils::quaternion(*pva));
  // Level, facing the direction of the (simulated) earth rotation: yaw -pi/2.
  EXPECT_TRUE(allclose(Vector(rpy), vec({0, 0, -nav::PI / 2}))) << rpy.transpose();
  Matrix cov = pva->get_covariance();
  cov.diagonal().setZero();
  EXPECT_TRUE(allclose(cov, Matrix::Zero(9, 9))) << cov;
  ASSERT_TRUE(sol.inertial_error_covariance);
  Matrix ecov = *sol.inertial_error_covariance;
  EXPECT_EQ(ecov.rows(), 6);
  ecov.diagonal().setZero();
  EXPECT_TRUE(allclose(ecov, Matrix::Zero(6, 6))) << ecov;
  EXPECT_EQ(sol.inertial_errors.has_value(), expect_inertial_errors);
  EXPECT_EQ(sol.status, InitializationStatus::INITIALIZED_GOOD);
}

TEST(StaticAlign, PortOfPythonTest) {
  Env env;
  const std::string id = "Cobra static align initialization plugin";
  cobra::StaticAlignInitializationPlugin plugin(id);
  plugin.init_plugin(std::nullopt, &env.med);
  const std::string group = "test/config/static_align";
  cobra::StaticAlignmentConfig c;
  c.group_ = group;
  c.static_time = 120.0;
  c.imu_model = cobra::inertial::hg1700_model().to_config(group);
  c.to_registry(env.med);
  check_inertial_align_plugin(plugin, group, c.static_time, id, false);
}

TEST(ManualHeadingAlign, PortOfPythonTest) {
  Env env;
  const std::string id = "Cobra static align initialization plugin";
  cobra::ManualHeadingAlignInitializationPlugin plugin(id);
  plugin.init_plugin(std::nullopt, &env.med);
  const std::string group = "test/config/static_align";
  cobra::ManualHeadingAlignmentConfig c;
  c.group_ = group;
  c.static_time = 120.0;
  c.imu_model = cobra::inertial::hg1700_model().to_config(group);
  c.heading = -nav::PI / 2;
  c.heading_sigma = 0.017453292519943295;
  c.to_registry(env.med);
  check_inertial_align_plugin(plugin, group, c.static_time, id, true);
}

TEST(QuaternionStaticAlignment, RecoversKnownAttitude) {
  // A sensor rotated by rpy relative to NED sees gravity and earth rate rotated into its frame.
  Vector3 rpy(0.05, -0.1, 0.8);
  Matrix3 C_s_to_n = nav::rpy_to_dcm(rpy);
  Vector3 f_n(0, 0, -9.81);
  Vector3 w_n(nav::ROTATION_RATE * std::cos(0.7), 0, -nav::ROTATION_RATE * std::sin(0.7));
  Vector3 dv = C_s_to_n.transpose() * f_n * 0.01;
  Vector3 dth = C_s_to_n.transpose() * w_n * 0.01;
  Matrix3 est = cobra::inertial::quaternion_static_alignment(dv, dth);
  EXPECT_TRUE(allclose(est, C_s_to_n, 1e-6, 1e-8)) << est << "\n" << C_s_to_n;
}

TEST(PvaMessageInitialization, TakesFirstPvaAfterStart) {
  Env env;
  cobra::PvaMessageInitializationPlugin plugin("pva init");
  plugin.init_plugin(std::nullopt, &env.med);
  cobra::PvaMessageInitializationConfig c;
  c.group_ = "config/pva_init";
  c.initial_pva_channel = "/pva";
  c.initial_accel_bias_sigma = {1e-3, 1e-3, 1e-3};
  c.initial_gyro_bias_sigma = {1e-5, 1e-5, 1e-5};
  c.initial_pva_sigma = std::array<double, 9>{1, 1, 1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01};
  c.start_time = 1.0;
  c.to_registry(env.med);
  auto back = cobra::PvaMessageInitializationConfig::from_registry(env.med, "config/pva_init");
  ASSERT_TRUE(back);
  EXPECT_EQ(back->start_time, 1.0);
  ASSERT_TRUE(back->initial_pva_sigma);
  EXPECT_EQ((*back->initial_pva_sigma)[8], 0.01);

  auto s = plugin.new_initialization_strategy(api::InitializationType::INERTIAL, "config/pva_init");
  ASSERT_TRUE(s);
  auto* init = dynamic_cast<api::InertialInitializationStrategy*>(s.get());
  ASSERT_TRUE(init);
  EXPECT_EQ(init->request_current_status(), InitializationStatus::WAITING);
  auto pva0 = make_pva(500'000'000, 0.1, 0.2, 10, 1, 2, 3, vec({1, 0, 0, 0}));
  init->process_pntos_message(Message(pva0, "/pva"));  // before start time
  EXPECT_EQ(init->request_current_status(), InitializationStatus::WAITING);
  init->process_pntos_message(Message(make_pva(2'000'000'000, 0.1, 0.2, 10, 1, 2, 3, vec({1, 0, 0, 0})), "/other"));
  EXPECT_EQ(init->request_current_status(), InitializationStatus::WAITING);
  init->process_pntos_message(Message(make_position(2'000'000'000, 0.1, 0.2, 10, Matrix::Identity(3, 3)), "/pva"));
  EXPECT_EQ(init->request_current_status(), InitializationStatus::WAITING);  // wrong type -> WARN
  init->process_pntos_message(Message(make_pva(2'000'000'000, 0.1, 0.2, 10, 1, 2, 3, vec({1, 0, 0, 0})), "/pva"));
  EXPECT_EQ(init->request_current_status(), InitializationStatus::INITIALIZED_GOOD);
  auto sol = init->request_solution();
  ASSERT_TRUE(sol.solution);
  auto pva = sol.solution->as<cobra::utils::PVA>();
  EXPECT_EQ(pva->get_time_of_validity().get_elapsed_nsec(), 2'000'000'000);
  EXPECT_NEAR(pva->get_covariance()(3, 3), 0.01, 1e-15);
  ASSERT_TRUE(sol.inertial_error_covariance);
  EXPECT_NEAR((*sol.inertial_error_covariance)(0, 0), 1e-6, 1e-20);
  EXPECT_NEAR((*sol.inertial_error_covariance)(5, 5), 1e-10, 1e-24);
}

}  // namespace
