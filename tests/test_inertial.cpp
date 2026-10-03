// Port of pntos-cobra/tests/test_inertial_plugin.py plus mechanization sanity checks.
#include <pntos/cobra/StandardRegistryPlugin.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/inertial/StandardInertialPlugin.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include "test_support.hpp"

using namespace pntos;
using namespace pntos::test;
using api::Matrix;
using api::Message;
using api::Timestamp;
using api::Vector;
using api::Vector3;
using api::Matrix3;
namespace nav = cobra::nav;
namespace inr = cobra::inertial;

namespace {
constexpr double kLat = 0.69, kLon = -0.8, kAlt = 0.0;

Message imu_at(std::int64_t tov) {
  // Integrated increments over 10 ms: gravity reaction + a hint of earth rotation.
  Vector3 g = nav::calculate_gravity_schwartz(kAlt, kLat);
  auto m = make_imu(tov, -g / 1e2, Vector3(1e-12, 1e-6, 1e-12) / 1e2);
  m->set_imu_type(ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED);
  return Message(m, "");
}
std::shared_ptr<cobra::utils::PVA> initial_pva() {
  return make_pva(0, kLat, kLon, kAlt, 0, 0, 0, Vector(nav::rpy_to_quat(Vector3::Zero())), Matrix::Identity(9, 9));
}
void expect_pva_equal(const cobra::utils::PVA& a, const cobra::utils::PVA& b) {
  EXPECT_EQ(a.get_time_of_validity().get_elapsed_nsec(), b.get_time_of_validity().get_elapsed_nsec());
  EXPECT_EQ(a.get_p1(), b.get_p1());
  EXPECT_EQ(a.get_p2(), b.get_p2());
  EXPECT_EQ(a.get_p3(), b.get_p3());
  EXPECT_EQ(a.get_v1(), b.get_v1());
  EXPECT_EQ(a.get_v2(), b.get_v2());
  EXPECT_EQ(a.get_v3(), b.get_v3());
  EXPECT_ALLCLOSE(Vector(a.get_quaternion()), Vector(b.get_quaternion()));
}
void expect_pva_close_but_unequal(const cobra::utils::PVA& a, const cobra::utils::PVA& b) {
  EXPECT_NE(a.get_p1(), b.get_p1());
  EXPECT_NEAR(a.get_p1(), b.get_p1(), 1e-9 * std::abs(b.get_p1()) + 1e-12);
  EXPECT_NE(a.get_p2(), b.get_p2());
  EXPECT_NEAR(a.get_p2(), b.get_p2(), 1e-9 * std::abs(b.get_p2()) + 1e-12);
  EXPECT_NE(a.get_p3(), b.get_p3());
  EXPECT_NEAR(a.get_p3(), b.get_p3(), 1e-12);
  EXPECT_NE(a.get_v1(), b.get_v1());
  EXPECT_NEAR(a.get_v1(), b.get_v1(), 1e-5);
  EXPECT_NE(a.get_v2(), b.get_v2());
  EXPECT_NEAR(a.get_v2(), b.get_v2(), 1e-5);
  EXPECT_NE(a.get_v3(), b.get_v3());
  EXPECT_NEAR(a.get_v3(), b.get_v3(), 1e-5);
  Vector qa = a.get_quaternion(), qb = b.get_quaternion();
  for (int i = 0; i < 4; ++i) EXPECT_NE(qa(i), qb(i));
  EXPECT_TRUE(allclose(qa, qb, 1e-5, 1e-5));
}

TEST(Mechanization, StationaryLevelImuStaysPut) {
  // With exact gravity increments and no rotation the solution should barely move over 10 s.
  inr::StandardPva s;
  s.llh = Vector3(kLat, kLon, kAlt);
  inr::Inertial ins(s);
  Vector3 g = nav::calculate_gravity_schwartz(kAlt, kLat);
  for (std::int64_t i = 1; i <= 1000; ++i) {
    ins.mechanize(Timestamp{i * 10'000'000}, -g * 0.01, Vector3::Zero());
  }
  const auto& out = ins.solution();
  EXPECT_EQ(out.time.elapsed_nsec, 10'000'000'000);
  // Earth rotation is not compensated in a static gyro (zero increments), so a small drift is expected:
  // ~ 0.5 * (omega*cos(lat)*g) * t^2 ... keep bounds loose but meaningful.
  EXPECT_LT((out.vned).norm(), 0.5);
  EXPECT_LT(nav::delta_lat_to_north(out.llh(0) - kLat, kLat, kAlt), 3.0);
  EXPECT_LT(std::abs(out.llh(2) - kAlt), 3.0);
}

TEST(Mechanization, ErrorModelAppliesBiasesAndScaleFactors) {
  inr::StandardPva s;
  s.llh = Vector3(kLat, kLon, kAlt);
  inr::Inertial a(s), b(s);
  inr::ImuErrors e;
  e.accel_biases = Vector3(0.1, 0, 0);  // m/s^2
  b.set_imu_errors(e);
  Vector3 g = nav::calculate_gravity_schwartz(kAlt, kLat);
  for (int i = 1; i <= 100; ++i) {
    a.mechanize(Timestamp{i * 10'000'000}, -g * 0.01, Vector3::Zero());
    // b's measurements carry the bias; after correction it should match a
    b.mechanize(Timestamp{i * 10'000'000}, -g * 0.01 + Vector3(0.1 * 0.01, 0, 0), Vector3::Zero());
  }
  EXPECT_ALLCLOSE(Vector(a.solution().vned), Vector(b.solution().vned));
  EXPECT_ALLCLOSE(Vector(a.solution().llh), Vector(b.solution().llh));
}

TEST(Mechanization, HelpersMatchClosedForms) {
  Matrix3 C = Matrix3::Identity();
  Vector3 dv(0.01, 0.02, -0.0981), dth(1e-4, 2e-4, 3e-4);
  Vector3 f = inr::calc_force_ned(C, 0.01, dth, dv);
  EXPECT_ALLCLOSE(Vector(f), Vector((dv + 0.5 * dth.cross(dv)) / 0.01));
  EXPECT_ALLCLOSE(Vector(nav::rot_vec_to_dcm(Vector3::Zero()) * Vector3(1, 2, 3)), vec({1, 2, 3}));
  // sixth-order rotation vector vs exact Rodrigues for a small angle
  Vector3 phi(0.01, -0.02, 0.005);
  EXPECT_TRUE(allclose(nav::rot_vec_to_dcm(phi), nav::axis_angle_to_dcm(phi, phi.norm()), 1e-9, 1e-12));
  EXPECT_NEAR(nav::wrap_to_pi(4.0), 4.0 - 2 * nav::PI, 1e-15);
  EXPECT_NEAR(nav::wrap_to_pi(-4.0), -4.0 + 2 * nav::PI, 1e-15);
  Vector3 rpy(0.1, -0.2, 0.3);
  EXPECT_ALLCLOSE(Vector(nav::quat_to_rpy(nav::rpy_to_quat(rpy))), Vector(rpy));
}

TEST(InertialPlugin, PortOfPythonTest) {
  cobra::StandardInertialPlugin plugin("Cobra inertial plugin");
  TestMediator med;
  cobra::StandardRegistryPlugin reg("registry", {});
  reg.init_plugin(std::nullopt, &med);
  med.set_registry(reg.new_registry());
  cobra::InertialConfig cfg;
  cfg.group_ = "test";
  cfg.expected_dt = 0.01;
  cfg.inertial_buffer_length = 5.0;
  cfg.channels = {"/sensor/imu"};
  cfg.to_registry(med);

  EXPECT_EQ(plugin.identifier(), "Cobra inertial plugin");
  plugin.init_plugin(std::nullopt, &med);
  EXPECT_TRUE(plugin.is_inertial_type_supported(api::InertialType::STANDARD_MECHANIZATION));
  EXPECT_FALSE(plugin.is_inertial_type_supported(api::InertialType::EXTERNAL));
  EXPECT_EQ(plugin.new_inertial(api::InertialType::STANDARD_MECHANIZATION, Message(initial_pva(), ""), std::nullopt),
            nullptr);
  EXPECT_EQ(plugin.new_inertial(api::InertialType::STANDARD_MECHANIZATION, Message(initial_pva(), ""), "nope"), nullptr);
  const std::size_t expected_errors = med.count(api::LoggingLevel::ERROR);  // the two rejections above

  auto pva = initial_pva();
  auto common = plugin.new_inertial(api::InertialType::STANDARD_MECHANIZATION, Message(pva, ""), "test");
  ASSERT_TRUE(common);
  auto* inertial = dynamic_cast<api::StandardInertialMechanization*>(common.get());
  ASSERT_TRUE(inertial);

  const Timestamp first{0}, mid{100'000'000}, final{130'000'000};
  EXPECT_TRUE(inertial->is_time_in_range(first));
  EXPECT_FALSE(inertial->is_time_in_range(mid));
  auto sol = inertial->request_current_solution().as<cobra::utils::PVA>();
  ASSERT_TRUE(sol);
  expect_pva_equal(*sol, *pva);
  EXPECT_EQ(inertial->request_earliest_time().elapsed_nsec, inertial->request_latest_time().elapsed_nsec);
  EXPECT_EQ(*inertial->request_reset_message_types(),
            std::vector<api::AspnMessageType>{ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE});
  EXPECT_EQ(inertial->request_solution_message_type(), ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE);

  for (int i = 0; i < 10; ++i) inertial->process_pntos_message(imu_at((i + 1) * 10'000'000));
  EXPECT_TRUE(inertial->is_time_in_range(first));
  EXPECT_TRUE(inertial->is_time_in_range(mid));
  sol = inertial->request_current_solution().as<cobra::utils::PVA>();
  EXPECT_EQ(sol->get_time_of_validity().get_elapsed_nsec(), mid.elapsed_nsec);
  expect_pva_close_but_unequal(*sol, *pva);
  EXPECT_EQ(inertial->request_earliest_time().elapsed_nsec, first.elapsed_nsec);
  EXPECT_EQ(inertial->request_latest_time().elapsed_nsec, mid.elapsed_nsec);

  // Reset to the initial solution at mid time.
  auto reset_pva = cobra::utils::copy_pva(*pva);
  reset_pva->set_time_of_validity(aspn23_eigen::TypeTimestamp(mid.elapsed_nsec));
  inertial->reset_solution(Message(reset_pva, ""));
  sol = inertial->request_current_solution().as<cobra::utils::PVA>();
  expect_pva_equal(*sol, *reset_pva);

  for (int i = 0; i < 3; ++i) inertial->process_pntos_message(imu_at((i + 11) * 10'000'000));
  auto continuous = inertial->request_solutions({first, final}, api::InertialSolutionRangeType::NO_UPDATES_WITHIN_RANGE);
  auto best = inertial->request_solutions({first, final}, api::InertialSolutionRangeType::BEST_KNOWN_SOLUTION);
  ASSERT_TRUE(continuous && best);
  ASSERT_TRUE((*continuous)[0] && (*continuous)[1] && (*best)[0] && (*best)[1]);
  auto c0 = (*continuous)[0]->as<cobra::utils::PVA>(), c1 = (*continuous)[1]->as<cobra::utils::PVA>();
  auto b0 = (*best)[0]->as<cobra::utils::PVA>(), b1 = (*best)[1]->as<cobra::utils::PVA>();
  EXPECT_EQ(c0->get_time_of_validity().get_elapsed_nsec(), first.elapsed_nsec);
  EXPECT_EQ(b0->get_time_of_validity().get_elapsed_nsec(), first.elapsed_nsec);
  EXPECT_EQ(c1->get_time_of_validity().get_elapsed_nsec(), final.elapsed_nsec);
  EXPECT_EQ(b1->get_time_of_validity().get_elapsed_nsec(), final.elapsed_nsec);
  expect_pva_equal(*c0, *b0);
  expect_pva_close_but_unequal(*c1, *b1);

  auto fr = inertial->request_forces_and_rates(first);
  ASSERT_TRUE(fr);
  EXPECT_EQ(fr->frame, api::InertialFrame::NED);
  EXPECT_EQ(fr->forces_and_rates->get_time_of_validity().get_elapsed_nsec(), first.elapsed_nsec);
  auto avg = inertial->request_average_forces_and_rates(first, final);
  ASSERT_TRUE(avg);
  EXPECT_EQ(avg->frame, api::InertialFrame::NED);
  const std::int64_t t = avg->forces_and_rates->get_time_of_validity().get_elapsed_nsec();
  EXPECT_GT(t, first.elapsed_nsec);
  EXPECT_LT(t, final.elapsed_nsec);
  // Static: force ~ -g in NED
  Vector3 f = Vector3(avg->forces_and_rates->get_meas_accel()(0), avg->forces_and_rates->get_meas_accel()(1),
                      avg->forces_and_rates->get_meas_accel()(2));
  EXPECT_NEAR(f(2), -nav::calculate_gravity_schwartz(kAlt, kLat)(2), 0.05);

  auto errs = inertial->request_sensor_errors(first);
  ASSERT_TRUE(errs);
  EXPECT_ALLCLOSE(Vector(errs->accel_biases), Vector::Zero(3));
  EXPECT_ALLCLOSE(Vector(errs->gyro_biases), Vector::Zero(3));
  errs->accel_biases = Vector3::Ones();
  inertial->correct_sensor_errors(first, *errs);
  errs = inertial->request_sensor_errors(first);
  ASSERT_TRUE(errs);
  EXPECT_ALLCLOSE(Vector(errs->accel_biases), Vector::Ones(3));

  // Non-IMU messages are ignored with a WARN
  inertial->process_pntos_message(Message(initial_pva(), "x"));
  EXPECT_GE(med.count(api::LoggingLevel::WARN), 1u);
  EXPECT_EQ(med.count(api::LoggingLevel::ERROR), expected_errors) << med.last_message();
}

TEST(BufferedImu, RingEvictsOldestAndInterpolates) {
  inr::BufferedImu buf(*initial_pva(), 0.01, 0.05);  // capacity 7
  for (int i = 1; i <= 20; ++i) {
    Vector3 g = nav::calculate_gravity_schwartz(kAlt, kLat);
    auto m = make_imu(i * 10'000'000, -g / 1e2, Vector3::Zero());
    m->set_imu_type(ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED);
    buf.add(m);
  }
  auto span = buf.time_span();
  EXPECT_EQ(span.second.elapsed_nsec, 200'000'000);
  EXPECT_GT(span.first.elapsed_nsec, 100'000'000);
  EXPECT_FALSE(buf.in_range(Timestamp{0}));
  auto mid = buf.calc_pva(Timestamp{185'000'000});
  ASSERT_TRUE(mid);
  EXPECT_EQ(mid->get_time_of_validity().get_elapsed_nsec(), 185'000'000);
  // out of range IMU in the past is rejected
  auto old = make_imu(50'000'000, Vector3::Zero(), Vector3::Zero());
  old->set_imu_type(ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED);
  EXPECT_FALSE(buf.add(old));
  // sampled type is rejected
  EXPECT_THROW(buf.add(make_imu(300'000'000, Vector3::Zero(), Vector3::Zero())), std::invalid_argument);
}

}  // namespace
