// Dumps numerical results of the C++ port for fixed inputs so tools/parity_check.py can diff them
// against the Python Cobra / navtk implementation. Build target: build/tools/parity_dump.
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/dummy/DummyPlugins.hpp>
#include <pntos/cobra/orchestration/OrchestrationUtils.hpp>
#include <pntos/cobra/state_modeling/MeasurementProcessors.hpp>
#include <pntos/cobra/state_modeling/Pinson15NedBlock.hpp>
#include <pntos/cobra/state_modeling/VirtualStateBlocks.hpp>
#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <aspn23/eigen/MeasurementVelocity.hpp>

#include <iomanip>
#include <iostream>

using namespace pntos;
using namespace pntos::cobra;
namespace nav = pntos::cobra::nav;
using api::Matrix;
using api::Vector;
using api::Vector3;

namespace {
std::ostream& out = std::cout;
bool first_key = true;
void key(const std::string& k) {
  out << (first_key ? "" : ",\n") << "  \"" << k << "\": ";
  first_key = false;
}
void dump(const std::string& k, const Matrix& m) {
  key(k);
  out << "[";
  for (Eigen::Index i = 0; i < m.rows(); ++i) {
    out << (i ? "," : "") << "[";
    for (Eigen::Index j = 0; j < m.cols(); ++j) out << (j ? "," : "") << std::setprecision(17) << m(i, j);
    out << "]";
  }
  out << "]";
}
void dump(const std::string& k, const Vector& v) { dump(k, Matrix(v.transpose())); }
void dump(const std::string& k, double d) {
  key(k);
  out << std::setprecision(17) << d;
}
}  // namespace

int main() {
  const double lat = 0.69, lon = -0.8, alt = 250.0;
  const Vector3 rpy(0.05, -0.1, 0.8);
  const Vector3 vel(5.0, -3.0, 0.5);
  const nav::Vector4 q = nav::rpy_to_quat(rpy);
  const Vector3 tilt(0.002, -0.001, 0.003);
  const Vector3 force(0.2, -0.1, -9.7), rate(0.01, -0.02, 0.03);
  const std::int64_t t_ns = 1'000'000'000;

  out << "{\n";
  dump("quat", Vector(q));
  dump("quat_to_dcm", Matrix(nav::quat_to_dcm(q)));
  dump("dcm_to_rpy", Vector(nav::dcm_to_rpy(nav::quat_to_dcm(q))));
  dump("correct_dcm_with_tilt", Matrix(nav::correct_dcm_with_tilt(nav::quat_to_dcm(q), tilt)));
  dump("ortho_dcm", Matrix(nav::ortho_dcm(nav::quat_to_dcm(q) + 1e-3 * Matrix::Ones(3, 3))));
  dump("gravity_schwartz", Vector(nav::calculate_gravity_schwartz(alt, lat)));
  dump("skew", Matrix(nav::skew(vel)));
  dump("d_rpy_to_dcm_wrt_r", Matrix(nav::d_rpy_to_dcm_wrt_r(rpy)));
  dump("d_rpy_to_dcm_wrt_y", Matrix(nav::d_rpy_to_dcm_wrt_y(rpy)));
  dump("meridian_radius", nav::meridian_radius(lat));
  dump("north_to_delta_lat", nav::north_to_delta_lat(100.0, lat, alt));

  DummyMediator med;
  ImuConfig imu;
  imu.accel_bias_sigma = {2.4e-3, 2.4e-3, 2.4e-3};
  imu.accel_bias_tau = {300, 300, 300};
  imu.accel_random_walk_sigma = {3.887e-6, 3.887e-6, 3.887e-6};
  imu.gyro_bias_sigma = {2e-4, 2e-4, 2e-4};
  imu.gyro_bias_tau = {500, 500, 500};
  imu.gyro_random_walk_sigma = {9.9e-4, 9.9e-4, 6.7e-5};
  auto pva = utils::make_pva(aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0),
                             api::Timestamp{t_ns}, Vector3(lat, lon, alt), vel, q, Matrix::Zero(9, 9));
  auto imu_msg = std::make_shared<aspn23_eigen::MeasurementImu>(
      aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0), aspn23_eigen::TypeTimestamp(t_ns),
      ASPN23_MEASUREMENT_IMU_IMU_TYPE_SAMPLED, Eigen::Matrix<double, Eigen::Dynamic, 1>(force),
      Eigen::Matrix<double, Eigen::Dynamic, 1>(rate), std::vector<aspn23_eigen::TypeIntegrity>{});
  api::Message pva_m(pva, "pva"), imu_m(imu_msg, "imu");
  Vector x = Vector::Constant(15, 0.01);
  auto gen = [&](const std::vector<std::string>& labels) -> std::optional<api::EstimateWithCovariance> {
    const int n = static_cast<int>(15 + 3 * (labels.size() - 1));
    return api::EstimateWithCovariance{api::EstimateWithCovarianceType::EWC_GENERIC, Vector::Constant(n, 0.01),
                                       Matrix::Identity(n, n)};
  };

  Pinson15NedBlock block("pinson15", &med, imu);
  block.receive_aux_data({pva_m, imu_m});
  auto dyn = block.generate_dynamics(gen, api::Timestamp{t_ns}, api::Timestamp{t_ns + 500'000'000});
  dump("pinson_Phi", dyn->Phi);
  dump("pinson_Qd", dyn->Qd);

  const Vector3 la(-0.5, 0.38, -0.05);
  auto pos = std::make_shared<aspn23_eigen::MeasurementPosition>(
      aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0), aspn23_eigen::TypeTimestamp(t_ns),
      ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC, lat + 2e-6, lon - 3e-6, alt + 1.5,
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(Matrix::Identity(3, 3) * 4.0),
      ASPN23_MEASUREMENT_POSITION_ERROR_MODEL_NONE, Eigen::Matrix<double, Eigen::Dynamic, 1>(0),
      std::vector<aspn23_eigen::TypeIntegrity>{});
  PinsonPositionMeasurementProcessor pmp(PinsonPositionMeasurementProcessor::Kind::Plain, "pos", {"pinson15"}, &med, la);
  pmp.receive_aux_data({pva_m});
  auto mm = pmp.generate_model(api::Message(pos, "gps"), gen);
  dump("pos_H", mm->H);
  dump("pos_z", mm->z);
  dump("pos_h", mm->h(x));
  dump("pos_R", mm->R);
  PinsonPositionMeasurementProcessor fmp(PinsonPositionMeasurementProcessor::Kind::WithNedFogm, "pos", {"pinson15", "f"}, &med, la);
  fmp.receive_aux_data({pva_m});
  auto fm = fmp.generate_model(api::Message(pos, "gps"), gen);
  dump("posfogm_H", fm->H);
  dump("posfogm_h", fm->h(Vector::Constant(18, 0.01)));

  auto velm = std::make_shared<aspn23_eigen::MeasurementVelocity>(
      aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_VELOCITY, 0, 0, 0, 0), aspn23_eigen::TypeTimestamp(t_ns),
      ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED, 5.2, -2.9, 0.4,
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(Matrix::Identity(3, 3) * 0.25),
      ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE, Eigen::Matrix<double, Eigen::Dynamic, 1>(0),
      std::vector<aspn23_eigen::TypeIntegrity>{});
  PinsonVelocityMeasurementProcessor vmp("vel", {"pinson15"}, &med);
  vmp.receive_aux_data({pva_m});
  auto vm = vmp.generate_model(api::Message(velm, "vel"), gen);
  dump("vel_H", vm->H);
  dump("vel_z", vm->z);
  dump("vel_h", vm->h(x));

  PinsonErrorToStandard pes(&med, "pinson15", "direct");
  pes.receive_aux_data({pva_m});
  dump("pes_convert", pes.convert_estimate(x, api::Timestamp{t_ns}));
  dump("pes_jacobian", pes.jacobian(x, api::Timestamp{t_ns}));

  auto corrected = orch::apply_error_states(*pva, x);
  Vector c(10);
  c << corrected->get_p1(), corrected->get_p2(), corrected->get_p3(), corrected->get_v1(), corrected->get_v2(),
      corrected->get_v3(), Vector(corrected->get_quaternion());
  dump("apply_error_states", c);
  out << "\n}\n";
  return 0;
}
