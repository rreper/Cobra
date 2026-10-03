// Replays the first seconds of an LCM log through the C++ alignment + BufferedImu and prints JSON
// for tools/inertial_parity_check.py (which does the same with navtk).
//   inertial_parity_dump <log> [seconds=25]
#include <pntos/cobra/inertial/BufferedImu.hpp>
#include <pntos/cobra/initialization/Alignment.hpp>
#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/transport/LcmLog.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include <iomanip>
#include <iostream>

using namespace pntos;
using namespace pntos::cobra;
using api::Matrix;
using api::Vector;
using api::Vector3;

namespace {
bool first = true;
void key(const std::string& k) { std::cout << (first ? "" : ",\n") << "  \"" << k << "\": "; first = false; }
void dump(const std::string& k, const Matrix& m) {
  key(k);
  std::cout << "[";
  for (Eigen::Index i = 0; i < m.rows(); ++i) {
    std::cout << (i ? "," : "") << "[";
    for (Eigen::Index j = 0; j < m.cols(); ++j) std::cout << (j ? "," : "") << std::setprecision(17) << m(i, j);
    std::cout << "]";
  }
  std::cout << "]";
}
void dump(const std::string& k, const Vector& v) { dump(k, Matrix(v.transpose())); }
Vector pva_vec(const utils::PVA& p) {
  Vector v(11);
  v << p.get_time_of_validity().get_elapsed_nsec() * 1e-9, p.get_p1(), p.get_p2(), p.get_p3(), p.get_v1(), p.get_v2(),
      p.get_v3(), Vector(p.get_quaternion());
  return v;
}
Vector imu_vec(const aspn23_eigen::MeasurementImu& m) {
  Vector v(7);
  v << m.get_time_of_validity().get_elapsed_nsec() * 1e-9, Vector(m.get_meas_accel()), Vector(m.get_meas_gyro());
  return v;
}
}  // namespace

int main(int argc, char** argv) {
  const std::string log = argv[1];
  const double seconds = argc > 2 ? std::atof(argv[2]) : 25.0;
  const std::string imu_ch = "/sensor/vn-100/imu", pos_ch = "/sensor/ublox-ZED-F9T/position";
  ImuConfig imu_cfg;
  imu_cfg.accel_bias_sigma = {2.4e-3, 2.4e-3, 2.4e-3};
  imu_cfg.accel_bias_tau = {300, 300, 300};
  imu_cfg.accel_random_walk_sigma = {3.887e-6, 3.887e-6, 3.887e-6};
  imu_cfg.gyro_bias_sigma = {2e-4, 2e-4, 2e-4};
  imu_cfg.gyro_bias_tau = {500, 500, 500};
  imu_cfg.gyro_random_walk_sigma = {9.9e-4, 9.9e-4, 6.7e-5};
  imu_cfg.accel_bias_initial_sigma = {0.072, 0.072, 0.072};
  imu_cfg.gyro_bias_initial_sigma = {0.003, 0.003, 0.003};
  inertial::ManualHeadingAlignment align(0.06895795874629593, 0.02236067977, inertial::ImuModel::from_config(imu_cfg), 10.0);

  lcm::LcmLogReader reader(log);
  std::optional<std::int64_t> t0;
  std::unique_ptr<inertial::BufferedImu> buf;
  std::int64_t align_t = 0;
  std::cout << "{\n";
  while (auto ev = reader.next()) {
    if (ev->channel != imu_ch && ev->channel != pos_ch) continue;
    auto msg = lcm::decode(ev->data);
    auto t = utils::time_of_validity(*msg)->elapsed_nsec;
    if (!t0) t0 = t;
    if ((t - *t0) * 1e-9 > seconds) break;
    if (!buf) {
      align.process(*msg);
      if (align.check_alignment_status() == inertial::AlignmentStatus::ALIGNED_GOOD) {
        auto [ok, sol] = align.get_computed_alignment();
        auto [okc, cov] = align.get_computed_covariance();
        auto [oke, err] = align.get_imu_errors();
        align_t = sol.time.elapsed_nsec;
        Vector s(10);
        s << sol.time.elapsed_nsec * 1e-9, sol.pos, sol.vel, nav::dcm_to_rpy(sol.rot_mat.transpose());
        dump("align_solution", s);
        dump("align_cov_diag", Vector(cov.diagonal()));
        Vector e(6);
        e << err.accel_biases, err.gyro_biases;
        dump("align_imu_errors", e);
        auto pva = utils::make_pva(aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE, 0, 0, 0, 0), sol.time,
                                   sol.pos, sol.vel, nav::dcm_to_quat(sol.rot_mat.transpose()), Matrix(cov.topLeftCorner(9, 9)));
        buf = std::make_unique<inertial::BufferedImu>(*pva, 0.01, 10.0);
        inertial::ImuErrors ie{err.accel_biases, err.gyro_biases, Vector3::Zero(), Vector3::Zero(), sol.time};
        buf->reset(nullptr, &ie);
      }
      continue;
    }
    if (auto imu = std::dynamic_pointer_cast<aspn23_eigen::MeasurementImu>(msg)) buf->add(imu);
  }
  auto [first_t, last_t] = buf->time_span();
  dump("span", Vector(Eigen::Vector2d(first_t.elapsed_nsec * 1e-9, last_t.elapsed_nsec * 1e-9)));
  // PVA at whole seconds after alignment, plus at the latest time
  Matrix pvas(0, 11);
  const std::int64_t grid0 = align_t + ((first_t.elapsed_nsec - align_t) / 1'000'000'000 + 1) * 1'000'000'000;
  for (std::int64_t t = grid0; t <= last_t.elapsed_nsec; t += 1'000'000'000) {
    auto p = buf->calc_pva(api::Timestamp{t});
    pvas.conservativeResize(pvas.rows() + 1, 11);
    Vector row = p ? pva_vec(*p) : Vector(Vector::Constant(11, std::nan("")));
    pvas.row(pvas.rows() - 1) = row.transpose();
  }
  dump("pva_grid", pvas);
  dump("pva_latest", pva_vec(*buf->calc_pva(last_t)));
  const api::Timestamp mid{align_t + 8'000'000'000};
  dump("force_rate_mid", imu_vec(*buf->calc_force_and_rate(mid)));
  dump("force_rate_avg", imu_vec(*buf->calc_force_and_rate(api::Timestamp{mid.elapsed_nsec - 500'000'000}, api::Timestamp{mid.elapsed_nsec + 500'000'000})));
  auto e0 = buf->imu_errors(mid);
  Vector ev(6);
  ev << e0.accel_biases, e0.gyro_biases;
  dump("imu_errors_mid", ev);
  // Reset at mid with a perturbed solution and new biases, then look at the end again
  auto p_mid = buf->calc_pva(mid);
  auto perturbed = utils::make_pva(p_mid->get_header(), mid, utils::position(*p_mid) + Vector3(1e-6, -1e-6, 2.0),
                                   utils::velocity(*p_mid) + Vector3(0.1, -0.1, 0.05),
                                   nav::dcm_to_quat(nav::correct_dcm_with_tilt(nav::quat_to_dcm(*utils::quaternion(*p_mid)), Vector3(0.001, -0.002, 0.003))),
                                   Matrix::Zero(9, 9));
  buf->reset(perturbed.get(), nullptr);
  inertial::ImuErrors ne{e0.accel_biases + Vector3(0.01, 0, 0), e0.gyro_biases + Vector3(0, 1e-4, 0), Vector3::Zero(), Vector3::Zero(), mid};
  buf->reset(nullptr, &ne);
  dump("pva_after_reset_latest", pva_vec(*buf->calc_pva(last_t)));
  dump("pva_after_reset_mid_plus_2", pva_vec(*buf->calc_pva(api::Timestamp{mid.elapsed_nsec + 2'000'000'000})));
  auto nr = buf->calc_pva_no_reset_since(last_t, api::Timestamp{mid.elapsed_nsec - 1'000'000'000});
  dump("pva_no_reset_since", pva_vec(*nr));
  std::cout << "\n}\n";
  return 0;
}
