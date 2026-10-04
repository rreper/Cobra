// Exports the IMU, position, velocity and truth PVA channels of an LCM log to the CSV files the CSV
// transport reads (see transport/CsvTransportPlugin.hpp).
//   lcm_to_csv input.log out_prefix [--imu ch] [--position ch] [--velocity ch] [--truth ch]
// Writes out_prefix_imu.csv, out_prefix_position.csv, out_prefix_velocity.csv, out_prefix_truth.csv.
#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/transport/LcmLog.hpp>
#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>
#include <aspn23/eigen/MeasurementPosition.hpp>
#include <aspn23/eigen/MeasurementVelocity.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: lcm_to_csv input.log out_prefix [--imu ch] [--position ch] [--velocity ch] [--truth ch]\n";
    return 2;
  }
  std::string imu_ch = "/sensor/vn-100/imu", pos_ch = "/sensor/ublox-ZED-F9T/position", vel_ch = "/sensor/ublox-ZED-F9T/velocity",
              truth_ch = "/sensor/ins-d/pva";
  for (int i = 3; i + 1 < argc; i += 2) {
    const std::string s = argv[i];
    if (s == "--imu") imu_ch = argv[i + 1];
    else if (s == "--position") pos_ch = argv[i + 1];
    else if (s == "--velocity") vel_ch = argv[i + 1];
    else if (s == "--truth") truth_ch = argv[i + 1];
  }
  const std::string prefix = argv[2];
  std::ofstream imu(prefix + "_imu.csv"), pos(prefix + "_position.csv"), vel(prefix + "_velocity.csv"), truth(prefix + "_truth.csv");
  imu << std::setprecision(17) << "time,ax,ay,az,gx,gy,gz\n";
  pos << std::setprecision(17) << "time,lat_deg,lon_deg,alt_m,sigma_n_m,sigma_e_m,sigma_d_m\n";
  vel << std::setprecision(17) << "time,vn,ve,vd,sigma_n,sigma_e,sigma_d\n";
  truth << std::setprecision(17) << "time_s,lat_deg,lon_deg,alt_m,vn_mps,ve_mps,vd_mps,roll_deg,pitch_deg,yaw_deg\n";
  const double r2d = 180.0 / M_PI;
  std::size_t n_imu = 0, n_pos = 0, n_vel = 0, n_truth = 0;
  bool integrated = false;
  pntos::cobra::lcm::LcmLogReader reader(argv[1]);
  while (auto ev = reader.next()) {
    if (ev->channel != imu_ch && ev->channel != pos_ch && ev->channel != vel_ch && ev->channel != truth_ch) continue;
    auto msg = pntos::cobra::lcm::decode(ev->data);
    if (!msg) continue;
    if (ev->channel == imu_ch) {
      auto m = std::dynamic_pointer_cast<const aspn23_eigen::MeasurementImu>(msg);
      if (!m) continue;
      integrated = m->get_imu_type() == ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED;
      const auto a = m->get_meas_accel(), g = m->get_meas_gyro();
      imu << m->get_time_of_validity().get_elapsed_nsec() * 1e-9 << ',' << a(0) << ',' << a(1) << ',' << a(2) << ',' << g(0) << ',' << g(1) << ',' << g(2) << '\n';
      ++n_imu;
    } else if (ev->channel == pos_ch) {
      auto m = std::dynamic_pointer_cast<const aspn23_eigen::MeasurementPosition>(msg);
      if (!m) continue;
      const auto c = m->get_covariance();
      pos << m->get_time_of_validity().get_elapsed_nsec() * 1e-9 << ',' << m->get_term1() * r2d << ',' << m->get_term2() * r2d << ',' << m->get_term3()
          << ',' << std::sqrt(c(0, 0)) << ',' << std::sqrt(c(1, 1)) << ',' << std::sqrt(c(2, 2)) << '\n';
      ++n_pos;
    } else if (ev->channel == vel_ch) {
      auto m = std::dynamic_pointer_cast<const aspn23_eigen::MeasurementVelocity>(msg);
      if (!m) continue;
      const auto c = m->get_covariance();
      vel << m->get_time_of_validity().get_elapsed_nsec() * 1e-9 << ',' << m->get_x() << ',' << m->get_y() << ',' << m->get_z() << ','
          << std::sqrt(c(0, 0)) << ',' << std::sqrt(c(1, 1)) << ',' << std::sqrt(c(2, 2)) << '\n';
      ++n_vel;
    } else {
      auto m = std::dynamic_pointer_cast<const pntos::cobra::utils::PVA>(msg);
      if (!m) continue;
      auto q = pntos::cobra::utils::quaternion(*m);
      pntos::api::Vector3 rpy = q ? pntos::api::Vector3(pntos::cobra::nav::dcm_to_rpy(pntos::cobra::nav::quat_to_dcm(*q)) * r2d) : pntos::api::Vector3::Zero();
      truth << m->get_time_of_validity().get_elapsed_nsec() * 1e-9 << ',' << m->get_p1() * r2d << ',' << m->get_p2() * r2d << ',' << m->get_p3() << ','
            << m->get_v1() << ',' << m->get_v2() << ',' << m->get_v3() << ',' << rpy(0) << ',' << rpy(1) << ',' << rpy(2) << '\n';
      ++n_truth;
    }
  }
  std::cerr << "lcm_to_csv: " << n_imu << " IMU (" << (integrated ? "integrated" : "sampled") << "), " << n_pos << " position, " << n_vel
            << " velocity, " << n_truth << " truth rows -> " << prefix << "_*.csv\n";
  return 0;
}
