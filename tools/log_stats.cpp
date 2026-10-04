// Prints, as JSON, the error statistics of a solution channel in an LCM log against a truth PVA
// channel, with no Python dependency (used by tools/ci_acceptance.py on the 60 s test log).
//   log_stats out.log [--solution /solution/pntos/pva] [--truth /sensor/ins-d/pva] [--truth-log in.log]
#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/transport/LcmLog.hpp>
#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace pntos;

namespace {
struct Sample {
  double t;
  api::Vector3 llh, vel;
  api::Matrix3 C;
};
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: log_stats out.log [--solution ch] [--truth ch]\n";
    return 2;
  }
  std::string solution = "/solution/pntos/pva", truth_ch = "/sensor/ins-d/pva", truth_log = argv[1];
  for (int i = 2; i + 1 < argc; i += 2) {
    const std::string s = argv[i];
    if (s == "--solution") solution = argv[i + 1];
    else if (s == "--truth") truth_ch = argv[i + 1];
    else if (s == "--truth-log") truth_log = argv[i + 1];
  }
  std::vector<Sample> sol, truth;
  std::size_t nan_count = 0;
  try {
    auto scan = [&](const std::string& path, bool want_sol, bool want_truth) {
      cobra::lcm::LcmLogReader reader(path);
      while (auto ev = reader.next()) {
        const bool is_sol = want_sol && ev->channel == solution, is_truth = want_truth && ev->channel == truth_ch;
        if (!is_sol && !is_truth) continue;
        auto pva = std::dynamic_pointer_cast<const cobra::utils::PVA>(cobra::lcm::decode(ev->data));
        if (!pva) continue;
        auto q = cobra::utils::quaternion(*pva);
        if (!q) continue;
        Sample s{pva->get_time_of_validity().get_elapsed_nsec() * 1e-9, cobra::utils::position(*pva), cobra::utils::velocity(*pva),
                 cobra::nav::quat_to_dcm(*q)};
        if (is_sol && (!s.llh.allFinite() || !s.vel.allFinite() || !s.C.allFinite())) ++nan_count;
        (is_sol ? sol : truth).push_back(s);
      }
    };
    scan(argv[1], true, truth_log == argv[1]);
    if (truth_log != argv[1]) scan(truth_log, false, true);
  } catch (const std::exception& e) {
    std::cerr << "log_stats: " << e.what() << "\n";
    return 1;
  }
  double sq_pos[3] = {0, 0, 0}, sq_vel[3] = {0, 0, 0}, sq_rpy[3] = {0, 0, 0};
  std::size_t n = 0, j = 0;
  for (const auto& s : sol) {
    if (truth.size() < 2 || s.t < truth.front().t || s.t > truth.back().t) continue;
    while (j + 1 < truth.size() && truth[j + 1].t < s.t) ++j;
    const auto& a = truth[j];
    const auto& b = truth[std::min(j + 1, truth.size() - 1)];
    const double w = b.t > a.t ? (s.t - a.t) / (b.t - a.t) : 0.0;
    const api::Vector3 llh_t = a.llh + w * (b.llh - a.llh), vel_t = a.vel + w * (b.vel - a.vel);
    const api::Matrix3& C_t = w < 0.5 ? a.C : b.C;
    const api::Vector3 dpos(cobra::nav::delta_lat_to_north(s.llh(0) - llh_t(0), llh_t(0), llh_t(2)),
                            cobra::nav::delta_lon_to_east(s.llh(1) - llh_t(1), llh_t(0), llh_t(2)), -(s.llh(2) - llh_t(2)));
    const api::Vector3 dvel = s.vel - vel_t;
    const api::Vector3 drpy = cobra::nav::dcm_to_rpy(s.C * C_t.transpose()) * (180.0 / M_PI);
    for (int i = 0; i < 3; ++i) {
      sq_pos[i] += dpos(i) * dpos(i);
      sq_vel[i] += dvel(i) * dvel(i);
      sq_rpy[i] += drpy(i) * drpy(i);
    }
    ++n;
  }
  auto rms = [n](const double* sq) {
    std::ostringstream os;
    os << std::setprecision(6) << "[" << std::sqrt(sq[0] / std::max<std::size_t>(n, 1)) << ", " << std::sqrt(sq[1] / std::max<std::size_t>(n, 1))
       << ", " << std::sqrt(sq[2] / std::max<std::size_t>(n, 1)) << "]";
    return os.str();
  };
  std::cout << std::setprecision(10) << "{\"solution_epochs\": " << sol.size() << ", \"truth_epochs\": " << truth.size()
            << ", \"compared_epochs\": " << n << ", \"nan_epochs\": " << nan_count
            << ", \"solution_start\": " << (sol.empty() ? 0.0 : sol.front().t) << ", \"solution_end\": " << (sol.empty() ? 0.0 : sol.back().t)
            << ", \"truth_start\": " << (truth.empty() ? 0.0 : truth.front().t) << ", \"truth_end\": " << (truth.empty() ? 0.0 : truth.back().t)
            << ", \"rms_pos_ned_m\": " << rms(sq_pos) << ", \"rms_vel_ned_mps\": " << rms(sq_vel) << ", \"rms_rpy_deg\": " << rms(sq_rpy) << "}\n";
  return 0;
}
