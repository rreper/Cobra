#include <pntos/cobra/tutorial/UiLogPlottingPlugin.hpp>

#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/transport/LcmLog.hpp>
#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/logging.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace pntos::cobra {

using api::LoggingLevel;

namespace {
struct PvaSample {
  double t = 0;
  api::Vector3 llh = api::Vector3::Zero();
  api::Vector3 vel = api::Vector3::Zero();
  api::Matrix3 C = api::Matrix3::Identity();  // platform -> NED
};
}  // namespace

void UiLogPlottingPlugin::log(LoggingLevel level, const std::string& message) const {
  if (mediator_)
    mediator_->log_message(level, message);
  else
    utils::print_message(level, identifier_, message);
}

void UiLogPlottingPlugin::init_plugin(const std::optional<std::string>&, api::Mediator* mediator) {
  mediator_ = mediator;
  if (!mediator_) return;
  config_ = UiLogPlottingConfig::from_registry(*mediator_);
  if (!config_)
    log(LoggingLevel::ERROR,
        std::string("Unable to retrieve config from registry. No config given or incorrect config group given. Expects config group ") +
            UiLogPlottingConfig::kGroup + ".");
}

std::optional<UiLogPlottingPlugin::Summary> UiLogPlottingPlugin::summarise() const {
  if (!config_) return std::nullopt;
  std::vector<PvaSample> sol, truth;
  try {
    lcm::LcmLogReader reader(config_->logfile);
    while (auto ev = reader.next()) {
      const bool is_sol = ev->channel == config_->solution_channel;
      const bool is_truth = ev->channel == config_->truth_channel;
      if (!is_sol && !is_truth) continue;
      auto msg = lcm::decode(ev->data);
      auto pva = std::dynamic_pointer_cast<const utils::PVA>(msg);
      if (!pva) continue;
      auto q = utils::quaternion(*pva);
      if (!q) continue;
      PvaSample s;
      s.t = pva->get_time_of_validity().get_elapsed_nsec() * 1e-9;
      s.llh = utils::position(*pva);
      s.vel = utils::velocity(*pva);
      s.C = nav::quat_to_dcm(*q);
      (is_sol ? sol : truth).push_back(s);
    }
  } catch (const std::exception& e) {
    log(LoggingLevel::WARN, std::string("Unable to read log file: ") + e.what());
    return std::nullopt;
  }
  if (sol.empty() || truth.size() < 2) {
    log(LoggingLevel::WARN, "No overlapping solution / truth PVA data found in " + config_->logfile + ".");
    return std::nullopt;
  }

  Summary out;
  const std::filesystem::path logpath(config_->logfile);
  const std::filesystem::path dir = logpath.parent_path() / logpath.stem();
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  out.csv_path = (dir / "pva_errors.csv").string();
  std::ofstream csv(out.csv_path);
  csv << "time_s,north_err_m,east_err_m,down_err_m,vn_err_mps,ve_err_mps,vd_err_mps,roll_err_deg,pitch_err_deg,yaw_err_deg\n";
  csv << std::setprecision(10);

  double sq_pos[3] = {0, 0, 0}, sq_vel[3] = {0, 0, 0}, sq_rpy[3] = {0, 0, 0};
  std::size_t j = 0;
  for (const auto& s : sol) {
    if (s.t < truth.front().t || s.t > truth.back().t) continue;
    while (j + 1 < truth.size() && truth[j + 1].t < s.t) ++j;
    const PvaSample& a = truth[j];
    const PvaSample& b = truth[std::min(j + 1, truth.size() - 1)];
    const double w = (b.t > a.t) ? (s.t - a.t) / (b.t - a.t) : 0.0;
    const api::Vector3 llh_t = a.llh + w * (b.llh - a.llh);
    const api::Vector3 vel_t = a.vel + w * (b.vel - a.vel);
    const api::Matrix3& C_t = (w < 0.5) ? a.C : b.C;
    const api::Vector3 dpos(nav::delta_lat_to_north(s.llh(0) - llh_t(0), llh_t(0), llh_t(2)),
                            nav::delta_lon_to_east(s.llh(1) - llh_t(1), llh_t(0), llh_t(2)), -(s.llh(2) - llh_t(2)));
    const api::Vector3 dvel = s.vel - vel_t;
    const api::Vector3 drpy = nav::dcm_to_rpy(s.C * C_t.transpose()) * (180.0 / M_PI);
    for (int i = 0; i < 3; ++i) {
      sq_pos[i] += dpos(i) * dpos(i);
      sq_vel[i] += dvel(i) * dvel(i);
      sq_rpy[i] += drpy(i) * drpy(i);
    }
    ++out.epochs;
    csv << s.t << ',' << dpos(0) << ',' << dpos(1) << ',' << dpos(2) << ',' << dvel(0) << ',' << dvel(1) << ',' << dvel(2)
        << ',' << drpy(0) << ',' << drpy(1) << ',' << drpy(2) << '\n';
  }
  if (out.epochs == 0) {
    log(LoggingLevel::WARN, "Solution and truth PVA data do not overlap in time.");
    return std::nullopt;
  }
  for (int i = 0; i < 3; ++i) {
    out.rms_pos[i] = std::sqrt(sq_pos[i] / static_cast<double>(out.epochs));
    out.rms_vel[i] = std::sqrt(sq_vel[i] / static_cast<double>(out.epochs));
    out.rms_rpy[i] = std::sqrt(sq_rpy[i] / static_cast<double>(out.epochs));
  }
  return out;
}

void UiLogPlottingPlugin::shutdown_plugin() {
  if (!config_) return;
  std::ifstream f(config_->logfile, std::ios::binary);
  if (!f) {
    log(LoggingLevel::WARN, "[" + config_->logfile + "] file does not exist. No results will be plotted.");
    return;
  }
  unsigned char magic[4] = {0, 0, 0, 0};
  f.read(reinterpret_cast<char*>(magic), 4);
  if (!(magic[0] == 0xED && magic[1] == 0xA1 && magic[2] == 0xDA && magic[3] == 0x01)) {
    log(LoggingLevel::WARN, "Invalid LCM log file. No results will be plotted.");
    return;
  }
  f.close();
  log(LoggingLevel::INFO, "Summarising results against truth. This may take a while...");
  auto s = summarise();
  if (!s) return;
  std::ostringstream os;
  os << std::fixed << std::setprecision(3) << "Cobra Solution vs Truth over " << s->epochs << " epochs: RMS position N/E/D "
     << s->rms_pos[0] << " / " << s->rms_pos[1] << " / " << s->rms_pos[2] << " m, velocity " << s->rms_vel[0] << " / "
     << s->rms_vel[1] << " / " << s->rms_vel[2] << " m/s, roll/pitch/yaw " << s->rms_rpy[0] << " / " << s->rms_rpy[1]
     << " / " << s->rms_rpy[2] << " deg";
  log(LoggingLevel::INFO, os.str());
  log(LoggingLevel::INFO, "Per-epoch errors saved to " + s->csv_path + " (the Python plugin saves its plots to this directory).");
}

}  // namespace pntos::cobra
