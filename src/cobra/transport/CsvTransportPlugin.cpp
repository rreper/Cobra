#include <pntos/cobra/transport/CsvTransportPlugin.hpp>

#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/logging.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>
#include <aspn23/eigen/MeasurementPosition.hpp>
#include <aspn23/eigen/MeasurementVelocity.hpp>
#include <aspn23/eigen/TypeHeader.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <map>
#include <sstream>

namespace pntos::cobra {

using api::LoggingLevel;

namespace {
template <std::size_t N>
std::optional<std::array<double, N>> arr(ConfigReader& r, const std::string& key) {
  auto v = r.optional<api::Vector>(key);
  if (!v || v->size() != static_cast<Eigen::Index>(N)) return std::nullopt;
  std::array<double, N> out{};
  for (std::size_t i = 0; i < N; ++i) out[i] = (*v)(static_cast<Eigen::Index>(i));
  return out;
}
}  // namespace

void CsvTransportConfig::to_registry(api::Mediator& m) const {
  ConfigWriter w(m, group_);
  w.scalar("imu_file", imu_file);
  w.optional("position_file", position_file);
  w.optional("velocity_file", velocity_file);
  w.scalar("imu_channel", imu_channel);
  w.scalar("position_channel", position_channel);
  w.scalar("velocity_channel", velocity_channel);
  w.scalar("imu_integrated", imu_integrated);
  w.scalar("time_unit", time_unit);
  w.vector("default_position_sigma", to_vector(default_position_sigma));
  w.vector("default_velocity_sigma", to_vector(default_velocity_sigma));
  w.optional("output_file", output_file);
}

std::optional<CsvTransportConfig> CsvTransportConfig::from_registry(api::Mediator& m, const std::string& group) {
  ConfigReader r(m, group);
  if (!r.ok()) return std::nullopt;
  CsvTransportConfig c;
  c.group_ = group;
  c.imu_file = r.require<std::string>("imu_file");
  c.position_file = r.optional<std::string>("position_file");
  c.velocity_file = r.optional<std::string>("velocity_file");
  c.imu_channel = r.optional<std::string>("imu_channel").value_or(c.imu_channel);
  c.position_channel = r.optional<std::string>("position_channel").value_or(c.position_channel);
  c.velocity_channel = r.optional<std::string>("velocity_channel").value_or(c.velocity_channel);
  c.imu_integrated = r.optional<bool>("imu_integrated").value_or(false);
  c.time_unit = r.optional<std::string>("time_unit").value_or("s");
  c.default_position_sigma = arr<3>(r, "default_position_sigma").value_or(c.default_position_sigma);
  c.default_velocity_sigma = arr<3>(r, "default_velocity_sigma").value_or(c.default_velocity_sigma);
  c.output_file = r.optional<std::string>("output_file");
  if (!r.ok()) return std::nullopt;
  return c;
}

// ----------------------------------------------------------------------------- CSV reading

namespace {
struct CsvTable {
  std::map<std::string, std::size_t> columns;
  std::vector<std::vector<double>> rows;
  std::optional<std::size_t> col(const std::string& name) const {
    auto it = columns.find(name);
    if (it == columns.end()) return std::nullopt;
    return it->second;
  }
};

std::string trim(std::string s) {
  const auto a = s.find_first_not_of(" \t\r\n\""), b = s.find_last_not_of(" \t\r\n\"");
  return a == std::string::npos ? "" : s.substr(a, b - a + 1);
}

std::optional<CsvTable> read_csv(const std::string& path, std::string* error) {
  std::ifstream in(path);
  if (!in) {
    if (error) *error = "cannot open " + path;
    return std::nullopt;
  }
  CsvTable t;
  std::string line;
  while (std::getline(in, line) && (line.empty() || line[0] == '#')) {
  }
  std::istringstream hs(line);
  std::string cell;
  std::size_t i = 0;
  while (std::getline(hs, cell, ',')) t.columns[trim(cell)] = i++;
  const std::size_t ncol = i;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<double> row;
    row.reserve(ncol);
    std::istringstream ls(line);
    while (std::getline(ls, cell, ',')) {
      const std::string c = trim(cell);
      row.push_back(c.empty() ? std::nan("") : std::atof(c.c_str()));
    }
    row.resize(ncol, std::nan(""));
    t.rows.push_back(std::move(row));
  }
  return t;
}

double time_scale(const std::string& unit) {
  if (unit == "ms") return 1e6;
  if (unit == "ns") return 1.0;
  return 1e9;
}

using RowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
RowMajor diag3(double a, double b, double c) {
  RowMajor m = RowMajor::Zero(3, 3);
  m(0, 0) = a * a;
  m(1, 1) = b * b;
  m(2, 2) = c * c;
  return m;
}
}  // namespace

CsvTransportPlugin::CsvTransportPlugin(std::string identifier, std::string config_group)
    : identifier_(std::move(identifier)), config_group_(std::move(config_group)) {}

CsvTransportPlugin::~CsvTransportPlugin() { stop_listening(); }

void CsvTransportPlugin::init_plugin(const std::optional<std::string>&, api::Mediator* mediator) {
  mediator_ = mediator;
  if (!mediator_) {
    utils::print_message(LoggingLevel::ERROR, identifier_, "CsvTransportPlugin requires a mediator.");
    return;
  }
  cfg_ = CsvTransportConfig::from_registry(*mediator_, config_group_);
  if (!cfg_) {
    mediator_->log_message(LoggingLevel::ERROR, "Unable to read CsvTransportConfig from group \"" + config_group_ + "\".");
    return;
  }
  if (cfg_->output_file) {
    out_.open(*cfg_->output_file);
    if (!out_) {
      mediator_->log_message(LoggingLevel::ERROR, "Cannot open the solutions CSV " + *cfg_->output_file + ".");
    } else {
      out_ << "time_s,lat_deg,lon_deg,alt_m,vn_mps,ve_mps,vd_mps,roll_deg,pitch_deg,yaw_deg,sigma_n_m,sigma_e_m,sigma_d_m,channel\n";
      out_ << std::setprecision(12);
    }
  }
}

void CsvTransportPlugin::start_listening() {
  if (!cfg_ || !mediator_) return;
  stop_ = false;
  thread_ = std::thread([this] { read_all(); });
}

void CsvTransportPlugin::stop_listening() {
  stop_ = true;
  if (thread_.joinable()) thread_.join();
}

void CsvTransportPlugin::shutdown_plugin() {
  stop_listening();
  std::lock_guard lk(out_mutex_);
  if (out_.is_open()) out_.close();
  if (mediator_) mediator_->log_message(LoggingLevel::INFO, "Shutdown plugin for " + identifier_ + ".");
}

void CsvTransportPlugin::read_all() {
  if (!cfg_ || !mediator_) return;
  std::string err;
  auto imu = read_csv(cfg_->imu_file, &err);
  if (!imu) {
    mediator_->log_message(LoggingLevel::ERROR, "CSV transport: " + err);
    return;
  }
  std::optional<CsvTable> pos, vel;
  if (cfg_->position_file && !(pos = read_csv(*cfg_->position_file, &err))) {
    mediator_->log_message(LoggingLevel::ERROR, "CSV transport: " + err);
    return;
  }
  if (cfg_->velocity_file && !(vel = read_csv(*cfg_->velocity_file, &err))) {
    mediator_->log_message(LoggingLevel::ERROR, "CSV transport: " + err);
    return;
  }
  const double scale = time_scale(cfg_->time_unit);
  auto need = [&](const CsvTable& t, const char* name, const char* file) -> std::optional<std::size_t> {
    auto c = t.col(name);
    if (!c) mediator_->log_message(LoggingLevel::ERROR, std::string("CSV transport: column \"") + name + "\" missing in " + file);
    return c;
  };
  const auto it = need(*imu, "time", "the IMU file"), ax = need(*imu, "ax", "the IMU file"), ay = need(*imu, "ay", "the IMU file"),
             az = need(*imu, "az", "the IMU file"), gx = need(*imu, "gx", "the IMU file"), gy = need(*imu, "gy", "the IMU file"),
             gz = need(*imu, "gz", "the IMU file");
  if (!(it && ax && ay && az && gx && gy && gz)) return;
  std::optional<std::size_t> pt, plat, plon, palt, psn, pse, psd, vt, vn, ve, vd, vsn, vse, vsd;
  if (pos) {
    pt = need(*pos, "time", "the position file"), plat = need(*pos, "lat_deg", "the position file"),
    plon = need(*pos, "lon_deg", "the position file"), palt = need(*pos, "alt_m", "the position file");
    if (!(pt && plat && plon && palt)) return;
    psn = pos->col("sigma_n_m"), pse = pos->col("sigma_e_m"), psd = pos->col("sigma_d_m");
  }
  if (vel) {
    vt = need(*vel, "time", "the velocity file"), vn = need(*vel, "vn", "the velocity file"), ve = need(*vel, "ve", "the velocity file"),
    vd = need(*vel, "vd", "the velocity file");
    if (!(vt && vn && ve && vd)) return;
    vsn = vel->col("sigma_n"), vse = vel->col("sigma_e"), vsd = vel->col("sigma_d");
  }
  std::vector<std::pair<std::int64_t, std::pair<int, std::size_t>>> order;  // (time ns, (stream, row))
  for (std::size_t i = 0; i < imu->rows.size(); ++i) order.push_back({static_cast<std::int64_t>(std::llround(imu->rows[i][*it] * scale)), {0, i}});
  if (pos)
    for (std::size_t i = 0; i < pos->rows.size(); ++i) order.push_back({static_cast<std::int64_t>(std::llround(pos->rows[i][*pt] * scale)), {1, i}});
  if (vel)
    for (std::size_t i = 0; i < vel->rows.size(); ++i) order.push_back({static_cast<std::int64_t>(std::llround(vel->rows[i][*vt] * scale)), {2, i}});
  std::stable_sort(order.begin(), order.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  const double d2r = M_PI / 180.0;
  const auto& ps = cfg_->default_position_sigma;
  const auto& vs = cfg_->default_velocity_sigma;
  for (const auto& [t, which] : order) {
    if (stop_) return;
    const auto [stream, i] = which;
    std::shared_ptr<api::AspnBase> msg;
    std::string channel;
    if (stream == 0) {
      const auto& r = imu->rows[i];
      Eigen::Matrix<double, Eigen::Dynamic, 1> a(3), g(3);
      a << r[*ax], r[*ay], r[*az];
      g << r[*gx], r[*gy], r[*gz];
      msg = std::make_shared<aspn23_eigen::MeasurementImu>(
          aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0), aspn23_eigen::TypeTimestamp(t),
          cfg_->imu_integrated ? ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED : ASPN23_MEASUREMENT_IMU_IMU_TYPE_SAMPLED, a, g,
          std::vector<aspn23_eigen::TypeIntegrity>{});
      channel = cfg_->imu_channel;
    } else if (stream == 1) {
      const auto& r = pos->rows[i];
      auto sig = [&](const std::optional<std::size_t>& c, double dflt) { return c && std::isfinite(r[*c]) ? r[*c] : dflt; };
      msg = std::make_shared<aspn23_eigen::MeasurementPosition>(
          aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0), aspn23_eigen::TypeTimestamp(t),
          ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC, r[*plat] * d2r, r[*plon] * d2r, r[*palt],
          diag3(sig(psn, ps[0]), sig(pse, ps[1]), sig(psd, ps[2])), ASPN23_MEASUREMENT_POSITION_ERROR_MODEL_NONE,
          Eigen::Matrix<double, Eigen::Dynamic, 1>(0), std::vector<aspn23_eigen::TypeIntegrity>{});
      channel = cfg_->position_channel;
    } else {
      const auto& r = vel->rows[i];
      auto sig = [&](const std::optional<std::size_t>& c, double dflt) { return c && std::isfinite(r[*c]) ? r[*c] : dflt; };
      msg = std::make_shared<aspn23_eigen::MeasurementVelocity>(
          aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_VELOCITY, 0, 0, 0, 0), aspn23_eigen::TypeTimestamp(t),
          ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED, r[*vn], r[*ve], r[*vd], diag3(sig(vsn, vs[0]), sig(vse, vs[1]), sig(vsd, vs[2])),
          ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE, Eigen::Matrix<double, Eigen::Dynamic, 1>(0), std::vector<aspn23_eigen::TypeIntegrity>{});
      channel = cfg_->velocity_channel;
    }
    mediator_->process_pntos_message(api::Message(msg, channel));
    ++processed_;
  }
  mediator_->log_message(LoggingLevel::INFO, "Done processing CSV input (" + std::to_string(processed_.load()) + " messages).");
  mediator_->registry().batch("controller/flags")->set("ready_to_shutdown", true);
}

void CsvTransportPlugin::broadcast_message(const api::Message& message, const std::optional<std::string>& channel_name) {
  auto pva = message.as<utils::PVA>();
  if (!pva) return;
  std::lock_guard lk(out_mutex_);
  if (!out_.is_open()) return;
  const double r2d = 180.0 / M_PI;
  api::Vector3 rpy = api::Vector3::Zero();
  if (auto q = utils::quaternion(*pva)) rpy = nav::dcm_to_rpy(nav::quat_to_dcm(*q)) * r2d;
  const auto cov = pva->get_covariance();
  auto sig = [&](int i) { return cov.rows() > i && cov(i, i) > 0 ? std::sqrt(cov(i, i)) : 0.0; };
  const std::int64_t ns = pva->get_time_of_validity().get_elapsed_nsec();
  out_ << ns / 1000000000 << '.' << std::setw(9) << std::setfill('0') << std::llabs(ns % 1000000000) << std::setfill(' ') << ','
       << pva->get_p1() * r2d << ',' << pva->get_p2() * r2d << ',' << pva->get_p3() << ',' << pva->get_v1() << ',' << pva->get_v2() << ','
       << pva->get_v3() << ',' << rpy(0) << ',' << rpy(1) << ',' << rpy(2) << ',' << sig(0) << ',' << sig(1) << ',' << sig(2) << ','
       << channel_name.value_or(message.source_identifier) << '\n';
  ++written_;
}

}  // namespace pntos::cobra
