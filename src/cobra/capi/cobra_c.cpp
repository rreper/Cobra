#include <pntos/cobra/capi/cobra.h>

#include <pntos/cobra/app/Filter.hpp>
#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>
#include <aspn23/eigen/MeasurementPosition.hpp>
#include <aspn23/eigen/MeasurementVelocity.hpp>
#include <aspn23/eigen/TypeHeader.hpp>

#include <cstring>
#include <memory>
#include <string>

struct cobra_filter {
  std::unique_ptr<pntos::cobra::Filter> filter;
};

namespace {
thread_local std::string g_error;

void set_error(const std::string& e) { g_error = e; }

using RowMajor = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
RowMajor cov3(const double* c) {
  RowMajor m(3, 3);
  for (int i = 0; i < 9; ++i) m(i / 3, i % 3) = c ? c[i] : (i % 4 == 0 ? 1.0 : 0.0);
  return m;
}

pntos::cobra::app::RunOptions options_from(const char* overrides_json) {
  pntos::cobra::app::RunOptions o;
  o.progress = false;
  if (!overrides_json || !*overrides_json) return o;
  auto j = nlohmann::json::parse(overrides_json);
  if (j.contains("legacy_q_rotation")) o.legacy_q_rotation = j.at("legacy_q_rotation").get<bool>();
  if (j.contains("joseph_form")) o.joseph_form = j.at("joseph_form").get<bool>();
  if (j.contains("output_log")) o.output_log = j.at("output_log").get<std::string>();
  if (j.contains("input_log")) o.input_log = j.at("input_log").get<std::string>();
  return o;
}

cobra_filter* create(pntos::cobra::AppConfig config, const char* overrides_json) {
  try {
    if (overrides_json && *overrides_json) {
      auto j = nlohmann::json::parse(overrides_json);
      if (j.contains("logging_level")) config.app.logging_level = j.at("logging_level").get<std::string>();
    }
    auto* f = new cobra_filter;
    f->filter = std::make_unique<pntos::cobra::Filter>(std::move(config), options_from(overrides_json));
    g_error.clear();
    return f;
  } catch (const std::exception& e) {
    set_error(e.what());
    return nullptr;
  }
}

bool fill(const pntos::api::Message& m, cobra_pva* out) {
  auto pva = m.as<pntos::cobra::utils::PVA>();
  if (!pva || !out) return false;
  std::memset(out, 0, sizeof(*out));
  out->tov_ns = pva->get_time_of_validity().get_elapsed_nsec();
  out->lat_rad = pva->get_p1();
  out->lon_rad = pva->get_p2();
  out->alt_m = pva->get_p3();
  out->vel_ned_mps[0] = pva->get_v1();
  out->vel_ned_mps[1] = pva->get_v2();
  out->vel_ned_mps[2] = pva->get_v3();
  const auto q = pva->get_quaternion();
  for (int i = 0; i < 4 && i < q.size(); ++i) out->quat_wxyz[i] = q(i);
  const auto c = pva->get_covariance();
  for (int r = 0; r < 9 && r < c.rows(); ++r)
    for (int k = 0; k < 9 && k < c.cols(); ++k) out->cov[r * 9 + k] = c(r, k);
  std::strncpy(out->channel, m.source_identifier.c_str(), sizeof(out->channel) - 1);
  return true;
}

template <class F>
int guarded(cobra_filter* f, F&& fn) {
  if (!f || !f->filter) {
    set_error("null filter");
    return 0;
  }
  try {
    return fn(*f->filter) ? 1 : 0;
  } catch (const std::exception& e) {
    set_error(e.what());
    return 0;
  }
}
}  // namespace

extern "C" {

const char* cobra_version(void) { return "0.2.0"; }
const char* cobra_last_error(void) { return g_error.c_str(); }

cobra_filter* cobra_filter_create(const char* config_json_path, const char* overrides_json) {
  try {
    return create(pntos::cobra::jsoncfg::load_app_config(config_json_path ? config_json_path : ""), overrides_json);
  } catch (const std::exception& e) {
    set_error(e.what());
    return nullptr;
  }
}

cobra_filter* cobra_filter_create_from_json(const char* config_json, const char* overrides_json) {
  try {
    return create(pntos::cobra::jsoncfg::app_config_from_json(nlohmann::json::parse(config_json ? config_json : "")), overrides_json);
  } catch (const std::exception& e) {
    set_error(e.what());
    return nullptr;
  }
}

void cobra_filter_destroy(cobra_filter* f) {
  if (!f) return;
  try {
    if (f->filter) f->filter->stop();
  } catch (...) {
  }
  delete f;
}

int cobra_filter_stop(cobra_filter* f) {
  if (!f || !f->filter) return 2;
  try {
    return f->filter->stop();
  } catch (const std::exception& e) {
    set_error(e.what());
    return 2;
  }
}

int cobra_filter_push_imu(cobra_filter* f, const char* channel, int64_t tov_ns, const double accel[3], const double gyro[3], int integrated) {
  return guarded(f, [&](pntos::cobra::Filter& filt) {
    if (!channel || !accel || !gyro) throw std::invalid_argument("push_imu: null argument");
    Eigen::Matrix<double, Eigen::Dynamic, 1> a(3), g(3);
    for (int i = 0; i < 3; ++i) {
      a(i) = accel[i];
      g(i) = gyro[i];
    }
    auto msg = std::make_shared<aspn23_eigen::MeasurementImu>(
        aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_IMU, 0, 0, 0, 0), aspn23_eigen::TypeTimestamp(tov_ns),
        integrated ? ASPN23_MEASUREMENT_IMU_IMU_TYPE_INTEGRATED : ASPN23_MEASUREMENT_IMU_IMU_TYPE_SAMPLED, a, g,
        std::vector<aspn23_eigen::TypeIntegrity>{});
    filt.push(msg, channel);
    return true;
  });
}

int cobra_filter_push_position(cobra_filter* f, const char* channel, int64_t tov_ns, double lat_rad, double lon_rad, double alt_m,
                               const double cov[9]) {
  return guarded(f, [&](pntos::cobra::Filter& filt) {
    if (!channel) throw std::invalid_argument("push_position: null channel");
    auto msg = std::make_shared<aspn23_eigen::MeasurementPosition>(
        aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_POSITION, 0, 0, 0, 0), aspn23_eigen::TypeTimestamp(tov_ns),
        ASPN23_MEASUREMENT_POSITION_REFERENCE_FRAME_GEODETIC, lat_rad, lon_rad, alt_m, cov3(cov),
        ASPN23_MEASUREMENT_POSITION_ERROR_MODEL_NONE, Eigen::Matrix<double, Eigen::Dynamic, 1>(0), std::vector<aspn23_eigen::TypeIntegrity>{});
    filt.push(msg, channel);
    return true;
  });
}

int cobra_filter_push_velocity_ned(cobra_filter* f, const char* channel, int64_t tov_ns, const double vel_ned[3], const double cov[9]) {
  return guarded(f, [&](pntos::cobra::Filter& filt) {
    if (!channel || !vel_ned) throw std::invalid_argument("push_velocity_ned: null argument");
    auto msg = std::make_shared<aspn23_eigen::MeasurementVelocity>(
        aspn23_eigen::TypeHeader(ASPN_MEASUREMENT_VELOCITY, 0, 0, 0, 0), aspn23_eigen::TypeTimestamp(tov_ns),
        ASPN23_MEASUREMENT_VELOCITY_REFERENCE_FRAME_NED, vel_ned[0], vel_ned[1], vel_ned[2], cov3(cov),
        ASPN23_MEASUREMENT_VELOCITY_ERROR_MODEL_NONE, Eigen::Matrix<double, Eigen::Dynamic, 1>(0), std::vector<aspn23_eigen::TypeIntegrity>{});
    filt.push(msg, channel);
    return true;
  });
}

int cobra_filter_push_lcm(cobra_filter* f, const char* channel, const uint8_t* data, size_t len) {
  return guarded(f, [&](pntos::cobra::Filter& filt) {
    if (!channel || !data) throw std::invalid_argument("push_lcm: null argument");
    auto msg = pntos::cobra::lcm::decode(data, len);
    if (!msg) throw std::runtime_error("push_lcm: cannot decode the message on " + std::string(channel));
    filt.push(msg, channel);
    return true;
  });
}

int cobra_filter_poll_solution(cobra_filter* f, cobra_pva* out) {
  return guarded(f, [&](pntos::cobra::Filter& filt) {
    // take_solutions() drains the queue: keep the rest for the next poll
    static thread_local std::vector<pntos::api::Message> pending;
    if (pending.empty()) pending = filt.take_solutions();
    if (pending.empty()) return false;
    const bool ok = fill(pending.front(), out);
    pending.erase(pending.begin());
    return ok;
  });
}

int cobra_filter_solution_at(cobra_filter* f, int64_t tov_ns, cobra_pva* out) {
  return guarded(f, [&](pntos::cobra::Filter& filt) {
    auto s = filt.solution(pntos::api::Timestamp{tov_ns});
    return s && fill(*s, out);
  });
}

int cobra_filter_error_logged(cobra_filter* f) {
  return guarded(f, [&](pntos::cobra::Filter& filt) { return filt.error_logged(); });
}

}  // extern "C"
