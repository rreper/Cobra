// CSV transport (roadmap Phase 3): sensor data from plain text files, solutions to a CSV file.
//
// Input files have a header line and comma-separated columns (names are matched by header, order free):
//   IMU:      time, ax, ay, az, gx, gy, gz            [m/s^2 and rad/s, or delta-v / delta-angle if integrated]
//   position: time, lat_deg, lon_deg, alt_m, sigma_n_m, sigma_e_m, sigma_d_m   (sigma columns optional)
//   velocity: time, vn, ve, vd, sigma_n, sigma_e, sigma_d                       (m/s; sigma optional)
// `time` is in seconds (time_unit "s", default), "ms" or "ns". The streams are merged in time order and
// handed to the mediator on the channels of CsvTransportConfig. Published solutions are written as
//   time, lat_deg, lon_deg, alt_m, vn, ve, vd, roll_deg, pitch_deg, yaw_deg, sigma_n, sigma_e, sigma_d
// `tools/lcm_to_csv` exports these files from an LCM log.
#pragma once

#include <pntos/api/api.hpp>
#include <pntos/cobra/config/configs.hpp>

#include <atomic>
#include <fstream>
#include <mutex>
#include <thread>

namespace pntos::cobra {

/// Config for CsvTransportPlugin (group "config/csv_transport"; C++ addition).
struct CsvTransportConfig final : BaseConfig {
  static constexpr const char* kGroup = "config/csv_transport";
  std::string group_ = kGroup;
  std::string imu_file;
  std::optional<std::string> position_file;
  std::optional<std::string> velocity_file;
  std::string imu_channel = "/sensor/imu";
  std::string position_channel = "/sensor/position";
  std::string velocity_channel = "/sensor/velocity";
  bool imu_integrated = false;      ///< delta-v / delta-angle samples instead of rates
  std::string time_unit = "s";      ///< s | ms | ns
  Vec3 default_position_sigma{2.0, 2.0, 4.0};  ///< m, used when the file has no sigma columns
  Vec3 default_velocity_sigma{0.2, 0.2, 0.2};  ///< m/s
  std::optional<std::string> output_file;      ///< solutions CSV

  const std::string& group() const override { return group_; }
  void to_registry(api::Mediator& m) const override;
  static std::optional<CsvTransportConfig> from_registry(api::Mediator& m, const std::string& group = kGroup);
};

class CsvTransportPlugin final : public api::TransportPlugin {
 public:
  explicit CsvTransportPlugin(std::string identifier, std::string config_group = CsvTransportConfig::kGroup);
  ~CsvTransportPlugin() override;

  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override;
  void shutdown_plugin() override;
  const std::string& identifier() const override { return identifier_; }
  void start_listening() override;
  void stop_listening() override;
  void broadcast_message(const api::Message& message, const std::optional<std::string>& channel_name) override;

  /// Reads and delivers everything synchronously (what the reader thread runs). Public for tests.
  void read_all();
  std::size_t messages_processed() const { return processed_.load(); }
  std::size_t solutions_written() const { return written_.load(); }

 private:
  std::string identifier_, config_group_;
  api::Mediator* mediator_ = nullptr;
  std::optional<CsvTransportConfig> cfg_;
  std::ofstream out_;
  std::mutex out_mutex_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<std::size_t> processed_{0}, written_{0};
};

}  // namespace pntos::cobra
