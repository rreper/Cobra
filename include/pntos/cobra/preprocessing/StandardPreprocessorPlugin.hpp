// Port of pntos.cobra.standard_plugins.preprocessor.* (six preprocessors + the plugin).
// Messages are immutable in the port: preprocessors that change a message return a modified copy
// instead of editing in place as the Python does.
#pragma once

#include <pntos/api/preprocessor.hpp>
#include <pntos/cobra/config/configs.hpp>

#include <map>

namespace pntos::cobra {

/// Keeps 1 of every N messages per configured channel (first message always passes).
class DownsamplerPreprocessor final : public api::Preprocessor {
 public:
  DownsamplerPreprocessor(const DownsamplerConfig& config, api::Mediator* mediator);
  std::optional<std::vector<api::Message>> process_pntos_message(const api::Message& message) override;

 private:
  std::map<std::string, std::int64_t> factors_;
  std::map<std::string, std::int64_t> counters_;
};

/// Rotates IMU measurements from the IMU frame into the platform frame.
class ImuRotationPreprocessor final : public api::Preprocessor {
 public:
  ImuRotationPreprocessor(api::Mediator* mediator, const api::Matrix3& C_imu_to_platform);
  std::optional<std::vector<api::Message>> process_pntos_message(const api::Message& message) override;

 private:
  api::Mediator* mediator_;
  api::Matrix3 C_;
};

/// Replaces timestamps that are not `expected_dt` after the previous one (within 0.1 ms) by
/// previous + expected_dt.
class TimeAdjusterPreprocessor final : public api::Preprocessor {
 public:
  TimeAdjusterPreprocessor(std::int64_t expected_dt_nsec, api::Mediator* mediator);
  std::optional<std::vector<api::Message>> process_pntos_message(const api::Message& message) override;

 private:
  api::Mediator* mediator_;
  std::optional<std::int64_t> last_nsec_;
  std::int64_t expected_dt_nsec_;
  std::int64_t tolerance_nsec_ = 100'000;
};

/// Barometric pressure -> MSL altitude (ISA, 288.15 K). Channel name `baro_pressure` -> `altitude`.
class BarometerToAltitudePreprocessor final : public api::Preprocessor {
 public:
  BarometerToAltitudePreprocessor(api::Mediator* mediator, std::optional<double> alt_sigma);
  std::optional<std::vector<api::Message>> process_pntos_message(const api::Message& message) override;
  static double pressure_to_alt(double pressure, double deg_k = 288.15, double ref_pressure = 101325.0,
                                double ref_alt = 0.0);

 private:
  api::Mediator* mediator_;
  std::optional<double> alt_sigma_;
  double deg_k_ = 288.15;
};

/// Subtracts a constant bias (ns) from every timestamp.
class TimeBiasPreprocessor final : public api::Preprocessor {
 public:
  TimeBiasPreprocessor(std::int64_t time_bias_nsec, api::Mediator* mediator);
  std::optional<std::vector<api::Message>> process_pntos_message(const api::Message& message) override;

 private:
  api::Mediator* mediator_;
  std::int64_t bias_;
};

/// Drops messages in [start, end) seconds relative to the first message seen.
class OutagePreprocessor final : public api::Preprocessor {
 public:
  OutagePreprocessor(double start_time, double end_time, api::Mediator* mediator);
  std::optional<std::vector<api::Message>> process_pntos_message(const api::Message& message) override;
  std::optional<std::int64_t> first_msg_time_ns() const { return first_ns_; }
  bool active() const { return active_; }

 private:
  api::Mediator* mediator_;
  double start_, end_;
  bool active_ = false;
  std::optional<std::int64_t> first_ns_;
};

class StandardPreprocessorPlugin final : public api::PreprocessorPlugin {
 public:
  explicit StandardPreprocessorPlugin(std::string identifier);
  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override;
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }
  /// Python order: downsampler, imu_rotator, time_adjuster, baro_converter, time_bias, outage.
  const std::vector<std::string>& preprocessor_identifiers() const override { return ids_; }
  std::unique_ptr<api::Preprocessor> new_preprocessor(std::size_t index,
                                                      const std::optional<std::string>& config_group) override;

 private:
  std::string identifier_;
  api::Mediator* mediator_ = nullptr;
  std::vector<std::string> ids_;
};

}  // namespace pntos::cobra
