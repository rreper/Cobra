// Port of pntos.cobra.StandardInertialPlugin (wrapping BufferedImu instead of navtk).
#pragma once

#include <pntos/api/inertial.hpp>
#include <pntos/cobra/inertial/BufferedImu.hpp>

namespace pntos::cobra {

class StandardInertial final : public api::StandardInertialMechanization {
 public:
  /// Reads InertialConfig from `config_group`; throws std::invalid_argument on a bad solution/config.
  StandardInertial(const std::string& config_group, api::Mediator* mediator, const api::Message& solution);

  api::AspnMessageType request_solution_message_type() const override {
    return ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE;
  }
  api::Message request_current_solution() override;
  std::optional<api::Message> request_solution(api::Timestamp time) override;
  std::optional<std::vector<std::optional<api::Message>>> request_solutions(
      const std::vector<api::Timestamp>& times, api::InertialSolutionRangeType type) override;
  bool is_time_in_range(api::Timestamp time) const override { return inertial_.in_range(time); }
  api::Timestamp request_earliest_time() const override { return inertial_.time_span().first; }
  api::Timestamp request_latest_time() const override { return inertial_.time_span().second; }
  std::vector<api::AspnMessageType> request_process_pntos_message_types() const override {
    return {ASPN_MEASUREMENT_IMU};
  }
  void process_pntos_message(const api::Message& message) override;
  std::optional<api::InertialForcesRates> request_forces_and_rates(api::Timestamp time) override;
  std::optional<api::InertialForcesRates> request_average_forces_and_rates(api::Timestamp t1,
                                                                           api::Timestamp t2) override;
  std::optional<std::vector<api::AspnMessageType>> request_reset_message_types() const override {
    return std::vector<api::AspnMessageType>{ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE};
  }
  void reset_solution(const api::Message& message) override;
  void correct_sensor_errors(api::Timestamp time, const api::StandardInertialErrors& errors) override;
  std::optional<api::StandardInertialErrors> request_sensor_errors(api::Timestamp time) override;

  static constexpr const char* kIdentifier = "Cobra standard inertial";
  inertial::BufferedImu& buffered() { return inertial_; }

 private:
  api::Mediator* mediator_;
  inertial::BufferedImu inertial_;
};

class StandardInertialPlugin final : public api::InertialPlugin {
 public:
  explicit StandardInertialPlugin(std::string identifier) : identifier_(std::move(identifier)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override;
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }
  bool is_inertial_type_supported(api::InertialType type) const override {
    return type == api::InertialType::STANDARD_MECHANIZATION;
  }
  std::unique_ptr<api::CommonInertial> new_inertial(api::InertialType type, const api::Message& solution,
                                                    const std::optional<std::string>& config_group) override;

 private:
  std::string identifier_;
  api::Mediator* mediator_ = nullptr;
};

}  // namespace pntos::cobra
