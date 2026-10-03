// Port of pntos.cobra.tutorial_plugins.TutorialPosOrchestrationPlugin / TutorialPosVelOrchestrationPlugin.
//
// The tutorial orchestration is the minimal closed-loop INS: a manual initial solution, an inertial
// fed with time-adjusted / rotated IMU messages, one Pinson15 block plus a 3-state FOGM position
// sensor error block, and a measurement update + full inertial feedback on every position (and,
// for the pos/vel variant, velocity) message. There is no alignment phase, no propagation chunking,
// no solution cache and no outage propagation — see StandardOrchestrationPlugin for those.
#pragma once

#include <pntos/api/api.hpp>
#include <pntos/cobra/orchestration/OrchestrationUtils.hpp>

#include <map>
#include <memory>

namespace pntos::cobra {

class TutorialOrchestrationPlugin : public api::OrchestrationPlugin {
 public:
  TutorialOrchestrationPlugin(std::string identifier, bool with_velocity)
      : identifier_(std::move(identifier)), with_velocity_(with_velocity) {}

  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override { mediator_ = mediator; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }

  void init_orchestration_plugin(const std::optional<api::PluginList>& plugins,
                                 api::MessageStreamConfig& stream_config) override;
  void process_pntos_message(const api::Message& message, bool sequenced) override;
  std::vector<std::string> filter_description_list() const override {
    return {"POS_INS_BEST_ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ESTIMATE"};
  }
  std::optional<std::vector<std::optional<api::Message>>> request_solutions(
      const std::vector<api::Timestamp>& solution_times,
      const std::optional<std::string>& filter_description = std::nullopt) override;

  // test hooks
  api::StandardFusionEngine* fusion_engine() const { return fusion_engine_.get(); }
  api::StandardInertialMechanization* inertial() const { return inertial_.get(); }
  bool ready() const { return ready_; }

 private:
  void log(api::LoggingLevel level, const std::string& message) const;
  bool set_up_fusion_engine(api::FusionPlugin& fusion, api::FusionStrategyPlugin& strategy,
                            api::StateModelingPlugin& state_modeling);
  bool initialize_inertial_and_fusion_engine(api::InitializationPlugin& init, api::InertialPlugin& inertial);
  std::optional<api::Message> get_best_solution(api::Timestamp time);
  void send_inertial_aux_to_pinson();
  void send_inertial_aux_to_measurement_processor(api::Timestamp time, const std::string& label);
  void apply_feedback(api::Timestamp time);

  std::string identifier_;
  bool with_velocity_;
  api::Mediator* mediator_ = nullptr;
  bool ready_ = false;

  std::unique_ptr<api::StandardFusionEngine> fusion_engine_;
  std::unique_ptr<api::StandardInertialMechanization> inertial_;
  std::unique_ptr<api::Preprocessor> time_adjust_, time_bias_, imu_rotator_;
  std::map<std::string, std::string> measurement_channels_;  ///< channel -> processor label
  std::string inertial_channel_;
};

class TutorialPosOrchestrationPlugin final : public TutorialOrchestrationPlugin {
 public:
  explicit TutorialPosOrchestrationPlugin(std::string identifier)
      : TutorialOrchestrationPlugin(std::move(identifier), false) {}
};

class TutorialPosVelOrchestrationPlugin final : public TutorialOrchestrationPlugin {
 public:
  explicit TutorialPosVelOrchestrationPlugin(std::string identifier)
      : TutorialOrchestrationPlugin(std::move(identifier), true) {}
};

}  // namespace pntos::cobra
