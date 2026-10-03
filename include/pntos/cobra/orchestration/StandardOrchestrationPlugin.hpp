// Port of pntos.cobra.standard_plugins.StandardOrchestrationPlugin.
#pragma once

#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/orchestration/OrchestrationUtils.hpp>

#include <map>
#include <memory>

namespace pntos::cobra {

/// Standard closed-loop INS orchestration: alignment -> inertial mechanization -> Pinson15 error
/// state EKF with configurable extra blocks / processors / virtual blocks, inertial feedback, and
/// BEST / DEAD_RECKONING solutions.
class StandardOrchestrationPlugin final : public api::OrchestrationPlugin {
 public:
  explicit StandardOrchestrationPlugin(std::string identifier) : identifier_(std::move(identifier)) {}

  void init_plugin(const std::optional<std::string>& plugin_resources_location, api::Mediator* mediator) override;
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }

  void init_orchestration_plugin(const std::optional<api::PluginList>& plugins,
                                 api::MessageStreamConfig& stream_config) override;
  void process_pntos_message(const api::Message& message, bool sequenced) override;
  std::vector<std::string> filter_description_list() const override;
  std::optional<std::vector<std::optional<api::Message>>> request_solutions(
      const std::vector<api::Timestamp>& solution_times,
      const std::optional<std::string>& filter_description = std::nullopt) override;

  // --- test / diagnostic hooks
  api::StandardFusionEngine* fusion_engine() const { return fusion_engine_.get(); }
  api::StandardInertialMechanization* inertial() const { return inertial_.get(); }
  api::InertialInitializationStrategy* initializer() const { return initializer_.get(); }
  bool is_initialized() const { return init_solution_.has_value(); }
  std::int64_t inertial_drift_prop_dt() const { return inertial_drift_prop_dt_; }
  const std::map<std::string, std::vector<std::string>>& measurement_channels() const { return measurement_channels_; }
  const std::map<std::string, std::vector<std::string>>& vsbs_needing_pva() const { return vsbs_needing_pva_; }

 private:
  using Providers = std::vector<std::unique_ptr<api::StandardStateModelProvider>>;

  void log(api::LoggingLevel level, const std::string& message) const;
  void set_stream_config(const StreamConfig& registry_cfg, api::MessageStreamConfig& controller_cfg);
  void store_config_data(const StandardOrchestrationConfig& cfg);
  Providers make_providers();
  bool set_up_fusion_engine(const StandardOrchestrationConfig& cfg);
  bool add_state_block(const Providers& providers, const StateBlockConfig& cfg);
  bool add_measurement_processor(const Providers& providers, const MeasurementProcessorConfig& cfg);
  bool add_virtual_state_block(const Providers& providers, const VirtualStateBlockConfig& cfg);
  void add_preprocessor(const std::vector<std::shared_ptr<api::PreprocessorPlugin>>& plugins,
                        const PreprocessorConfig& cfg);
  void map_vsb_inertial_needs(const std::vector<std::shared_ptr<const MeasurementProcessorConfig>>& mp_configs);
  std::optional<api::EstimateWithCovariance> create_state_block_ewc(
      const std::string& identifier, std::size_t num_states, const std::optional<api::EstimateWithCovariance>& ewc);

  bool generate_initial_inertial_solution();
  bool initialize_filter();
  void try_initialize();

  void propagate_to_time(api::Timestamp target);
  void propagate_during_outage();
  void publish_solution(const std::optional<api::Message>& solution, const std::string& group,
                        const std::string& key);
  std::optional<api::Message> get_inertial_forces(std::optional<api::Timestamp> t1 = std::nullopt,
                                                  std::optional<api::Timestamp> t2 = std::nullopt);
  void send_inertial_aux_to_measurement_processor(const std::string& mp_label);
  void send_inertial_aux_to_pinson();
  void send_inertial_aux_to_vsbs(const std::string& mp_label);
  bool ready_to_apply_feedback();
  void apply_inertial_feedback();
  void perform_measurement_update(const api::Message& message, const std::string& target_mp);
  void send_message_as_aux_data(const api::Message& message);
  /// Runs the preprocessor chain. `effective_tov` (optional) receives the time the original message
  /// would carry after Python's in-place preprocessing (utils/effective_time.hpp).
  std::optional<std::vector<api::Message>> preprocess_message(const api::Message& message,
                                                              std::optional<api::Timestamp>* effective_tov = nullptr);

  std::string identifier_;
  api::Mediator* mediator_ = nullptr;

  // plugins (owned by the controller / app)
  std::shared_ptr<api::FusionPlugin> fusion_plugin_;
  std::shared_ptr<api::FusionStrategyPlugin> fusion_strategy_plugin_;
  std::shared_ptr<api::InertialPlugin> inertial_plugin_;
  std::shared_ptr<api::InitializationPlugin> initialization_plugin_;
  std::vector<std::shared_ptr<api::StateModelingPlugin>> state_modeling_plugins_;

  // config data
  PinsonStateBlockConfig pinson_sb_config_;
  std::string best_sol_channel_, imu_sol_channel_;
  bool publish_before_update_ = false, publish_after_update_ = false;
  std::vector<std::string> alignment_channels_;
  std::vector<std::string> inertial_channels_;
  std::string inertial_group_;
  std::int64_t max_prop_dt_ns_ = 2'000'000'000;
  std::int64_t buffer_time_ns_ = 2'000'000'000;
  std::optional<FeedbackConfig> feedback_config_;
  std::int64_t inertial_drift_prop_dt_ = 100'000'000;

  // runtime state
  std::vector<std::pair<std::unique_ptr<api::Preprocessor>, std::optional<std::vector<std::string>>>> preprocessors_;
  std::unique_ptr<api::InertialInitializationStrategy> initializer_;
  std::unique_ptr<api::StandardInertialMechanization> inertial_;
  std::unique_ptr<api::StandardFusionEngine> fusion_engine_;
  std::optional<api::InitialInertialSolution> init_solution_;
  std::optional<api::Matrix> init_pinson_cov_;
  std::unique_ptr<orch::SolutionCache> cache_;
  std::int64_t last_feedback_time_ns_ = 0;
  bool engine_ready_ = false;

  std::map<std::string, std::vector<std::string>> measurement_channels_;  ///< channel -> MP labels
  std::map<std::string, std::vector<std::string>> sb_aux_channels_, mp_aux_channels_, vsb_aux_channels_;
  std::map<std::string, bool> needs_inertial_pva_, needs_inertial_f_and_r_;
  std::map<std::string, std::vector<std::string>> vsbs_needing_pva_, vsbs_needing_f_and_r_;
  std::map<std::string, std::string> vsb_target_to_source_;
};

}  // namespace pntos::cobra
