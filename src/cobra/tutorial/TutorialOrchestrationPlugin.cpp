#include <pntos/cobra/tutorial/TutorialOrchestrationPlugin.hpp>

#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/utils/effective_time.hpp>
#include <pntos/cobra/utils/logging.hpp>
#include <pntos/cobra/utils/plugins.hpp>

#include <aspn23/eigen/MeasurementImu.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace pntos::cobra {

using api::EstimateWithCovariance;
using api::EstimateWithCovarianceType;
using api::LoggingLevel;
using api::Message;
using api::Timestamp;

namespace {
std::string secs(Timestamp t) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(9) << t.seconds();
  return os.str();
}
std::optional<std::size_t> index_of(const std::vector<std::string>& ids, const std::string& id) {
  auto it = std::find(ids.begin(), ids.end(), id);
  if (it == ids.end()) return std::nullopt;
  return static_cast<std::size_t>(it - ids.begin());
}
}  // namespace

void TutorialOrchestrationPlugin::log(LoggingLevel level, const std::string& message) const {
  if (mediator_)
    mediator_->log_message(level, message);
  else
    utils::print_message(level, "TutorialOrchestrationPlugin", message);
}

void TutorialOrchestrationPlugin::init_orchestration_plugin(const std::optional<api::PluginList>& plugins,
                                                            api::MessageStreamConfig& stream_config) {
  stream_config.sequenced_stream_all(true);
  stream_config.immediate_stream_add(ASPN_MEASUREMENT_IMU, std::nullopt);
  if (!plugins || !mediator_) {
    log(LoggingLevel::ERROR, "Tutorial orchestration needs a mediator and a plugin list. Filter cannot be implemented.");
    return;
  }
  // The Python tutorial indexes the plugin list by position; the C++ controller hands the same plugins
  // over, so pick them by kind to be independent of the order.
  SortedPlugins sorted = sort_plugins(*plugins);
  auto logfn = [this](LoggingLevel l, const std::string& m) { log(l, m); };
  if (!validate_plugins(sorted, logfn,
                        {{"fusion_plugins", {1, 1}},
                         {"fusion_strategy_plugins", {1, 1}},
                         {"inertial_plugins", {1, 1}},
                         {"initialization_plugins", {1, 1}},
                         {"state_modeling_plugins", {1, 1}},
                         {"preprocessor_plugins", {1, 1}}}))
    return;

  if (!set_up_fusion_engine(*sorted.fusion_plugins[0], *sorted.fusion_strategy_plugins[0],
                            *sorted.state_modeling_plugins[0]))
    return;
  if (!initialize_inertial_and_fusion_engine(*sorted.initialization_plugins[0], *sorted.inertial_plugins[0])) return;

  auto orch_config = TutorialOrchestrationConfig::from_registry(*mediator_);
  if (!orch_config) {
    log(LoggingLevel::ERROR, "Unable to read TutorialOrchestrationConfig from the registry.");
    return;
  }
  measurement_channels_[orch_config->position_channel] = "pos";
  if (with_velocity_) measurement_channels_[orch_config->velocity_channel] = "vel";

  auto inertial_config = InertialConfig::from_registry(*mediator_, "config/inertial");
  if (!inertial_config || inertial_config->channels.empty()) {
    log(LoggingLevel::ERROR, "Unable to read InertialConfig (group config/inertial) from the registry.");
    return;
  }
  inertial_channel_ = inertial_config->channels[0];

  auto& pp = *sorted.preprocessor_plugins[0];
  auto make = [&](const char* id, const char* group) -> std::unique_ptr<api::Preprocessor> {
    auto idx = index_of(pp.preprocessor_identifiers(), id);
    if (!idx) {
      log(LoggingLevel::ERROR, std::string("Preprocessor plugin does not provide \"") + id + "\".");
      return nullptr;
    }
    return pp.new_preprocessor(*idx, std::string(group));
  };
  time_adjust_ = make("time_adjuster", "config/time_adjuster");
  time_bias_ = make("time_bias", "config/time_bias");
  imu_rotator_ = make("imu_rotator", "config/imu_rotator");
  if (!time_adjust_ || !time_bias_ || !imu_rotator_) return;
  ready_ = true;
}

bool TutorialOrchestrationPlugin::set_up_fusion_engine(api::FusionPlugin& fusion, api::FusionStrategyPlugin& strategy,
                                                       api::StateModelingPlugin& state_modeling) {
  auto engine = fusion.new_fusion_engine(api::FusionType::STANDARD);
  if (!engine) {
    log(LoggingLevel::ERROR, "Unable to make new fusion engine - cannot continue.");
    return false;
  }
  auto strat = strategy.new_fusion_strategy(api::FusionType::STANDARD);
  if (!strat) {
    log(LoggingLevel::ERROR, "Unable to make new fusion strategy - cannot continue.");
    return false;
  }
  engine->set_strategy(std::move(strat));
  auto provider = state_modeling.new_state_model_provider(api::FusionType::STANDARD);
  if (!provider) {
    log(LoggingLevel::ERROR, "Unable to make a state model provider - cannot continue.");
    return false;
  }

  auto pinson_idx = index_of(provider->block_identifiers(), "pinson15");
  auto fogm_idx = index_of(provider->block_identifiers(), "fogm");
  auto pos_idx = index_of(provider->processor_identifiers(), "pinson_with_ned_fogm_position");
  auto vel_idx = index_of(provider->processor_identifiers(), "pinson_velocity");
  if (!pinson_idx || !fogm_idx || !pos_idx || (with_velocity_ && !vel_idx)) {
    log(LoggingLevel::ERROR, "State model provider does not offer the tutorial blocks/processors.");
    return false;
  }

  auto pinson = provider->new_block(*pinson_idx, engine.get(), "pinson15", std::string("config/inertial_state"));
  if (!pinson) return false;
  api::Vector diag(15);
  diag << 0.1, 0.1, 0.1, 1e-3, 1e-3, 1e-3, 5e-4, 5e-4, 5e-4, 5.2e-3, 5.2e-3, 5.2e-3, 9e-6, 9e-6, 9e-6;
  engine->add_state_block(std::move(pinson),
                          EstimateWithCovariance{EstimateWithCovarianceType::EWC_GENERIC, api::Vector::Zero(15),
                                                 api::Matrix(diag.asDiagonal())},
                          std::nullopt);

  auto fogm = provider->new_block(*fogm_idx, engine.get(), "pos_fogm", std::string("config/pos_sensor_error"));
  if (!fogm) return false;
  engine->add_state_block(std::move(fogm),
                          EstimateWithCovariance{EstimateWithCovarianceType::EWC_GENERIC, api::Vector::Zero(3),
                                                 api::Matrix::Identity(3, 3) * 9.0},
                          std::nullopt);

  auto pos = provider->new_processor(*pos_idx, engine.get(), "pos", {"pinson15", "pos_fogm"},
                                     std::string("config/gp3d_state_modeling"));
  if (!pos) return false;
  engine->add_measurement_processor(std::move(pos));
  if (with_velocity_) {
    auto vel = provider->new_processor(*vel_idx, engine.get(), "vel", {"pinson15"}, std::string("config/gp3d_state_modeling"));
    if (!vel) return false;
    engine->add_measurement_processor(std::move(vel));
  }
  fusion_engine_ = std::move(engine);
  return true;
}

bool TutorialOrchestrationPlugin::initialize_inertial_and_fusion_engine(api::InitializationPlugin& init,
                                                                        api::InertialPlugin& inertial_plugin) {
  auto logfn = [this](LoggingLevel l, const std::string& m) { log(l, m); };
  auto initializer = orch::set_up_initializer(init, "config/default/alignment", logfn);
  if (!initializer) return false;
  if (!orch::initialization_ready(*initializer)) {
    log(LoggingLevel::ERROR, "Initializer not ready.");
    return false;
  }
  auto setup = orch::set_up_inertial_mechanization(*initializer, inertial_plugin, "config/inertial", logfn);
  if (!setup) return false;
  inertial_ = std::move(setup->inertial);
  auto pva = setup->init_solution.solution->as<utils::PVA>();
  const Timestamp init_time = utils::time_of_validity(*pva).value_or(Timestamp{0});
  fusion_engine_->set_time(init_time);
  send_inertial_aux_to_pinson();
  log(LoggingLevel::INFO, "Aligned filter at " + secs(init_time) + "s");
  return true;
}

std::optional<Message> TutorialOrchestrationPlugin::get_best_solution(Timestamp time) {
  return orch::get_best_solution(*fusion_engine_, *inertial_, time, "pinson15", "/solution/pntos/pva",
                                 [this](LoggingLevel l, const std::string& m) { log(l, m); });
}

void TutorialOrchestrationPlugin::send_inertial_aux_to_pinson() {
  const Timestamp time = fusion_engine_->time();
  api::AuxData aux;
  aux.push_back(inertial_->request_solution(time));
  auto forces = inertial_->request_forces_and_rates(time);
  if (forces && forces->forces_and_rates)
    aux.push_back(Message(forces->forces_and_rates, "Orchestration forces and rates"));
  else
    aux.push_back(std::nullopt);  // before the first IMU message there are no forces yet
  fusion_engine_->give_state_block_aux_data("pinson15", aux);
}

void TutorialOrchestrationPlugin::send_inertial_aux_to_measurement_processor(Timestamp time, const std::string& label) {
  fusion_engine_->give_measurement_processor_aux_data(label, {inertial_->request_solution(time)});
}

void TutorialOrchestrationPlugin::apply_feedback(Timestamp time) {
  auto corrected = get_best_solution(time);
  if (!corrected) return;
  inertial_->reset_solution(*corrected);
  auto estimate = fusion_engine_->get_state_block_estimate("pinson15");
  auto errors = inertial_->request_sensor_errors(time);
  if (!estimate || !errors) {
    log(LoggingLevel::ERROR, "Unable to apply inertial feedback at time " + secs(time) + "s.");
    return;
  }
  errors->accel_biases -= estimate->segment<3>(9);
  errors->gyro_biases -= estimate->segment<3>(12);
  inertial_->correct_sensor_errors(time, *errors);
  fusion_engine_->set_state_block_estimate("pinson15", api::Vector::Zero(15));
}

void TutorialOrchestrationPlugin::process_pntos_message(const Message& message, bool sequenced) {
  if (!ready_) return;
  if (message.source_identifier == inertial_channel_) {
    auto adjusted = time_adjust_->process_pntos_message(message);
    if (!adjusted || adjusted->empty()) return;
    // Python's time adjuster replaces the timestamp on the message object the mediator still holds.
    if (!sequenced && adjusted->front().wrapped_message)
      report_effective_time(mediator_, utils::time_of_validity(*adjusted->front().wrapped_message));
    auto rotated = imu_rotator_->process_pntos_message(adjusted->front());
    if (!rotated || rotated->empty()) return;
    inertial_->process_pntos_message(rotated->front());
    return;
  }
  auto it = measurement_channels_.find(message.source_identifier);
  if (it == measurement_channels_.end()) return;
  auto biased = time_bias_->process_pntos_message(message);
  if (!biased || biased->empty()) return;
  const Message& msg = biased->front();
  auto tov = utils::time_of_validity(*msg.wrapped_message);
  if (!tov) return;
  const std::string& label = it->second;
  send_inertial_aux_to_measurement_processor(*tov, label);
  send_inertial_aux_to_pinson();
  fusion_engine_->propagate(*tov);
  fusion_engine_->update(label, msg);
  apply_feedback(*tov);
}

std::optional<std::vector<std::optional<Message>>> TutorialOrchestrationPlugin::request_solutions(
    const std::vector<Timestamp>&, const std::optional<std::string>&) {
  if (!ready_) return std::nullopt;
  auto sol = get_best_solution(inertial_->request_latest_time());
  if (!sol) return std::nullopt;
  return std::vector<std::optional<Message>>{sol};
}

}  // namespace pntos::cobra
