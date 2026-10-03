#include <pntos/cobra/orchestration/StandardOrchestrationPlugin.hpp>
#include <pntos/cobra/utils/arrays.hpp>
#include <pntos/cobra/utils/logging.hpp>
#include <pntos/cobra/utils/plugins.hpp>

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
template <class T>
bool contains(const std::vector<T>& v, const T& x) {
  return std::find(v.begin(), v.end(), x) != v.end();
}
std::optional<std::size_t> index_of(const std::vector<std::string>& ids, const std::string& id) {
  auto it = std::find(ids.begin(), ids.end(), id);
  if (it == ids.end()) return std::nullopt;
  return static_cast<std::size_t>(it - ids.begin());
}
}  // namespace

void StandardOrchestrationPlugin::log(LoggingLevel level, const std::string& message) const {
  if (mediator_)
    mediator_->log_message(level, message);
  else
    utils::print_message(level, "OrchestrationPlugin", message);
}

void StandardOrchestrationPlugin::init_plugin(const std::optional<std::string>&, api::Mediator* mediator) {
  if (!mediator) {
    log(LoggingLevel::ERROR, "Orchestration was not handed a mediator. Orchestration will be disabled.");
    return;
  }
  mediator_ = mediator;
}

void StandardOrchestrationPlugin::set_stream_config(const StreamConfig& reg, api::MessageStreamConfig& ctrl) {
  const bool sequenced_default = reg.default_buffer_mode == BufferMode::SEQUENCED;
  if (sequenced_default)
    ctrl.sequenced_stream_all(true);
  else
    ctrl.immediate_stream_all(true);
  if (!reg.override_streams) return;
  for (const auto& s : *reg.override_streams) {
    if (sequenced_default)
      ctrl.immediate_stream_add(s.message_type, s.source_identifier);
    else
      ctrl.sequenced_stream_add(s.message_type, s.source_identifier);
  }
}

void StandardOrchestrationPlugin::store_config_data(const StandardOrchestrationConfig& cfg) {
  pinson_sb_config_ = cfg.pinson_sb_config;
  best_sol_channel_ = cfg.best_sol_channel;
  imu_sol_channel_ = cfg.imu_sol_channel;
  publish_before_update_ = cfg.publish_before_update;
  publish_after_update_ = cfg.publish_after_update;
  alignment_channels_ = cfg.alignment_channels;
  inertial_channels_ = cfg.inertial_config.channels;
  inertial_group_ = cfg.inertial_config.group_;
  max_prop_dt_ns_ = static_cast<std::int64_t>(cfg.max_prop_interval * 1e9);
  buffer_time_ns_ = static_cast<std::int64_t>(cfg.max_filter_lag * 1e9);
  feedback_config_ = cfg.feedback_config;
}

void StandardOrchestrationPlugin::init_orchestration_plugin(const std::optional<api::PluginList>& plugins,
                                                            api::MessageStreamConfig& stream_config) {
  if (!plugins) {
    log(LoggingLevel::ERROR, "No plugins were provided. Filter cannot be implemented.");
    return;
  }
  if (!mediator_) {
    log(LoggingLevel::ERROR, "Orchestration has no mediator. Filter cannot be implemented.");
    return;
  }
  auto cfg = StandardOrchestrationConfig::from_registry(*mediator_);
  if (!cfg) {
    log(LoggingLevel::ERROR,
        "Unable to grab the orchestration config from the registry. Filter cannot be implemented.");
    return;
  }
  set_stream_config(cfg->stream_config, stream_config);
  store_config_data(*cfg);

  SortedPlugins sorted = sort_plugins(*plugins);
  auto logfn = [this](LoggingLevel l, const std::string& m) { log(l, m); };
  if (!validate_plugins(sorted, logfn,
                        {{"fusion_plugins", {1, 1}},
                         {"fusion_strategy_plugins", {1, 1}},
                         {"inertial_plugins", {1, 1}},
                         {"initialization_plugins", {1, 1}},
                         {"state_modeling_plugins", {1, 1000}}}))
    return;
  fusion_plugin_ = sorted.fusion_plugins[0];
  fusion_strategy_plugin_ = sorted.fusion_strategy_plugins[0];
  inertial_plugin_ = sorted.inertial_plugins[0];
  initialization_plugin_ = sorted.initialization_plugins[0];
  state_modeling_plugins_ = sorted.state_modeling_plugins;

  if (cfg->preprocessor_configs)
    for (const auto& pc : *cfg->preprocessor_configs) add_preprocessor(sorted.preprocessor_plugins, *pc);

  if (!set_up_fusion_engine(*cfg)) return;

  initializer_ = orch::set_up_initializer(*initialization_plugin_, cfg->alignment_config_group, logfn);
  if (!initializer_) return;
  try_initialize();
}

void StandardOrchestrationPlugin::try_initialize() {
  if (!initializer_ || !orch::initialization_ready(*initializer_)) return;
  if (!generate_initial_inertial_solution()) return;
  if (!initialize_filter()) return;
  send_inertial_aux_to_pinson();
}

bool StandardOrchestrationPlugin::generate_initial_inertial_solution() {
  auto logfn = [this](LoggingLevel l, const std::string& m) { log(l, m); };
  auto setup = orch::set_up_inertial_mechanization(*initializer_, *inertial_plugin_, inertial_group_, logfn);
  if (!setup) return false;
  inertial_ = std::move(setup->inertial);
  init_solution_ = std::move(setup->init_solution);
  return true;
}

StandardOrchestrationPlugin::Providers StandardOrchestrationPlugin::make_providers() {
  Providers out;
  for (const auto& p : state_modeling_plugins_)
    if (auto prov = p->new_state_model_provider(api::FusionType::STANDARD)) out.push_back(std::move(prov));
  return out;
}

bool StandardOrchestrationPlugin::initialize_filter() {
  auto pva = init_solution_->solution->as<utils::PVA>();
  api::Matrix pva_cov = pva->get_covariance();
  api::Matrix bias_cov = init_solution_->inertial_error_covariance.value_or(api::Matrix::Zero(6, 6));
  api::Matrix cov = api::Matrix::Zero(pva_cov.rows() + bias_cov.rows(), pva_cov.cols() + bias_cov.cols());
  cov.topLeftCorner(pva_cov.rows(), pva_cov.cols()) = pva_cov;
  cov.bottomRightCorner(bias_cov.rows(), bias_cov.cols()) = bias_cov;
  init_pinson_cov_ = cov;

  if (!add_state_block(make_providers(), pinson_sb_config_)) return false;

  const Timestamp init_time = utils::time_of_validity(*pva).value_or(Timestamp{0});
  fusion_engine_->set_time(init_time);
  last_feedback_time_ns_ = init_time.elapsed_nsec;
  cache_ = std::make_unique<orch::SolutionCache>(
      *fusion_engine_, *inertial_, imu_sol_channel_, best_sol_channel_, pinson_sb_config_.label,
      [this](LoggingLevel l, const std::string& m) { log(l, m); });
  log(LoggingLevel::INFO, "Aligned filter at " + secs(init_time) + "s");
  return true;
}

bool StandardOrchestrationPlugin::add_state_block(const Providers& providers, const StateBlockConfig& cfg) {
  for (const auto& provider : providers) {
    auto idx = index_of(provider->block_identifiers(), cfg.identifier);
    if (!idx) continue;
    auto block = provider->new_block(*idx, fusion_engine_.get(), cfg.label, cfg.group_);
    if (!block) {
      log(LoggingLevel::ERROR, "Unable to create state block \"" + cfg.label + "\" with identifier \"" +
                                   cfg.identifier + "\" from config group \"" + cfg.group_ +
                                   "\" - cannot set up fusion engine or initialize filter.");
      return false;
    }
    auto ewc = create_state_block_ewc(cfg.identifier, block->num_states(), cfg.estimate_with_covariance);
    if (!ewc) return false;
    fusion_engine_->add_state_block(std::move(block), *ewc, std::nullopt);
    if (cfg.aux_channels)
      for (const auto& ch : *cfg.aux_channels) sb_aux_channels_[ch].push_back(cfg.label);
    return true;
  }
  return true;  // no provider knows this identifier: Python silently skips too
}

bool StandardOrchestrationPlugin::add_measurement_processor(const Providers& providers,
                                                            const MeasurementProcessorConfig& cfg) {
  for (const auto& provider : providers) {
    auto idx = index_of(provider->processor_identifiers(), cfg.identifier);
    if (!idx) continue;
    auto proc = provider->new_processor(*idx, fusion_engine_.get(), cfg.label, cfg.state_block_labels, cfg.group_);
    if (!proc) {
      log(LoggingLevel::ERROR, "Unable to create measurement processor \"" + cfg.label + "\" with identifier \"" +
                                   cfg.identifier + "\" from config group \"" + cfg.group_ +
                                   "\" - cannot set up fusion engine or initialize filter.");
      return false;
    }
    fusion_engine_->add_measurement_processor(std::move(proc));
    needs_inertial_pva_[cfg.label] = false;
    needs_inertial_f_and_r_[cfg.label] = false;
    if (cfg.aux_channels) {
      for (const auto& ch : *cfg.aux_channels) {
        mp_aux_channels_[ch].push_back(cfg.label);
        if (ch == kAuxInertialPva) needs_inertial_pva_[cfg.label] = true;
        if (ch == kAuxInertialForcesAndRates) needs_inertial_f_and_r_[cfg.label] = true;
      }
    }
    return true;
  }
  return true;
}

bool StandardOrchestrationPlugin::add_virtual_state_block(const Providers& providers,
                                                          const VirtualStateBlockConfig& cfg) {
  for (const auto& provider : providers) {
    auto idx = index_of(provider->virtual_block_identifiers(), cfg.identifier);
    if (!idx) continue;
    auto vsb = provider->new_virtual_block(*idx, cfg.source, cfg.target, cfg.group_);
    if (!vsb) {
      log(LoggingLevel::ERROR, "Unable to create virtual state block \"" + cfg.identifier +
                                   "\" from config group \"" + cfg.group_ +
                                   "\" - cannot set up fusion engine or initialize filter.");
      return false;
    }
    fusion_engine_->add_virtual_state_block(std::move(vsb));
    needs_inertial_pva_[cfg.target] = false;
    needs_inertial_f_and_r_[cfg.target] = false;
    vsb_target_to_source_[cfg.target] = cfg.source;
    if (cfg.aux_channels) {
      for (const auto& ch : *cfg.aux_channels) {
        vsb_aux_channels_[ch].push_back(cfg.target);
        if (ch == kAuxInertialPva) needs_inertial_pva_[cfg.target] = true;
        if (ch == kAuxInertialForcesAndRates) needs_inertial_f_and_r_[cfg.target] = true;
      }
    }
    return true;
  }
  return true;
}

void StandardOrchestrationPlugin::map_vsb_inertial_needs(
    const std::vector<std::shared_ptr<const MeasurementProcessorConfig>>& mp_configs) {
  for (const auto& mp : mp_configs) {
    std::vector<std::string> needs_pva, needs_fnr;
    for (const auto& label : mp->state_block_labels) {
      // Walk the source chain: every VSB between the MP's target and the real block may need aux.
      std::optional<std::string> cur = label;
      while (cur) {
        if (auto it = needs_inertial_pva_.find(*cur); it != needs_inertial_pva_.end() && it->second)
          needs_pva.push_back(*cur);
        if (auto it = needs_inertial_f_and_r_.find(*cur); it != needs_inertial_f_and_r_.end() && it->second)
          needs_fnr.push_back(*cur);
        auto s = vsb_target_to_source_.find(*cur);
        cur = s == vsb_target_to_source_.end() ? std::nullopt : std::optional<std::string>(s->second);
      }
    }
    vsbs_needing_pva_[mp->label] = needs_pva;
    vsbs_needing_f_and_r_[mp->label] = needs_fnr;
  }
}

void StandardOrchestrationPlugin::add_preprocessor(const std::vector<std::shared_ptr<api::PreprocessorPlugin>>& plugins,
                                                   const PreprocessorConfig& cfg) {
  if (cfg.regex) {
    log(LoggingLevel::ERROR, "Regex channels are not supported for \"PreprocessorConfig.channels\" in "
                             "StandardOrchestrationPlugin.");
    return;
  }
  for (const auto& plugin : plugins) {
    auto idx = index_of(plugin->preprocessor_identifiers(), cfg.identifier);
    if (!idx) continue;
    auto pp = plugin->new_preprocessor(*idx, cfg.group_);
    if (!pp) {
      log(LoggingLevel::ERROR, "Unable to create preprocessor with identifier \"" + cfg.identifier +
                                   "\" from config group \"" + cfg.group_ + "\".");
      return;
    }
    preprocessors_.emplace_back(std::move(pp), cfg.channels);
    return;
  }
}

bool StandardOrchestrationPlugin::set_up_fusion_engine(const StandardOrchestrationConfig& cfg) {
  auto engine = fusion_plugin_->new_fusion_engine(api::FusionType::STANDARD);
  if (!engine) {
    log(LoggingLevel::ERROR, "Unable to make new fusion engine - cannot continue.");
    return false;
  }
  auto strategy = fusion_strategy_plugin_->new_fusion_strategy(api::FusionType::STANDARD);
  if (!strategy) {
    log(LoggingLevel::ERROR, "Unable to make new fusion strategy - cannot continue.");
    return false;
  }
  engine->set_strategy(std::move(strategy));
  fusion_engine_ = std::move(engine);

  Providers providers = make_providers();
  if (cfg.additional_sb_configs)
    for (const auto& sb : *cfg.additional_sb_configs)
      if (!add_state_block(providers, *sb)) return false;
  if (cfg.mp_configs) {
    for (const auto& mp : *cfg.mp_configs) {
      if (!add_measurement_processor(providers, *mp)) return false;
      measurement_channels_[mp->channel].push_back(mp->label);
    }
  }
  if (cfg.vsb_configs)
    for (const auto& vsb : *cfg.vsb_configs)
      if (!add_virtual_state_block(providers, *vsb)) return false;
  if (cfg.mp_configs) map_vsb_inertial_needs(*cfg.mp_configs);
  engine_ready_ = true;
  return true;
}

std::optional<EstimateWithCovariance> StandardOrchestrationPlugin::create_state_block_ewc(
    const std::string& identifier, std::size_t num_states, const std::optional<EstimateWithCovariance>& ewc) {
  if (identifier == PinsonStateBlockConfig::kIdentifier && !ewc && init_pinson_cov_) {
    return EstimateWithCovariance{EstimateWithCovarianceType::EWC_GENERIC,
                                  api::Vector::Zero(static_cast<Eigen::Index>(num_states)), *init_pinson_cov_};
  }
  if (ewc) return utils::validate_manual_ewc(*ewc, num_states, *mediator_);
  log(LoggingLevel::ERROR, "No initial estimate/covariance available for state block \"" + identifier + "\".");
  return std::nullopt;
}

// ----------------------------------------------------------------------------- runtime

void StandardOrchestrationPlugin::propagate_to_time(Timestamp target) {
  Timestamp filter_time = fusion_engine_->time();
  while (filter_time.elapsed_nsec < target.elapsed_nsec) {
    send_inertial_aux_to_pinson();
    Timestamp prop_time{filter_time.elapsed_nsec + max_prop_dt_ns_};
    if (target.elapsed_nsec < prop_time.elapsed_nsec) prop_time = target;
    fusion_engine_->propagate(prop_time);
    if (fusion_engine_->time().elapsed_nsec == filter_time.elapsed_nsec) break;  // propagate refused
    filter_time = prop_time;
  }
}

void StandardOrchestrationPlugin::publish_solution(const std::optional<Message>& solution, const std::string& group,
                                                   const std::string& key) {
  if (!solution) return;
  mediator_->registry().batch(group)->set(key, *solution);
  mediator_->broadcast_aspn_message(*solution, std::nullopt, solution->source_identifier);
}

std::optional<Message> StandardOrchestrationPlugin::get_inertial_forces(std::optional<Timestamp> t1,
                                                                        std::optional<Timestamp> t2) {
  Timestamp a = t1 ? Timestamp{std::max(t1->elapsed_nsec, inertial_->request_earliest_time().elapsed_nsec)}
                   : fusion_engine_->time();
  Timestamp b = t2 ? Timestamp{std::min(t2->elapsed_nsec, inertial_->request_latest_time().elapsed_nsec)}
                   : fusion_engine_->time();
  std::optional<api::InertialForcesRates> forces = (a.elapsed_nsec == b.elapsed_nsec)
                                                       ? inertial_->request_forces_and_rates(a)
                                                       : inertial_->request_average_forces_and_rates(a, b);
  if (!forces || !forces->forces_and_rates) {
    log(LoggingLevel::ERROR, "Cannot get inertial aux. Forces not available spanning time [" + secs(a) + ", " +
                                 secs(b) + "]");
    return std::nullopt;
  }
  return Message(forces->forces_and_rates, "Orchestration forces and rates");
}

void StandardOrchestrationPlugin::send_inertial_aux_to_measurement_processor(const std::string& mp_label) {
  const bool needs_pva = needs_inertial_pva_[mp_label];
  const bool needs_forces = needs_inertial_f_and_r_[mp_label];
  if (!needs_pva && !needs_forces) return;
  api::AuxData aux;
  if (needs_pva) aux.push_back(cache_->inertial_solution());
  if (needs_forces) {
    const Timestamp ft = fusion_engine_->time();
    aux.push_back(get_inertial_forces(Timestamp{ft.elapsed_nsec - 500'000'000}, Timestamp{ft.elapsed_nsec + 500'000'000}));
  }
  fusion_engine_->give_measurement_processor_aux_data(mp_label, aux);
}

void StandardOrchestrationPlugin::send_inertial_aux_to_pinson() {
  api::AuxData aux{cache_->inertial_solution(), get_inertial_forces()};
  fusion_engine_->give_state_block_aux_data(pinson_sb_config_.label, aux);
}

void StandardOrchestrationPlugin::send_inertial_aux_to_vsbs(const std::string& mp_label) {
  const auto& needs_pva = vsbs_needing_pva_[mp_label];
  const auto& needs_forces = vsbs_needing_f_and_r_[mp_label];
  if (needs_pva.empty() && needs_forces.empty()) return;
  if (!needs_pva.empty()) {
    auto pva = cache_->inertial_solution();
    for (const auto& l : needs_pva) fusion_engine_->give_virtual_state_block_aux_data(l, {pva});
  }
  if (!needs_forces.empty()) {
    auto forces = get_inertial_forces();
    for (const auto& l : needs_forces) fusion_engine_->give_virtual_state_block_aux_data(l, {forces});
  }
}

bool StandardOrchestrationPlugin::ready_to_apply_feedback() {
  if (!feedback_config_) return true;
  bool time_ok = true, error_ok = true;
  if (feedback_config_->time_threshold > 0) {
    const double since = (fusion_engine_->time().elapsed_nsec - last_feedback_time_ns_) * 1e-9;
    time_ok = since >= feedback_config_->time_threshold;
  }
  if (feedback_config_->pos_error_threshold > 0) {
    auto xp = cache_->pinson();
    error_ok = xp && (xp->estimate.head<3>().cwiseAbs().array() >= feedback_config_->pos_error_threshold).any();
  }
  return time_ok && error_ok;
}

void StandardOrchestrationPlugin::apply_inertial_feedback() {
  if (!ready_to_apply_feedback()) return;
  auto solution = cache_->filter_solution();
  if (!solution) return;
  inertial_->reset_solution(*solution);

  const Timestamp cur = fusion_engine_->time();
  auto errors = inertial_->request_sensor_errors(cur);
  if (!errors) {
    log(LoggingLevel::ERROR, "Unable to obtain sensor errors from inertial at time " + secs(cur) + "s.");
    return;
  }
  auto xp = cache_->pinson();
  if (!xp) return;
  errors->accel_biases -= xp->estimate.segment<3>(9);
  errors->gyro_biases -= xp->estimate.segment<3>(12);
  inertial_->correct_sensor_errors(cur, *errors);

  fusion_engine_->set_state_block_estimate(pinson_sb_config_.label, api::Vector::Zero(15));
  cache_->clear_inertial_solution();
  cache_->clear_pinson();
  cache_->clear_filter_solution();
  last_feedback_time_ns_ = cur.elapsed_nsec;
}

void StandardOrchestrationPlugin::perform_measurement_update(const Message& message, const std::string& target_mp) {
  send_inertial_aux_to_measurement_processor(target_mp);
  send_inertial_aux_to_vsbs(target_mp);
  if (publish_before_update_) {
    publish_solution(cache_->inertial_solution(), "inertial solution", "before feedback");
    publish_solution(cache_->filter_solution(), "filter solution", "before update");
  }
  fusion_engine_->update(target_mp, message);
  cache_->clear_filter_solution();
  cache_->clear_pinson();
  apply_inertial_feedback();
  if (publish_after_update_) {
    publish_solution(cache_->inertial_solution(), "inertial solution", "after feedback");
    publish_solution(cache_->filter_solution(), "filter solution", "after update");
  }
}

void StandardOrchestrationPlugin::propagate_during_outage() {
  if (!init_solution_) return;
  const Timestamp earliest = inertial_->request_earliest_time();
  const Timestamp latest = inertial_->request_latest_time();
  const Timestamp prop_time{earliest.elapsed_nsec + inertial_drift_prop_dt_};
  if (prop_time.elapsed_nsec < latest.elapsed_nsec - buffer_time_ns_)
    propagate_to_time(prop_time);
  else
    propagate_to_time(earliest);
}

void StandardOrchestrationPlugin::send_message_as_aux_data(const Message& message) {
  const auto& ch = message.source_identifier;
  if (auto it = sb_aux_channels_.find(ch); it != sb_aux_channels_.end())
    for (const auto& l : it->second) fusion_engine_->give_state_block_aux_data(l, {message});
  if (auto it = mp_aux_channels_.find(ch); it != mp_aux_channels_.end())
    for (const auto& l : it->second) fusion_engine_->give_measurement_processor_aux_data(l, {message});
  if (auto it = vsb_aux_channels_.find(ch); it != vsb_aux_channels_.end())
    for (const auto& l : it->second) fusion_engine_->give_virtual_state_block_aux_data(l, {message});
}

std::optional<std::vector<Message>> StandardOrchestrationPlugin::preprocess_message(const Message& message) {
  std::vector<Message> out{message};
  for (auto& [pp, channels] : preprocessors_) {
    if (out.empty()) return std::nullopt;
    std::vector<Message> in = std::move(out);
    out.clear();
    for (const auto& m : in) {
      if (!channels || contains(*channels, m.source_identifier)) {
        if (auto produced = pp->process_pntos_message(m)) out.insert(out.end(), produced->begin(), produced->end());
      } else {
        out.push_back(m);
      }
    }
  }
  if (out.empty()) return std::nullopt;
  return out;
}

void StandardOrchestrationPlugin::process_pntos_message(const Message& message, bool) {
  if (!engine_ready_ || !initializer_) return;
  std::optional<std::vector<Message>> msgs =
      preprocessors_.empty() ? std::optional<std::vector<Message>>{std::vector<Message>{message}}
                             : preprocess_message(message);
  if (!msgs) return;
  auto logfn = [this](LoggingLevel l, const std::string& m) { log(l, m); };

  for (const auto& msg : *msgs) {
    const std::string& channel = msg.source_identifier;
    if (!init_solution_) {
      if (contains(alignment_channels_, channel)) {
        initializer_->process_pntos_message(msg);
        try_initialize();
      }
      continue;
    }
    if (!orch::has_valid_time(true, *fusion_engine_, msg, logfn)) continue;

    if (contains(inertial_channels_, channel)) {
      inertial_->process_pntos_message(msg);
    } else if (auto it = measurement_channels_.find(channel); it != measurement_channels_.end()) {
      auto tov = utils::time_of_validity(*msg.wrapped_message);
      if (tov) {
        propagate_to_time(*tov);
        for (const auto& mp : it->second) perform_measurement_update(msg, mp);
      }
    }
    if (sb_aux_channels_.count(channel) || mp_aux_channels_.count(channel) || vsb_aux_channels_.count(channel))
      send_message_as_aux_data(msg);
  }
  propagate_during_outage();
}

std::vector<std::string> StandardOrchestrationPlugin::filter_description_list() const {
  const std::string pva = "ASPN_MEASUREMENT_POSITION_VELOCITY_ATTITUDE_ESTIMATE";
  return {"POS_INS_BEST_" + pva, "POS_INS_DEAD_RECKONING_" + pva};
}

std::optional<std::vector<std::optional<Message>>> StandardOrchestrationPlugin::request_solutions(
    const std::vector<Timestamp>& solution_times, const std::optional<std::string>& filter_description) {
  if (!initializer_ || !orch::initialization_ready(*initializer_) || !init_solution_) {
    log(LoggingLevel::DEBUG, "Unable to provide a solution - initialization not ready.");
    return std::nullopt;
  }
  if (solution_times.size() != 1) {
    log(LoggingLevel::ERROR,
        "This implementation of request_solutions requires a time array of length one but received a time array "
        "of length " +
            std::to_string(solution_times.size()));
    return std::nullopt;
  }
  Timestamp time = solution_times[0];
  if (!inertial_->is_time_in_range(time)) {
    const Timestamp latest = inertial_->request_latest_time();
    const Timestamp earliest = inertial_->request_earliest_time();
    log(LoggingLevel::DEBUG, "Requested time (" + std::to_string(time.elapsed_nsec) +
                                 ") is outside inertial time range (" + std::to_string(earliest.elapsed_nsec) +
                                 " - " + std::to_string(latest.elapsed_nsec) +
                                 "). Replacing requested time with the latest inertial time.");
    time = latest;
  }
  auto logfn = [this](LoggingLevel l, const std::string& m) { log(l, m); };
  std::optional<Message> out;
  if (!filter_description || filter_description->find("BEST") != std::string::npos) {
    out = orch::get_best_solution(*fusion_engine_, *inertial_, time, pinson_sb_config_.label, best_sol_channel_, logfn);
  } else if (filter_description->find("DEAD_RECKONING") != std::string::npos) {
    out = orch::get_dead_reckoning_solution(*inertial_, time, imu_sol_channel_, logfn);
  } else {
    std::string descs;
    for (const auto& d : filter_description_list()) descs += (descs.empty() ? "" : ", ") + d;
    log(LoggingLevel::ERROR,
        "Solution " + *filter_description + " was requested, but available solution types are: " + descs);
    return std::nullopt;
  }
  if (!out) return std::nullopt;
  return std::vector<std::optional<Message>>{out};
}

}  // namespace pntos::cobra
