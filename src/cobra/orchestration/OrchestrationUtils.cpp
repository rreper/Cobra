#include <pntos/cobra/orchestration/OrchestrationUtils.hpp>
#include <pntos/cobra/utils/navutils.hpp>

#include <iomanip>
#include <sstream>

namespace pntos::cobra::orch {

using api::LoggingLevel;
using api::Message;
using api::Timestamp;

namespace {
std::string secs(Timestamp t) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(9) << t.seconds();
  return os.str();
}
}  // namespace

std::unique_ptr<api::InertialInitializationStrategy> set_up_initializer(api::InitializationPlugin& plugin,
                                                                        const std::string& alignment_config_group,
                                                                        const LogFn& log) {
  auto strategy = plugin.new_initialization_strategy(api::InitializationType::INERTIAL, alignment_config_group);
  auto* inertial = dynamic_cast<api::InertialInitializationStrategy*>(strategy.get());
  if (!inertial) {
    log(LoggingLevel::ERROR, "InertialInitializationStrategy not supported by " + plugin.identifier() +
                                 ". Unable to continue.");
    return nullptr;
  }
  strategy.release();
  return std::unique_ptr<api::InertialInitializationStrategy>(inertial);
}

bool has_valid_time(bool have_init_solution, const api::StandardFusionEngine& engine, const Message& message,
                    const LogFn& log) {
  if (!have_init_solution) return true;
  if (message.wrapped_message) {
    if (auto t = utils::time_of_validity(*message.wrapped_message)) {
      if (engine.time().elapsed_nsec <= t->elapsed_nsec) return true;
      log(LoggingLevel::DEBUG, "Received old message at time " + secs(*t) + "s on channel " +
                                   message.source_identifier + ". Filter is at time " + secs(engine.time()) +
                                   "s. Discarding message");
      return false;
    }
  }
  log(LoggingLevel::ERROR, "Measurement from \"" + message.source_identifier +
                               "\" does not contain a \"time_of_validity\" field.");
  return false;
}

std::optional<Message> get_dead_reckoning_solution(api::StandardInertialMechanization& inertial, Timestamp time,
                                                   const std::string& imu_sol_channel, const LogFn& log) {
  auto m = inertial.request_solution(time);
  if (m) return Message(m->wrapped_message, imu_sol_channel);
  log(LoggingLevel::ERROR, "Unable to get PVA message from inertial. Cannot generate DEAD_RECKONING solution.");
  return std::nullopt;
}

std::optional<Message> get_best_solution(api::StandardFusionEngine& engine,
                                         api::StandardInertialMechanization& inertial, Timestamp time,
                                         const std::string& sb_label, const std::string& best_sol_channel,
                                         const LogFn& log) {
  auto x_and_p = engine.peek_ahead(time, {sb_label});
  if (!x_and_p) {
    log(LoggingLevel::WARN, "Cannot get filter solution at time " + secs(time) + "s. Filter is already at time " +
                                secs(engine.time()) + "s.");
    return std::nullopt;
  }
  auto inertial_solution = inertial.request_solution(time);
  if (!inertial_solution) {
    log(LoggingLevel::ERROR,
        "Unable to obtain solution from inertial at time " + secs(time) + "s. Cannot generate BEST solution.");
    return std::nullopt;
  }
  auto pva = inertial_solution->as<utils::PVA>();
  if (!pva) {
    log(LoggingLevel::ERROR, "Expected PVA solution from inertial. Cannot generate BEST solution.");
    return std::nullopt;
  }
  auto corrected = apply_error_states(*pva, x_and_p->estimate);
  corrected->set_covariance(x_and_p->covariance.topLeftCorner(9, 9));
  return Message(corrected, best_sol_channel);
}

std::optional<InertialSetup> set_up_inertial_mechanization(api::InertialInitializationStrategy& initializer,
                                                           api::InertialPlugin& inertial_plugin,
                                                           const std::string& inertial_group, const LogFn& log) {
  api::InitialInertialSolution init = initializer.request_solution();
  if (!init.solution) {
    log(LoggingLevel::ERROR, "Invalid InitialInertialSolution returned from init strategy - unable to proceed.");
    return std::nullopt;
  }
  auto pva = init.solution->as<utils::PVA>();
  if (!pva) {
    log(LoggingLevel::ERROR, "Expected PVA solution from init strategy - unable to proceed.");
    return std::nullopt;
  }
  Message init_message(init.solution->wrapped_message, "orchestration");
  auto common = inertial_plugin.new_inertial(api::InertialType::STANDARD_MECHANIZATION, init_message, inertial_group);
  auto* mech = dynamic_cast<api::StandardInertialMechanization*>(common.get());
  if (!mech) {
    log(LoggingLevel::ERROR,
        "StandardInertialMechanization not supported by inertial (" + inertial_plugin.identifier() + ")");
    return std::nullopt;
  }
  common.release();
  InertialSetup out{std::unique_ptr<api::StandardInertialMechanization>(mech), std::move(init)};
  if (out.init_solution.inertial_errors && out.init_solution.inertial_error_covariance) {
    auto tov = utils::time_of_validity(*pva);
    out.inertial->correct_sensor_errors(tov.value_or(Timestamp{0}), *out.init_solution.inertial_errors);
  }
  return out;
}

std::shared_ptr<utils::PVA> apply_error_states(const utils::PVA& pva, const api::Vector& x) {
  if (x.size() < 9) throw std::invalid_argument("apply_error_states: need at least 9 error states");
  auto q = utils::quaternion(pva);
  if (!utils::has_position(pva) || !utils::has_velocity(pva) || !q)
    throw std::invalid_argument("apply_error_states: PVA is missing position, velocity or quaternion");
  const double lat = pva.get_p1(), alt = pva.get_p3();
  api::Vector3 llh(pva.get_p1() + nav::north_to_delta_lat(x(0), lat, alt),
                   pva.get_p2() + nav::east_to_delta_lon(x(1), lat, alt), pva.get_p3() - x(2));
  api::Vector3 vel(pva.get_v1() + x(3), pva.get_v2() + x(4), pva.get_v3() + x(5));
  nav::Vector4 q_new = nav::dcm_to_quat(nav::correct_dcm_with_tilt(nav::quat_to_dcm(*q), x.segment<3>(6)));
  auto tov = utils::time_of_validity(pva).value_or(Timestamp{0});
  auto out = utils::make_pva(pva.get_header(), tov, llh, vel, q_new, api::Matrix(pva.get_covariance()));
  out->set_reference_frame(pva.get_reference_frame());
  return out;
}

// ----------------------------------------------------------------------------- SolutionCache

SolutionCache::SolutionCache(api::StandardFusionEngine& engine, api::StandardInertialMechanization& inertial,
                             std::string imu_sol_channel, std::string best_sol_channel, std::string pinson_label,
                             LogFn log)
    : engine_(engine),
      inertial_(inertial),
      imu_sol_channel_(std::move(imu_sol_channel)),
      best_sol_channel_(std::move(best_sol_channel)),
      pinson_label_(std::move(pinson_label)),
      log_(std::move(log)) {}

bool SolutionCache::valid(const std::optional<Timestamp>& t) const {
  return t && t->elapsed_nsec == engine_.time().elapsed_nsec;
}

std::optional<Message> SolutionCache::inertial_solution() {
  if (!valid(inertial_time_)) {
    const Timestamp t = engine_.time();
    inertial_value_ = get_dead_reckoning_solution(inertial_, t, imu_sol_channel_, log_);
    inertial_time_ = t;
  }
  return inertial_value_;
}

std::optional<api::EstimateWithCovariance> SolutionCache::pinson() {
  if (!valid(pinson_time_)) {
    auto xp = engine_.generate_x_and_p({pinson_label_});
    if (!xp) return std::nullopt;
    pinson_value_ = std::move(xp);
    pinson_time_ = engine_.time();
  }
  return pinson_value_;
}

std::optional<Message> SolutionCache::filter_solution() {
  if (!valid(filter_time_)) {
    auto xp = pinson();
    if (!xp) return std::nullopt;
    auto inertial = inertial_solution();
    if (!inertial) return std::nullopt;
    auto pva = inertial->as<utils::PVA>();
    if (!pva) return std::nullopt;
    auto corrected = apply_error_states(*pva, xp->estimate);
    corrected->set_covariance(xp->covariance.topLeftCorner(9, 9));
    filter_value_ = Message(corrected, best_sol_channel_);
    filter_time_ = engine_.time();
  }
  return filter_value_;
}

}  // namespace pntos::cobra::orch
