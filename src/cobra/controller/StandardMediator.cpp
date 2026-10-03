#include <pntos/cobra/controller/StandardMediator.hpp>
#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/logging.hpp>

#include <algorithm>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Message;
using api::PluginType;

// ----------------------------------------------------------------------------- ExitEvent

void ExitEvent::set(ExitCode code) {
  {
    std::lock_guard lk(mutex_);
    code_ = code;
    set_ = true;
  }
  cv_.notify_all();
}
void ExitEvent::clear() {
  std::lock_guard lk(mutex_);
  set_ = false;
  code_ = ExitCode::SUCCESS;
}
bool ExitEvent::is_set() const {
  std::lock_guard lk(mutex_);
  return set_;
}
ExitCode ExitEvent::exit_code() const {
  std::lock_guard lk(mutex_);
  return code_;
}
bool ExitEvent::wait(std::optional<std::chrono::milliseconds> timeout) const {
  std::unique_lock lk(mutex_);
  if (timeout) return cv_.wait_for(lk, *timeout, [&] { return set_; });
  cv_.wait(lk, [&] { return set_; });
  return true;
}

// ----------------------------------------------------------------------------- UiMediatorInterface

namespace {
constexpr const char* kUiChannelPrefix = "ui/channel/";
constexpr const char* kEnabledMediator = "enabled_mediator";
constexpr const char* kMessageCount = "message_count";
constexpr const char* kType = "type";
constexpr const char* kTovLast = "tov_last_message";
}  // namespace

UiMediatorInterface::UiMediatorInterface(std::shared_ptr<api::Registry> registry, double update_interval_sec)
    : registry_(std::move(registry)),
      interval_(static_cast<std::int64_t>(update_interval_sec * 1000.0)) {}

UiMediatorInterface::~UiMediatorInterface() {
  if (!registry_) return;
  for (auto& [source, ch] : channels_) {
    if (ch->token) {
      auto kv = registry_->batch(kUiChannelPrefix + source);
      kv->remove_notify(*ch->token);
    }
  }
}

UiMediatorInterface::Channel& UiMediatorInterface::ensure(const std::string& source, const Message& message) {
  auto it = channels_.find(source);
  if (it != channels_.end()) return *it->second;
  auto ch = std::make_unique<Channel>();
  ch->type = message.wrapped_message ? "aspn_type_" + std::to_string(static_cast<int>(message.message_type())) : std::string("Unknown");
  if (registry_) {
    auto kv = registry_->batch(kUiChannelPrefix + source);
    if (!kv->has_key(kEnabledMediator)) kv->set(kEnabledMediator, true);
    if (!kv->has_key(kMessageCount)) kv->set(kMessageCount, std::int64_t{0});
    if (!kv->has_key(kType)) kv->set(kType, std::string("Unknown"));
    if (auto e = kv->get_value<bool>(kEnabledMediator)) ch->enabled = *e;
    Channel* raw = ch.get();
    ch->token = kv->request_notify(
        kEnabledMediator, [raw](const std::string&, const std::vector<std::string>&, api::KeyValueStore& store) {
          if (auto e = store.get_value<bool>(kEnabledMediator)) raw->enabled = *e;
        });
  }
  auto& ref = *ch;
  channels_.emplace(source, std::move(ch));
  return ref;
}

void UiMediatorInterface::publish(const std::string& source, Channel& ch) {
  if (!registry_) return;
  auto kv = registry_->batch(kUiChannelPrefix + source);
  kv->set(kMessageCount, ch.count);
  kv->set(kType, ch.type);
  if (ch.last_tov) kv->set(kTovLast, *ch.last_tov);
}

bool UiMediatorInterface::new_mediator_message(const Message& message) {
  std::lock_guard lk(mutex_);
  Channel& ch = ensure(message.source_identifier, message);
  ++ch.count;
  if (message.wrapped_message) {
    if (auto t = utils::time_of_validity(*message.wrapped_message)) ch.last_tov = t->elapsed_nsec;
  }
  const auto now = std::chrono::steady_clock::now();
  if (ch.last_update.time_since_epoch().count() == 0 || now - ch.last_update > interval_) {
    publish(message.source_identifier, ch);
    ch.last_update = now;
  }
  return ch.enabled.load();
}

// ----------------------------------------------------------------------------- StandardMediator

StandardMediator::StandardMediator(std::shared_ptr<MediatorContext> ctx, std::string attached_plugin_identifier,
                                   PluginType attached_plugin_type)
    : ctx_(std::move(ctx)), identifier_(std::move(attached_plugin_identifier)), type_(attached_plugin_type) {
  if (!ctx_) throw std::invalid_argument("StandardMediator requires a MediatorContext");
}

std::vector<std::string> StandardMediator::filter_description_list() const {
  if (!ctx_->orchestration_plugin)
    throw std::logic_error("Orchestration plugin used before initialized and passed to mediator.");
  return ctx_->orchestration_plugin->filter_description_list();
}

std::optional<std::vector<std::optional<Message>>> StandardMediator::request_solutions(
    const std::vector<api::Timestamp>& solution_times, const std::optional<std::string>& filter_description) {
  if (!ctx_->orchestration_plugin)
    throw std::logic_error("Orchestration plugin used before initialized and passed to mediator.");
  return ctx_->orchestration_plugin->request_solutions(solution_times, filter_description);
}

namespace {
std::int64_t tov_nsec(const Message& m) {
  if (m.wrapped_message)
    if (auto t = utils::time_of_validity(*m.wrapped_message)) return t->elapsed_nsec;
  throw std::invalid_argument("Message from \"" + m.source_identifier + "\" has no time of validity");
}
}  // namespace

void StandardMediator::process_pntos_message(const Message& message) {
  if (!ctx_->orchestration_plugin)
    throw std::logic_error("Orchestration plugin used before initialized and passed to mediator.");
  std::lock_guard lk(ctx_->mutex);
  if (ctx_->ui_interface && !ctx_->ui_interface->new_mediator_message(message)) return;

  const std::int64_t cur_nsec = tov_nsec(message);
  if (ctx_->stream_config->is_sequenced(message.message_type(), message.source_identifier)) {
    auto& buf = ctx_->messages;
    auto pos = std::upper_bound(buf.begin(), buf.end(), cur_nsec,
                                [](std::int64_t t, const Message& m) { return t < tov_nsec(m); });
    buf.insert(pos, message);
  } else {
    ctx_->orchestration_plugin->process_pntos_message(message, false);
  }

  const std::int64_t process_until = cur_nsec - ctx_->buffer_time_nsec;
  auto& buf = ctx_->messages;
  auto split = std::lower_bound(buf.begin(), buf.end(), process_until,
                                [](const Message& m, std::int64_t t) { return tov_nsec(m) < t; });
  std::vector<Message> ready(buf.begin(), split);
  buf.erase(buf.begin(), split);
  for (const auto& m : ready) ctx_->orchestration_plugin->process_pntos_message(m, true);

  const api::Timestamp cur_time{cur_nsec};
  if (!ctx_->last_solution_time) {
    ctx_->last_solution_time = cur_time;
    return;
  }
  if (ctx_->publish_interval_nsec && cur_nsec - ctx_->last_solution_time->elapsed_nsec > *ctx_->publish_interval_nsec) {
    auto solution = request_solutions({cur_time});
    if (solution && !solution->empty() && (*solution)[0]) {
      const Message& sol = *(*solution)[0];
      if (ctx_->ui_interface) ctx_->ui_interface->new_mediator_message(sol);
      for (const auto& transport : ctx_->transport_plugins)
        broadcast_aspn_message(sol, transport->identifier(), sol.source_identifier);
    } else {
      log_as(LoggingLevel::DEBUG, "Could not receive solution from orchestration.", PluginType::CONTROLLER);
    }
    ctx_->last_solution_time = cur_time;
  }
}

void StandardMediator::broadcast_aspn_message(const Message& message, const std::optional<std::string>& transport,
                                              const std::optional<std::string>& destination_identifier) {
  if (ctx_->transport_plugins.empty())
    throw std::logic_error("Transport plugin used before initialized and passed to mediator.");
  bool sent = false;
  for (const auto& t : ctx_->transport_plugins) {
    if (!transport || t->identifier() == *transport) {
      t->broadcast_message(message, destination_identifier);
      sent = true;
    }
  }
  if (!sent)
    log_as(LoggingLevel::WARN, "Transport \"" + transport.value_or("") + "\" not found. Unable to broadcast message.",
           PluginType::CONTROLLER);
}

void StandardMediator::log_message(LoggingLevel level, const std::string& message) { log_as(level, message, type_); }

void StandardMediator::log_as(LoggingLevel level, const std::string& message, PluginType type) {
  if (ctx_->logging_plugin)
    ctx_->logging_plugin->log(type, identifier_, level, message);
  else
    utils::print_message(level, api::to_string(type), message);
  if (level == LoggingLevel::ERROR && ctx_->controller_plugin) ctx_->exit_event.set(ExitCode::ERROR);
}

api::Registry& StandardMediator::registry() {
  if (!ctx_->registry) throw std::logic_error("Registry used before the controller created it.");
  return *ctx_->registry;
}

}  // namespace pntos::cobra
