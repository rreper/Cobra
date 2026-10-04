#include <pntos/cobra/app/Filter.hpp>

#include <pntos/cobra/app/AppBuilder.hpp>

#include <stdexcept>

namespace pntos::cobra {

void PushTransportPlugin::broadcast_message(const api::Message& message, const std::optional<std::string>& channel_name) {
  if (sink_) sink_(message, channel_name.value_or(message.source_identifier));
}

bool PushTransportPlugin::push(const api::Message& message) {
  if (!listening_ || !mediator_) return false;
  mediator_->process_pntos_message(message);
  return true;
}

Filter::Filter(AppConfig config, const app::RunOptions& options) : config_(app::apply_overrides(config, options)) {
  config_.app.transport = "push";
  plugins_ = app::build_plugins(config_);
  for (const auto& p : plugins_)
    if (auto t = std::dynamic_pointer_cast<PushTransportPlugin>(p)) transport_ = t;
  if (!transport_) throw std::runtime_error("Filter: no push transport in the plugin set");
  transport_->set_sink([this](const api::Message& m, const std::string& channel) {
    api::Message copy(m.wrapped_message, channel);
    {
      std::lock_guard lk(mutex_);
      solutions_.push_back(copy);
    }
    if (callback_) callback_(copy);
  });
  controller_ = std::make_unique<StandardControllerPlugin>("Cobra Standard Controller Plugin");
  controller_->init_plugin(std::nullopt, nullptr);
  if (!controller_->start(plugins_)) throw std::runtime_error("Filter: the controller could not start (see the log)");
}

Filter::~Filter() { stop(); }

void Filter::push(const api::Message& message) {
  if (stopped_) throw std::logic_error("Filter::push after stop()");
  transport_->push(message);
}

std::vector<api::Message> Filter::take_solutions() {
  std::lock_guard lk(mutex_);
  std::vector<api::Message> out;
  out.swap(solutions_);
  return out;
}

void Filter::set_solution_callback(std::function<void(const api::Message&)> callback) { callback_ = std::move(callback); }

std::optional<api::Message> Filter::solution(api::Timestamp time) {
  if (stopped_ || !transport_->mediator()) return std::nullopt;
  auto sols = transport_->mediator()->request_solutions({time});
  if (!sols || sols->empty() || !(*sols)[0]) return std::nullopt;
  return (*sols)[0];
}

api::Registry& Filter::registry() { return transport_->mediator()->registry(); }

int Filter::stop() {
  if (!stopped_) {
    stopped_ = true;
    controller_->stop();
  }
  return controller_->exit_code() == ExitCode::SUCCESS ? 0 : 1;
}

}  // namespace pntos::cobra
