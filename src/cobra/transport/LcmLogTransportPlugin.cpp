#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/transport/LcmLogTransportPlugin.hpp>
#include <pntos/cobra/utils/aspn.hpp>

#include <iomanip>
#include <sstream>

namespace pntos::cobra {

using api::LoggingLevel;
using api::Message;

LcmLogTransportPlugin::LcmLogTransportPlugin(std::string identifier, std::string config_group)
    : identifier_(std::move(identifier)), config_group_(std::move(config_group)) {}

LcmLogTransportPlugin::~LcmLogTransportPlugin() {
  stop_ = true;
  if (thread_.joinable()) thread_.join();
}

void LcmLogTransportPlugin::init_plugin(const std::optional<std::string>&, api::Mediator* mediator) {
  if (!mediator) return;
  mediator_ = mediator;
  auto cfg = LcmLogTransportConfig::from_registry(*mediator_, config_group_);
  if (!cfg) {
    mediator_->log_message(LoggingLevel::ERROR, "Unable to retrieve config from registry.");
    return;
  }
  if (cfg->input_file) {
    try {
      input_ = std::make_unique<lcm::LcmLogReader>(*cfg->input_file);
    } catch (const std::exception& e) {
      mediator_->log_message(LoggingLevel::ERROR, e.what());
      return;
    }
  }
  if (cfg->output_file && cfg->input_file && *cfg->output_file == *cfg->input_file) {
    mediator_->log_message(LoggingLevel::ERROR,
                           "Output file of " + *cfg->output_file + " cannot be the same as input file.");
    return;
  }
  if (cfg->output_file) {
    try {
      output_ = std::make_unique<lcm::LcmLogWriter>(*cfg->output_file);
    } catch (const std::exception& e) {
      mediator_->log_message(LoggingLevel::ERROR, e.what());
      return;
    }
  }
  if (cfg->channels_to_process)
    channels_to_process_ = std::set<std::string>(cfg->channels_to_process->begin(), cfg->channels_to_process->end());
  record_input_ = cfg->record_input_channels;
}

void LcmLogTransportPlugin::shutdown_plugin() {
  stop_listening();
  if (thread_.joinable()) thread_.join();
  {
    std::lock_guard lk(output_mutex_);
    if (output_) output_->close();
  }
  if (mediator_) mediator_->log_message(LoggingLevel::INFO, "Shutdown plugin for " + identifier_ + ".");
}

bool LcmLogTransportPlugin::source_enabled(const std::string& channel) {
  // ui/channel/<channel>: enabled_source (UiSourceInterface). Read once per channel; a UI that flips
  // the flag later is honoured on the next registry notify (kept simple: re-read every 1000 messages).
  auto it = source_gate_.find(channel);
  if (it != source_gate_.end() && processed_ % 1000 != 0) return it->second;
  bool enabled = true;
  try {
    auto kv = mediator_->registry().batch("ui/channel/" + channel);
    if (!kv->has_key("enabled_source")) kv->set("enabled_source", true);
    enabled = kv->get_value<bool>("enabled_source").value_or(true);
  } catch (const std::exception&) {
    enabled = true;  // no registry (tests)
  }
  source_gate_[channel] = enabled;
  return enabled;
}

void LcmLogTransportPlugin::process(const std::string& channel, const std::vector<std::uint8_t>& data) {
  std::shared_ptr<api::AspnBase> msg;
  try {
    msg = lcm::decode(data);
  } catch (const std::exception& e) {
    mediator_->log_message(LoggingLevel::WARN, std::string("Failed to decode lcm message: ") + e.what());
    return;
  }
  if (!msg) {
    mediator_->log_message(LoggingLevel::WARN, "Cannot decode message on channel " + channel + ". Ignoring message.");
    return;
  }
  if (!channels_found_.count(channel)) {
    auto t = utils::time_of_validity(*msg);
    std::ostringstream os;
    os << "Found new channel " << channel << "\t with a timestamp of " << std::fixed << std::setprecision(9)
       << (t ? t->seconds() : 0.0) << "s";
    mediator_->log_message(LoggingLevel::INFO, os.str());
    channels_found_.insert(channel);
  }
  mediator_->process_pntos_message(Message(msg, channel));
  ++processed_;
}

void LcmLogTransportPlugin::read_log() {
  if (!input_ || !mediator_) return;
  const std::uint64_t total = input_->size();
  std::uint64_t next_report = total / 100;
  std::optional<lcm::LcmEvent> ev = input_->next();
  while (ev && !stop_) {
    if (output_ && record_input_) {
      std::lock_guard lk(output_mutex_);
      output_->write(lcm::now_us(), ev->channel, ev->data);
    }
    if (!channels_to_process_ || channels_to_process_->count(ev->channel)) {
      if (source_enabled(ev->channel)) process(ev->channel, ev->data);
    }
    if (progress_ && input_->tell() >= next_report) {
      progress_(input_->tell(), total);
      next_report += std::max<std::uint64_t>(total / 100, 1);
    }
    ev = input_->next();
  }
  if (!ev) {
    mediator_->log_message(LoggingLevel::INFO, "Done processing LCM log.");
    mediator_->registry().batch("controller/flags")->set("ready_to_shutdown", true);
  }
}

void LcmLogTransportPlugin::start_listening() {
  if (!input_ || thread_.joinable()) return;
  stop_ = false;
  thread_ = std::thread([this] { read_log(); });
  if (mediator_) mediator_->log_message(LoggingLevel::INFO, "LCM log reader is running.");
}

void LcmLogTransportPlugin::stop_listening() { stop_ = true; }

void LcmLogTransportPlugin::broadcast_message(const Message& message, const std::optional<std::string>& channel_name) {
  if (!output_) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::ERROR,
                             "Cannot output message via LcmLogTransportPlugin. Output file has not been set in "
                             "LcmLogTransportConfig.");
    return;
  }
  if (!channel_name) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::WARN, "No channel name specified. This implementation requires a channel name.");
    return;
  }
  auto bytes = message.wrapped_message ? lcm::encode(*message.wrapped_message) : std::nullopt;
  if (!bytes) {
    if (mediator_)
      mediator_->log_message(LoggingLevel::WARN,
                             "Cannot marshal message on channel " + *channel_name + ". Ignoring message.");
    return;
  }
  std::lock_guard lk(output_mutex_);
  output_->write(lcm::now_us(), *channel_name, *bytes);
}

}  // namespace pntos::cobra
