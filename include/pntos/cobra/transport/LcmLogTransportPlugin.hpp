// Port of pntos.cobra.LcmLogTransportPlugin: replays an LCM log into the mediator on its own
// thread and records broadcast messages (and optionally the input) to an output log.
#pragma once

#include <pntos/api/transport.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/transport/LcmLog.hpp>

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

namespace pntos::cobra {

class LcmLogTransportPlugin final : public api::TransportPlugin {
 public:
  explicit LcmLogTransportPlugin(std::string identifier, std::string config_group = LcmLogTransportConfig::kGroup);
  ~LcmLogTransportPlugin() override;

  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override;
  void shutdown_plugin() override;
  const std::string& identifier() const override { return identifier_; }

  void start_listening() override;
  void stop_listening() override;
  void broadcast_message(const api::Message& message, const std::optional<std::string>& channel_name) override;

  /// Replays the whole input log synchronously (what the reader thread runs). Public for tests.
  void read_log();
  const std::set<std::string>& channels_found() const { return channels_found_; }
  std::size_t messages_processed() const { return processed_.load(); }
  /// Progress callback (bytes read, total bytes), called at most every ~1% of the file.
  void set_progress_callback(std::function<void(std::uint64_t, std::uint64_t)> cb) { progress_ = std::move(cb); }

 private:
  bool source_enabled(const std::string& channel);
  void process(const std::string& channel, const std::vector<std::uint8_t>& data);

  std::string identifier_, config_group_;
  api::Mediator* mediator_ = nullptr;
  std::unique_ptr<lcm::LcmLogReader> input_;
  std::unique_ptr<lcm::LcmLogWriter> output_;
  std::mutex output_mutex_;
  std::optional<std::set<std::string>> channels_to_process_;
  bool record_input_ = true;
  std::set<std::string> channels_found_;
  std::map<std::string, bool> source_gate_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<std::size_t> processed_{0};
  std::function<void(std::uint64_t, std::uint64_t)> progress_;
};

}  // namespace pntos::cobra
