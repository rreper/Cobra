// Network LCM transport over UDP multicast (roadmap Phase 3), implementing the LCM wire protocol
// directly (no liblcm): short messages (magic 0x4C433032) and fragmented messages (0x4C433033) as
// lcm_udpm sends them, so it interoperates with lcm-logplayer, lcm-logger and any LCM 1.x process.
//
// Config: LcmTransportConfig (url "udpm://239.255.76.67:7667?ttl=0", subscribe_to regex, plus the C++
// additions idle_timeout_sec and output_file). Received messages matching the regex are decoded as
// ASPN-23 and handed to the mediator; broadcasts are encoded and sent on their channel. With
// idle_timeout_sec > 0 the transport sets controller/flags ready_to_shutdown after that long without a
// message (once something has arrived), which is how a replayed log ends a run.
#pragma once

#include <pntos/api/api.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/transport/LcmLog.hpp>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <set>
#include <thread>

namespace pntos::cobra {

struct UdpmUrl {
  std::string address = "239.255.76.67";
  std::uint16_t port = 7667;
  int ttl = 0;
};
/// Parses `udpm://addr:port?ttl=N`; anything else (e.g. Cobra's "tcpq://") yields the default with ok=false.
UdpmUrl parse_udpm_url(const std::string& url, bool* ok = nullptr);

/// LCM datagram encoding/decoding (public for tests and the log player).
namespace lcm {
constexpr std::uint32_t kLcmMagicShort = 0x4C433032;
constexpr std::uint32_t kLcmMagicFragment = 0x4C433033;
constexpr std::size_t kLcmMaxDatagram = 65499;
/// One or more datagrams carrying `data` on `channel` with sequence number `seq`.
std::vector<std::vector<std::uint8_t>> lcm_datagrams(const std::string& channel, const std::vector<std::uint8_t>& data,
                                                     std::uint32_t seq);

/// Reassembles datagrams into (channel, payload) messages.
class LcmReassembler {
 public:
  /// Feeds one datagram; returns the completed message, if any.
  std::optional<std::pair<std::string, std::vector<std::uint8_t>>> feed(const std::uint8_t* d, std::size_t len,
                                                                        const std::string& sender_key);

 private:
  struct Partial {
    std::string channel;
    std::vector<std::uint8_t> data;
    std::vector<bool> have;
    std::size_t received = 0;
    std::chrono::steady_clock::time_point started;
  };
  std::map<std::pair<std::string, std::uint32_t>, Partial> partials_;
};
}  // namespace lcm

class LcmUdpTransportPlugin final : public api::TransportPlugin {
 public:
  explicit LcmUdpTransportPlugin(std::string identifier, std::string config_group = LcmTransportConfig::kGroup);
  ~LcmUdpTransportPlugin() override;

  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override;
  void shutdown_plugin() override;
  const std::string& identifier() const override { return identifier_; }

  void start_listening() override;
  void stop_listening() override;
  void broadcast_message(const api::Message& message, const std::optional<std::string>& channel_name) override;

  std::size_t messages_received() const { return received_.load(); }
  std::size_t messages_sent() const { return sent_.load(); }
  bool socket_open() const { return fd_ >= 0; }
  /// Sends raw LCM-encoded bytes (what the log player does). Returns false if the socket is closed.
  bool send_raw(const std::string& channel, const std::vector<std::uint8_t>& data);

 private:
  bool open_socket();
  void receive_loop();
  void handle(const std::string& channel, const std::vector<std::uint8_t>& data);

  std::string identifier_, config_group_;
  api::Mediator* mediator_ = nullptr;
  UdpmUrl url_;
  std::regex subscribe_;
  double idle_timeout_sec_ = 0;
  std::unique_ptr<lcm::LcmLogWriter> output_;
  std::mutex output_mutex_;
  int fd_ = -1;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<std::size_t> received_{0}, sent_{0};
  std::atomic<std::uint32_t> seq_{0};
  std::set<std::string> channels_found_;
  lcm::LcmReassembler reassembler_;
};

}  // namespace pntos::cobra
