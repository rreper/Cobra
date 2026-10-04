#include <pntos/cobra/transport/LcmUdpTransportPlugin.hpp>

#include <pntos/cobra/transport/LcmConversions.hpp>
#include <pntos/cobra/utils/aspn.hpp>
#include <pntos/cobra/utils/logging.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstring>
#include <iomanip>
#include <sstream>

namespace pntos::cobra {

using api::LoggingLevel;

UdpmUrl parse_udpm_url(const std::string& url, bool* ok) {
  UdpmUrl u;
  if (ok) *ok = false;
  const std::string prefix = "udpm://";
  if (url.rfind(prefix, 0) != 0) return u;
  std::string rest = url.substr(prefix.size());
  std::string query;
  if (auto q = rest.find('?'); q != std::string::npos) {
    query = rest.substr(q + 1);
    rest = rest.substr(0, q);
  }
  if (auto c = rest.rfind(':'); c != std::string::npos) {
    u.address = rest.substr(0, c);
    try {
      u.port = static_cast<std::uint16_t>(std::stoi(rest.substr(c + 1)));
    } catch (...) {
      return u;
    }
  } else if (!rest.empty()) {
    u.address = rest;
  }
  std::istringstream qs(query);
  std::string kv;
  while (std::getline(qs, kv, '&')) {
    if (kv.rfind("ttl=", 0) == 0) u.ttl = std::atoi(kv.c_str() + 4);
  }
  if (ok) *ok = !u.address.empty();
  return u;
}

namespace lcm {

namespace {
void put32(std::vector<std::uint8_t>& v, std::uint32_t x) {
  for (int i = 3; i >= 0; --i) v.push_back(static_cast<std::uint8_t>(x >> (8 * i)));
}
void put16(std::vector<std::uint8_t>& v, std::uint16_t x) {
  v.push_back(static_cast<std::uint8_t>(x >> 8));
  v.push_back(static_cast<std::uint8_t>(x));
}
std::uint32_t get32(const std::uint8_t* p) { return (std::uint32_t(p[0]) << 24) | (std::uint32_t(p[1]) << 16) | (std::uint32_t(p[2]) << 8) | p[3]; }
std::uint16_t get16(const std::uint8_t* p) { return static_cast<std::uint16_t>((p[0] << 8) | p[1]); }
constexpr std::size_t kFragmentPayload = 60000;
}  // namespace

std::vector<std::vector<std::uint8_t>> lcm_datagrams(const std::string& channel, const std::vector<std::uint8_t>& data,
                                                     std::uint32_t seq) {
  std::vector<std::vector<std::uint8_t>> out;
  if (8 + channel.size() + 1 + data.size() <= kLcmMaxDatagram) {
    std::vector<std::uint8_t> d;
    put32(d, kLcmMagicShort);
    put32(d, seq);
    d.insert(d.end(), channel.begin(), channel.end());
    d.push_back(0);
    d.insert(d.end(), data.begin(), data.end());
    out.push_back(std::move(d));
    return out;
  }
  // fragmented: fragment 0 carries the channel; offsets count payload bytes only
  const std::size_t first_payload = kFragmentPayload - channel.size() - 1;
  const std::size_t rest = data.size() > first_payload ? data.size() - first_payload : 0;
  const std::uint16_t n = static_cast<std::uint16_t>(1 + (rest + kFragmentPayload - 1) / kFragmentPayload);
  std::size_t offset = 0;
  for (std::uint16_t i = 0; i < n; ++i) {
    std::vector<std::uint8_t> d;
    put32(d, kLcmMagicFragment);
    put32(d, seq);
    put32(d, static_cast<std::uint32_t>(data.size()));
    put32(d, static_cast<std::uint32_t>(offset));
    put16(d, i);
    put16(d, n);
    std::size_t take = i == 0 ? std::min(first_payload, data.size()) : std::min(kFragmentPayload, data.size() - offset);
    if (i == 0) {
      d.insert(d.end(), channel.begin(), channel.end());
      d.push_back(0);
    }
    d.insert(d.end(), data.begin() + static_cast<std::ptrdiff_t>(offset), data.begin() + static_cast<std::ptrdiff_t>(offset + take));
    offset += take;
    out.push_back(std::move(d));
  }
  return out;
}

std::optional<std::pair<std::string, std::vector<std::uint8_t>>> LcmReassembler::feed(const std::uint8_t* d, std::size_t len,
                                                                                      const std::string& sender_key) {
  if (len < 8) return std::nullopt;
  const std::uint32_t magic = get32(d);
  if (magic == kLcmMagicShort) {
    const std::uint8_t* end = d + len;
    const std::uint8_t* c = d + 8;
    const std::uint8_t* nul = static_cast<const std::uint8_t*>(std::memchr(c, 0, static_cast<std::size_t>(end - c)));
    if (!nul) return std::nullopt;
    return std::make_pair(std::string(reinterpret_cast<const char*>(c), static_cast<std::size_t>(nul - c)),
                          std::vector<std::uint8_t>(nul + 1, end));
  }
  if (magic != kLcmMagicFragment || len < 20) return std::nullopt;
  const std::uint32_t seq = get32(d + 4), size = get32(d + 8), offset = get32(d + 12);
  const std::uint16_t fragno = get16(d + 16), nfrag = get16(d + 18);
  if (nfrag == 0 || fragno >= nfrag || size > 64u * 1024u * 1024u) return std::nullopt;
  const auto now = std::chrono::steady_clock::now();
  for (auto it = partials_.begin(); it != partials_.end();)  // drop stale partials (lost fragments)
    it = (now - it->second.started > std::chrono::seconds(2)) ? partials_.erase(it) : std::next(it);
  auto key = std::make_pair(sender_key, seq);
  auto& p = partials_[key];
  if (p.data.empty()) {
    p.data.resize(size);
    p.have.assign(nfrag, false);
    p.started = now;
  }
  const std::uint8_t* payload = d + 20;
  const std::uint8_t* end = d + len;
  if (fragno == 0) {
    const std::uint8_t* nul = static_cast<const std::uint8_t*>(std::memchr(payload, 0, static_cast<std::size_t>(end - payload)));
    if (!nul) return std::nullopt;
    p.channel.assign(reinterpret_cast<const char*>(payload), static_cast<std::size_t>(nul - payload));
    payload = nul + 1;
  }
  const std::size_t n = static_cast<std::size_t>(end - payload);
  if (offset + n > size) {
    partials_.erase(key);
    return std::nullopt;
  }
  std::memcpy(p.data.data() + offset, payload, n);
  if (!p.have[fragno]) {
    p.have[fragno] = true;
    ++p.received;
  }
  if (p.received < nfrag) return std::nullopt;
  auto done = std::make_pair(p.channel, std::move(p.data));
  partials_.erase(key);
  return done;
}

}  // namespace lcm

// ----------------------------------------------------------------------------- plugin

LcmUdpTransportPlugin::LcmUdpTransportPlugin(std::string identifier, std::string config_group)
    : identifier_(std::move(identifier)), config_group_(std::move(config_group)) {}

LcmUdpTransportPlugin::~LcmUdpTransportPlugin() { stop_listening(); }

void LcmUdpTransportPlugin::init_plugin(const std::optional<std::string>&, api::Mediator* mediator) {
  mediator_ = mediator;
  if (!mediator_) {
    utils::print_message(LoggingLevel::ERROR, identifier_, "LcmUdpTransportPlugin requires a mediator.");
    return;
  }
  auto cfg = LcmTransportConfig::from_registry(*mediator_, config_group_);
  if (!cfg) {
    mediator_->log_message(LoggingLevel::ERROR, "Unable to read LcmTransportConfig from group \"" + config_group_ + "\".");
    return;
  }
  bool ok = false;
  url_ = parse_udpm_url(cfg->url, &ok);
  if (!ok)
    mediator_->log_message(LoggingLevel::WARN, "LCM url \"" + cfg->url + "\" is not udpm://; using udpm://" + url_.address + ":" +
                                                   std::to_string(url_.port) + "?ttl=" + std::to_string(url_.ttl) + ".");
  try {
    subscribe_ = std::regex(cfg->subscribe_to);
  } catch (const std::regex_error& e) {
    mediator_->log_message(LoggingLevel::ERROR, "Invalid subscribe_to regex \"" + cfg->subscribe_to + "\": " + e.what());
    subscribe_ = std::regex(".*");
  }
  idle_timeout_sec_ = cfg->idle_timeout_sec;
  if (cfg->output_file) {
    try {
      output_ = std::make_unique<lcm::LcmLogWriter>(*cfg->output_file);
    } catch (const std::exception& e) {
      mediator_->log_message(LoggingLevel::ERROR, std::string("Cannot open the output log: ") + e.what());
    }
  }
}

bool LcmUdpTransportPlugin::open_socket() {
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) return false;
  int one = 1;
  ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
#ifdef SO_REUSEPORT
  ::setsockopt(fd_, SOL_SOCKET, SO_REUSEPORT, &one, sizeof one);
#endif
  int rcvbuf = 8 * 1024 * 1024;
  ::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof rcvbuf);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(url_.port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  ip_mreq mreq{};
  mreq.imr_multiaddr.s_addr = inet_addr(url_.address.c_str());
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);
  if (::setsockopt(fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof mreq) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  unsigned char ttl = static_cast<unsigned char>(url_.ttl), loop = 1;
  ::setsockopt(fd_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof ttl);
  ::setsockopt(fd_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof loop);
  timeval tv{0, 100000};
  ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  return true;
}

void LcmUdpTransportPlugin::start_listening() {
  if (!mediator_) return;
  if (fd_ < 0 && !open_socket()) {
    mediator_->log_message(LoggingLevel::ERROR, "Cannot open the LCM multicast socket on udpm://" + url_.address + ":" +
                                                    std::to_string(url_.port) + " (" + std::strerror(errno) + ").");
    return;
  }
  stop_ = false;
  thread_ = std::thread([this] { receive_loop(); });
  mediator_->log_message(LoggingLevel::INFO, "LCM UDP transport listening on udpm://" + url_.address + ":" + std::to_string(url_.port) + ".");
}

void LcmUdpTransportPlugin::stop_listening() {
  stop_ = true;
  if (thread_.joinable()) thread_.join();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void LcmUdpTransportPlugin::shutdown_plugin() {
  stop_listening();
  {
    std::lock_guard lk(output_mutex_);
    if (output_) output_->close();
  }
  if (mediator_) mediator_->log_message(LoggingLevel::INFO, "Shutdown plugin for " + identifier_ + ".");
}

void LcmUdpTransportPlugin::receive_loop() {
  std::vector<std::uint8_t> buf(65536);
  std::optional<std::chrono::steady_clock::time_point> last_message;
  while (!stop_) {
    sockaddr_in from{};
    socklen_t fromlen = sizeof from;
    const ssize_t n = ::recvfrom(fd_, buf.data(), buf.size(), 0, reinterpret_cast<sockaddr*>(&from), &fromlen);
    if (n < 0) {
      if (idle_timeout_sec_ > 0 && last_message &&
          std::chrono::steady_clock::now() - *last_message > std::chrono::duration<double>(idle_timeout_sec_)) {
        mediator_->log_message(LoggingLevel::INFO, "No LCM message for " + std::to_string(idle_timeout_sec_) + " s; done.");
        mediator_->registry().batch("controller/flags")->set("ready_to_shutdown", true);
        last_message.reset();
      }
      continue;
    }
    const std::string sender = std::to_string(from.sin_addr.s_addr) + ":" + std::to_string(from.sin_port);
    auto msg = reassembler_.feed(buf.data(), static_cast<std::size_t>(n), sender);
    if (!msg) continue;
    last_message = std::chrono::steady_clock::now();
    handle(msg->first, msg->second);
  }
}

void LcmUdpTransportPlugin::handle(const std::string& channel, const std::vector<std::uint8_t>& data) {
  if (!std::regex_search(channel, subscribe_)) return;
  {
    std::lock_guard lk(output_mutex_);
    if (output_) output_->write(lcm::now_us(), channel, data);
  }
  std::shared_ptr<api::AspnBase> msg;
  try {
    msg = lcm::decode(data);
  } catch (const std::exception&) {
    msg = nullptr;
  }
  if (!msg) {
    if (!channels_found_.count(channel)) {
      mediator_->log_message(LoggingLevel::WARN, "Cannot decode messages on channel " + channel + " as ASPN-23; ignoring that channel.");
      channels_found_.insert(channel);
    }
    return;
  }
  if (!channels_found_.count(channel)) {
    auto t = utils::time_of_validity(*msg);
    std::ostringstream os;
    os << "Found new channel " << channel << "\t with a timestamp of " << std::fixed << std::setprecision(9) << (t ? t->seconds() : 0.0) << "s";
    mediator_->log_message(LoggingLevel::INFO, os.str());
    channels_found_.insert(channel);
  }
  mediator_->process_pntos_message(api::Message(msg, channel));
  ++received_;
}

bool LcmUdpTransportPlugin::send_raw(const std::string& channel, const std::vector<std::uint8_t>& data) {
  if (fd_ < 0) return false;
  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_port = htons(url_.port);
  to.sin_addr.s_addr = inet_addr(url_.address.c_str());
  for (const auto& dgram : lcm::lcm_datagrams(channel, data, seq_++))
    if (::sendto(fd_, dgram.data(), dgram.size(), 0, reinterpret_cast<sockaddr*>(&to), sizeof to) < 0) return false;
  ++sent_;
  return true;
}

void LcmUdpTransportPlugin::broadcast_message(const api::Message& message, const std::optional<std::string>& channel_name) {
  if (!message.wrapped_message) return;
  const std::string channel = channel_name.value_or(message.source_identifier);
  auto bytes = lcm::encode(*message.wrapped_message);
  if (!bytes) {
    if (mediator_) mediator_->log_message(LoggingLevel::WARN, "Cannot encode a message for LCM on channel " + channel + ".");
    return;
  }
  {
    std::lock_guard lk(output_mutex_);
    if (output_) output_->write(lcm::now_us(), channel, *bytes);
  }
  if (!send_raw(channel, *bytes) && mediator_) mediator_->log_message(LoggingLevel::WARN, "Failed to publish message over LCM on " + channel + ".");
}

}  // namespace pntos::cobra
