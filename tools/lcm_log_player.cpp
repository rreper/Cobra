// Plays an LCM event log over UDP multicast at a chosen speed, like lcm-logplayer, without liblcm.
//   lcm_log_player input.log [--speed 1.0] [--url udpm://239.255.76.67:7667?ttl=0] [--channels a,b]
//                            [--start-delay 1.0]
// --speed 0 sends as fast as possible (expect receive-buffer loss at 100 Hz IMU rates; 20 is safe locally).
#include <pntos/cobra/transport/LcmLog.hpp>
#include <pntos/cobra/transport/LcmUdpTransportPlugin.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <set>
#include <sstream>
#include <thread>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: lcm_log_player input.log [--speed 1.0] [--url udpm://239.255.76.67:7667?ttl=0] [--channels a,b] [--start-delay s]\n";
    return 2;
  }
  double speed = 1.0, start_delay = 1.0;
  std::string url = "udpm://239.255.76.67:7667?ttl=0";
  std::optional<std::set<std::string>> channels;
  for (int i = 2; i + 1 < argc; i += 2) {
    const std::string s = argv[i];
    if (s == "--speed") speed = std::atof(argv[i + 1]);
    else if (s == "--url") url = argv[i + 1];
    else if (s == "--start-delay") start_delay = std::atof(argv[i + 1]);
    else if (s == "--channels") {
      channels.emplace();
      std::istringstream is(argv[i + 1]);
      std::string c;
      while (std::getline(is, c, ',')) channels->insert(c);
    }
  }
  using namespace pntos::cobra;
  const UdpmUrl u = parse_udpm_url(url);
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  unsigned char ttl = static_cast<unsigned char>(u.ttl), loop = 1;
  ::setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof ttl);
  ::setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof loop);
  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_port = htons(u.port);
  to.sin_addr.s_addr = inet_addr(u.address.c_str());

  lcm::LcmLogReader reader(argv[1]);
  std::this_thread::sleep_for(std::chrono::duration<double>(start_delay));
  const auto wall0 = std::chrono::steady_clock::now();
  std::optional<std::int64_t> t0;
  std::uint32_t seq = 0;
  std::size_t sent = 0;
  while (auto ev = reader.next()) {
    if (channels && !channels->count(ev->channel)) continue;
    if (!t0) t0 = ev->timestamp_us;
    if (speed > 0) {
      const auto due = wall0 + std::chrono::microseconds(static_cast<std::int64_t>((ev->timestamp_us - *t0) / speed));
      std::this_thread::sleep_until(due);
    }
    for (const auto& d : lcm::lcm_datagrams(ev->channel, ev->data, seq))
      ::sendto(fd, d.data(), d.size(), 0, reinterpret_cast<sockaddr*>(&to), sizeof to);
    ++seq;
    ++sent;
  }
  ::close(fd);
  std::cerr << "lcm_log_player: sent " << sent << " messages\n";
  return 0;
}
