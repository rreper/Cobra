#include <pntos/cobra/transport/LcmLog.hpp>

#include <chrono>
#include <stdexcept>

namespace pntos::cobra::lcm {

namespace {
constexpr std::uint32_t kSync = 0xEDA1DA01u;

template <class T>
bool read_be(std::istream& in, T& out) {
  std::uint8_t b[sizeof(T)];
  if (!in.read(reinterpret_cast<char*>(b), sizeof(T))) return false;
  std::uint64_t v = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) v = (v << 8) | b[i];
  out = static_cast<T>(v);
  return true;
}
template <class T>
void write_be(std::ostream& out, T value) {
  std::uint8_t b[sizeof(T)];
  auto v = static_cast<std::uint64_t>(value);
  for (std::size_t i = sizeof(T); i-- > 0;) {
    b[i] = static_cast<std::uint8_t>(v & 0xFF);
    v >>= 8;
  }
  out.write(reinterpret_cast<const char*>(b), sizeof(T));
}
}  // namespace

LcmLogReader::LcmLogReader(const std::string& path) : in_(path, std::ios::binary) {
  if (!in_) throw std::runtime_error("LcmLogReader: cannot open " + path);
  in_.seekg(0, std::ios::end);
  size_ = static_cast<std::uint64_t>(in_.tellg());
  in_.seekg(0, std::ios::beg);
}

std::optional<LcmEvent> LcmLogReader::next() {
  // Scan for the sync word byte by byte so a corrupt event does not end the replay.
  std::uint32_t sync = 0;
  int have = 0;
  while (true) {
    int c = in_.get();
    if (c == std::char_traits<char>::eof()) return std::nullopt;
    sync = (sync << 8) | static_cast<std::uint32_t>(c);
    if (have < 4) ++have;
    if (have == 4 && sync == kSync) break;
  }
  LcmEvent ev;
  std::int32_t channel_len = 0, data_len = 0;
  if (!read_be(in_, ev.event_number) || !read_be(in_, ev.timestamp_us) || !read_be(in_, channel_len) ||
      !read_be(in_, data_len))
    return std::nullopt;
  if (channel_len < 0 || data_len < 0) return next();
  ev.channel.resize(static_cast<std::size_t>(channel_len));
  ev.data.resize(static_cast<std::size_t>(data_len));
  if (channel_len && !in_.read(ev.channel.data(), channel_len)) return std::nullopt;
  if (data_len && !in_.read(reinterpret_cast<char*>(ev.data.data()), data_len)) return std::nullopt;
  return ev;
}

LcmLogWriter::LcmLogWriter(const std::string& path) : out_(path, std::ios::binary | std::ios::trunc) {
  if (!out_) throw std::runtime_error("LcmLogWriter: cannot open " + path);
}

void LcmLogWriter::write(std::int64_t timestamp_us, const std::string& channel, const std::uint8_t* data,
                         std::size_t len) {
  write_be(out_, kSync);
  write_be(out_, next_event_++);
  write_be(out_, timestamp_us);
  write_be(out_, static_cast<std::int32_t>(channel.size()));
  write_be(out_, static_cast<std::int32_t>(len));
  out_.write(channel.data(), static_cast<std::streamsize>(channel.size()));
  out_.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(len));
}

void LcmLogWriter::close() {
  if (out_.is_open()) out_.close();
}

std::int64_t now_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace pntos::cobra::lcm
