// Minimal LCM event-log reader/writer (the ".lcmlog" format). No liblcm needed.
//
// Format, all big-endian: 0xEDA1DA01 sync | int64 event number | int64 timestamp (µs) |
// int32 channel length | int32 data length | channel bytes | data bytes.
#pragma once

#include <cstdint>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace pntos::cobra::lcm {

struct LcmEvent {
  std::int64_t event_number = 0;
  std::int64_t timestamp_us = 0;
  std::string channel;
  std::vector<std::uint8_t> data;
};

class LcmLogReader {
 public:
  /// Throws std::runtime_error if the file cannot be opened.
  explicit LcmLogReader(const std::string& path);
  /// Next event, or nullopt at end of file. Resynchronises on a bad sync word.
  std::optional<LcmEvent> next();
  std::uint64_t size() const { return size_; }
  std::uint64_t tell() { return static_cast<std::uint64_t>(in_.tellg()); }

 private:
  std::ifstream in_;
  std::uint64_t size_ = 0;
};

class LcmLogWriter {
 public:
  /// Truncates/creates `path`. Throws std::runtime_error if it cannot be opened.
  explicit LcmLogWriter(const std::string& path);
  void write(std::int64_t timestamp_us, const std::string& channel, const std::uint8_t* data, std::size_t len);
  void write(std::int64_t timestamp_us, const std::string& channel, const std::vector<std::uint8_t>& data) {
    write(timestamp_us, channel, data.data(), data.size());
  }
  void close();
  std::int64_t events_written() const { return next_event_; }

 private:
  std::ofstream out_;
  std::int64_t next_event_ = 0;
};

/// Wall-clock microseconds (what the Python transport stamps output events with).
std::int64_t now_us();

}  // namespace pntos::cobra::lcm
