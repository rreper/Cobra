#include <pntos/cobra/utils/logging.hpp>

#include <ctime>
#include <iostream>
#include <mutex>

namespace pntos::cobra::utils {
namespace {

constexpr const char* OKBLUE = "\033[94m";
constexpr const char* OKGREEN = "\033[92m";
constexpr const char* DRKGRAY = "\033[90m";
constexpr const char* LTGRAY = "\033[37m";
constexpr const char* WARNING = "\033[93m";
constexpr const char* FAIL = "\033[91m";
constexpr const char* ENDC = "\033[0m";

const char* level_color(api::LoggingLevel level) {
  switch (level) {
    case api::LoggingLevel::INFO: return OKGREEN;
    case api::LoggingLevel::WARN: return WARNING;
    case api::LoggingLevel::DEBUG: return OKBLUE;
    case api::LoggingLevel::ERROR: return FAIL;
  }
  return ENDC;
}

std::string time_str(const std::string& fmt) {
  std::time_t now = std::time(nullptr);
  std::tm tm{};
  localtime_r(&now, &tm);
  char buf[128];
  if (std::strftime(buf, sizeof buf, fmt.c_str(), &tm) == 0) return {};
  return buf;
}

std::mutex& print_mutex() {
  static std::mutex m;
  return m;
}

}  // namespace

std::string format_message(api::LoggingLevel level, const std::string& plugin_id, const std::string& message,
                           bool colorize, const std::string& date_time_format) {
  std::string out;
  if (colorize) {
    out += LTGRAY;
    out += '[' + time_str(date_time_format) + ']';
    out += ENDC;
    out += DRKGRAY;
    out += " [" + plugin_id + ']';
    out += ENDC;
    out += level_color(level);
    out += std::string(" [") + api::to_string(level) + "] ";
    out += ENDC;
  } else {
    out += '[' + time_str(date_time_format) + ']';
    out += " [" + plugin_id + ']';
    out += std::string(" [") + api::to_string(level) + "] ";
  }
  out += message;
  return out;
}

namespace {
LogSink& sink_slot() {
  static LogSink s;
  return s;
}
bool& console_slot() {
  static bool on = true;
  return on;
}
}  // namespace

void set_log_sink(LogSink sink) {
  std::lock_guard<std::mutex> lock(print_mutex());
  sink_slot() = std::move(sink);
}

void set_log_to_console(bool on) {
  std::lock_guard<std::mutex> lock(print_mutex());
  console_slot() = on;
}

void print_message(api::LoggingLevel level, const std::string& plugin_id, const std::string& message,
                   bool colorize, const std::string& date_time_format) {
  std::string line = format_message(level, plugin_id, message, colorize, date_time_format);
  std::lock_guard<std::mutex> lock(print_mutex());
  if (console_slot()) std::cout << line << '\n' << std::flush;
  if (sink_slot()) sink_slot()(level, plugin_id, message);
}

}  // namespace pntos::cobra::utils
