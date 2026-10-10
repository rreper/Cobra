// Console log formatting (port of pntos.cobra.utils.logging).
#pragma once

#include <pntos/api/types.hpp>

#include <functional>
#include <string>

namespace pntos::cobra::utils {

/// Process-wide log sink (C++ addition): when set, every message printed through print_message is also handed
/// to `sink(level, plugin_id, message)` after printing. Called under the print lock; must not log. An empty
/// function removes the sink. Hosts that embed the filter (an appliance daemon with a web page) use it to keep
/// the recent log in memory.
using LogSink = std::function<void(api::LoggingLevel level, const std::string& plugin_id, const std::string& message)>;
void set_log_sink(LogSink sink);
/// When false, print_message only feeds the sink (a host that writes its own log). Default true.
void set_log_to_console(bool on);

/// Print `[time] [plugin_id] [LEVEL] message` to stdout, optionally ANSI-colourised.
/// `date_time_format` is a strftime format (Cobra default '%d/%m/%Y %H:%M:%S').
void print_message(api::LoggingLevel level, const std::string& plugin_id, const std::string& message,
                   bool colorize = true, const std::string& date_time_format = "%d/%m/%Y %H:%M:%S");

/// Same, but returns the formatted line instead of printing it.
std::string format_message(api::LoggingLevel level, const std::string& plugin_id, const std::string& message,
                           bool colorize = true, const std::string& date_time_format = "%d/%m/%Y %H:%M:%S");

}  // namespace pntos::cobra::utils
