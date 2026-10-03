// Console log formatting (port of pntos.cobra.utils.logging).
#pragma once

#include <pntos/api/types.hpp>

#include <string>

namespace pntos::cobra::utils {

/// Print `[time] [plugin_id] [LEVEL] message` to stdout, optionally ANSI-colourised.
/// `date_time_format` is a strftime format (Cobra default '%d/%m/%Y %H:%M:%S').
void print_message(api::LoggingLevel level, const std::string& plugin_id, const std::string& message,
                   bool colorize = true, const std::string& date_time_format = "%d/%m/%Y %H:%M:%S");

/// Same, but returns the formatted line instead of printing it.
std::string format_message(api::LoggingLevel level, const std::string& plugin_id, const std::string& message,
                           bool colorize = true, const std::string& date_time_format = "%d/%m/%Y %H:%M:%S");

}  // namespace pntos::cobra::utils
