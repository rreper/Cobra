// ASPN-23 LCM wire format <-> aspn23_eigen messages (C++ counterpart of aspn23_lcm_conversions,
// for the types Cobra's apps use: IMU, position, velocity, PVA, altitude, barometer).
#pragma once

#include <pntos/api/types.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace pntos::cobra::lcm {

/// Decode an LCM-encoded ASPN message (first 8 bytes = type fingerprint). nullptr if the type is
/// unsupported; throws std::runtime_error on a malformed payload.
std::shared_ptr<api::AspnBase> decode(const std::uint8_t* data, std::size_t len);
inline std::shared_ptr<api::AspnBase> decode(const std::vector<std::uint8_t>& d) { return decode(d.data(), d.size()); }

/// Encode an aspn23_eigen message to LCM bytes. nullopt if the type is unsupported.
std::optional<std::vector<std::uint8_t>> encode(const api::AspnBase& msg);

/// Type name for a fingerprint ("measurement_IMU", ...), or nullopt.
std::optional<std::string> type_name_for(const std::uint8_t* data, std::size_t len);

}  // namespace pntos::cobra::lcm
