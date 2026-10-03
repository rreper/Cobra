// pntOS C++ API — fundamental value types shared by all plugins.
//
// This mirrors pntos.api (Python) / pntOS-C `plugins/common.h`. Linear algebra uses Eigen.
#pragma once

#include <Eigen/Dense>
#include <aspn23/eigen/TypeHeader.hpp>
#include <aspn23/eigen/TypeTimestamp.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace pntos::api {

// ---------------------------------------------------------------------------
// Linear algebra aliases
// ---------------------------------------------------------------------------
using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

/// A plain-old-data timestamp: whole nanoseconds since the ASPN epoch (negative = before).
/// Equivalent to aspn23 TypeTimestamp / AspnTypeTimestamp but cheap to copy and compare.
struct Timestamp {
  std::int64_t elapsed_nsec = 0;

  constexpr Timestamp() = default;
  constexpr explicit Timestamp(std::int64_t nsec) : elapsed_nsec(nsec) {}
  explicit Timestamp(const aspn23_eigen::TypeTimestamp& t) : elapsed_nsec(t.get_elapsed_nsec()) {}

  static constexpr Timestamp from_seconds(double s) {
    return Timestamp(static_cast<std::int64_t>(s * 1e9));
  }
  constexpr double seconds() const { return static_cast<double>(elapsed_nsec) * 1e-9; }
  aspn23_eigen::TypeTimestamp to_aspn() const { return aspn23_eigen::TypeTimestamp(elapsed_nsec); }

  friend constexpr bool operator==(Timestamp a, Timestamp b) { return a.elapsed_nsec == b.elapsed_nsec; }
  friend constexpr bool operator!=(Timestamp a, Timestamp b) { return a.elapsed_nsec != b.elapsed_nsec; }
  friend constexpr bool operator<(Timestamp a, Timestamp b) { return a.elapsed_nsec < b.elapsed_nsec; }
  friend constexpr bool operator<=(Timestamp a, Timestamp b) { return a.elapsed_nsec <= b.elapsed_nsec; }
  friend constexpr bool operator>(Timestamp a, Timestamp b) { return a.elapsed_nsec > b.elapsed_nsec; }
  friend constexpr bool operator>=(Timestamp a, Timestamp b) { return a.elapsed_nsec >= b.elapsed_nsec; }
};

// ---------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------

/// The common base of every ASPN-23 message in the Eigen flavour of aspn-generated.
using AspnBase = aspn23_eigen::TypeHeader;
using AspnMessageType = Aspn23MessageType;

/// A container for an ASPN message.
///
/// `wrapped_message` is either a proper ASPN message or a pntOS extension message.
/// `source_identifier` says where the message came from (a transport channel/topic when
/// available, otherwise plugin-defined).
///
/// Messages are shared, immutable-by-convention objects (see docs/concurrency.md in Cobra:
/// "all memory passed to another plugin ... must be assumed shared and must not be mutated").
/// Plugins that need to modify a message copy it first.
struct Message {
  std::shared_ptr<AspnBase> wrapped_message;
  std::string source_identifier;

  Message() = default;
  Message(std::shared_ptr<AspnBase> msg, std::string source)
      : wrapped_message(std::move(msg)), source_identifier(std::move(source)) {}

  AspnMessageType message_type() const { return wrapped_message->get_message_type(); }

  /// Downcast helper: returns nullptr if the wrapped message is not a T.
  template <class T>
  std::shared_ptr<T> as() const {
    return std::dynamic_pointer_cast<T>(wrapped_message);
  }

  /// Identity comparison (same wrapped object and source), enough for registry/variant use.
  friend bool operator==(const Message& a, const Message& b) {
    return a.wrapped_message == b.wrapped_message && a.source_identifier == b.source_identifier;
  }
  friend bool operator!=(const Message& a, const Message& b) { return !(a == b); }
};

// ---------------------------------------------------------------------------
// Estimate with covariance
// ---------------------------------------------------------------------------

/// Describes how the fields in EstimateWithCovariance are used.
enum class EstimateWithCovarianceType : int {
  /// Mean (N×1) and covariance (N×N) of a multivariate Gaussian.
  EWC_GENERIC = 0,
  /// Estimate is a 4×1 quaternion, covariance is a 3×3 tilt-error covariance (rad²).
  EWC_ATTITUDE_QUAT = 1,
};

struct EstimateWithCovariance {
  EstimateWithCovarianceType type = EstimateWithCovarianceType::EWC_GENERIC;
  Vector estimate;    ///< N×1
  Matrix covariance;  ///< N×N
};

/// Covariances relating one state block to a set of other state blocks.
/// `cross_covariances[i]` has shape [n_i, a] where n_i is the size of block `block_labels[i]`
/// and a is the size of the block being added.
struct CrossCovariances {
  std::vector<std::string> block_labels;
  std::vector<Matrix> cross_covariances;
};

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

enum class LoggingLevel : int {
  ERROR = 0,
  WARN = 1,
  INFO = 2,
  DEBUG = 3,
};

inline const char* to_string(LoggingLevel l) {
  switch (l) {
    case LoggingLevel::ERROR: return "ERROR";
    case LoggingLevel::WARN: return "WARN";
    case LoggingLevel::INFO: return "INFO";
    case LoggingLevel::DEBUG: return "DEBUG";
  }
  return "?";
}

// ---------------------------------------------------------------------------
// Plugin types (mirrors PntosPluginTypes in pntOS-C)
// ---------------------------------------------------------------------------

enum class PluginType : int {
  UNDEFINED = 0,
  CONTROLLER,
  FUSION,
  FUSION_STRATEGY,
  PLATFORM_INTEGRATION,
  INITIALIZATION,
  DATABASE,  // placeholder in the C API
  TRANSPORT,
  UI,
  ORCHESTRATION,
  ORCHESTRATION_STRATEGY,  // placeholder in the C API
  REGISTRY,
  INERTIAL,
  STATE_MODELING,
  LOGGING,
  UTILITY,
  PREPROCESSOR,
  NUM_PLUGIN_TYPES
};

inline const char* to_string(PluginType t) {
  switch (t) {
    case PluginType::CONTROLLER: return "ControllerPlugin";
    case PluginType::FUSION: return "FusionPlugin";
    case PluginType::FUSION_STRATEGY: return "FusionStrategyPlugin";
    case PluginType::PLATFORM_INTEGRATION: return "PlatformIntegrationPlugin";
    case PluginType::INITIALIZATION: return "InitializationPlugin";
    case PluginType::DATABASE: return "DatabasePlugin";
    case PluginType::TRANSPORT: return "TransportPlugin";
    case PluginType::UI: return "UiPlugin";
    case PluginType::ORCHESTRATION: return "OrchestrationPlugin";
    case PluginType::ORCHESTRATION_STRATEGY: return "OrchestrationStrategyPlugin";
    case PluginType::REGISTRY: return "RegistryPlugin";
    case PluginType::INERTIAL: return "InertialPlugin";
    case PluginType::STATE_MODELING: return "StateModelingPlugin";
    case PluginType::LOGGING: return "LoggingPlugin";
    case PluginType::UTILITY: return "UtilityPlugin";
    case PluginType::PREPROCESSOR: return "PreprocessorPlugin";
    default: return "UndefinedPlugin";
  }
}

/// Which fusion model a factory produces (mirrors PntosFusionType). Only STANDARD is defined.
enum class FusionType : int { STANDARD = 0, SAMPLED, TIME_DELAYED, STANDARD_COMPILED };

// ---------------------------------------------------------------------------
// Registry value type
// ---------------------------------------------------------------------------

/// The set of value types a KeyValueStore can hold (mirrors RegistryValueTypeUnion /
/// PntosKeyValueStoreType). Numeric arrays are stored as a 2-D double matrix; a 1-D array is
/// stored as N×1. Only the element count is guaranteed to round-trip, as in the Python API.
using StringArray = std::vector<std::string>;
using RegistryValue = std::variant<std::string, StringArray, std::int64_t, bool, double, Matrix, Message>;

enum class RegistryValueType : int {
  STR = 0,
  STR_ARRAY,
  INT,
  BOOL,
  DOUBLE,
  DOUBLE_ARRAY,
  MESSAGE,
  RAW,
  KEY_DNE,
};

inline RegistryValueType registry_value_type(const RegistryValue& v) {
  switch (v.index()) {
    case 0: return RegistryValueType::STR;
    case 1: return RegistryValueType::STR_ARRAY;
    case 2: return RegistryValueType::INT;
    case 3: return RegistryValueType::BOOL;
    case 4: return RegistryValueType::DOUBLE;
    case 5: return RegistryValueType::DOUBLE_ARRAY;
    case 6: return RegistryValueType::MESSAGE;
  }
  return RegistryValueType::KEY_DNE;
}

enum class KeyValueStoreDataFormat : int { INI = 0, UNSPECIFIED = 1 };

}  // namespace pntos::api
