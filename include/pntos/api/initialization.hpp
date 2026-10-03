// pntOS C++ API — initialization (alignment) strategies and InitializationPlugin (unstable upstream).
#pragma once

#include <pntos/api/inertial.hpp>

namespace pntos::api {

enum class InitializationStatus : int {
  WAITING = 0,
  INITIALIZING_COARSE = 1,
  INITIALIZING_FINE = 2,
  INITIALIZED_GOOD = 3,
  INITIALIZATION_FAILED = 4,
};

enum class InitializationMotionNeeded : int {
  NO_MOTION = 0,
  MOTION_NEEDED = 1,
  ANY_MOTION = 2,
};

enum class InitializationType : int {
  INERTIAL = 0,
  EWC = 1,
};

class CommonInitializationStrategy {
 public:
  virtual ~CommonInitializationStrategy() = default;

  virtual InitializationMotionNeeded request_motion_needed() const = 0;
  virtual InitializationStatus request_current_status() const = 0;
  virtual void process_pntos_message(const Message& message) = 0;
};

/// Initial PVA + inertial errors + their covariance + status, coupled to avoid TOCTOU.
struct InitialInertialSolution {
  std::optional<Message> solution;
  std::optional<StandardInertialErrors> inertial_errors;
  std::optional<Matrix> inertial_error_covariance;  ///< 6×6 (accel biases, gyro biases)
  InitializationStatus status = InitializationStatus::WAITING;
};

class InertialInitializationStrategy : public CommonInitializationStrategy {
 public:
  virtual InitialInertialSolution request_solution() = 0;
};

struct InitialEstimateWithCovariance {
  Timestamp time;
  std::optional<EstimateWithCovariance> estimate_with_covariance;
  InitializationStatus status = InitializationStatus::WAITING;
};

class EwcInitializationStrategy : public CommonInitializationStrategy {
 public:
  virtual std::optional<InitialEstimateWithCovariance> request_solution() = 0;
};

/// Factory for initialization strategies.
class InitializationPlugin : public CommonPlugin {
 public:
  PluginType plugin_type() const override { return PluginType::INITIALIZATION; }

  virtual bool is_initialization_type_supported(InitializationType type) const = 0;
  /// nullptr if unsupported or the config group is invalid.
  virtual std::unique_ptr<CommonInitializationStrategy> new_initialization_strategy(
      InitializationType type, const std::optional<std::string>& config_group = std::nullopt) = 0;
};

}  // namespace pntos::api
