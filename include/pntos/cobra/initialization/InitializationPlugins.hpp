// Ports of Cobra's inertial initialization plugins: TutorialInitializationPlugin (manual),
// StaticAlignInitializationPlugin, ManualHeadingAlignInitializationPlugin, PvaMessageInitializationPlugin.
#pragma once

#include <pntos/api/initialization.hpp>
#include <pntos/cobra/config/configs.hpp>
#include <pntos/cobra/initialization/Alignment.hpp>

namespace pntos::cobra {

/// Immediately INITIALIZED_GOOD with the solution described by a ManualAlignmentConfig.
class ManualInitialization final : public api::InertialInitializationStrategy {
 public:
  /// Throws std::invalid_argument if the config group cannot be read.
  ManualInitialization(const std::string& config_group, api::Mediator* mediator);
  api::InitializationMotionNeeded request_motion_needed() const override {
    return api::InitializationMotionNeeded::ANY_MOTION;
  }
  api::InitializationStatus request_current_status() const override { return api::InitializationStatus::INITIALIZED_GOOD; }
  void process_pntos_message(const api::Message&) override {}
  api::InitialInertialSolution request_solution() override { return solution_; }
  static constexpr const char* kSourceIdentifier = "Cobra simple initialization";

 private:
  api::InitialInertialSolution solution_;
};

/// Adapts a NavToolkit-style AlignBase to the pntOS strategy interface (Python StaticAlign /
/// ManualHeadingAlign share this shape).
class AlignmentStrategy final : public api::InertialInitializationStrategy {
 public:
  explicit AlignmentStrategy(std::unique_ptr<inertial::AlignBase> aligner, api::Mediator* mediator);
  api::InitializationMotionNeeded request_motion_needed() const override;
  api::InitializationStatus request_current_status() const override;
  void process_pntos_message(const api::Message& message) override;
  api::InitialInertialSolution request_solution() override;
  static constexpr const char* kSourceIdentifier = "Cobra initializer";
  inertial::AlignBase& aligner() { return *aligner_; }

 private:
  std::unique_ptr<inertial::AlignBase> aligner_;
  api::Mediator* mediator_;
};

/// Takes the first PVA on a channel (after an optional start time) as the initial solution.
class PvaMessageInitialization final : public api::InertialInitializationStrategy {
 public:
  PvaMessageInitialization(api::Mediator* mediator, const PvaMessageInitializationConfig& cfg);
  api::InitializationMotionNeeded request_motion_needed() const override {
    return api::InitializationMotionNeeded::ANY_MOTION;
  }
  api::InitializationStatus request_current_status() const override { return status_; }
  void process_pntos_message(const api::Message& message) override;
  api::InitialInertialSolution request_solution() override { return solution_; }

 private:
  api::Mediator* mediator_;
  std::string channel_;
  std::optional<std::int64_t> start_time_ns_;
  std::optional<api::Matrix> pva_cov_;
  api::Matrix imu_error_cov_;
  api::InitializationStatus status_ = api::InitializationStatus::WAITING;
  api::InitialInertialSolution solution_;
};

/// Common plugin shell: `make` builds the strategy from a config group.
class InitializationPluginBase : public api::InitializationPlugin {
 public:
  explicit InitializationPluginBase(std::string identifier) : identifier_(std::move(identifier)) {}
  void init_plugin(const std::optional<std::string>&, api::Mediator* mediator) override { mediator_ = mediator; }
  void shutdown_plugin() override {}
  const std::string& identifier() const override { return identifier_; }
  bool is_initialization_type_supported(api::InitializationType t) const override {
    return t == api::InitializationType::INERTIAL;
  }
  std::unique_ptr<api::CommonInitializationStrategy> new_initialization_strategy(
      api::InitializationType type, const std::optional<std::string>& config_group) override;

 protected:
  virtual std::unique_ptr<api::CommonInitializationStrategy> make(const std::string& group) = 0;
  bool config_group_ok(const std::optional<std::string>& group) const;
  std::string identifier_;
  api::Mediator* mediator_ = nullptr;
};

class TutorialInitializationPlugin final : public InitializationPluginBase {
 public:
  using InitializationPluginBase::InitializationPluginBase;

 protected:
  std::unique_ptr<api::CommonInitializationStrategy> make(const std::string& group) override;
};
using ManualAlignInitializationPlugin = TutorialInitializationPlugin;

class StaticAlignInitializationPlugin final : public InitializationPluginBase {
 public:
  using InitializationPluginBase::InitializationPluginBase;

 protected:
  std::unique_ptr<api::CommonInitializationStrategy> make(const std::string& group) override;
};

class ManualHeadingAlignInitializationPlugin final : public InitializationPluginBase {
 public:
  using InitializationPluginBase::InitializationPluginBase;

 protected:
  std::unique_ptr<api::CommonInitializationStrategy> make(const std::string& group) override;
};

class PvaMessageInitializationPlugin final : public InitializationPluginBase {
 public:
  using InitializationPluginBase::InitializationPluginBase;

 protected:
  std::unique_ptr<api::CommonInitializationStrategy> make(const std::string& group) override;
};

}  // namespace pntos::cobra
