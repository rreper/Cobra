// C++-only hook that stands in for a Python side effect.
//
// Python preprocessors mutate the message object in place, so after an orchestration plugin has
// processed an *immediate* message the Python mediator reads the preprocessed time of validity from
// that same object and uses it for its buffer release and 1 Hz solution grid. Messages are immutable
// in the port (DESIGN.md deviation 13), so the orchestration reports the effective time explicitly
// through this interface and the mediator picks it up after process_pntos_message() returns.
#pragma once

#include <pntos/api/api.hpp>

namespace pntos::cobra {

class EffectiveTimeSink {
 public:
  virtual ~EffectiveTimeSink() = default;
  /// Time of validity the message would have carried after in-place preprocessing in Python.
  virtual void set_effective_time_of_validity(api::Timestamp tov) = 0;
};

/// No-op when the mediator is not a StandardMediator (tests, dummies).
inline void report_effective_time(api::Mediator* mediator, const std::optional<api::Timestamp>& tov) {
  if (!tov) return;
  if (auto* sink = dynamic_cast<EffectiveTimeSink*>(mediator)) sink->set_effective_time_of_validity(*tov);
}

}  // namespace pntos::cobra
