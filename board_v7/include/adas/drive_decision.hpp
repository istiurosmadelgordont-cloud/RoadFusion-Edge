#pragma once

#include "adas/types.hpp"

namespace adas {

struct DriveConfig {
  // A light that stops being detected must not blank the indicator on the very
  // next frame, so the decision only falls back to UNKNOWN after this many
  // consecutive updates without a usable reading.
  int unknown_grace_updates = 8;
};

// Turns the traffic-light state into a pass/stop decision.
//
// The mapping is direct (red -> STOP, yellow -> SLOW, green -> GO) because the
// debouncing already happens upstream: SignalLogic only reports a state once a
// majority of recent frames agree, and this class ignores anything it has not
// marked stable.
class DriveDecisionLogic {
 public:
  explicit DriveDecisionLogic(const DriveConfig& config);

  DriveResult update(const SignalResult& signal);
  void reset();

 private:
  static DriveDecision from_signal(SignalState state);

  DriveConfig config_;
  DriveResult current_;
  int unknown_streak_ = 0;
};

}  // namespace adas
