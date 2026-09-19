#include "adas/drive_decision.hpp"

namespace adas {

DriveDecisionLogic::DriveDecisionLogic(const DriveConfig& config) : config_(config) {}

void DriveDecisionLogic::reset() {
  current_ = DriveResult();
  unknown_streak_ = 0;
}

DriveDecision DriveDecisionLogic::from_signal(SignalState state) {
  switch (state) {
    case SignalState::RED: return DriveDecision::STOP;
    case SignalState::YELLOW: return DriveDecision::SLOW;
    case SignalState::GREEN: return DriveDecision::GO;
    default: return DriveDecision::UNKNOWN;
  }
}

DriveResult DriveDecisionLogic::update(const SignalResult& signal) {
  current_.signal = signal.state;

  // SignalLogic has not reached a majority yet, so keep the previous decision
  // rather than reacting to a single uncertain frame.
  if (!signal.stable) {
    current_.held = current_.decision != DriveDecision::UNKNOWN;
    return current_;
  }

  const DriveDecision candidate = from_signal(signal.state);
  if (candidate == DriveDecision::UNKNOWN) {
    if (++unknown_streak_ >= config_.unknown_grace_updates) {
      current_.decision = DriveDecision::UNKNOWN;
      current_.held = false;
    } else {
      current_.held = current_.decision != DriveDecision::UNKNOWN;
    }
  } else {
    unknown_streak_ = 0;
    current_.decision = candidate;
    current_.held = false;
  }
  return current_;
}

}  // namespace adas
