#include "adas/signal_logic.hpp"

#include <algorithm>
#include <cmath>
#include <map>

namespace adas {

const char* signal_name(SignalState state) {
  switch (state) {
    case SignalState::RED: return "RED";
    case SignalState::YELLOW: return "YELLOW";
    case SignalState::GREEN: return "GREEN";
    case SignalState::UNKNOWN: return "UNKNOWN";
    default: return "NONE";
  }
}

SignalResult SignalLogic::update(const std::vector<Detection>& detections, int width, int height) {
  SignalResult result;
  float best_relevance = -1.0f;
  for (const Detection& detection : detections) {
    if (detection.class_id < 7 || detection.class_id > 10) continue;
    const float cx = detection.box.x + detection.box.width * 0.5f;
    const float cy = detection.box.y + detection.box.height * 0.5f;
    if (cy > height * 0.68f || cx < width * 0.12f || cx > width * 0.88f) continue;
    const float center_penalty = std::abs(cx - width * 0.5f) / (width * 0.5f);
    const float size_bonus = std::min(0.25f, detection.box.height / height);
    const float relevance = detection.score + size_bonus - center_penalty * 0.42f;
    if (relevance <= best_relevance) continue;
    best_relevance = relevance;
    result.box = detection.box;
    result.score = detection.score;
    if (detection.class_id == 7) result.state = SignalState::RED;
    else if (detection.class_id == 8) result.state = SignalState::YELLOW;
    else if (detection.class_id == 9) result.state = SignalState::GREEN;
    else result.state = SignalState::UNKNOWN;
  }

  history_.push_back(result.state);
  if (history_.size() > 5) history_.pop_front();
  std::map<SignalState, int> counts;
  for (SignalState value : history_) ++counts[value];
  SignalState majority = SignalState::NONE;
  int majority_count = 0;
  for (const auto& item : counts) {
    if (item.first != SignalState::NONE && item.second > majority_count) {
      majority = item.first;
      majority_count = item.second;
    }
  }
  result.stable = majority_count >= 3;
  if (result.stable) result.state = majority;
  return result;
}

}  // namespace adas
