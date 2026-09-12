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

const char* signal_direction_name(SignalDirection direction) {
  switch (direction) {
    case SignalDirection::CIRCLE: return "CIRCLE";
    case SignalDirection::LEFT: return "LEFT";
    case SignalDirection::RIGHT: return "RIGHT";
    case SignalDirection::STRAIGHT: return "STRAIGHT";
    default: return "NONE";
  }
}

SignalResult SignalLogic::update(const std::vector<Detection>& detections, int width, int height) {
  SignalResult result;
  float best_relevance = -1.0f;
  for (const Detection& detection : detections) {
    const bool directional = detection.name == "traffic_red_circle" ||
                             detection.name == "traffic_red_left" ||
                             detection.name == "traffic_red_right" ||
                             detection.name == "traffic_red_straight" ||
                             detection.name == "traffic_green_circle" ||
                             detection.name == "traffic_green_left" ||
                             detection.name == "traffic_green_right" ||
                             detection.name == "traffic_green_straight";
    const bool legacy = detection.name == "traffic_red" || detection.name == "traffic_yellow" ||
                        detection.name == "traffic_green" || detection.name == "traffic_unknown";
    if (!directional && !legacy) continue;
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
    if (legacy) {
      if (detection.name == "traffic_red") result.state = SignalState::RED;
      else if (detection.name == "traffic_yellow") result.state = SignalState::YELLOW;
      else if (detection.name == "traffic_green") result.state = SignalState::GREEN;
      else result.state = SignalState::UNKNOWN;
      result.direction = SignalDirection::NONE;
    } else {
      result.state = detection.name.find("traffic_red_") == 0 ? SignalState::RED : SignalState::GREEN;
      if (detection.name.find("_circle") != std::string::npos) result.direction = SignalDirection::CIRCLE;
      else if (detection.name.find("_left") != std::string::npos) result.direction = SignalDirection::LEFT;
      else if (detection.name.find("_right") != std::string::npos) result.direction = SignalDirection::RIGHT;
      else result.direction = SignalDirection::STRAIGHT;
    }
  }

  history_.push_back(std::make_pair(result.state, result.direction));
  if (history_.size() > 5) history_.pop_front();
  std::map<std::pair<SignalState, SignalDirection>, int> counts;
  for (const auto& value : history_) ++counts[value];
  std::pair<SignalState, SignalDirection> majority(SignalState::NONE, SignalDirection::NONE);
  int majority_count = 0;
  for (const auto& item : counts) {
    if (item.first.first != SignalState::NONE && item.second > majority_count) {
      majority = item.first;
      majority_count = item.second;
    }
  }
  result.stable = majority_count >= 3;
  if (result.stable) {
    result.state = majority.first;
    result.direction = majority.second;
  }
  return result;
}

}  // namespace adas
