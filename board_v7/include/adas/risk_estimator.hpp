#pragma once

#include "adas/types.hpp"
#include <chrono>
#include <vector>

namespace adas {

class RiskEstimator {
 public:
  RiskResult update(const std::vector<Detection>& detections, int width, int height);

 private:
  float previous_distance_ = -1.0f;
  float smoothed_speed_ = 0.0f;
  std::chrono::steady_clock::time_point previous_time_{};
  bool initialized_ = false;
};

}  // namespace adas
