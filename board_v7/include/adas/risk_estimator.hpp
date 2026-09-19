#pragma once

#include "adas/types.hpp"
#include <chrono>
#include <vector>

namespace adas {

struct RiskConfig {
  float focal_scale = 0.72f;
  float warning_distance_m = 8.0f;
  float clear_distance_m = 10.0f;
  float warning_ttc_s = 3.0f;
  float clear_ttc_s = 4.2f;
  int enter_updates = 2;
  int clear_updates = 3;
};

class RiskEstimator {
 public:
  explicit RiskEstimator(const RiskConfig& config = RiskConfig());
  RiskResult update(const std::vector<Detection>& detections, int width, int height,
                    const LaneResult* lane = nullptr, bool fresh_measurement = true);
  void reset();

 private:
  RiskConfig config_;
  float previous_distance_ = -1.0f;
  float filtered_distance_ = -1.0f;
  float smoothed_speed_ = 0.0f;
  std::chrono::steady_clock::time_point previous_time_{};
  bool initialized_ = false;
  bool warning_ = false;
  int target_track_id_ = -1;
  int target_hits_ = 0;
  int target_misses_ = 0;
  int warning_streak_ = 0;
  int clear_streak_ = 0;
  RiskResult last_result_;
};

}  // namespace adas
