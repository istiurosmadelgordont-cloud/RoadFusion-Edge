#pragma once

#include "adas/types.hpp"

#include <opencv2/core.hpp>
#include <vector>
#include <chrono>

namespace adas {

// Until CAN is wired, callers deliberately use visual-only demonstration mode.
struct VehicleWarningContext {
  bool valid = false;
  float speed_kmh = 0.0f;
  bool left_blinker = false;
  bool right_blinker = false;
  bool lateral_control_active = false;
};

class LaneDepartureMonitor {
 public:
  void set_reference_from_roi(const std::vector<cv::Point2f>& ordered_roi);
  LaneResult update(const LaneResult& measured,
      std::chrono::steady_clock::time_point captured_at = std::chrono::steady_clock::now(),
      const VehicleWarningContext& vehicle = VehicleWarningContext());
  void reset();

 private:
  float reference_offset_ = 0.0f;
  float filtered_offset_ = 0.0f;
  bool initialized_ = false;
  bool warning_ = false;
  int enter_streak_ = 0;
  int clear_streak_ = 0;
  int unreliable_streak_ = 0;
  std::chrono::steady_clock::time_point previous_at_{}, enter_at_{}, clear_at_{}, blinker_at_{};
  bool blinker_seen_ = false;
};

struct BlindSpotResult {
  bool occupied = false;
  int track_id = -1;
};

class BlindSpotMonitor {
 public:
  BlindSpotResult update(const std::vector<Detection>& detections,
                         int width, int height, bool fresh_measurement);
  void reset();

 private:
  bool occupied_ = false;
  int candidate_track_id_ = -1;
  int enter_streak_ = 0;
  int clear_streak_ = 0;
  BlindSpotResult last_result_;
};

}  // namespace adas
