#pragma once

#include "adas/types.hpp"

#include <opencv2/core.hpp>
#include <vector>

namespace adas {

class LaneDepartureMonitor {
 public:
  void set_reference_from_roi(const std::vector<cv::Point2f>& ordered_roi);
  LaneResult update(const LaneResult& measured);
  void reset();

 private:
  float reference_offset_ = 0.0f;
  float filtered_offset_ = 0.0f;
  bool initialized_ = false;
  bool warning_ = false;
  int enter_streak_ = 0;
  int clear_streak_ = 0;
  int unreliable_streak_ = 0;
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
