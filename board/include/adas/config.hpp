#pragma once

#include <opencv2/core.hpp>
#include <string>

namespace adas {

struct DetectorConfig {
  std::string model_path = "models/unified17_v10_candidate_640_int8.rknn";
  int input_size = 640;
  int class_count = 17;
  float confidence = 0.25f;
  float nms = 0.45f;
};

struct LaneConfig {
  // Four normalized ROI points: top-left, top-right, bottom-right, bottom-left.
  cv::Point2f top_left{0.42f, 0.62f};
  cv::Point2f top_right{0.58f, 0.62f};
  cv::Point2f bottom_right{0.92f, 0.95f};
  cv::Point2f bottom_left{0.08f, 0.95f};
  float smoothing = 0.78f;
  float departure_ratio = 0.12f;
  int processing_width = 640;
  int missing_hold_updates = 15;
};

struct AppConfig {
  DetectorConfig detector;
  LaneConfig lane;
  int detection_interval = 2;
  bool display = true;
  std::string output_path;
};

}  // namespace adas
