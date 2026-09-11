#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace adas {

struct Detection {
  int class_id = -1;
  float score = 0.0f;
  cv::Rect2f box;
  std::string name;
};

struct LaneResult {
  bool valid = false;
  bool partial = false;
  bool departure = false;
  float offset_ratio = 0.0f;
  std::vector<cv::Point> left;
  std::vector<cv::Point> right;
  std::vector<cv::Point> polygon;
  cv::Mat binary;
  cv::Mat bird_eye;
  cv::Mat curve_fit;
};

enum class SignalState { NONE, RED, YELLOW, GREEN, UNKNOWN };

struct SignalResult {
  SignalState state = SignalState::NONE;
  bool stable = false;
  cv::Rect box;
  float score = 0.0f;
};

struct RiskResult {
  bool target = false;
  bool warning = false;
  float distance_m = -1.0f;
  float relative_speed_kmh = 0.0f;
  float ttc_s = -1.0f;
  cv::Rect box;
};

inline const char* class_name(int id) {
  static const char* names[] = {
      "pedestrian", "rider", "car", "bus", "truck", "motorcycle", "bicycle",
      "traffic_red", "traffic_yellow", "traffic_green", "traffic_unknown",
      "traffic_sign", "crosswalk", "guide_arrows", "traffic_cone",
      "roadworks_sign", "delineator"};
  return (id >= 0 && id < 17) ? names[id] : "unknown";
}

}  // namespace adas
