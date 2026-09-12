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
enum class SignalDirection { NONE, CIRCLE, LEFT, RIGHT, STRAIGHT };

struct SignalResult {
  SignalState state = SignalState::NONE;
  SignalDirection direction = SignalDirection::NONE;
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

inline const char* class_name(int id, int class_count = 21) {
  static const char* names21[] = {
      "pedestrian", "rider", "car", "bus", "truck", "motorcycle", "bicycle",
      "traffic_red_circle", "traffic_red_left", "traffic_red_right", "traffic_red_straight",
      "traffic_green_circle", "traffic_green_left", "traffic_green_right", "traffic_green_straight",
      "traffic_sign", "crosswalk", "guide_arrows", "traffic_cone", "roadworks_sign", "delineator"};
  static const char* names17[] = {
      "pedestrian", "rider", "car", "bus", "truck", "motorcycle", "bicycle",
      "traffic_red", "traffic_yellow", "traffic_green", "traffic_unknown",
      "traffic_sign", "crosswalk", "guide_arrows", "traffic_cone", "roadworks_sign", "delineator"};
  if (class_count == 21) return (id >= 0 && id < 21) ? names21[id] : "unknown";
  return (id >= 0 && id < 17) ? names17[id] : "unknown";
}

}  // namespace adas
