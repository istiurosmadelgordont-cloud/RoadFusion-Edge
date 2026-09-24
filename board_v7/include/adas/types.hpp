#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace adas {

enum class LaneDepartureSide { NONE, LEFT, RIGHT };

struct Detection {
  int class_id = -1;
  int track_id = -1;
  float score = 0.0f;
  cv::Rect2f box;
  std::string name;
};

struct LaneResult {
  bool valid = false;
  bool partial = false;
  bool departure = false;
  bool identity_uncertain = false;
  LaneDepartureSide departure_side = LaneDepartureSide::NONE;
  // Set by temporal geometry tracking when UFLD appears to replace the
  // original ego-lane pair with an adjacent pair in one update.
  LaneDepartureSide reassignment_side = LaneDepartureSide::NONE;
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

// What the vehicle should do, as opposed to what the light shows. Driving logic
// and the on-screen indicator react to this instead of to SignalState.
enum class DriveDecision { UNKNOWN, GO, SLOW, STOP };

struct DriveResult {
  DriveDecision decision = DriveDecision::UNKNOWN;
  SignalState signal = SignalState::NONE;
  bool held = false;  // carried over from an earlier frame, no fresh reading
};

inline const char* decision_name(DriveDecision decision) {
  switch (decision) {
    case DriveDecision::GO: return "GO";
    case DriveDecision::SLOW: return "SLOW";
    case DriveDecision::STOP: return "STOP";
    default: return "UNKNOWN";
  }
}

// Short technical captions remain ASCII; the UI translates them where needed.
inline const char* decision_caption(DriveDecision decision) {
  switch (decision) {
    case DriveDecision::GO: return "GREEN LIGHT";
    case DriveDecision::SLOW: return "YELLOW LIGHT";
    case DriveDecision::STOP: return "RED LIGHT";
    default: return "";
  }
}

struct RiskResult {
  bool target = false;
  bool warning = false;
  bool reliable = false;
  int track_id = -1;
  float distance_m = -1.0f;
  float relative_speed_kmh = 0.0f;
  float ttc_s = -1.0f;
  cv::Rect box;
};

inline const char* lane_departure_name(LaneDepartureSide side) {
  switch (side) {
    case LaneDepartureSide::LEFT: return "LDW-L";
    case LaneDepartureSide::RIGHT: return "LDW-R";
    default: return "LDW";
  }
}

inline const char* class_name(int id) {
  static const char* names[] = {
      "pedestrian", "rider", "car", "bus", "truck", "motorcycle", "bicycle",
      "traffic_red_circle", "traffic_red_left", "traffic_red_right", "traffic_red_straight",
      "traffic_green_circle", "traffic_green_left", "traffic_green_right", "traffic_green_straight",
      "traffic_sign", "crosswalk", "guide_arrows", "traffic_cone", "roadworks_sign", "delineator"};
  return (id >= 0 && id < 21) ? names[id] : "unknown";
}

}  // namespace adas
