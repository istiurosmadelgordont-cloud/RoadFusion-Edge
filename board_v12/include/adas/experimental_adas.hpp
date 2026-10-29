#pragma once

#include "adas/types.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

namespace adas {

// Independent implementation of Autoware's ROI/HSV/UNKNOWN design. This
// verifies color only; arrow/circle shape remains the YOLO prediction.
inline SignalState roi_light_color(const cv::Mat& bgr, const cv::Rect2f& box) {
  if (bgr.empty() || bgr.type() != CV_8UC3 || !std::isfinite(box.x) ||
      !std::isfinite(box.y) || !std::isfinite(box.width) ||
      !std::isfinite(box.height) || box.width < 4 || box.height < 4) {
    return SignalState::UNKNOWN;
  }
  const float x0 = std::max(0.0f, box.x);
  const float y0 = std::max(0.0f, box.y);
  const float x1 = std::min(float(bgr.cols), box.x + box.width);
  const float y1 = std::min(float(bgr.rows), box.y + box.height);
  if (x1 - x0 < 4 || y1 - y0 < 4) return SignalState::UNKNOWN;
  const cv::Rect roi(int(x0), int(y0), int(x1 - x0), int(y1 - y0));
  cv::Mat crop = bgr(roi);
  cv::Mat reduced;
  cv::Mat hsv;
  if (std::max(crop.cols, crop.rows) > 96) {
    const double scale = 96.0 / std::max(crop.cols, crop.rows);
    cv::resize(crop, reduced, cv::Size(), scale, scale, cv::INTER_AREA);
    crop = reduced;
  }
  cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);
  std::array<int, 3> votes{{0, 0, 0}};
  int white = 0;
  for (int y = 0; y < hsv.rows; ++y) {
    for (int x = 0; x < hsv.cols; ++x) {
      const auto p = hsv.at<cv::Vec3b>(y, x);
      if (p[1] < 35 && p[2] > 245) ++white;
      if (p[1] < 90 || p[2] < 140) continue;
      if (p[0] <= 10 || p[0] >= 170)
        ++votes[0];
      else if (p[0] >= 18 && p[0] <= 38)
        ++votes[1];
      else if (p[0] >= 40 && p[0] <= 100)
        ++votes[2];
    }
  }
  const int total = hsv.rows * hsv.cols;
  const int best = static_cast<int>(
      std::max_element(votes.begin(), votes.end()) - votes.begin());
  const int others = votes[(best + 1) % 3] + votes[(best + 2) % 3];
  if (white > total * 0.65 || votes[best] < std::max(4, total / 50) ||
      votes[best] < 3 * others) {
    return SignalState::UNKNOWN;
  }
  if (best == 0) return SignalState::RED;
  if (best == 1) return SignalState::YELLOW;
  return SignalState::GREEN;
}

inline std::vector<Detection> verify_light_colors(
    const cv::Mat& frame, const std::vector<Detection>& detections,
    int* checked = nullptr, int* rejected = nullptr) {
  std::vector<Detection> accepted;
  int count = 0;
  int veto = 0;
  for (const auto& d : detections) {
    if (d.class_id < 7 || d.class_id > 14 || d.score < 0.4f) continue;
    ++count;
    const auto color = roi_light_color(frame, d.box);
    const auto expected =
        d.class_id <= 10 ? SignalState::RED : SignalState::GREEN;
    if (color == expected)
      accepted.push_back(d);
    else
      ++veto;  // Never convert a contradictory red into a GO candidate.
  }
  if (checked) *checked = count;
  if (rejected) *rejected = veto;
  return accepted;
}

enum class Intent { NONE, LEFT, RIGHT };
enum class LaneChangeAdvice { IDLE, UNKNOWN, CHECK, BLOCKED };
// Advisory subset inspired by openpilot desire_helper: directional blindspot
// gating, explicit intent and timeout. No steering command or change approval.
class LaneChangeAdvisor {
 public:
  void set_intent(Intent intent, std::chrono::steady_clock::time_point now) {
    intent_ = intent;
    since_ = now;
    clear_pending_ = false;
    state_ = intent == Intent::NONE ? LaneChangeAdvice::IDLE
                                    : LaneChangeAdvice::UNKNOWN;
  }

  Intent intent() const { return intent_; }

  LaneChangeAdvice update(bool left, bool right, bool side_fresh,
                          bool lane_valid,
                          std::chrono::steady_clock::time_point now) {
    if (intent_ == Intent::NONE ||
        now - since_ >= std::chrono::seconds(10)) {
      set_intent(Intent::NONE, now);
      return state_;
    }
    if (!side_fresh) {
      clear_pending_ = false;
      state_ = LaneChangeAdvice::UNKNOWN;
      return state_;
    }
    if (intent_ == Intent::LEFT ? left : right) {
      clear_pending_ = false;
      state_ = LaneChangeAdvice::BLOCKED;
      return state_;
    }
    if (state_ == LaneChangeAdvice::BLOCKED) {
      if (!clear_pending_) {
        clear_pending_ = true;
        clear_since_ = now;
      }
      if (now - clear_since_ < std::chrono::milliseconds(500)) return state_;
    }
    state_ = lane_valid ? LaneChangeAdvice::CHECK : LaneChangeAdvice::UNKNOWN;
    return state_;
  }

 private:
  Intent intent_ = Intent::NONE;
  LaneChangeAdvice state_ = LaneChangeAdvice::IDLE;
  bool clear_pending_ = false;
  std::chrono::steady_clock::time_point since_{};
  std::chrono::steady_clock::time_point clear_since_{};
};

inline const char* lane_change_caption(LaneChangeAdvice state) {
  switch (state) {
    case LaneChangeAdvice::BLOCKED: return "目标侧盲区占用，暂勿变道";
    case LaneChangeAdvice::CHECK: return "未见盲区目标，仍需确认线型与路况";
    case LaneChangeAdvice::UNKNOWN: return "感知不足，无法判断变道条件";
    default: return "模拟变道提示：未请求";
  }
}

}  // namespace adas
