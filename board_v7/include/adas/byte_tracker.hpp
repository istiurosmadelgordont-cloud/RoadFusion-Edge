#pragma once

#include "adas/types.hpp"
#include <chrono>
#include <vector>

namespace adas {

// Lightweight ByteTrack-style tracker for the edge demo. It performs the
// characteristic two-stage association: high confidence detections first,
// then low confidence detections to recover temporarily weak tracks.
class ByteTracker {
 public:
  std::vector<Detection> update(const std::vector<Detection>& detections,
                                std::chrono::steady_clock::time_point measured_at,
                                std::chrono::steady_clock::time_point now);
  std::vector<Detection> predict(std::chrono::steady_clock::time_point now) const;
  std::vector<Detection> observations() const;
  void reset();

 private:
  struct Track {
    int id = -1;
    int class_id = -1;
    std::string name;
    cv::Rect2f box;
    cv::Vec4f velocity{0, 0, 0, 0};
    float score = 0.0f;
    int missed = 0;
    int hits = 0;
    std::chrono::steady_clock::time_point measured_at{};
  };

  static cv::Rect2f project(const Track& track,
                            std::chrono::steady_clock::time_point when);
  static float iou(const cv::Rect2f& a, const cv::Rect2f& b);
  void associate(const std::vector<Detection>& detections,
                 const std::vector<int>& indices, float threshold,
                 std::chrono::steady_clock::time_point measured_at,
                 std::vector<bool>& track_used,
                 std::vector<bool>& detection_used);

  std::vector<Track> tracks_;
  int next_id_ = 1;
};

}  // namespace adas
