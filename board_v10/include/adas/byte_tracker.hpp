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
  // 为经过 HSV 筛选的原始观测补上 ID；不改变框、类别、分数或观测数量。
  std::vector<Detection> identify_measurements(const std::vector<Detection>& detections) const;
  void reset();

 private:
  struct Track {
    int id = -1;
    int class_id = -1;
    std::string name;
    cv::Rect2f box;
    // Rendering only. Association and observations keep the unsmoothed measurement.
    cv::Rect2f display_box;
    cv::Vec4f velocity{0, 0, 0, 0};
    float score = 0.0f;
    int missed = 0;
    int hits = 0;
    std::chrono::steady_clock::time_point measured_at{};
    // Display lifetime starts when a result arrives; motion/evidence keep source time.
    std::chrono::steady_clock::time_point received_at{};
    int source_housing_index = -1;
    int parent_track_id = -1;
    // 小灯相对于同一灯组的归一化 x/y/宽/高，仅用于几何跟踪。
    cv::Rect2f relative_box;
  };

  static cv::Rect2f project(const Track& track,
                            std::chrono::steady_clock::time_point when);
  static cv::Rect2f project_display(const Track& track,
                                  std::chrono::steady_clock::time_point when);
  static float iou(const cv::Rect2f& a, const cv::Rect2f& b);
  const Track* parent_of(const Track& track,
                         std::chrono::steady_clock::time_point when) const;
  cv::Rect2f display_box_at(const Track& track,
                            std::chrono::steady_clock::time_point when) const;
  void associate(const std::vector<Detection>& detections,
                 const std::vector<int>& indices, float threshold,
                 std::chrono::steady_clock::time_point measured_at,
                 std::chrono::steady_clock::time_point received_at,
                 std::vector<bool>& track_used,
                 std::vector<bool>& detection_used,
                 std::vector<int>& detection_track_ids,
                 const std::vector<int>& parent_ids);

  std::vector<Track> tracks_;
  int next_id_ = 1;
};

}  // namespace adas
