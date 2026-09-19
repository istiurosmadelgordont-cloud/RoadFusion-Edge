#include "adas/warning_logic.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace adas {

void LaneDepartureMonitor::set_reference_from_roi(
    const std::vector<cv::Point2f>& ordered_roi) {
  reference_offset_ = 0.0f;
  if (ordered_roi.size() == 4) {
    const float bottom_width = ordered_roi[2].x - ordered_roi[3].x;
    if (bottom_width > 0.05f) {
      const float bottom_center = 0.5f * (ordered_roi[2].x + ordered_roi[3].x);
      reference_offset_ = (0.5f - bottom_center) / bottom_width;
    }
  }
  reset();
}

void LaneDepartureMonitor::reset() {
  filtered_offset_ = 0.0f;
  initialized_ = false;
  warning_ = false;
  enter_streak_ = 0;
  clear_streak_ = 0;
  unreliable_streak_ = 0;
}

LaneResult LaneDepartureMonitor::update(const LaneResult& measured) {
  LaneResult result = measured;
  if (!result.valid) {
    if (++unreliable_streak_ >= 3) warning_ = false;
    result.departure = false;
    return result;
  }

  const float corrected = result.offset_ratio - reference_offset_;
  if (!initialized_) {
    filtered_offset_ = corrected;
    initialized_ = true;
  } else {
    filtered_offset_ = 0.65f * filtered_offset_ + 0.35f * corrected;
  }
  result.offset_ratio = filtered_offset_;

  // A lane inferred from one visible marking is useful for drawing, but is
  // not reliable enough to create a new driver warning.
  if (result.partial) {
    enter_streak_ = 0;
    if (++unreliable_streak_ >= 3) warning_ = false;
    result.departure = warning_;
    return result;
  }

  unreliable_streak_ = 0;
  const float magnitude = std::abs(filtered_offset_);
  if (!warning_) {
    clear_streak_ = 0;
    enter_streak_ = magnitude >= 0.16f ? enter_streak_ + 1 : 0;
    if (enter_streak_ >= 3) {
      warning_ = true;
      enter_streak_ = 0;
    }
  } else {
    enter_streak_ = 0;
    clear_streak_ = magnitude <= 0.10f ? clear_streak_ + 1 : 0;
    if (clear_streak_ >= 4) {
      warning_ = false;
      clear_streak_ = 0;
    }
  }
  result.departure = warning_;
  return result;
}

void BlindSpotMonitor::reset() {
  occupied_ = false;
  candidate_track_id_ = -1;
  enter_streak_ = 0;
  clear_streak_ = 0;
  last_result_ = BlindSpotResult();
}

BlindSpotResult BlindSpotMonitor::update(
    const std::vector<Detection>& detections, int width, int height,
    bool fresh_measurement) {
  if (!fresh_measurement) return last_result_;

  const std::vector<cv::Point> region = {
      cv::Point(cvRound(width * 0.08f), cvRound(height * 0.34f)),
      cv::Point(cvRound(width * 0.92f), cvRound(height * 0.34f)),
      cv::Point(cvRound(width * 0.98f), cvRound(height * 0.98f)),
      cv::Point(cvRound(width * 0.02f), cvRound(height * 0.98f))};
  const Detection* candidate = nullptr;
  float best_area = 0.0f;
  for (const Detection& detection : detections) {
    if (detection.class_id < 1 || detection.class_id > 6 || detection.score < 0.25f) continue;
    const cv::Point2f contact(detection.box.x + detection.box.width * 0.5f,
                              detection.box.y + detection.box.height);
    const float area = detection.box.area() /
        std::max(1.0f, static_cast<float>(width * height));
    if (area < 0.006f || cv::pointPolygonTest(region, contact, false) < 0.0) continue;
    if (area > best_area) {
      best_area = area;
      candidate = &detection;
    }
  }

  if (candidate) {
    clear_streak_ = 0;
    if (candidate->track_id >= 0 && candidate->track_id != candidate_track_id_) {
      candidate_track_id_ = candidate->track_id;
      enter_streak_ = 1;
    } else {
      ++enter_streak_;
    }
    if (enter_streak_ >= 2) occupied_ = true;
  } else {
    enter_streak_ = 0;
    if (++clear_streak_ >= 3) {
      occupied_ = false;
      candidate_track_id_ = -1;
      clear_streak_ = 0;
    }
  }

  last_result_.occupied = occupied_;
  last_result_.track_id = candidate_track_id_;
  return last_result_;
}

}  // namespace adas
