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
  warning_side_ = LaneDepartureSide::NONE;
  enter_streak_ = 0;
  clear_streak_ = 0;
  unreliable_streak_ = 0;
  previous_at_ = enter_at_ = clear_at_ = warning_at_ = blinker_at_ =
      std::chrono::steady_clock::time_point();
  blinker_seen_ = false;
}

LaneResult LaneDepartureMonitor::update(const LaneResult& measured,
    std::chrono::steady_clock::time_point captured_at, const VehicleWarningContext& vehicle) {
  if (initialized_ && captured_at <= previous_at_) {
    LaneResult stale = measured;
    stale.departure = warning_;
    stale.departure_side = warning_side_;
    return stale;
  }
  const float dt = initialized_ ? std::chrono::duration<float>(captured_at - previous_at_).count() : 0.0f;
  if (initialized_ && dt > 0.8f) {
    initialized_ = warning_ = false;
    warning_side_ = LaneDepartureSide::NONE;
    enter_streak_ = clear_streak_ = 0;
  }
  previous_at_ = captured_at;
  LaneResult result = measured;
  if (!result.valid || !std::isfinite(result.offset_ratio)) {
    ++unreliable_streak_;
    if (unreliable_streak_ > 3) {
      warning_ = false;
      warning_side_ = LaneDepartureSide::NONE;
      enter_streak_ = clear_streak_ = 0;
    }
    result.departure = warning_;
    result.departure_side = warning_side_;
    return result;
  }

  const float corrected = result.offset_ratio - reference_offset_;
  if (!initialized_) {
    filtered_offset_ = corrected;
    initialized_ = true;
  } else {
    const float alpha = 1.0f - std::exp(-std::max(0.0f, dt) / 0.12f);
    filtered_offset_ += alpha * (corrected - filtered_offset_);
  }
  result.offset_ratio = filtered_offset_;

  if (vehicle.valid && (vehicle.left_blinker || vehicle.right_blinker)) {
    blinker_at_ = captured_at;
    blinker_seen_ = true;
  }
  const bool recent_blinker = blinker_seen_ &&
      std::chrono::duration<float>(captured_at - blinker_at_).count() < 5.0f;
  if (vehicle.valid && (vehicle.speed_kmh < 50.0f || recent_blinker || vehicle.lateral_control_active)) {
    warning_ = false;
    warning_side_ = LaneDepartureSide::NONE;
    enter_streak_ = clear_streak_ = 0;
    result.departure = false;
    return result;
  }

  if ((result.partial || result.identity_uncertain) &&
      result.reassignment_side == LaneDepartureSide::NONE) {
    ++unreliable_streak_;
    // Weak observations cannot confirm recovery, but must not keep an old
    // departure alive indefinitely either. Expiry means unknown, not centred.
    if (unreliable_streak_ > 3 || (warning_ &&
        captured_at - warning_at_ > std::chrono::milliseconds(1200))) {
      warning_ = false;
      warning_side_ = LaneDepartureSide::NONE;
    }
    enter_streak_ = clear_streak_ = 0;
    result.departure = warning_;
    result.departure_side = warning_side_;
    return result;
  }
  unreliable_streak_ = 0;
  const float magnitude = std::abs(filtered_offset_);
  const LaneDepartureSide measured_side = filtered_offset_ < 0.0f
      ? LaneDepartureSide::LEFT : LaneDepartureSide::RIGHT;
  const LaneDepartureSide candidate_side =
      result.reassignment_side != LaneDepartureSide::NONE
          ? result.reassignment_side : measured_side;
  if (!warning_) {
    clear_streak_ = 0;
    // A lane-pair reassignment is strong evidence that the vehicle crossed a
    // boundary.  Keep the warning on the original side even if the newly
    // selected pair makes the numerical offset look centred again.
    const bool reassigned = result.reassignment_side != LaneDepartureSide::NONE;
    const bool severe = magnitude >= 0.24f;
    enter_streak_ = (reassigned || magnitude >= 0.15f) ? enter_streak_ + 1 : 0;
    if (enter_streak_ == 1) enter_at_ = captured_at;
    if (reassigned || (severe && enter_streak_ >= 2) || (enter_streak_ >= 2 &&
        std::chrono::duration<float>(captured_at - enter_at_).count() >= 0.12f)) {
      warning_ = true;
      warning_side_ = candidate_side;
      warning_at_ = captured_at;
      enter_streak_ = 0;
    }
  } else {
    enter_streak_ = 0;
    const bool minimum_hold = warning_at_ != std::chrono::steady_clock::time_point() &&
        captured_at - warning_at_ >= std::chrono::milliseconds(800);
    const bool centred = magnitude <= 0.08f &&
        result.reassignment_side == LaneDepartureSide::NONE &&
        !result.identity_uncertain;
    clear_streak_ = minimum_hold && centred ? clear_streak_ + 1 : 0;
    if (clear_streak_ == 1) clear_at_ = captured_at;
    if (clear_streak_ >= 3 && std::chrono::duration<float>(captured_at - clear_at_).count() >= 0.30f) {
      warning_ = false;
      warning_side_ = LaneDepartureSide::NONE;
      clear_streak_ = 0;
    }
  }
  result.departure = warning_;
  result.departure_side = warning_side_;
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
