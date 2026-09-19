#include "adas/risk_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace adas {
namespace {

bool is_obstacle(int id) { return id >= 0 && id <= 6; }

float physical_height(int id) {
  if (id == 0 || id == 1) return 1.70f;
  if (id == 3 || id == 4) return 2.70f;
  if (id == 5 || id == 6) return 1.35f;
  return 1.50f;
}

}  // namespace

RiskEstimator::RiskEstimator(const RiskConfig& config) : config_(config) {}

void RiskEstimator::reset() {
  previous_distance_ = -1.0f;
  filtered_distance_ = -1.0f;
  smoothed_speed_ = 0.0f;
  previous_time_ = std::chrono::steady_clock::time_point();
  initialized_ = false;
  warning_ = false;
  target_track_id_ = -1;
  target_hits_ = target_misses_ = 0;
  warning_streak_ = clear_streak_ = 0;
  last_result_ = RiskResult();
}

RiskResult RiskEstimator::update(const std::vector<Detection>& detections,
                                 int width, int height,
                                 const LaneResult* lane,
                                 bool fresh_measurement) {
  if (!fresh_measurement) return last_result_;

  std::vector<const Detection*> candidates;
  for (const Detection& detection : detections) {
    if (!is_obstacle(detection.class_id) || detection.score < 0.25f) continue;
    const float cx = detection.box.x + detection.box.width * 0.5f;
    const float bottom = detection.box.y + detection.box.height;
    bool inside = false;
    if (lane && lane->valid && !lane->polygon.empty()) {
      inside = cv::pointPolygonTest(lane->polygon, cv::Point2f(cx, bottom), false) >= 0.0;
    } else {
      const float corridor_half = width * (0.07f + 0.17f * bottom / height);
      inside = std::abs(cx - width * 0.5f) <= corridor_half && bottom >= height * 0.42f;
    }
    if (inside) candidates.push_back(&detection);
  }

  const Detection* best = nullptr;
  if (target_track_id_ >= 0) {
    for (const Detection* candidate : candidates) {
      if (candidate->track_id == target_track_id_) { best = candidate; break; }
    }
    if (!best && ++target_misses_ <= 2) return last_result_;
  }
  if (!best) {
    float best_rank = -1.0f;
    for (const Detection* candidate : candidates) {
      const float bottom = candidate->box.y + candidate->box.height;
      const float rank = bottom / height + candidate->box.height / height +
                         candidate->score * 0.20f;
      if (rank > best_rank) { best_rank = rank; best = candidate; }
    }
  }
  if (!best) {
    if (++target_misses_ <= 2) return last_result_;
    reset();
    return RiskResult();
  }

  const bool changed = target_track_id_ >= 0 && best->track_id >= 0 &&
                       target_track_id_ != best->track_id;
  if (changed) {
    previous_distance_ = filtered_distance_ = -1.0f;
    smoothed_speed_ = 0.0f;
    initialized_ = warning_ = false;
    warning_streak_ = clear_streak_ = target_hits_ = 0;
  }
  target_track_id_ = best->track_id;
  target_misses_ = 0;
  ++target_hits_;

  const auto now = std::chrono::steady_clock::now();
  const float focal_px = width * config_.focal_scale;
  float raw_distance = focal_px * physical_height(best->class_id) /
      std::max(8.0f, best->box.height);
  raw_distance = std::max(1.0f, std::min(raw_distance, 150.0f));
  if (!initialized_) {
    filtered_distance_ = previous_distance_ = raw_distance;
    previous_time_ = now;
    initialized_ = true;
  } else {
    const float dt = std::chrono::duration<float>(now - previous_time_).count();
    if (dt > 0.05f && dt < 2.0f) {
      const float max_jump = std::max(2.5f, filtered_distance_ * 0.22f);
      const float bounded = std::max(filtered_distance_ - max_jump,
          std::min(raw_distance, filtered_distance_ + max_jump));
      const float next_distance = 0.68f * filtered_distance_ + 0.32f * bounded;
      float closing_mps = (filtered_distance_ - next_distance) / dt;
      closing_mps = std::max(-15.0f, std::min(closing_mps, 40.0f));
      smoothed_speed_ = 0.72f * smoothed_speed_ + 0.28f * closing_mps;
      previous_distance_ = filtered_distance_;
      filtered_distance_ = next_distance;
    }
    previous_time_ = now;
  }

  RiskResult result;
  result.target = true;
  result.track_id = target_track_id_;
  result.box = best->box;
  result.distance_m = filtered_distance_;
  result.reliable = target_hits_ >= 2;
  result.relative_speed_kmh = smoothed_speed_ * 3.6f;
  if (result.reliable && smoothed_speed_ > 0.8f)
    result.ttc_s = result.distance_m / smoothed_speed_;

  const bool danger = result.reliable &&
      (result.distance_m < config_.warning_distance_m ||
       (result.ttc_s > 0.0f && result.ttc_s < config_.warning_ttc_s));
  const bool clear = result.distance_m > config_.clear_distance_m &&
      (result.ttc_s < 0.0f || result.ttc_s > config_.clear_ttc_s);
  if (!warning_) {
    clear_streak_ = 0;
    warning_streak_ = danger ? warning_streak_ + 1 : 0;
    if (warning_streak_ >= config_.enter_updates) {
      warning_ = true;
      warning_streak_ = 0;
    }
  } else {
    warning_streak_ = 0;
    clear_streak_ = clear ? clear_streak_ + 1 : 0;
    if (clear_streak_ >= config_.clear_updates) {
      warning_ = false;
      clear_streak_ = 0;
    }
  }
  result.warning = warning_;
  last_result_ = result;
  return result;
}

}  // namespace adas
