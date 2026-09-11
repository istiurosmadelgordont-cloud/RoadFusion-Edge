#include "adas/risk_estimator.hpp"

#include <algorithm>
#include <cmath>

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

RiskResult RiskEstimator::update(const std::vector<Detection>& detections, int width, int height) {
  RiskResult result;
  const Detection* best = nullptr;
  float best_rank = -1.0f;
  for (const Detection& detection : detections) {
    if (!is_obstacle(detection.class_id)) continue;
    const float cx = detection.box.x + detection.box.width * 0.5f;
    const float bottom = detection.box.y + detection.box.height;
    const float corridor_half = width * (0.10f + 0.23f * bottom / height);
    if (std::abs(cx - width * 0.5f) > corridor_half || bottom < height * 0.38f) continue;
    const float rank = bottom / height + detection.box.height / height + detection.score * 0.25f;
    if (rank > best_rank) {
      best_rank = rank;
      best = &detection;
    }
  }
  const auto now = std::chrono::steady_clock::now();
  if (!best) {
    initialized_ = false;
    previous_distance_ = -1.0f;
    return result;
  }

  result.target = true;
  result.box = best->box;
  const float focal_px = width * 0.72f;
  result.distance_m = focal_px * physical_height(best->class_id) / std::max(8.0f, best->box.height);
  result.distance_m = std::max(1.0f, std::min(result.distance_m, 150.0f));
  if (initialized_) {
    const float dt = std::chrono::duration<float>(now - previous_time_).count();
    if (dt > 0.02f && dt < 1.0f) {
      const float closing_mps = (previous_distance_ - result.distance_m) / dt;
      smoothed_speed_ = 0.82f * smoothed_speed_ + 0.18f * closing_mps;
    }
  }
  previous_distance_ = result.distance_m;
  previous_time_ = now;
  initialized_ = true;
  result.relative_speed_kmh = smoothed_speed_ * 3.6f;
  if (smoothed_speed_ > 0.35f) result.ttc_s = result.distance_m / smoothed_speed_;
  result.warning = result.distance_m < 8.0f || (result.ttc_s > 0.0f && result.ttc_s < 3.0f);
  return result;
}

}  // namespace adas
