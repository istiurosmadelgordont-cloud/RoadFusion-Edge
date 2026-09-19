#include "adas/byte_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace adas {

float ByteTracker::iou(const cv::Rect2f& a, const cv::Rect2f& b) {
  const float left = std::max(a.x, b.x);
  const float top = std::max(a.y, b.y);
  const float right = std::min(a.x + a.width, b.x + b.width);
  const float bottom = std::min(a.y + a.height, b.y + b.height);
  const float intersection = std::max(0.0f, right - left) * std::max(0.0f, bottom - top);
  const float total = a.area() + b.area() - intersection;
  return total > 0.0f ? intersection / total : 0.0f;
}

cv::Rect2f ByteTracker::project(const Track& track,
                                std::chrono::steady_clock::time_point when) {
  const float dt = std::max(0.0f, std::min(0.6f,
      std::chrono::duration<float>(when - track.measured_at).count()));
  cv::Rect2f box(track.box.x + track.velocity[0] * dt,
                 track.box.y + track.velocity[1] * dt,
                 std::max(2.0f, track.box.width + track.velocity[2] * dt),
                 std::max(2.0f, track.box.height + track.velocity[3] * dt));
  return box;
}

void ByteTracker::associate(const std::vector<Detection>& detections,
                            const std::vector<int>& indices, float threshold,
                            std::chrono::steady_clock::time_point measured_at,
                            std::vector<bool>& track_used,
                            std::vector<bool>& detection_used) {
  while (true) {
    float best = threshold;
    int best_track = -1;
    int best_detection = -1;
    for (size_t t = 0; t < tracks_.size(); ++t) {
      if (track_used[t]) continue;
      const cv::Rect2f predicted = project(tracks_[t], measured_at);
      for (int d : indices) {
        if (detection_used[d] || tracks_[t].class_id != detections[d].class_id) continue;
        const float overlap = iou(predicted, detections[d].box);
        if (overlap > best) { best = overlap; best_track = static_cast<int>(t); best_detection = d; }
      }
    }
    if (best_track < 0) break;
    Track& track = tracks_[best_track];
    const float dt = std::chrono::duration<float>(measured_at - track.measured_at).count();
    if (dt > 0.03f && dt < 1.5f) {
      const cv::Vec4f measured_velocity(
          (detections[best_detection].box.x - track.box.x) / dt,
          (detections[best_detection].box.y - track.box.y) / dt,
          (detections[best_detection].box.width - track.box.width) / dt,
          (detections[best_detection].box.height - track.box.height) / dt);
      track.velocity = track.hits < 2 ? measured_velocity
                                     : track.velocity * 0.65f + measured_velocity * 0.35f;
    }
    track.box = detections[best_detection].box;
    track.score = detections[best_detection].score;
    track.name = detections[best_detection].name;
    track.measured_at = measured_at;
    track.missed = 0;
    ++track.hits;
    track_used[best_track] = true;
    detection_used[best_detection] = true;
  }
}

std::vector<Detection> ByteTracker::update(
    const std::vector<Detection>& detections,
    std::chrono::steady_clock::time_point measured_at,
    std::chrono::steady_clock::time_point now) {
  std::vector<int> high, low;
  for (size_t i = 0; i < detections.size(); ++i)
    (detections[i].score >= 0.45f ? high : low).push_back(static_cast<int>(i));
  std::vector<bool> track_used(tracks_.size(), false);
  std::vector<bool> detection_used(detections.size(), false);
  associate(detections, high, 0.25f, measured_at, track_used, detection_used);
  associate(detections, low, 0.15f, measured_at, track_used, detection_used);
  for (size_t i = 0; i < tracks_.size(); ++i) if (!track_used[i]) ++tracks_[i].missed;
  for (int d : high) {
    if (detection_used[d]) continue;
    Track track;
    track.id = next_id_++;
    track.class_id = detections[d].class_id;
    track.name = detections[d].name;
    track.box = detections[d].box;
    track.score = detections[d].score;
    track.hits = 1;
    track.measured_at = measured_at;
    tracks_.push_back(track);
  }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& track) {
    return track.missed > 8 || now - track.measured_at > std::chrono::milliseconds(1200);
  }), tracks_.end());
  return predict(now);
}

std::vector<Detection> ByteTracker::predict(std::chrono::steady_clock::time_point now) const {
  std::vector<Detection> result;
  for (const Track& track : tracks_) {
    Detection detection;
    detection.class_id = track.class_id;
    detection.track_id = track.id;
    detection.name = track.name;
    detection.score = track.score;
    detection.box = project(track, now);
    result.push_back(detection);
  }
  return result;
}

void ByteTracker::reset() { tracks_.clear(); next_id_ = 1; }

}  // namespace adas
