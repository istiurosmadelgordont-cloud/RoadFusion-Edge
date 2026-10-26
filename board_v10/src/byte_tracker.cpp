#include "adas/byte_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace adas {

namespace {
bool classified_lamp(int cls) { return cls>=7 && cls<=14; }
cv::Point2f center_of(const cv::Rect2f& box) {
  return {box.x+box.width*.5f,box.y+box.height*.5f};
}
float clamp(float value, float low, float high) {
  return std::max(low, std::min(high, value));
}
bool display_fresh(std::chrono::steady_clock::time_point measured,
                   std::chrono::steady_clock::time_point received,
                   std::chrono::steady_clock::time_point when) {
  // Receipt time bridges a delayed result; source time still places a hard
  // bound on all rendered tracks. This never refreshes measured lamp evidence.
  return when-measured<=std::chrono::milliseconds(1000) &&
         when-received<=std::chrono::milliseconds(650);
}
cv::Rect2f from_relative(const cv::Rect2f& relative, const cv::Rect2f& parent) {
  return {parent.x + relative.x * parent.width, parent.y + relative.y * parent.height,
          std::max(2.f, relative.width * parent.width),
          std::max(2.f, relative.height * parent.height)};
}
}  // namespace


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

cv::Rect2f ByteTracker::project_display(const Track& track,
                                      std::chrono::steady_clock::time_point when) {
  const float dt = std::max(0.0f, std::min(0.6f,
      std::chrono::duration<float>(when - track.measured_at).count()));
  if(classified_lamp(track.class_id) || track.class_id==21) {
    // 用短预测时域限制漂移，而不是只允许移动“半个小灯宽”。
    // 未绑定灯组的小灯最多外推 200ms，灯组最多 300ms，断流后不无限前进。
    const float horizon = std::min(dt, track.class_id == 21 ? .30f : .20f);
    const float xlimit = std::max(12.f, track.display_box.width * 2.f);
    const float ylimit = std::max(12.f, track.display_box.height * 2.f);
    const float dx = clamp(track.velocity[0] * horizon, -xlimit, xlimit);
    const float dy = clamp(track.velocity[1] * horizon, -ylimit, ylimit);
    const float width = clamp(track.display_box.width + track.velocity[2] * horizon,
                              track.display_box.width * .75f, track.display_box.width * 1.5f);
    const float height = clamp(track.display_box.height + track.velocity[3] * horizon,
                               track.display_box.height * .75f, track.display_box.height * 1.5f);
    return cv::Rect2f(track.display_box.x+dx,track.display_box.y+dy,
                      std::max(2.f,width),std::max(2.f,height));
  }
  return cv::Rect2f(track.display_box.x + track.velocity[0] * dt,
                    track.display_box.y + track.velocity[1] * dt,
                    std::max(2.0f, track.display_box.width + track.velocity[2] * dt),
                    std::max(2.0f, track.display_box.height + track.velocity[3] * dt));
}

const ByteTracker::Track* ByteTracker::parent_of(
    const Track& track, std::chrono::steady_clock::time_point when) const {
  if (track.parent_track_id < 0) return nullptr;
  for (const auto& parent : tracks_) {
    if (parent.id == track.parent_track_id && parent.class_id == 21 && parent.missed <= 1 &&
        display_fresh(parent.measured_at,parent.received_at,when) &&
        parent.box.width > 0 && parent.box.height > 0) return &parent;
  }
  return nullptr;
}

cv::Rect2f ByteTracker::display_box_at(
    const Track& track, std::chrono::steady_clock::time_point when) const {
  if (classified_lamp(track.class_id)) {
    if (const auto* parent = parent_of(track, when))
      return from_relative(track.relative_box, project_display(*parent, when));
  }
  return project_display(track, when);
}

void ByteTracker::associate(const std::vector<Detection>& detections,
                            const std::vector<int>& indices, float threshold,
                            std::chrono::steady_clock::time_point measured_at,
                            std::chrono::steady_clock::time_point received_at,
                            std::vector<bool>& track_used,
                            std::vector<bool>& detection_used,
                            std::vector<int>& detection_track_ids,
                            const std::vector<int>& parent_ids) {
  while (true) {
    float best = threshold;
    int best_track = -1;
    int best_detection = -1;
    for (size_t t = 0; t < tracks_.size(); ++t) {
      if (track_used[t]) continue;
      const cv::Rect2f predicted = classified_lamp(tracks_[t].class_id) &&
          tracks_[t].parent_track_id >= 0 ? display_box_at(tracks_[t], measured_at) :
                                          project(tracks_[t], measured_at);
      for (int d : indices) {
        const bool same_lamp = tracks_[t].class_id >= 7 && tracks_[t].class_id <= 14 &&
                               detections[d].class_id >= 7 && detections[d].class_id <= 14;
        if (detection_used[d] || (!same_lamp && tracks_[t].class_id != detections[d].class_id)) continue;
        // 灯组先完成关联：不同父目标的子灯不能因距离接近而交换身份。
        if (same_lamp && tracks_[t].parent_track_id >= 0 && parent_ids[d] >= 0 &&
            tracks_[t].parent_track_id != parent_ids[d]) continue;
        float overlap = iou(predicted, detections[d].box);
        if (same_lamp && (tracks_[t].parent_track_id < 0 || parent_ids[d] < 0)) {
          overlap=std::max(overlap,iou(tracks_[t].box,detections[d].box));
        }
        if (overlap > best) { best = overlap; best_track = static_cast<int>(t); best_detection = d; }
      }
    }
    if (best_track < 0) break;
    Track& track = tracks_[best_track];
    const float dt = std::chrono::duration<float>(measured_at - track.measured_at).count();
    const cv::Rect2f measured_box = detections[best_detection].box;
    const cv::Rect2f previous_display = display_box_at(track, measured_at);
    const bool lamp = track.class_id >= 7 && track.class_id <= 14;
    // 位置立即采用新观测，避免追着旧坐标缓慢移动；仅对小幅宽高变化轻微平滑。
    // 换色、方向改变或大幅运动时，宽高也立即校正。
    const bool smooth_display = lamp && track.class_id == detections[best_detection].class_id &&
        dt > 0.0f && dt < 0.8f && iou(track.box, measured_box) >= 0.6f &&
        iou(previous_display, measured_box) >= 0.6f;
    track.display_box = measured_box;
    if (smooth_display) {
      track.display_box.width = measured_box.width * .85f + previous_display.width * .15f;
      track.display_box.height = measured_box.height * .85f + previous_display.height * .15f;
    }
    if (lamp && track.class_id != detections[best_detection].class_id) {
      // 红、绿灯泡可能在壳体的不同位置，不能把亮灯切换误估为高速运动。
      track.velocity = cv::Vec4f(0, 0, 0, 0);
    } else if (dt > 0.03f && dt < 1.5f) {
      const cv::Vec4f measured_velocity(
          (detections[best_detection].box.x - track.box.x) / dt,
          (detections[best_detection].box.y - track.box.y) / dt,
          (detections[best_detection].box.width - track.box.width) / dt,
          (detections[best_detection].box.height - track.box.height) / dt);
      const float fresh_weight = lamp || track.class_id == 21 ? .75f : .35f;
      track.velocity = track.hits < 2 ? measured_velocity :
          track.velocity * (1.f - fresh_weight) + measured_velocity * fresh_weight;
    }
    track.box = detections[best_detection].box;
    track.class_id = detections[best_detection].class_id;
    track.score = detections[best_detection].score;
    track.name = detections[best_detection].name;
    track.source_housing_index = detections[best_detection].source_housing_index;
    track.measured_at = measured_at;
    track.received_at = received_at;
    track.missed = 0;
    ++track.hits;
    track_used[best_track] = true;
    detection_used[best_detection] = true;
    detection_track_ids[best_detection] = track.id;
  }
}

std::vector<Detection> ByteTracker::update(
    const std::vector<Detection>& detections,
    std::chrono::steady_clock::time_point measured_at,
    std::chrono::steady_clock::time_point now) {
  std::vector<int> object_high, object_low, lamp_high, lamp_low;
  for (size_t i = 0; i < detections.size(); ++i) {
    auto& high = classified_lamp(detections[i].class_id) ? lamp_high : object_high;
    auto& low = classified_lamp(detections[i].class_id) ? lamp_low : object_low;
    (detections[i].score >= .45f ? high : low).push_back(static_cast<int>(i));
  }
  std::vector<bool> track_used(tracks_.size(), false);
  std::vector<bool> detection_used(detections.size(), false);
  std::vector<int> detection_track_ids(detections.size(), -1), parent_ids(detections.size(), -1);
  const auto add_new = [&](const std::vector<int>& indices) {
    for (int d : indices) {
      if (detection_used[d]) continue;
      Track track;
      track.id = next_id_++;
      track.class_id = detections[d].class_id;
      track.name = detections[d].name;
      track.box = detections[d].box;
      track.display_box = track.box;
      track.score = detections[d].score;
      track.source_housing_index = detections[d].source_housing_index;
      track.hits = 1;
      track.measured_at = measured_at;
      track.received_at = now;
      detection_track_ids[d] = track.id;
      detection_used[d] = true;
      tracks_.push_back(track);
      track_used.push_back(true);
    }
  };
  // 两阶段仍按高/低置信度关联，但先更新所有灯组，再关联子灯。
  associate(detections, object_high, .25f, measured_at, now, track_used, detection_used,
            detection_track_ids, parent_ids);
  associate(detections, object_low, .15f, measured_at, now, track_used, detection_used,
            detection_track_ids, parent_ids);
  add_new(object_high);
  for (size_t d = 0; d < detections.size(); ++d) {
    if (!classified_lamp(detections[d].class_id)) continue;
    int parent = -1, candidates = 0;
    for (size_t h = 0; h < detections.size(); ++h) {
      const auto& housing = detections[h];
      if (housing.class_id != 21 || detection_track_ids[h] < 0 || housing.box.width <= 0 ||
          housing.box.height <= 0 || !housing.box.contains(center_of(detections[d].box))) continue;
      if (detections[d].source_housing_index >= 0 &&
          housing.source_housing_index != detections[d].source_housing_index) continue;
      parent = detection_track_ids[h];
      ++candidates;
    }
    // 无来源信息时只接受唯一包含它的灯组，不猜测重叠灯组之间的归属。
    if (candidates == 1) parent_ids[d] = parent;
  }
  associate(detections, lamp_high, .25f, measured_at, now, track_used, detection_used,
            detection_track_ids, parent_ids);
  associate(detections, lamp_low, .15f, measured_at, now, track_used, detection_used,
            detection_track_ids, parent_ids);
  add_new(lamp_high);
  for (size_t d = 0; d < detections.size(); ++d) {
    if (!classified_lamp(detections[d].class_id) || detection_track_ids[d] < 0) continue;
    for (auto& track : tracks_) {
      if (track.id != detection_track_ids[d]) continue;
      track.parent_track_id = parent_ids[d];
      if (const auto* parent = parent_of(track, measured_at)) {
        track.relative_box = {(track.display_box.x - parent->box.x) / parent->box.width,
                              (track.display_box.y - parent->box.y) / parent->box.height,
                              track.display_box.width / parent->box.width,
                              track.display_box.height / parent->box.height};
      }
      break;
    }
  }
  for (size_t i = 0; i < tracks_.size(); ++i) if (!track_used[i]) ++tracks_[i].missed;
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& track) {
    return track.missed > 8 || now - track.measured_at > std::chrono::milliseconds(1200);
  }), tracks_.end());
  return predict(now);
}

std::vector<Detection> ByteTracker::predict(std::chrono::steady_clock::time_point now) const {
  std::vector<Detection> result;
  for (const Track& track : tracks_) {
    const bool lamp=(track.class_id>=7 && track.class_id<=14) || track.class_id==21;
    // Bridge the inter-result gap, but a fresh ordinary-object miss must hide
    // its old geometry immediately. Retain the internal ID for reassociation,
    // not for drawing another box beside a newly created track after a turn.
    // Lamps keep their longer, parent/child display lifetime independently.
    if (!display_fresh(track.measured_at,track.received_at,now) ||
        (!lamp && (track.missed!=0 ||
                   now-track.measured_at>std::chrono::milliseconds(650)))) continue;
    // 父灯组失效时一并隐藏其子灯，不能继续用邻近灯组或无限预测接替。
    if (classified_lamp(track.class_id) && track.parent_track_id >= 0 && !parent_of(track, now)) continue;
    if(lamp && track.missed>0) {
      // Keep current coloured evidence when this measurement classified lamps.
      // Current tracks continue rendering smoothly between measurements.
      bool current_coloured_lamp=false;
      for(const auto& current:tracks_)if(current.missed==0 && classified_lamp(current.class_id) &&
          (track.parent_track_id < 0 || current.parent_track_id == track.parent_track_id)) {
        current_coloured_lamp=true;break;
      }
      if(classified_lamp(track.class_id) && current_coloured_lamp)continue;
      // An unconfirmed candidate is not retained as another visible light.
      if((track.class_id==21 && track.name!="交通灯") || track.missed>1)continue;
      bool replaced=false;
      for(const auto& current:tracks_) {
        if(current.id==track.id || current.missed!=0)continue;
        if(classified_lamp(current.class_id) && iou(track.box,current.box)>.1f) {
          replaced=true;break;
        }
        if(current.class_id==21 && current.id!=track.parent_track_id) {
          const auto& box=current.box;
          const cv::Rect2f expanded(box.x-box.width*.25f,box.y-box.height*.25f,
                                    box.width*1.5f,box.height*1.5f);
          if(expanded.contains(center_of(track.box))){replaced=true;break;}
        }
      }
      if(replaced)continue;
    }
    Detection detection;
    detection.class_id = track.class_id;
    detection.track_id = track.id;
    detection.source_housing_index = track.source_housing_index;
    detection.name = track.name;
    // Never present a carried-over label as a fresh MO classification. Generic
    // hidden housing entries keep their exact name so draw_boxes still skips them.
    if (lamp && !(track.class_id == 21 && track.name == "交通灯") &&
        (track.missed > 0 || now - track.measured_at > std::chrono::milliseconds(650)))
      detection.name += " · 暂存";
    detection.score = track.score;
    detection.box = display_box_at(track, now);
    detection.motion_valid = track.hits >= 3 && track.missed == 0 &&
        now - track.measured_at < std::chrono::milliseconds(800);
    detection.image_velocity = cv::Point2f(track.velocity[0] + 0.5f * track.velocity[2],
                                           track.velocity[1] + 0.5f * track.velocity[3]);
    result.push_back(detection);
  }
  return result;
}

std::vector<Detection> ByteTracker::observations() const {
  std::vector<Detection> result;
  for (const auto& track : tracks_) {
    if (track.missed != 0) continue;
    Detection d;
    d.class_id = track.class_id;
    d.track_id = track.id;
    d.source_housing_index = track.source_housing_index;
    d.name = track.name;
    d.score = track.score;
    d.box = track.box;
    result.push_back(d);
  }
  return result;
}

std::vector<Detection> ByteTracker::identify_measurements(
    const std::vector<Detection>& detections) const {
  auto result = detections;
  for (auto& detection : result) {
    detection.track_id = -1;
    for (const auto& track : tracks_) {
      if (track.missed != 0 || track.class_id != detection.class_id ||
          track.source_housing_index != detection.source_housing_index) continue;
      const auto& a = track.box;
      const auto& b = detection.box;
      // HSV 筛选只删观测、不移动框；只认同一原始测量，绝不按邻近预测框补证据。
      if (std::abs(a.x-b.x) + std::abs(a.y-b.y) + std::abs(a.width-b.width) +
          std::abs(a.height-b.height) > .01f) continue;
      detection.track_id = track.id;
      break;
    }
  }
  return result;
}

void ByteTracker::reset() { tracks_.clear(); next_id_ = 1; }

}  // namespace adas
