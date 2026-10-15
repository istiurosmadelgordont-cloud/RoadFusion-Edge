#include "adas/byte_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace adas {

namespace {
bool classified_lamp(int cls) { return cls>=7 && cls<=14; }
cv::Point2f center_of(const cv::Rect2f& box) {
  return {box.x+box.width*.5f,box.y+box.height*.5f};
}
// A housing anchors a tiny illuminated crop without merging nearby units.
// Wide housings containing several distinct signal heads are excluded.
float housing_affinity(const cv::Rect2f& previous,const cv::Rect2f& measured,
                       const std::vector<Detection>& detections) {
  float best=0;
  const auto old_center=center_of(previous),new_center=center_of(measured);
  const float size_ratio=previous.area()/std::max(1.f,measured.area());
  if(size_ratio<.25f || size_ratio>4.f)return 0;
  for(const auto& housing:detections) {
    const auto& box=housing.box;
    if(housing.class_id!=21 || !box.contains(new_center) || box.width<=0 || box.height<=0 ||
       box.width>6*std::max(previous.width,measured.width) ||
       box.height>12*std::max(previous.height,measured.height))continue;
    const auto anchor=center_of(box);
    const float dx=std::abs(old_center.x-anchor.x)/box.width;
    const float dy=std::abs(old_center.y-anchor.y)/box.height;
    if(dx>.75f || dy>.75f)continue;
    best=std::max(best,.26f+.15f*(1.f-std::max(dx,dy)/.75f));
  }
  return best;
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
    // Lamp crops must not fly away or grow from noisy velocity estimates.
    const float xlimit=std::max(2.f,track.display_box.width*.5f);
    const float ylimit=std::max(2.f,track.display_box.height*.5f);
    const float dx=std::max(-xlimit,std::min(xlimit,track.velocity[0]*dt));
    const float dy=std::max(-ylimit,std::min(ylimit,track.velocity[1]*dt));
    return cv::Rect2f(track.display_box.x+dx,track.display_box.y+dy,
                      track.display_box.width,track.display_box.height);
  }
  return cv::Rect2f(track.display_box.x + track.velocity[0] * dt,
                    track.display_box.y + track.velocity[1] * dt,
                    std::max(2.0f, track.display_box.width + track.velocity[2] * dt),
                    std::max(2.0f, track.display_box.height + track.velocity[3] * dt));
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
        const bool same_lamp = tracks_[t].class_id >= 7 && tracks_[t].class_id <= 14 &&
                               detections[d].class_id >= 7 && detections[d].class_id <= 14;
        if (detection_used[d] || (!same_lamp && tracks_[t].class_id != detections[d].class_id)) continue;
        float overlap = iou(predicted, detections[d].box);
        if(same_lamp) {
          overlap=std::max(overlap,iou(tracks_[t].box,detections[d].box));
          if((tracks_[t].class_id-7)%4==(detections[d].class_id-7)%4)
            overlap=std::max(overlap,housing_affinity(tracks_[t].box,detections[d].box,detections));
        }
        if (overlap > best) { best = overlap; best_track = static_cast<int>(t); best_detection = d; }
      }
    }
    if (best_track < 0) break;
    Track& track = tracks_[best_track];
    const float dt = std::chrono::duration<float>(measured_at - track.measured_at).count();
    const cv::Rect2f measured_box = detections[best_detection].box;
    const cv::Rect2f previous_display = project_display(track, measured_at);
    const bool lamp = track.class_id >= 7 && track.class_id <= 14;
    // Smooth only small geometric variations of a fresh, matching lamp class.
    // Colour/direction changes and large camera movement snap to the new reading.
    const bool smooth_display = lamp && track.class_id == detections[best_detection].class_id &&
        dt > 0.0f && dt < 0.8f && iou(track.box, measured_box) >= 0.6f &&
        iou(previous_display, measured_box) >= 0.6f;
    track.display_box = smooth_display ? cv::Rect2f(
        measured_box.x * 0.65f + previous_display.x * 0.35f,
        measured_box.y * 0.65f + previous_display.y * 0.35f,
        measured_box.width * 0.65f + previous_display.width * 0.35f,
        measured_box.height * 0.65f + previous_display.height * 0.35f) : measured_box;
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
    track.class_id = detections[best_detection].class_id;
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
    track.display_box = track.box;
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
    const bool lamp=(track.class_id>=7 && track.class_id<=14) || track.class_id==21;
    const auto maximum_age=std::chrono::milliseconds(lamp ? 1000 : 650);
    if (now-track.measured_at>maximum_age || (!lamp && track.missed>1)) continue;
    if(lamp && track.missed>0) {
      // Keep current coloured evidence when this measurement classified lamps.
      // Current tracks continue rendering smoothly between measurements.
      bool current_coloured_lamp=false;
      for(const auto& current:tracks_)if(current.missed==0 && classified_lamp(current.class_id)) {
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
        if(current.class_id==21) {
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
    detection.name = track.name;
    // Never present a carried-over label as a fresh MO classification. Generic
    // hidden housing entries keep their exact name so draw_boxes still skips them.
    if (lamp && !(track.class_id == 21 && track.name == "交通灯") &&
        (track.missed > 0 || now - track.measured_at > std::chrono::milliseconds(650)))
      detection.name += " · 暂存";
    detection.score = track.score;
    detection.box = project_display(track, now);
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
    d.name = track.name;
    d.score = track.score;
    d.box = track.box;
    result.push_back(d);
  }
  return result;
}

void ByteTracker::reset() { tracks_.clear(); next_id_ = 1; }

}  // namespace adas
