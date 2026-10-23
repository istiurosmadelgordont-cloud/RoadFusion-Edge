#pragma once

#include "adas/types.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <map>

namespace adas {

enum class Test4Hazard { NONE, PEDESTRIAN, FRONT_OBSTACLE };

struct Test4Result {
  bool enabled = false, brake = false, stale = true, can_resume = false;
  bool lane_based = false, target_fresh = false;
  Test4Hazard hazard = Test4Hazard::NONE;
  int track_id = -1;
  cv::Rect2f box;
  std::vector<cv::Point> corridor;
  float estimated_distance_m = -1, expansion_ttc_s = -1;
};

// Visual demonstration only. All votes and motion estimates come from fresh,
// timestamped observations, never display predictions. STOP is latched until
// fresh clear observations and an explicit acknowledgement; it never starts
// the vehicle. No dependence on the optional FCW/TTC display switch.
class Test4Safety {
 public:
  void reset() {
    const bool enabled = result_.enabled;
    result_ = Test4Result(); result_.enabled = enabled;
    history_.clear(); last_sample_ = -1; clear_since_ = -1; clear_count_ = 0;
  }
  void set_enabled(bool enabled) {
    if (enabled != result_.enabled) { reset(); result_.enabled = enabled; }
  }
  bool acknowledge(double now) {
    tick(now);
    if (!result_.enabled || !result_.brake || !result_.can_resume) return false;
    reset(); return true;
  }
  const Test4Result& current() const { return result_; }
  const Test4Result& tick(double now) {
    result_.stale = !std::isfinite(now) || last_sample_ < 0 ||
        now < last_sample_ || now - last_sample_ > .65;
    if (result_.stale) { result_.can_resume = false; result_.target_fresh = false; }
    return result_;
  }
  const Test4Result& update(const std::vector<Detection>& observations,
      int width, int height, const LaneResult* lane, bool lane_fresh,
      bool fresh, double sample, double now) {
    tick(now);
    if (!result_.enabled || !fresh || !std::isfinite(sample) ||
        !std::isfinite(now) || sample <= last_sample_ || now < sample ||
        now - sample > .5 || width <= 0 || height <= 0) return result_;
    if (last_sample_ >= 0 && sample - last_sample_ > .7) {
      history_.clear(); clear_count_ = 0; clear_since_ = -1;
    }
    last_sample_ = sample; result_.stale = false; result_.target_fresh = false;
    result_.lane_based = lane_fresh && lane && lane->valid &&
        !lane->identity_uncertain && !lane->partial && lane->polygon.size() >= 3;
    result_.corridor = result_.lane_based ? lane->polygon : std::vector<cv::Point>{
        {int(width*.44),int(height*.45)}, {int(width*.56),int(height*.45)},
        {int(width*.72),height-1}, {int(width*.28),height-1}};
    auto inside = [&](cv::Point2f foot) {
      return foot.y >= height*.45 && foot.y < height && foot.x >= 0 && foot.x < width &&
          cv::pointPolygonTest(result_.corridor, foot, false) >= 0;
    };
    bool any_danger = false;
    Test4Result selected = result_; selected.hazard = Test4Hazard::NONE;
    for (const auto& d : observations) {
      if (d.track_id < 0 || d.class_id < 0 || d.class_id > 6 ||
          !std::isfinite(d.score) || d.score < .4f ||
          !std::isfinite(d.box.x) || !std::isfinite(d.box.y) ||
          !std::isfinite(d.box.width) || !std::isfinite(d.box.height) ||
          d.box.width < 3 || d.box.height < 6) continue;
      const cv::Rect2f clipped = d.box & cv::Rect2f(0,0,width,height);
      if (clipped.area() < d.box.area()*.7f) continue;
      const cv::Point2f foot(clipped.x+clipped.width*.5f,
          std::min(float(height-1),clipped.y+clipped.height));
      auto& h = history_[d.track_id];
      const double dt = sample - h.time;
      const bool linked = h.hits > 0 && h.cls == d.class_id && dt > .03 && dt <= .7;
      float vx = 0, ttc = -1;
      if (linked) {
        vx = float((foot.x-h.foot.x)/dt);
        const float growth = float((clipped.height-h.height)/dt);
        if (growth > clipped.height*.05f) ttc = clipped.height/growth;
      } else h = History();
      const bool in_path = inside(foot);
      const bool pedestrian = d.class_id == 0 || d.class_id == 1;
      // Only a measured inward lateral motion may predict an imminent entry.
      const bool inward = linked && std::abs(vx) > width*.03f &&
          std::abs(vx) < width*.6f && (foot.x-width*.5f)*vx < 0;
      const bool imminent_entry = pedestrian && foot.y > height*.60f && inward &&
          inside(foot+cv::Point2f(vx*.5f,0));
      const float physical_height = d.class_id == 3 || d.class_id == 4 ? 2.7f :
          pedestrian ? 1.7f : d.class_id == 5 || d.class_id == 6 ? 1.35f : 1.5f;
      // Existing NVIDIA front-wide approximation; this number is informational.
      // The uncalibrated metre estimate never authorizes braking or resuming.
      const float distance = .30f*width*physical_height/clipped.height;
      const bool danger = pedestrian ?
          ((in_path || imminent_entry) && (clipped.height >= height*.10f || foot.y > height*.70f)) :
          (in_path && foot.y > height*.60f &&
           ((clipped.height > height*.28f && foot.y > height*.82f) ||
            (linked && ttc > 0 && ttc < 3.f)));
      h.danger_hits = danger ? h.danger_hits+1 : 0;
      h.hits++; h.time=sample; h.cls=d.class_id; h.foot=foot; h.height=clipped.height;
      any_danger = any_danger || danger;
      if (result_.brake && result_.track_id == d.track_id && result_.hazard != Test4Hazard::NONE) {
        result_.box=clipped; result_.target_fresh=true;
      }
      if (danger && h.danger_hits >= 2 &&
          (selected.hazard == Test4Hazard::NONE || pedestrian)) {
        selected.hazard=pedestrian?Test4Hazard::PEDESTRIAN:Test4Hazard::FRONT_OBSTACLE;
        selected.track_id=d.track_id; selected.box=clipped; selected.target_fresh=true;
        selected.estimated_distance_m=pedestrian?-1:distance;
        selected.expansion_ttc_s=pedestrian?-1:ttc;
      }
    }
    for (auto it=history_.begin(); it!=history_.end();) {
      if (it->second.time != sample) it->second.danger_hits=0;
      if (sample-it->second.time > .7) history_.erase(it++); else ++it;
    }
    if (selected.hazard != Test4Hazard::NONE) {
      result_.brake=true; result_.hazard=selected.hazard;
      result_.track_id=selected.track_id; result_.box=selected.box;
      result_.target_fresh=true; result_.estimated_distance_m=selected.estimated_distance_m;
      result_.expansion_ttc_s=selected.expansion_ttc_s;
    }
    if (any_danger || !result_.brake) { clear_count_=0; clear_since_=-1; }
    else { if (clear_count_++ == 0) clear_since_=sample; }
    result_.can_resume = result_.brake && clear_count_>=4 && sample-clear_since_>=1.-1e-6;
    return result_;
  }
 private:
  struct History {
    int cls=-1, hits=0, danger_hits=0;
    double time=-1;
    cv::Point2f foot;
    float height=0;
  };
  Test4Result result_;
  std::map<int,History> history_;
  double last_sample_=-1, clear_since_=-1;
  int clear_count_=0;
};

}  // namespace adas
