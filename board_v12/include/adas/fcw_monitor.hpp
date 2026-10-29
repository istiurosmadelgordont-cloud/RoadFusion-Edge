#pragma once
#include "adas/risk_estimator.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace adas {
enum class FcwLevel { UNKNOWN, NORMAL, CAUTION, DANGER };
enum class FcwStatus { OFF, WAIT_LANE, STALE, NO_TARGET, CONFIRMING, TRACKING };
struct FcwState {
  bool enabled=false;
  FcwLevel level=FcwLevel::UNKNOWN;
  FcwStatus status=FcwStatus::OFF;
  RiskResult risk;
};

// Front vehicles only, using fresh detector observations and observed ego lanes.
// No center-screen fallback, display predictions, automatic braking or ego speed.
class FcwMonitor {
 public:
  FcwMonitor():estimator_(configuration()){}
  void set_enabled(bool value) { if(value!=enabled_){enabled_=value;reset();} }
  void reset() { clear(enabled_?FcwStatus::NO_TARGET:FcwStatus::OFF);last_sample_=-1;last_now_=-1; }
  const FcwState& state() const { return state_; }
  FcwState update(const std::vector<Detection>& detections,int width,int height,
                  const LaneResult& lane,bool lane_fresh,bool fresh_measurement,
                  double sample_s,double now_s) {
    if(!enabled_){clear(FcwStatus::OFF);return state_;}
    if(last_now_>=0 && now_s<last_now_)reset();
    last_now_=now_s;
    const bool geometry=lane_fresh && lane.valid && !lane.identity_uncertain &&
        (!lane.partial || (lane.boundary_coverage[1]>=.45f && lane.boundary_coverage[2]>=.45f)) &&
        lane.polygon.size()>=3;
    if(!geometry){clear(FcwStatus::WAIT_LANE);return state_;}
    if(!std::isfinite(sample_s) || !std::isfinite(now_s) || sample_s<0 ||
        now_s<sample_s || now_s-sample_s>.65){clear(FcwStatus::STALE);return state_;}
    if(!fresh_measurement)return state_;
    if(last_sample_>=0 && sample_s<=last_sample_)return state_;
    last_sample_=sample_s;
    const Detection* nearest=nullptr;float nearest_distance=1e9f;
    for(const auto& d:detections) {
      if(d.class_id<2 || d.class_id>6 || d.track_id<0 || d.score<.35f ||
          !std::isfinite(d.box.x) || !std::isfinite(d.box.y) ||
          !std::isfinite(d.box.width) || !std::isfinite(d.box.height) ||
          d.box.width<=0 || d.box.height<10)continue;
      const cv::Point2f foot(d.box.x+d.box.width*.5f,d.box.y+d.box.height);
      if(foot.x<0 || foot.x>=width || foot.y<0 || foot.y>height ||
          cv::pointPolygonTest(lane.polygon,foot,false)<0)continue;
      const float physical_height=(d.class_id==3 || d.class_id==4)?2.70f:
                                  (d.class_id==5 || d.class_id==6)?1.35f:1.50f;
      const float distance=width*.30f*physical_height/d.box.height;
      if(distance<nearest_distance){nearest_distance=distance;nearest=&d;}
    }
    if(!nearest){clear(FcwStatus::NO_TARGET);return state_;}
    // A cut-in can be closer than the former lead; do not retain the old lead
    // for two missing detections while a different current vehicle is present.
    if(state_.risk.target && state_.risk.track_id!=nearest->track_id)clear(FcwStatus::CONFIRMING);
    auto risk=estimator_.update({*nearest},width,height,&lane,true,sample_s);
    state_.risk=risk;state_.enabled=true;
    if(!risk.reliable){state_.status=FcwStatus::CONFIRMING;state_.level=FcwLevel::UNKNOWN;return state_;}
    const bool approaching=std::isfinite(risk.ttc_s) && risk.ttc_s>0;
    const FcwLevel candidate=risk.warning?FcwLevel::DANGER:
        (risk.distance_m<14.f || (approaching && risk.ttc_s<4.f))?FcwLevel::CAUTION:FcwLevel::NORMAL;
    if(state_.level==FcwLevel::UNKNOWN || candidate==FcwLevel::DANGER) {
      state_.level=candidate;votes_=0;pending_=candidate;
    } else if(candidate!=state_.level) {
      if(candidate!=pending_){pending_=candidate;votes_=0;}
      const int required=candidate>state_.level?2:3;
      if(++votes_>=required){state_.level=candidate;votes_=0;}
    } else {votes_=0;pending_=candidate;}
    state_.status=FcwStatus::TRACKING;
    state_.risk.warning=state_.level==FcwLevel::DANGER;
    return state_;
  }
 private:
  static RiskConfig configuration() {
    RiskConfig c;c.focal_scale=.30f;c.warning_distance_m=8.f;c.clear_distance_m=11.f;
    c.warning_ttc_s=2.5f;c.clear_ttc_s=3.5f;return c;
  }
  void clear(FcwStatus status) {
    estimator_.reset();state_=FcwState();state_.enabled=enabled_;state_.status=status;
    votes_=0;pending_=FcwLevel::UNKNOWN;
  }
  bool enabled_=false;
  RiskEstimator estimator_;
  FcwState state_;
  double last_sample_=-1,last_now_=-1;
  int votes_=0;
  FcwLevel pending_=FcwLevel::UNKNOWN;
};

inline cv::Scalar fcw_color(FcwLevel level) {
  return level==FcwLevel::DANGER?cv::Scalar(35,45,245):
         level==FcwLevel::CAUTION?cv::Scalar(30,220,245):
         level==FcwLevel::NORMAL?cv::Scalar(65,220,75):cv::Scalar(85,115,90);
}
struct FcwPaint {
  bool enabled=false;
  FcwLevel level=FcwLevel::UNKNOWN;
  cv::Scalar color=fcw_color(FcwLevel::UNKNOWN);
};
class FcwPaintTracker {
 public:
  FcwPaint update(const FcwState& state,double now_s) {
    const auto desired=fcw_color(state.level);
    const bool reset=!state.enabled || state.level==FcwLevel::UNKNOWN || last_<0 ||
        now_s<last_ || now_s-last_>.65;
    const double gain=reset?1.:1.-std::exp(-(now_s-last_)/.18);
    value_.color=value_.color*(1.-gain)+desired*gain;
    value_.enabled=state.enabled;value_.level=state.level;
    last_=state.enabled && state.level!=FcwLevel::UNKNOWN?now_s:-1.;return value_;
  }
 private:
  FcwPaint value_;double last_=-1;
};
}
