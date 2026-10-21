#pragma once
#include "adas/types.hpp"
#include "adas/lane_displacement_kalman.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace adas {
// Board port of pc_lane_visual.py: display-only outer boundaries, independent
// confirmation/expiry, valid visible spans, and reprojection against ego edges.
// Model existence/coverage thresholds and decision LaneResult stay unchanged.
class LaneVisualTracker {
  using Clock=std::chrono::steady_clock;
  struct Observation {
    std::vector<float> y,ratio;
    bool valid() const {return y.size()>=5;}
  };
  struct Side {
    Observation active,pending;
    int hits=0;
    float coverage=0;
    float hold_seconds=.65f;
    LaneDisplacementKalman kalman;
    Clock::time_point seen{},pending_at{};
  };
  struct Ego {
    cv::Vec3d left,right;
    float far=0,near=0;
    bool valid=false;
  };
 public:
  void reset() {sides_={};last_={};previous_=Ego();update_period_=.30f;}
  void update(const LaneResult& measured,const LaneResult& lane,
              Clock::time_point at,int width=640,int height=360,
              Clock::time_point presented_at=Clock::time_point()) {
    width_=width;height_=height;
    if(last_!=Clock::time_point() && (at<=last_ || at-last_>std::chrono::milliseconds(750)))reset();
    if(last_!=Clock::time_point())update_period_=.75f*update_period_+.25f*seconds(at-last_);
    const float lag=presented_at>=at?seconds(presented_at-at):0.f;
    const float hold=std::min(.90f,std::max(.65f,2*update_period_+std::min(.30f,lag)+.06f));
    last_=at;
    Ego ego=make_ego(lane);
    if(!measured.valid || measured.partial || lane.identity_uncertain || lane.partial || !ego.valid) {
      reset();last_=at;return;
    }
    if(previous_.valid) {
      const float y=std::min(ego.near,previous_.near);
      const float old_width=x(previous_.right,y)-x(previous_.left,y);
      const float new_width=x(ego.right,y)-x(ego.left,y);
      const float shift=std::abs(x(ego.left,y)+x(ego.right,y)-x(previous_.left,y)-x(previous_.right,y))*.5f;
      if(old_width<=0 || new_width<old_width*.75f || new_width>old_width*1.34f || shift>old_width*.20f)sides_={};
    }
    previous_=ego;
    for(int side=0;side<2;++side) {
      const int slot=side==0?0:3;
      bool contradictory=false;
      Observation observed;
      if(measured.boundary_coverage[slot]>.25f)
        observed=observe(measured.boundaries[slot],side,ego,contradictory);
      auto& state=sides_[side];
      if(contradictory)state=Side();
      bool fresh=false;
      if(observed.valid()) {
        if(state.active.valid() && distance(state.active,observed)<.32f) {
          std::vector<float> shifts;
          for(size_t i=0;i<observed.y.size();++i)
            if(observed.y[i]>=state.active.y.front() && observed.y[i]<=state.active.y.back())
              shifts.push_back(observed.ratio[i]-interp(state.active,observed.y[i]));
          if(!shifts.empty()) {
            std::sort(shifts.begin(),shifts.end());const float shift=shifts[shifts.size()/2];
            std::vector<float> residuals;for(float value:shifts)residuals.push_back(std::abs(value-shift));
            std::sort(residuals.begin(),residuals.end());
            const float sigma=std::min(.06f,.015f+.02f*(1-measured.boundary_coverage[slot])+1.4826f*residuals[residuals.size()/2]);
            const float elapsed=std::max(.001f,seconds(at-state.seen));
            const auto filtered=state.kalman.update(shift,elapsed,sigma);
            if(filtered.accepted) {
              const float alpha=1-std::exp(-std::min(.067f,elapsed)/.06f);
              for(size_t i=0;i<observed.y.size();++i) {
                if(observed.y[i]>=state.active.y.front() && observed.y[i]<=state.active.y.back()) {
                  const float old=interp(state.active,observed.y[i]);
                  observed.ratio[i]=old+filtered.displacement+alpha*(observed.ratio[i]-old-shift);
                } else observed.ratio[i]+=filtered.displacement-shift;
              }
              state.active=observed;state.pending=Observation();state.hits=0;fresh=true;
            }
          }
        }
        if(!fresh) {
          const bool consecutive=state.pending.valid() && at-state.pending_at<=std::chrono::milliseconds(600) &&
              distance(state.pending,observed)<.24f;
          state.hits=consecutive?state.hits+1:1;state.pending=observed;state.pending_at=at;
          if(state.hits>=2){state.active=observed;state.pending=Observation();state.hits=0;state.kalman.reset();fresh=true;}
        }
      } else {state.pending=Observation();state.hits=0;}
      if(fresh){state.seen=at;state.coverage=measured.boundary_coverage[slot];state.hold_seconds=hold;}
      if(state.active.valid() && seconds(at-state.seen)>state.hold_seconds)state.active=Observation();
    }
  }
  LaneResult render(const LaneResult& measured,Clock::time_point at) const {
    LaneResult visual=measured;
    const Ego ego=make_ego(measured);
    for(int side=0;side<2;++side) {
      const int slot=side==0?0:3,area=side==0?0:2;
      visual.boundaries[slot].clear();visual.lane_polygons[area].clear();
      const auto& state=sides_[side];
      if(!ego.valid || measured.partial || measured.identity_uncertain || !state.active.valid() || at<state.seen ||
          seconds(at-state.seen)>state.hold_seconds)continue;
      const float top=std::max(ego.far,state.active.y.front());
      const float bottom=std::min(ego.near,state.active.y.back());
      auto outside=[&](float y){const float w=x(ego.right,y)-x(ego.left,y);
        return x(side==0?ego.left:ego.right,y)+(side==0?-1:1)*interp(state.active,y)*w;};
      float lo=top,hi=bottom;
      if(!visible_span(ego,side,outside,lo,hi))continue;
      std::vector<cv::Point> inner;
      for(int i=0;i<32;++i) {
        const float y=hi-(hi-lo)*i/31.f;
        visual.boundaries[slot].emplace_back(cvRound(outside(y)),cvRound(y));
        inner.emplace_back(cvRound(x(side==0?ego.left:ego.right,y)),cvRound(y));
      }
      auto& polygon=visual.lane_polygons[area];
      polygon=side==0?visual.boundaries[slot]:inner;
      const auto& reverse=side==0?inner:visual.boundaries[slot];
      polygon.insert(polygon.end(),reverse.rbegin(),reverse.rend());
      // Held curves are for presentation; retain no fabricated coverage value.
      visual.boundary_coverage[slot]=state.coverage;
    }
    return visual;
  }
  // Fresh supported outer spans for paint classification, never held curves.
  static LaneResult supported_observation(const LaneResult& measured,int width,int height) {
    LaneVisualTracker tracker;tracker.width_=width;tracker.height_=height;
    LaneResult result=measured;const Ego ego=make_ego(measured);
    for(int side=0;side<2;++side){
      const int slot=side==0?0:3,area=side==0?0:2;
      result.boundaries[slot].clear();result.lane_polygons[area].clear();
      if(!ego.valid || measured.partial || measured.identity_uncertain || measured.boundary_coverage[slot]<=.25f)continue;
      bool contradiction=false;auto o=tracker.observe(measured.boundaries[slot],side,ego,contradiction);
      if(!o.valid() || contradiction)continue;
      std::vector<cv::Point> inner;
      for(size_t i=o.y.size();i-->0;){float y=o.y[i],w=x(ego.right,y)-x(ego.left,y),in=x(side==0?ego.left:ego.right,y);
        result.boundaries[slot].emplace_back(cvRound(in+(side==0?-1:1)*o.ratio[i]*w),cvRound(y));
        inner.emplace_back(cvRound(in),cvRound(y));}
      auto& polygon=result.lane_polygons[area];polygon=side==0?result.boundaries[slot]:inner;
      const auto& reverse=side==0?inner:result.boundaries[slot];polygon.insert(polygon.end(),reverse.rbegin(),reverse.rend());
    }
    return result;
  }
  std::array<float,2> observation_age_ms(Clock::time_point at) const {
    std::array<float,2> result{{-1,-1}};
    for(int side=0;side<2;++side)if(sides_[side].active.valid() && at>=sides_[side].seen)
      result[side]=seconds(at-sides_[side].seen)*1000;
    return result;
  }
 private:
  static float seconds(Clock::duration d){return std::chrono::duration<float>(d).count();}
  static float x(const cv::Vec3d& f,float y){return (f[0]*y+f[1])*y+f[2];}
  static bool fit(const std::vector<cv::Point>& p,cv::Vec3d& f) {
    if(p.size()<5)return false;
    cv::Mat a(p.size(),3,CV_64F),b(p.size(),1,CV_64F),solution;
    for(size_t i=0;i<p.size();++i){double y=p[i].y;a.at<double>(i,0)=y*y;a.at<double>(i,1)=y;a.at<double>(i,2)=1;b.at<double>(i)=p[i].x;}
    if(!cv::solve(a,b,solution,cv::DECOMP_SVD))return false;
    f={solution.at<double>(0),solution.at<double>(1),solution.at<double>(2)};
    return std::isfinite(f[0])&&std::isfinite(f[1])&&std::isfinite(f[2]);
  }
  static Ego make_ego(const LaneResult& lane) {
    Ego e;if(!lane.valid || lane.left.size()<5 || lane.right.size()<5)return e;
    if(!fit(lane.left,e.left)||!fit(lane.right,e.right))return e;
    auto limits=[](const std::vector<cv::Point>& p){return std::minmax_element(p.begin(),p.end(),[](const cv::Point&a,const cv::Point&b){return a.y<b.y;});};
    auto l=limits(lane.left),r=limits(lane.right);
    e.far=std::max(l.first->y,r.first->y);e.near=std::min(l.second->y,r.second->y);e.valid=e.near>e.far;return e;
  }
  template<class F> bool visible_span(const Ego& ego,int side,F outside,float& top,float& bottom) const {
    int run=-1,best=-1,end=-1;
    const float start=top,step=(bottom-top)/95.f;
    for(int i=0;i<96;++i) {
      const float y=start+step*i,o=outside(y),in=x(side==0?ego.left:ego.right,y);
      const float w=x(ego.right,y)-x(ego.left,y),gap=side==0?in-o:o-in;
      const bool good=std::isfinite(o)&&std::isfinite(in)&&o>=0&&o<width_&&in>=0&&in<width_&&w>=10&&
          gap>=std::max(8.f,w*.35f)&&gap<=w*2.2f;
      if(good){if(run<0)run=i;if(best<0 || i-run>end-best){best=run;end=i;}}
      else run=-1;
    }
    if(best<0)return false;
    top=start+step*best;bottom=start+step*end;return bottom-top>=height_*.15f;
  }
  Observation observe(const std::vector<cv::Point>& points,int side,const Ego& ego,bool& contradictory) const {
    Observation o;cv::Vec3d f;
    if(!fit(points,f))return o;
    auto limits=std::minmax_element(points.begin(),points.end(),[](const cv::Point&a,const cv::Point&b){return a.y<b.y;});
    float top=std::max(ego.far,float(limits.first->y)),bottom=std::min(ego.near,float(limits.second->y));
    std::vector<float> gaps;
    for(int i=0;i<16;++i){float y=top+(bottom-top)*i/15.f;gaps.push_back(side==0?x(ego.left,y)-x(f,y):x(f,y)-x(ego.right,y));}
    std::sort(gaps.begin(),gaps.end());if(gaps[8]<=0){contradictory=true;return o;}
    if(!visible_span(ego,side,[&](float y){return x(f,y);},top,bottom))return o;
    float lo=bottom,hi=top;int supported=0;
    for(const auto& p:points)if(p.y>=top && p.y<=bottom){lo=std::min(lo,float(p.y));hi=std::max(hi,float(p.y));++supported;}
    if(supported<5 || hi-lo<height_*.15f)return o;
    for(int i=0;i<32;++i){const float y=lo+(hi-lo)*i/31.f,w=x(ego.right,y)-x(ego.left,y);
      o.y.push_back(y);o.ratio.push_back((side==0?x(ego.left,y)-x(f,y):x(f,y)-x(ego.right,y))/w);}
    return o;
  }
  static float interp(const Observation& o,float y) {
    auto upper=std::lower_bound(o.y.begin(),o.y.end(),y);
    if(upper==o.y.begin())return o.ratio.front();if(upper==o.y.end())return o.ratio.back();
    size_t i=upper-o.y.begin();float t=(y-o.y[i-1])/(o.y[i]-o.y[i-1]);return o.ratio[i-1]*(1-t)+o.ratio[i]*t;
  }
  float distance(const Observation& a,const Observation& b) const {
    const float top=std::max(a.y.front(),b.y.front()),bottom=std::min(a.y.back(),b.y.back());
    if(bottom-top<height_*.15f)return std::numeric_limits<float>::infinity();
    std::vector<float> d;for(int i=0;i<16;++i){float y=top+(bottom-top)*i/15.f;d.push_back(std::abs(interp(a,y)-interp(b,y)));}
    std::sort(d.begin(),d.end());return d[14];
  }
  std::array<Side,2> sides_{};
  Clock::time_point last_{};
  Ego previous_;
  int width_=640,height_=360;
  float update_period_=.30f;
};
}
