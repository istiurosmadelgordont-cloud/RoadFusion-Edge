#pragma once
#include "adas/adas_logic_v2.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>

namespace adas {

// 路口演示状态机：主视角依赖明确指定的灯组 ROI，不会自动推断本车道归属。
// observe() 只消费新检测结果；tick() 仅刷新显示和超时，不能重复投票。
class SignalDemo {
 public:
  bool load(const std::string& file) {
    enabled_ = false;direction_filter_=true;
    std::ifstream input(file);
    std::string key;
    float x1=0, y1=0, x2=0, y2=0;
    if (input >> key >> x1 >> y1 >> x2 >> y2 && key == "roi" &&
        std::isfinite(x1) && std::isfinite(y1) && std::isfinite(x2) && std::isfinite(y2) &&
        x1 >= 0 && y1 >= 0 && x2 <= 1 && y2 <= 1 && x2 > x1 && y2 > y1) {
      region_ = cv::Rect2f(x1,y1,x2-x1,y2-y1);
      enabled_ = true;
    }
    direction_ = ManeuverDirection::STRAIGHT;
    reset();
    return enabled_;
  }
  void enable_full_view(bool enabled=true) {
    region_=cv::Rect2f(0,0,1,1);enabled_=enabled;direction_filter_=false;reset();
  }
  void set_direction_filter(bool enabled) {
    if(direction_filter_!=enabled){direction_filter_=enabled;reset();}
  }
  bool enabled() const { return enabled_; }
  void reset() {
    committed_=candidate_=raw_=SignalState::UNKNOWN;
    count_=0;last_event_=last_valid_=-1e9;since_=0;
    selected_=SignalResult();action_=IntersectionAction::UNKNOWN;held_=false;
    motion_mode_=0;confidence_=0;
  }
  void set_direction(ManeuverDirection value) {
    if(value==ManeuverDirection::LEFT || value==ManeuverDirection::RIGHT ||
       value==ManeuverDirection::STRAIGHT) {
      if(value!=direction_ && direction_filter_){const int motion=motion_mode_;reset();motion_mode_=motion;}
      direction_=value;
    }
  }
  ManeuverDirection direction() const { return direction_; }
  const char* direction_text() const {
    return direction_==ManeuverDirection::LEFT ? "左转" :
           direction_==ManeuverDirection::RIGHT ? "右转" : "直行";
  }
  void set_motion(int value) { motion_mode_=std::max(0,std::min(2,value)); }
  void toggle_motion() { motion_mode_=moving()?2:1; }
  int motion_mode() const { return motion_mode_; }
  bool moving() const { return motion_mode_==1 || (motion_mode_==0 && action_==IntersectionAction::GO); }
  bool alarm() const { return moving() && action_==IntersectionAction::STOP; }
  IntersectionAction action() const { return action_; }
  SignalState raw() const { return raw_; }
  float confidence() const { return confidence_; }
  int confirmations() const { return count_; }
  bool held() const { return held_; }
  const char* reason() const {
    if(alarm())return "模拟闯红灯：请立即停车";
    if(action_==IntersectionAction::STOP && raw_==SignalState::GREEN)return "绿灯确认中，继续停车";
    if(held_)return "短时漏检，保持上次灯态";
    if(action_==IntersectionAction::GO)return "绿灯已确认，可以起步";
    if(action_==IntersectionAction::STOP)return "红灯已确认，停车等待";
    if(action_==IntersectionAction::WAIT)return "灯态连续确认中";
    return "灯态证据不足，暂不提示通行";
  }
  const char* action_text() const {
    if(alarm())return "闯红灯报警（模拟）";
    if(action_==IntersectionAction::STOP)return "停车等待";
    if(action_==IntersectionAction::GO)return moving()?"绿灯通行 / 起步":"绿灯可通行 / 模拟停车";
    if(action_==IntersectionAction::WAIT)return "确认中 / 等待";
    return "暂无法确认 / 等待";
  }
  void observe(double t,const std::vector<Detection>& lights,int width,int height) {
    if(!enabled_ || t<=last_event_)return;
    const double gap=t-last_event_;last_event_=t;
    expire(t);
    float red=0,green=0;SignalResult red_signal,green_signal;
    for(const auto& d:lights) {
      if(d.class_id<7 || d.class_id>14 || !std::isfinite(d.score))continue;
      // 内部灯态 ID 7..14：红/绿各含圆灯、左转、右转、直行。
      // 圆灯保留为候选；箭头只允许与当前模拟方向匹配的类别参与判断。
      const int shape=(d.class_id-7)%4;  // circle, left, right, straight
      const int expected=direction_==ManeuverDirection::LEFT?1:direction_==ManeuverDirection::RIGHT?2:3;
      if(direction_filter_ && shape!=0 && shape!=expected)continue;
      const cv::Point2f center((d.box.x+d.box.width*.5f)/width,
                               (d.box.y+d.box.height*.5f)/height);
      if(!region_.contains(center))continue;
      float& score=d.class_id<=10?red:green;
      SignalResult& signal=d.class_id<=10?red_signal:green_signal;
      if(d.score>score){score=d.score;signal.box=cv::Rect(d.box);signal.score=d.score;
        signal.state=d.class_id<=10?SignalState::RED:SignalState::GREEN;}
    }
    const bool conflict=red>=.65f && green>=.8f;
    raw_=conflict?SignalState::UNKNOWN:red>=.65f?SignalState::RED:
         green>=.8f?SignalState::GREEN:SignalState::UNKNOWN;
    confidence_=std::max(red,green);
    // 已经通行时出现红灯或红绿冲突，立即撤销 GO，再重新积累确认票数。
    if(committed_==SignalState::GREEN && (red>=.65f || conflict)) {
      committed_=SignalState::UNKNOWN;last_valid_=-1e9;selected_=SignalResult();
    }
    if(raw_==SignalState::UNKNOWN){
      // Two real red readings can bridge one brief dropout in the bound ROI.
      // Green always requires consecutive fresh confirmations.
      if(candidate_!=SignalState::RED || t-since_>1.5){candidate_=SignalState::UNKNOWN;count_=0;}
    }
    else {
      if(raw_!=candidate_ || gap>1.0){candidate_=raw_;count_=1;since_=t;}
      else ++count_;
      // 红灯需至少 2 次新结果且持续 0.2 秒；绿灯需连续 3 次且持续 0.6 秒。
      // 两个条件同时满足，避免把显示帧刷新次数当成检测置信度。
      const int required=raw_==SignalState::RED?2:3;
      const double elapsed=raw_==SignalState::RED?.2:.6;
      if(count_>=required && t-since_>=elapsed-1e-6){
        committed_=raw_;last_valid_=t;selected_=raw_==SignalState::RED?red_signal:green_signal;
      }else if(raw_==committed_){last_valid_=t;selected_=raw_==SignalState::RED?red_signal:green_signal;}
    }
    tick(t);
  }
  void tick(double t) {
    if(!enabled_)return;
    expire(t);
    action_=committed_==SignalState::RED?IntersectionAction::STOP:
        committed_==SignalState::GREEN?IntersectionAction::GO:
        raw_!=SignalState::UNKNOWN && t-last_event_<=1.0?IntersectionAction::WAIT:IntersectionAction::UNKNOWN;
    held_=committed_!=SignalState::UNKNOWN &&
         (raw_!=committed_ || t-last_event_>.5);
  }
  IntersectionResult intersection() const {
    IntersectionResult result;
    result.action=action_;result.route=direction_;result.route_valid=enabled_;
    result.stable=action_==IntersectionAction::STOP || action_==IntersectionAction::GO;
    result.selected_signal=selected_;result.selected_signal.state=committed_;
    result.selected_signal.stable=result.stable;result.reason=reason();return result;
  }
 private:
  void expire(double t) {
    // 短时漏检暂存红灯 2 秒、绿灯 1.5 秒；证据过期后回到无法确认。
    // 不能无限沿用旧绿灯，否则换灯、换路口或遮挡时会错误提示通行。
    const double limit=committed_==SignalState::RED?2.0:1.5;
    if(committed_!=SignalState::UNKNOWN && t-last_valid_>limit+1e-6){
      const bool was_green=committed_==SignalState::GREEN;
      committed_=SignalState::UNKNOWN;last_valid_=-1e9;selected_=SignalResult();
      // Reacquisition needs new confirmations after evidence has expired.
      if(was_green){candidate_=SignalState::UNKNOWN;count_=0;}
    }
  }
  bool enabled_=false,held_=false,direction_filter_=true;
  cv::Rect2f region_;
  ManeuverDirection direction_=ManeuverDirection::STRAIGHT;
  SignalState committed_=SignalState::UNKNOWN,candidate_=SignalState::UNKNOWN,raw_=SignalState::UNKNOWN;
  IntersectionAction action_=IntersectionAction::UNKNOWN;
  SignalResult selected_;
  int count_=0,motion_mode_=0;
  double last_event_=-1e9,last_valid_=-1e9,since_=0;
  float confidence_=0;
};
}  // namespace adas
