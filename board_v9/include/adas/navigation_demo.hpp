#pragma once
#include "adas/adas_logic_v2.hpp"
#include <array>

namespace adas {

// Explicit user-operated mock input. These values are never presented as OCR,
// measured road distance, or navigation extracted from the PCIe video.
class NavigationDemo {
 public:
  enum class Action { DISABLED, WAIT, STOP, STRAIGHT, LEFT, RIGHT };
  struct Advice {
    Action action=Action::DISABLED;
    bool preparing=false;
    const char* text="导航模拟已关闭";
  };
  void set_enabled(bool value) { enabled_=value; }
  bool enabled() const { return enabled_; }
  ManeuverDirection direction() const { return direction_; }
  int distance() const { return distance_; }
  void set_direction(ManeuverDirection value) {
    if (value>=ManeuverDirection::LEFT && value<=ManeuverDirection::RIGHT) direction_=value;
  }
  void adjust_distance(int delta) { distance_=std::max(0,std::min(990,distance_+delta)); }
  ManeuverDirection current_route() const {
    return distance_>20?ManeuverDirection::STRAIGHT:direction_;
  }
  const char* direction_text() const {
    return direction_==ManeuverDirection::LEFT?"左转":direction_==ManeuverDirection::RIGHT?"右转":"直行";
  }
  Advice advice(IntersectionAction signal, bool danger_stop) const {
    Advice a;if(!enabled_)return a;
    a.preparing=distance_>20;
    if(danger_stop){a.action=Action::STOP;a.text="危险停车优先";return a;}
    if(signal==IntersectionAction::STOP){a.action=Action::STOP;a.text="红灯停车";return a;}
    if(signal!=IntersectionAction::GO){a.action=Action::WAIT;a.text="灯态未确认，等待";return a;}
    a.action=current_route()==ManeuverDirection::LEFT?Action::LEFT:
        current_route()==ManeuverDirection::RIGHT?Action::RIGHT:Action::STRAIGHT;
    a.text=a.action==Action::LEFT?"模拟左转":a.action==Action::RIGHT?"模拟右转":"保持直行";
    return a;
  }
  static cv::Rect toggle_button() { return {1408,312,220,30}; }
  static std::array<cv::Rect,3> direction_buttons() {
    return {{{1408,353,148,30},{1568,353,148,30},{1728,353,148,30}}};
  }
  static std::array<cv::Rect,3> distance_buttons() {
    return {{{1408,395,148,30},{1568,395,148,30},{1728,395,148,30}}};
  }
  bool click(cv::Point point) {
    if(toggle_button().contains(point)){enabled_=!enabled_;return true;}
    const auto dirs=direction_buttons(),distances=distance_buttons();
    for(int i=0;i<3;++i){
      if(dirs[i].contains(point)){set_direction(static_cast<ManeuverDirection>(i+1));return true;}
      if(distances[i].contains(point)){if(i==2)distance_=120;else adjust_distance(i==0?-30:30);return true;}
    }
    return false;
  }
 private:
  bool enabled_=false;
  ManeuverDirection direction_=ManeuverDirection::RIGHT;
  int distance_=120;
};
} // namespace adas
