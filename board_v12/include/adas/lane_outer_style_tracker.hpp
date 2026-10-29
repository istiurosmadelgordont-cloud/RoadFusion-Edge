#pragma once
#include "adas/adas_logic_v2.hpp"

namespace adas {
// Presentation-only paint-style hysteresis. Ego paint and manoeuvre evidence
// continue to come directly from LaneSemanticTracker's fresh observations.
class LaneOuterStyleTracker {
  using Clock=std::chrono::steady_clock;
  struct Side {
    LaneMarking confirmed=LaneMarking::UNKNOWN,candidate=LaneMarking::UNKNOWN;
    int streak=0,misses=0;
    Clock::time_point confirmed_at{};
  };
 public:
  void reset(){sides_={};last_={};}
  void update(const LaneMarkingObservation& observed,const LaneResult& lane,Clock::time_point at){
    if(last_!=Clock::time_point() && (at<=last_ || at-last_>std::chrono::milliseconds(750)))reset();
    last_=at;
    if(!lane.valid || lane.partial || lane.identity_uncertain || lane.reassignment_side!=LaneDepartureSide::NONE){sides_={};return;}
    for(int side=0;side<2;++side){
      auto& state=sides_[side];const auto value=observed.boundaries[side==0?0:3];
      if(state.confirmed!=LaneMarking::UNKNOWN && at-state.confirmed_at>std::chrono::milliseconds(650))state=Side();
      if(value==LaneMarking::UNKNOWN){
        if(++state.misses>=2 || state.confirmed==LaneMarking::UNKNOWN)state=Side();
        continue;
      }
      state.misses=0;
      if(value!=state.candidate){state.candidate=value;state.streak=1;}else ++state.streak;
      if(state.streak>=2){state.confirmed=value;state.confirmed_at=at;}
    }
  }
  void apply(LaneSemantic& display,const LaneResult& visual,Clock::time_point at) const {
    for(int side=0;side<2;++side){
      const auto& state=sides_[side];
      const bool recent=visual.valid && !visual.partial && !visual.identity_uncertain &&
          visual.reassignment_side==LaneDepartureSide::NONE && state.confirmed!=LaneMarking::UNKNOWN &&
          at>=state.confirmed_at && at-state.confirmed_at<=std::chrono::milliseconds(650);
      display.boundaries[side==0?0:3]=recent?state.confirmed:LaneMarking::UNKNOWN;
    }
  }
 private:
  std::array<Side,2> sides_{};
  Clock::time_point last_{};
};
}
