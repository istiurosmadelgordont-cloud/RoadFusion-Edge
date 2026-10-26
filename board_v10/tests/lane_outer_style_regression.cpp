#include "adas/lane_outer_style_tracker.hpp"
#include <cassert>
#include <iostream>
int main(){
  using namespace adas;using Clock=std::chrono::steady_clock;
  const auto base=Clock::time_point(std::chrono::seconds(1));auto at=[&](int ms){return base+std::chrono::milliseconds(ms);};
  LaneOuterStyleTracker tracker;LaneResult lane;lane.valid=true;LaneMarkingObservation observed;
  LaneSemantic original;original.left=LaneMarking::UNKNOWN;original.left_change_allowed=false;
  auto displayed=[&](int ms){LaneSemantic result=original;tracker.apply(result,lane,at(ms));assert(result.left==LaneMarking::UNKNOWN && !result.left_change_allowed);return result.boundaries[3];};
  observed.boundaries[3]=LaneMarking::SOLID;tracker.update(observed,lane,at(0));assert(displayed(0)==LaneMarking::UNKNOWN);
  tracker.update(observed,lane,at(250));assert(displayed(250)==LaneMarking::SOLID);
  observed.boundaries[3]=LaneMarking::UNKNOWN;tracker.update(observed,lane,at(500));assert(displayed(500)==LaneMarking::SOLID);
  observed.boundaries[3]=LaneMarking::SOLID;tracker.update(observed,lane,at(750));assert(displayed(750)==LaneMarking::SOLID);
  observed.boundaries[3]=LaneMarking::DASHED;tracker.update(observed,lane,at(1000));assert(displayed(1000)==LaneMarking::SOLID);
  tracker.update(observed,lane,at(1250));assert(displayed(1250)==LaneMarking::DASHED);
  observed.boundaries[3]=LaneMarking::UNKNOWN;tracker.update(observed,lane,at(1500));assert(displayed(1500)==LaneMarking::DASHED);
  tracker.update(observed,lane,at(1750));assert(displayed(1750)==LaneMarking::UNKNOWN);
  observed.boundaries[3]=LaneMarking::SOLID;tracker.update(observed,lane,at(2000));tracker.update(observed,lane,at(2250));
  assert(displayed(2901)==LaneMarking::UNKNOWN);
  lane.identity_uncertain=true;tracker.update(observed,lane,at(2500));assert(displayed(2500)==LaneMarking::UNKNOWN);
  lane.identity_uncertain=false;tracker.update(observed,lane,at(2750));assert(displayed(2750)==LaneMarking::UNKNOWN);
  tracker.update(observed,lane,at(3000));assert(displayed(3000)==LaneMarking::SOLID);
  std::cout<<"Outer style PASS: two confirmations, one missing frame grace, two misses clear, type transition,650ms expiry, identity reset; ego evidence unchanged\n";
}
