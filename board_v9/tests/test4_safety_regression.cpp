#include "adas/test4_safety.hpp"
#include <cassert>
#include <iostream>
#include <limits>

using adas::Test4Safety;
using adas::Test4Hazard;
adas::Detection object(int cls, int id, cv::Rect2f box) {
  adas::Detection d; d.class_id=cls; d.track_id=id; d.score=.9f; d.box=box; return d;
}
const adas::Test4Result& feed(Test4Safety& s, double t,
    std::vector<adas::Detection> detections, const adas::LaneResult* lane=nullptr, bool lane_fresh=false) {
  return s.update(detections,640,360,lane,lane_fresh,true,t,t);
}
int main() {
  const auto ped=object(0,7,{300,140,40,160});
  const auto roadside=object(0,8,{30,140,40,160});
  {
    Test4Safety s;
    feed(s,0,{ped}); assert(!feed(s,.3,{ped}).brake);
    s.set_enabled(true);
    for(double t:{1.,1.3,1.6,1.9}) assert(!feed(s,t,{roadside}).brake);
    assert(!feed(s,2.2,{ped}).brake);
    assert(feed(s,2.5,{ped}).brake);
    assert(s.current().hazard==Test4Hazard::PEDESTRIAN);
  }
  {
    Test4Safety s; s.set_enabled(true);
    feed(s,0,{ped});
    // Repaint, repeated source timestamp and stale arrivals are not votes.
    for(int i=0;i<20;++i) s.update({ped},640,360,nullptr,false,false,0,.1);
    assert(!s.current().brake);
    assert(!feed(s,0,{ped}).brake);
    s.update({ped},640,360,nullptr,false,true,.2,.8);
    assert(!s.current().brake);
    assert(!feed(s,1.,{ped}).brake); // gap resets confirmation
    assert(feed(s,1.3,{ped}).brake);
    assert(!s.acknowledge(1.31));
    s.tick(2.); assert(s.current().brake && s.current().stale && !s.current().target_fresh);
    assert(!s.acknowledge(2.)); // stale data cannot release STOP
    for(double t:{2.1,2.45,2.8}) { feed(s,t,{}); assert(!s.current().can_resume); }
    assert(feed(s,3.15,{}).can_resume);
    assert(s.acknowledge(3.16));
    assert(!s.current().brake && s.current().enabled);
  }
  {
    Test4Safety s; s.set_enabled(true);
    auto other=ped; other.track_id=9;
    assert(!feed(s,0,{ped}).brake);
    assert(!feed(s,.3,{other}).brake); // identities do not share votes
    assert(!feed(s,.6,{ped}).brake); // one real miss breaks consecutive danger votes
    assert(feed(s,.9,{ped}).brake);
    feed(s,1.2,{}); feed(s,1.55,{}); feed(s,1.9,{});
    feed(s,2.25,{ped}); assert(!s.current().can_resume); // danger interrupts release
    s.set_enabled(false); assert(!s.current().brake && !s.current().enabled);
  }
  {
    Test4Safety s; s.set_enabled(true);
    const auto near=object(2,10,{260,145,120,160});
    const auto adjacent=object(2,11,{20,145,120,160});
    for(double t:{0.,.3,.6}) assert(!feed(s,t,{adjacent}).brake);
    assert(!feed(s,.9,{near}).brake);
    assert(feed(s,1.2,{near}).brake);
    assert(s.current().hazard==Test4Hazard::FRONT_OBSTACLE);
  }
  {
    Test4Safety s; s.set_enabled(true);
    for(double t:{0.,.3,.6}) assert(!feed(s,t,{object(2,10,{280,200,80,60})}).brake);
    // Expansion TTC detects approach before the near-image size condition.
    feed(s,.9,{object(2,12,{280,230,80,30})});
    feed(s,1.2,{object(2,12,{280,215,80,45})});
    assert(feed(s,1.5,{object(2,12,{280,200,80,60})}).brake);
    assert(s.current().expansion_ttc_s>0 && s.current().expansion_ttc_s<3);
  }
  {
    Test4Safety s; s.set_enabled(true);
    adas::LaneResult lane; lane.valid=true; lane.polygon={{440,150},{500,150},{630,359},{400,359}};
    for(double t:{0.,.3,.6}) assert(!feed(s,t,{ped},&lane,true).brake);
    assert(s.current().lane_based);
    // Old/ambiguous lane display geometry cannot establish the driving region.
    assert(!feed(s,.9,{ped},&lane,false).brake);
    assert(feed(s,1.2,{ped},&lane,false).brake);
    assert(!s.current().lane_based);
  }
  {
    Test4Safety s; s.set_enabled(true);
    auto bad=ped; bad.box.x=std::numeric_limits<float>::quiet_NaN();
    auto low=ped; low.score=.1f;
    auto sign=ped; sign.class_id=15;
    for(double t:{0.,.3,.6}) assert(!feed(s,t,{bad,low,sign}).brake);
    s.reset(); assert(s.current().enabled && !s.current().brake);
  }
  std::cout<<"PASS: Test4 pedestrian/vehicle hazards, roadside exclusion, fresh-only votes, identity, stale hold, manual release and lane freshness\n";
}
