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
    assert(!s.acknowledge(3.16)); // already released automatically
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
    const auto second=object(0,9,{360,140,30,160});
    feed(s,0,{ped,second,roadside});
    const auto& both=feed(s,.3,{ped,second,roadside});
    assert(both.brake && both.hazards.size()==2);
    assert(both.hazards[0].track_id==7 && both.hazards[1].track_id==9);
    const auto& one=feed(s,.6,{second,roadside});
    assert(one.brake && one.hazards.size()==1 && one.hazards[0].track_id==9);
    // One pedestrian leaving cannot release STOP while another remains.
    for(double t:{.9,1.2,1.5,1.8}) assert(feed(s,t,{second}).brake);
    for(double t:{2.1,2.4,2.7,3.0}) assert(feed(s,t,{roadside}).brake);
    const auto& clear=feed(s,3.2,{roadside});
    assert(!clear.brake && clear.can_resume && clear.hazards.empty());
    assert(clear.hazard==Test4Hazard::NONE && clear.track_id==-1);
    // Re-entry must be confirmed again; clearance is not a permanent bypass.
    assert(!feed(s,3.5,{ped}).brake);
    assert(feed(s,3.8,{ped}).brake && !s.current().can_resume);
    s.tick(4.5); assert(s.current().brake && s.current().hazards.empty());
  }
  {
    Test4Safety s; s.set_enabled(true);
    feed(s,0,{ped}); feed(s,.3,{ped});
    // Four fast clear observations are not one second of clearance.
    for(double t:{.4,.5,.6,.7}) assert(feed(s,t,{}).brake);
    for(int i=0;i<20;++i) s.update({},640,360,nullptr,false,false,.7,1.2);
    assert(s.current().brake);
    s.tick(1.4); assert(s.current().stale && s.current().brake);
    // A stale interval resets clearance, even with no intervening repaint.
    for(double t:{1.5,1.8,2.1,2.4}) assert(feed(s,t,{}).brake);
    assert(!feed(s,2.55,{}).brake);
  }
  {
    Test4Safety s; s.set_enabled(true);
    feed(s,0,{ped}); feed(s,.3,{ped});
    feed(s,.6,{}); feed(s,.9,{}); feed(s,1.2,{});
    feed(s,1.5,{ped}); assert(s.current().brake && !s.current().can_resume);
    // Any fresh danger, even before its second vote, restarts clearance.
    for(double t:{1.8,2.1,2.4,2.7}) assert(feed(s,t,{}).brake);
    assert(!feed(s,2.85,{}).brake);
  }
  {
    // Near-field pedestrians just beyond the earlier narrow fallback region
    // are included on both sides; genuinely roadside people remain excluded.
    for(float x:{169.f,439.f}) {
      Test4Safety s;s.set_enabled(true);
      const auto edge=object(0,20,{x,140,32,190});
      assert(!feed(s,0,{edge}).brake);
      assert(feed(s,.3,{edge}).brake && s.current().hazards.size()==1);
      assert(!s.current().lane_based);
    }
    Test4Safety s;s.set_enabled(true);
    const auto outside=object(0,21,{100,140,32,190});
    for(double t:{0.,.3,.6})assert(!feed(s,t,{outside}).brake);
    for(float x:{140.f,410.f}) {
      Test4Safety vehicle_s;vehicle_s.set_enabled(true);
      const auto parked=object(2,22,{x,140,90,190});
      for(double t:{0.,.3,.6})assert(!feed(vehicle_s,t,{parked}).brake);
    }
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
    // A 32px person in a 360px frame was below the old 36px size gate.
    // The slightly earlier policy must still require two measured votes.
    Test4Safety s;s.set_enabled(true);
    const auto farther=object(0,30,{312,145,16,32});
    assert(!feed(s,0,{farther}).brake);
    assert(feed(s,.3,{farther}).brake);
    assert(s.current().hazard==Test4Hazard::PEDESTRIAN);
    for(double t:{.6,.9,1.2,1.5})assert(feed(s,t,{}).brake);
    assert(!feed(s,1.65,{}).brake && s.current().can_resume);
  }
  {
    // A slightly more distant footpoint can enter the centre estimate;
    // roadside people and tiny distant boxes must not become blanket alarms.
    Test4Safety s;s.set_enabled(true);
    const auto far_centre=object(0,31,{312,125,16,30});
    assert(!feed(s,0,{far_centre}).brake);
    assert(feed(s,.3,{far_centre}).brake);
    for(const auto& ignored:{object(0,32,{70,125,16,32}),
                             object(0,33,{312,145,16,24}),
                             object(0,34,{312,100,16,30})}) {
      Test4Safety excluded;excluded.set_enabled(true);
      for(double t:{0.,.3,.6})assert(!feed(excluded,t,{ignored}).brake);
    }
  }
  {
    // Predict entry earlier only from repeated inward lateral measurements.
    Test4Safety s;s.set_enabled(true);
    assert(!feed(s,0,{object(0,35,{417,170,16,40})}).brake);
    assert(!feed(s,.3,{object(0,35,{406,170,16,40})}).brake);
    assert(feed(s,.6,{object(0,35,{395,170,16,40})}).brake);
    Test4Safety outward;outward.set_enabled(true);
    for(int i=0;i<3;++i)
      assert(!feed(outward,i*.3,{object(0,36,{406.f+i*11,170,16,40})}).brake);
  }
  {
    Test4Safety s;s.set_enabled(true);
    const auto a=object(0,37,{300,145,16,32});
    const auto b=object(0,38,{330,145,16,32});
    feed(s,0,{a,b});
    assert(feed(s,.3,{a,b}).brake && s.current().hazards.size()==2);
    for(double t:{.6,.9,1.2})assert(feed(s,t,{b}).brake);
    s.tick(2.);assert(s.current().brake && s.current().stale);
  }
  {
    Test4Safety s; s.set_enabled(true);
    auto bad=ped; bad.box.x=std::numeric_limits<float>::quiet_NaN();
    auto low=ped; low.score=.1f;
    auto sign=ped; sign.class_id=15;
    for(double t:{0.,.3,.6}) assert(!feed(s,t,{bad,low,sign}).brake);
    s.reset(); assert(s.current().enabled && !s.current().brake);
  }
  std::cout<<"PASS: Test4 earlier pedestrian warning, multiple hazards, roadside exclusion, fresh-only votes, identity, stale hold, sustained automatic release, re-entry and lane freshness\n";
}
