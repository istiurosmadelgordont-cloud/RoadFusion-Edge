#include "adas/fcw_monitor.hpp"
#include "adas/overlay.hpp"
#include "adas/adas_logic_v2.hpp"
#include <cassert>
#include <iostream>

adas::LaneResult road() {
  adas::LaneResult l;l.valid=true;l.boundary_coverage[1]=l.boundary_coverage[2]=.9f;
  for(int i=0;i<32;++i) {
    l.left.emplace_back(160+i*3,350-i*6);
    l.right.emplace_back(387+i*3,164+i*6);
  }
  l.polygon=l.left;l.polygon.insert(l.polygon.end(),l.right.begin(),l.right.end());return l;
}
adas::Detection car(float distance,int id=1,float cx=320,int cls=2) {
  adas::Detection d;d.class_id=cls;d.track_id=id;d.score=.95f;
  const float h=640*.30f*1.5f/distance;d.box={cx-16,320-h,32,h};return d;
}
int main() {
  auto lane=road();adas::FcwMonitor m;
  auto state=m.update({car(6)},640,360,lane,true,true,0,0);
  assert(!state.enabled && !state.risk.target);
  m.set_enabled(true);
  state=m.update({car(6)},640,360,lane,true,true,0,0);assert(!state.risk.reliable);
  state=m.update({car(6)},640,360,lane,true,true,0,.1);assert(!state.risk.reliable);
  state=m.update({car(6)},640,360,lane,true,true,.3,.35);assert(state.risk.reliable);
  state=m.update({car(6)},640,360,lane,true,true,.6,.65);
  assert(state.level==adas::FcwLevel::DANGER && state.risk.warning);
  assert(state.risk.distance_m>5.9 && state.risk.distance_m<6.1);
  // A real miss cannot retain red, a range label or a ghost vehicle.
  state=m.update({},640,360,lane,true,true,.9,.95);
  assert(!state.risk.target && state.level==adas::FcwLevel::UNKNOWN);
  // Vehicles in other lanes and pedestrians are excluded from this switch.
  state=m.update({car(6,2,60),car(6,3,320,0)},640,360,lane,true,true,1.2,1.25);
  assert(!state.risk.target);
  state=m.update({car(20,4)},640,360,lane,true,true,1.5,1.55);
  state=m.update({car(20,4)},640,360,lane,true,true,1.8,1.85);
  assert(state.level==adas::FcwLevel::NORMAL && state.risk.reliable);
  // A closer cut-in is re-confirmed; its speed never inherits the old lead.
  state=m.update({car(20,4),car(6,5)},640,360,lane,true,true,2.1,2.15);
  assert(state.risk.track_id==5 && !state.risk.reliable && state.risk.ttc_s<0);
  state=m.update({car(6,5)},640,360,lane,true,true,2.4,2.45);
  state=m.update({car(6,5)},640,360,lane,true,true,2.7,2.75);
  assert(state.level==adas::FcwLevel::DANGER);
  state=m.update({car(6,5)},640,360,lane,true,false,2.7,3.4);
  assert(!state.risk.target && state.status==adas::FcwStatus::STALE);
  auto unknown=lane;unknown.identity_uncertain=true;
  state=m.update({car(6)},640,360,unknown,true,true,3.6,3.65);
  assert(!state.risk.target && state.status==adas::FcwStatus::WAIT_LANE);
  // No ego-lane inference or expired lane means no center-screen fallback.
  state=m.update({car(6)},640,360,lane,false,true,3.9,3.95);assert(!state.risk.target);
  state=m.update({car(10,6)},640,360,lane,true,true,4.2,4.25);
  state=m.update({car(10,6)},640,360,lane,true,true,4.5,4.55);
  assert(state.level==adas::FcwLevel::CAUTION && !state.risk.warning);
  m.set_enabled(false);assert(!m.state().enabled && !m.state().risk.target);
  m.set_enabled(true);assert(m.state().level==adas::FcwLevel::UNKNOWN);
  // Approaching ranges produce positive relative closing speed, never ego speed.
  for(int i=0;i<12;++i)state=m.update({car(25-i,7)},640,360,lane,true,true,5.+i*.3,5.05+i*.3);
  assert(state.risk.relative_speed_kmh>0 && state.risk.ttc_s>0);
  adas::LaneSemantic semantics;semantics.left=adas::LaneMarking::DASHED;semantics.right=adas::LaneMarking::SOLID;
  cv::Mat image(360,640,CV_8UC3,cv::Scalar(40,40,40));
  cv::Mat green,yellow,red;
  for(auto level:{adas::FcwLevel::NORMAL,adas::FcwLevel::CAUTION,adas::FcwLevel::DANGER}) {
    adas::FcwPaint paint;paint.enabled=true;paint.level=level;paint.color=adas::fcw_color(level);
    auto out=image.clone();auto risk=state.risk;risk.box=cv::Rect(car(10,7).box);
    adas::draw_overlay(out,{car(10,7)},lane,{},risk,{},3.5,100,&semantics,nullptr,&paint);
    if(level==adas::FcwLevel::NORMAL)green=out;
    if(level==adas::FcwLevel::CAUTION)yellow=out;
    if(level==adas::FcwLevel::DANGER)red=out;
  }
  const cv::Rect fill(250,290,40,28);
  assert(cv::mean(green(fill))[1]>cv::mean(green(fill))[2]+10);
  assert(cv::mean(yellow(fill))[2]>cv::mean(green(fill))[2]+15);
  assert(cv::mean(red(fill))[2]>cv::mean(red(fill))[1]+10);
  assert(semantics.left==adas::LaneMarking::DASHED && semantics.right==adas::LaneMarking::SOLID);
  adas::FcwPaintTracker paint_tracker;state.enabled=true;state.level=adas::FcwLevel::NORMAL;
  auto p=paint_tracker.update(state,10);state.level=adas::FcwLevel::DANGER;p=paint_tracker.update(state,10.05);
  assert(p.color[1]>100 && p.color[2]>75 && p.color[2]<245);
  state.enabled=false;p=paint_tracker.update(state,10.1);assert(!p.enabled);
  std::cout<<"FCW_PASS: switch, measured-only confirmation, nearest ego vehicle, cut-in, true miss, expiry, lane gating, relative speed, three colours and animation\n";
}
