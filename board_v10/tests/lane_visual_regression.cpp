#include "adas/lane_visual_tracker.hpp"
#include <cassert>
#include <iostream>
using namespace adas;
LaneResult scene(bool outer=true) {
  LaneResult l;l.valid=true;l.boundary_coverage={{.5f,.9f,.9f,.5f}};
  for(int i=0;i<32;++i){int y=350-i*5;int left=120+(350-y),right=520-(350-y);
    l.left.emplace_back(left,y);l.right.emplace_back(right,y);
    if(outer){l.boundaries[0].emplace_back(left-(right-left),y);l.boundaries[3].emplace_back(right+(right-left),y);}}
  std::reverse(l.right.begin(),l.right.end());l.polygon=l.left;l.polygon.insert(l.polygon.end(),l.right.begin(),l.right.end());
  l.boundaries[1]=l.left;l.boundaries[2].assign(l.right.rbegin(),l.right.rend());return l;
}
LaneResult motion_scene(float ratio) {
  LaneResult l;l.valid=true;l.boundary_coverage={{.5f,.9f,.9f,.5f}};
  for(int i=0;i<32;++i){const int y=320-i*4,left=cvRound(260-.4*(y-200)),right=640-left,w=right-left;
    l.left.emplace_back(left,y);l.right.emplace_back(right,y);
    l.boundaries[0].emplace_back(cvRound(left-ratio*w),y);l.boundaries[3].emplace_back(cvRound(right+ratio*w),y);}
  std::reverse(l.right.begin(),l.right.end());l.boundaries[1]=l.left;l.boundaries[2].assign(l.right.rbegin(),l.right.rend());
  l.polygon=l.left;l.polygon.insert(l.polygon.end(),l.right.begin(),l.right.end());return l;
}
float boundary_x(const LaneResult& lane) {
  auto p=lane.boundaries[3];assert(p.size()>5);std::sort(p.begin(),p.end(),[](cv::Point a,cv::Point b){return a.y<b.y;});
  for(size_t i=1;i<p.size();++i)if(p[i].y>=260)return p[i-1].x+(p[i].x-p[i-1].x)*float(260-p[i-1].y)/(p[i].y-p[i-1].y);
  assert(false);return 0;
}
int main(){
  using Clock=std::chrono::steady_clock;const auto base=Clock::time_point(std::chrono::seconds(1));
  auto at=[base](int ms){return base+std::chrono::milliseconds(ms);};
  LaneVisualTracker t;auto l=scene();
  t.update(l,l,at(0));assert(t.render(l,at(0)).lane_polygons[0].empty());
  t.update(l,l,at(330));auto v=t.render(l,at(340));
  assert(!v.lane_polygons[0].empty() && !v.lane_polygons[2].empty());
  for(int slot:{0,3})for(auto p:v.boundaries[slot])assert(p.x>=0&&p.x<640);
  // A partly off-screen outer curve survives only within supported geometry.
  assert(v.boundaries[0].front().y<350 && v.boundaries[0].back().y>=195);
  auto missing=scene(false);missing.boundary_coverage[0]=missing.boundary_coverage[3]=0;
  t.update(missing,missing,at(660));v=t.render(missing,at(700));
  assert(!v.lane_polygons[0].empty() && v.boundary_coverage[0]==.5f);
  assert(t.render(missing,at(1240)).lane_polygons[0].empty());
  // The actual board delivers each 330ms update about240ms after exposure.
  // Keep one missed outer update until the next result can arrive, then expire.
  t.reset();t.update(l,l,at(0),640,360,at(240));t.update(l,l,at(330),640,360,at(570));
  t.update(missing,missing,at(660),640,360,at(900));
  assert(!t.render(missing,at(1100)).lane_polygons[2].empty());
  assert(t.render(missing,at(1231)).lane_polygons[2].empty());
  auto supported=LaneVisualTracker::supported_observation(l,640,360);
  assert(!supported.lane_polygons[2].empty());
  assert(l.lane_polygons[2].empty()); // observed decision geometry is untouched
  for(auto point:supported.boundaries[3])assert(point.x>=0 && point.x<640);
  t.reset();t.update(l,l,at(0));t.update(l,l,at(330));
  auto crossing=l;crossing.boundaries[0]=l.boundaries[2];
  t.update(crossing,crossing,at(660));assert(t.render(crossing,at(660)).lane_polygons[0].empty());
  assert(!t.render(crossing,at(660)).lane_polygons[2].empty());
  l.identity_uncertain=true;t.update(l,l,at(700));assert(t.render(l,at(700)).lane_polygons[2].empty());
  l.identity_uncertain=false;t.update(l,l,at(0));assert(t.render(l,at(0)).lane_polygons[0].empty());
  // A single coherent but displaced outer detection must not move the display;
  // two consistent observations of a sustained change must still take effect.
  t.reset();auto stable=motion_scene(.8f),jump=motion_scene(1.05f);
  for(int i=0;i<6;++i)t.update(stable,stable,at(i*300));
  const float old_x=boundary_x(t.render(stable,at(1500)));
  t.update(jump,jump,at(1800));assert(std::abs(boundary_x(t.render(jump,at(1800)))-old_x)<2);
  t.update(stable,stable,at(2100));assert(std::abs(boundary_x(t.render(stable,at(2100)))-old_x)<2);
  t.update(jump,jump,at(2400));t.update(jump,jump,at(2700));
  assert(boundary_x(t.render(jump,at(2700)))>old_x+20);
  jump.identity_uncertain=true;t.update(jump,jump,at(3000));assert(t.render(jump,at(3000)).lane_polygons[2].empty());
  t.update(stable,stable,at(3300));assert(t.render(stable,at(3300)).lane_polygons[2].empty());
  t.update(stable,stable,at(3600));assert(std::abs(boundary_x(t.render(stable,at(3600)))-old_x)<2);
  std::cout<<"Lane visual PASS: confirmation, clipped span, independent sides, measured coverage, latency-aware bounded expiry, fresh paint geometry, contradiction and reset\n";
}
