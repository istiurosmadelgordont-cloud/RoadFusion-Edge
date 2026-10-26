#include "adas/lane_paint_contrast.hpp"
#include <cassert>
#include <iostream>
using namespace adas;
int main(){
  std::vector<unsigned char> profile(72,1);assert(classify_contrast_profile(profile).label==LaneMarking::SOLID);
  for(int i=0;i<72;++i)profile[i]=i%24<12;assert(classify_contrast_profile(profile).label==LaneMarking::DASHED);
  std::fill(profile.begin(),profile.end(),0);assert(classify_contrast_profile(profile).label==LaneMarking::UNKNOWN);
  for(int i=0;i<72;i+=8)profile[i]=1;assert(classify_contrast_profile(profile).label==LaneMarking::UNKNOWN);
  profile.assign(72,1);std::fill(profile.begin(),profile.begin()+11,0);
  assert(classify_contrast_profile(profile).label==LaneMarking::UNKNOWN);
  assert(classify_contrast_profile(profile,true).label==LaneMarking::SOLID);
  profile.assign(72,1);std::fill(profile.begin()+28,profile.begin()+38,0);
  assert(classify_contrast_profile(profile,true).label==LaneMarking::UNKNOWN);
  profile.assign(72,0);std::fill(profile.begin()+8,profile.end()-8,1);
  assert(classify_contrast_profile(profile,true).label==LaneMarking::DASHED);
  profile.assign(72,1);std::fill(profile.end()-10,profile.end(),0);
  assert(classify_contrast_profile(profile,true).label==LaneMarking::UNKNOWN);
  profile.assign(36,1);std::fill(profile.begin(),profile.begin()+6,0);
  assert(classify_contrast_profile(profile,true).label==LaneMarking::UNKNOWN);
  LaneResult lane;lane.valid=true;
  for(int i=0;i<32;++i)lane.left.emplace_back(200,350-i*5);
  lane.right=lane.left;for(auto& p:lane.right)p.x=440;std::reverse(lane.right.begin(),lane.right.end());
  cv::Mat image(360,640,CV_8UC3,cv::Scalar(60,60,60));
  cv::line(image,{200,160},{200,350},{110,110,110},3);
  assert(observe_lane_markings_contrast(image,lane).left==LaneMarking::SOLID);
  image.setTo(cv::Scalar(60,60,60));
  for(int y=180;y<260;y+=24)cv::line(image,{208,y},{208,y+11},{110,110,110},3);
  assert(observe_lane_markings_contrast(image,lane).left==LaneMarking::DASHED);
  image.setTo(cv::Scalar(60,60,60));cv::rectangle(image,{178,160,45,190},{200,200,200},cv::FILLED);
  assert(observe_lane_markings_contrast(image,lane).left==LaneMarking::UNKNOWN);
  LaneSemanticTracker tracker;LaneMarkingObservation observation;observation.left=LaneMarking::DASHED;
  auto base=std::chrono::steady_clock::now();
  assert(!tracker.update(observation,lane,base).left_change_allowed);
  assert(tracker.update(observation,lane,base+std::chrono::milliseconds(300)).left_change_allowed);
  observation.left=LaneMarking::UNKNOWN;
  assert(!tracker.update(observation,lane,base+std::chrono::milliseconds(600)).left_change_allowed);
  observation.left=LaneMarking::DASHED;
  assert(!tracker.update(observation,lane,base+std::chrono::milliseconds(900)).left_change_allowed);
  assert(tracker.update(observation,lane,base+std::chrono::milliseconds(1200)).left_change_allowed);
  image.setTo(cv::Scalar(60,60,60));
  lane.boundaries[3]=lane.left;for(auto& p:lane.boundaries[3])p.x=560;
  cv::line(image,{560,205},{560,350},{110,110,110},3);
  const auto outer=observe_lane_markings_contrast(image,lane);
  assert(outer.boundaries[3]==LaneMarking::SOLID);
  assert(lane.lane_polygons[2].empty()); // Clipped raw polygon need not exist.
  tracker.reset();tracker.update(outer,lane,base);
  const auto confirmed=tracker.update(outer,lane,base+std::chrono::milliseconds(300));
  assert(confirmed.boundaries[3]==LaneMarking::SOLID);
  assert(!confirmed.left_change_allowed && !confirmed.right_change_allowed);
  {
    // Reproduce the current PCIe camera framing: all supported ego paint is
    // below y=.69H, with around .47 model coverage. The old fixed band saw
    // zero evidence and the semantic cache incorrectly kept previous dashes.
    LaneResult low;low.valid=true;low.partial=true;
    low.boundary_coverage[1]=low.boundary_coverage[2]=.47f;
    for(int i=0;i<32;++i)low.left.emplace_back(200,350-cvRound(i*2.5));
    low.right=low.left;for(auto& p:low.right)p.x=440;std::reverse(low.right.begin(),low.right.end());
    image.setTo(cv::Scalar(60,60,60));
    cv::line(image,{200,260},{200,352},{110,110,110},3);
    for(int y=270;y<350;y+=24)cv::line(image,{440,y},{440,y+11},{110,110,110},3);
    const auto actual=observe_lane_markings_contrast(image,low);
    assert(actual.left==LaneMarking::SOLID && actual.left_coverage>.83f);
    assert(actual.right==LaneMarking::DASHED);
    LaneSemanticTracker semantics;
    LaneMarkingObservation dashed;dashed.left=dashed.right=LaneMarking::DASHED;
    auto healthy=low;healthy.partial=false;
    semantics.update(dashed,healthy,base);
    assert(semantics.update(dashed,healthy,base+std::chrono::milliseconds(300)).left==LaneMarking::DASHED);
    const auto lost=semantics.update({},low,base+std::chrono::milliseconds(600));
    assert(!lost.lane_valid && lost.left==LaneMarking::UNKNOWN && lost.right==LaneMarking::UNKNOWN);
    assert(!lost.left_change_allowed && !lost.right_change_allowed);
    const auto first=semantics.update(actual,low,base+std::chrono::milliseconds(900));
    assert(first.lane_valid && first.left==LaneMarking::UNKNOWN && !first.left_change_allowed);
    const auto second=semantics.update(actual,low,base+std::chrono::milliseconds(1200));
    assert(second.left==LaneMarking::SOLID && !second.left_change_allowed);
    assert(second.right==LaneMarking::DASHED && second.right_change_allowed);
    LaneResult sparse=low;sparse.left.clear();sparse.right.clear();
    for(int i=0;i<16;++i)sparse.left.emplace_back(200,315-i);
    sparse.right=sparse.left;for(auto& p:sparse.right)p.x=440;
    assert(observe_lane_markings_contrast(image,sparse).left==LaneMarking::UNKNOWN);
    std::cout<<"LOW_ROAD_PAINT_PASS: adaptive observed span, solid/dashed distinction, invalid-cache reset and fresh reconfirmation\n";
  }
  std::cout<<"Lane paint PASS: solid/dashed/unknown, dim contrast, ridge offset, broad-object rejection, outer far-end occlusion, internal-gap rejection, fresh clipped outer evidence, confirmations and loss reset\n";
}
