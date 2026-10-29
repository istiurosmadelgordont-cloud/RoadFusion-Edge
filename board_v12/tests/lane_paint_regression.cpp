#include "adas/lane_paint_contrast.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cassert>
#include <iostream>
using namespace adas;
int main(int argc,char** argv){
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
  // A single dash fills the old far-road band. The longer measured span
  // must include the gap and next dash, rather than authorize SOLID.
  image.setTo(cv::Scalar(60,60,60));
  cv::line(image,{200,160},{200,350},{110,110,110},3);
  cv::line(image,{440,180},{440,248},{110,110,110},3);
  cv::line(image,{440,290},{440,310},{110,110,110},3);
  const auto long_dash=observe_lane_markings_contrast(image,lane);
  assert(long_dash.left==LaneMarking::SOLID && long_dash.right==LaneMarking::DASHED);
  // Low-horizon footage still has a measured road region below the old band.
  LaneResult low;low.valid=true;
  for(int y=270;y<=350;y+=2){low.left.emplace_back(200,y);low.right.emplace_back(440,y);}
  image.setTo(cv::Scalar(60,60,60));
  cv::line(image,{200,265},{200,350},{110,110,110},3);
  for(int y:{270,300,330})cv::line(image,{440,y},{440,y+12},{110,110,110},3);
  const auto lower=observe_lane_markings_contrast(image,low);
  assert(lower.left==LaneMarking::SOLID && lower.right==LaneMarking::DASHED);
  // Broad vehicle-body occlusion is not a dashed gap in otherwise solid paint.
  image.setTo(cv::Scalar(60,60,60));
  cv::line(image,{200,160},{200,350},{110,110,110},3);
  cv::line(image,{440,160},{440,350},{110,110,110},3);
  cv::rectangle(image,cv::Rect(0,270,640,90),cv::Scalar(15,15,15),cv::FILLED);
  const auto occluded=observe_lane_markings_contrast(image,lane);
  assert(occluded.left==LaneMarking::SOLID && occluded.right==LaneMarking::SOLID);
  // If the remaining road is too short, an occluded near field cannot invent
  // confident paint evidence for a low-horizon or clipped observation.
  const auto hidden=observe_lane_markings_contrast(image,low);
  assert(hidden.left==LaneMarking::UNKNOWN && hidden.right==LaneMarking::UNKNOWN);
  if(argc==2)for(int index=0;index<10;++index){
    const std::string stem=std::string(argv[1])+cv::format("/sample_%02d",index);
    cv::Mat captured_image=cv::imread(stem+"_front.png");assert(!captured_image.empty());
    cv::FileStorage file(stem+"_lane.yml",cv::FileStorage::READ);assert(file.isOpened());
    LaneResult captured;captured.valid=true;file["left"]>>captured.left;file["right"]>>captured.right;
    const auto observed=observe_lane_markings_contrast(captured_image,captured);
    std::cout<<"CAPTURE_EVIDENCE sample="<<index<<" left="<<int(observed.left)
      <<" right="<<int(observed.right)<<" coverage="<<observed.left_coverage<<','<<observed.right_coverage<<'\n';
    assert(observed.right==LaneMarking::DASHED);
    assert(observed.left!=LaneMarking::DASHED);
  }
  std::cout<<"Lane paint PASS: solid/dashed/unknown, dim contrast, ridge offset, broad-object rejection, outer far-end occlusion, internal-gap rejection, fresh clipped outer evidence, confirmations and loss reset\n";
}
