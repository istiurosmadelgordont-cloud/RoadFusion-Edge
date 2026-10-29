#include "adas/light_verification_frames.hpp"
#include "adas/experimental_adas.hpp"
#include <cassert>
#include <iostream>
int main(){
  cv::setNumThreads(1);cv::Mat source(1080,1920,CV_8UC3);cv::randu(source,0,256);
  std::array<std::vector<adas::Detection>,4> detections;std::array<cv::Mat,4> frames;
  adas::prepare_light_verification_frames(source,detections,frames);
  for(const auto& frame:frames)assert(frame.empty());
  cv::Mat reference;adas::resize_preview(source,reference);
  for(int view=0;view<4;++view){
    detections={};adas::Detection lamp;lamp.class_id=7;lamp.score=.9f;lamp.box={120,80,20,30};detections[view].push_back(lamp);
    adas::prepare_light_verification_frames(source,detections,frames);
    for(int k=0;k<4;++k)if(k==view){
      cv::Mat expected=reference(cv::Rect((k%2)*640,(k/2)*360,640,360));
      assert(!frames[k].empty() && cv::norm(expected,frames[k],cv::NORM_INF)==0);
      int ca=0,ra=0,cb=0,rb=0;
      auto a=adas::verify_light_colors(expected,detections[k],&ca,&ra),b=adas::verify_light_colors(frames[k],detections[k],&cb,&rb);
      assert(a.size()==b.size() && ca==cb && ra==rb);
    }else assert(frames[k].empty());
  }
  detections={};adas::Detection low;low.class_id=7;low.score=.399f;detections[0].push_back(low);
  low.class_id=21;low.score=.99f;detections[1].push_back(low);
  adas::prepare_light_verification_frames(source,detections,frames);for(const auto& frame:frames)assert(frame.empty());
  std::cout<<"HSV frame PASS: empty fast path, all four view mappings byte-identical, same verification, candidate threshold unchanged\n";
}
