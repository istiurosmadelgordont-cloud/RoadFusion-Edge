#include "adas/camera_slot_order.hpp"
#include "adas/camera_view_layout.hpp"
#include "adas/light_verification_frames.hpp"
#include <cassert>
#include <iostream>

int main() {
  adas::CameraSlotOrder order("left,front,right,rear");
  const std::array<int,4> raw{{0,1,2,3}};
  const auto cameras=order.semantic(raw);
  assert((cameras==std::array<int,4>{{1,3,0,2}}));
  const auto screen=adas::CameraViewLayout::grid_rects();
  const cv::Point slot_screen[]={{32,218},{704,218},{32,666},{704,666}};
  cv::Mat composite(1080,1920,CV_8UC3);
  for(int slot=0;slot<4;++slot)
    composite(cv::Rect((slot%2)*960,(slot/2)*540,960,540)).setTo(cv::Scalar(25+slot*40,70,170));
  std::array<std::vector<adas::Detection>,4> detections;
  for(int i=0;i<4;++i) {
    assert(screen[i].tl()==slot_screen[order.slot(i)]); // No input image moves.
    const auto roi=order.roi(i,composite.size());
    assert(composite(roi).at<cv::Vec3b>(0,0)[0]==25+order.slot(i)*40);
    adas::Detection lamp;lamp.class_id=7;lamp.score=.9f;lamp.box={10,10,20,30};
    detections[i].push_back(lamp);
  }
  std::array<cv::Mat,4> hsv;
  adas::prepare_light_verification_frames(composite,detections,hsv,order.slots());
  for(int i=0;i<4;++i)assert(hsv[i].at<cv::Vec3b>(20,20)[0]==25+order.slot(i)*40);
  assert(order.roi(0,composite.size())==cv::Rect(960,0,960,540));
  assert(order.roi(1,composite.size())==cv::Rect(960,540,960,540));
  adas::CameraSlotOrder replay;
  assert(replay.semantic(raw)==raw); // Existing file camera identities stay intact.
  bool rejected=false;try{adas::CameraSlotOrder bad("front,front,left,right");}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);
  std::cout<<"PASS: fixed slot pixel positions; semantic detections; native front/rear ROI; matching HSV input; replay unchanged\n";
}
