#include "adas/box_motion_compensator.hpp"
#include <cassert>
#include <iostream>
using namespace adas;
static BoxMotionCompensator::Time at(int ms) {
  return BoxMotionCompensator::Time(std::chrono::seconds(1))+std::chrono::milliseconds(ms);
}
int main() {
  cv::setNumThreads(1);
  cv::Mat source(360,640,CV_8UC3,cv::Scalar(30,30,30));
  cv::RNG rng(123);rng.fill(source(cv::Rect(180,140,120,100)),cv::RNG::UNIFORM,0,255);
  Detection car;car.track_id=8;car.class_id=2;car.score=.95f;car.box={180,140,120,100};
  BoxMotionCompensator motion;motion.observe(motion.reference(source),{car},at(0));
  auto transform=[](const cv::Mat& in,float dx,float dy,float scale=1.f) {
    cv::Mat out;cv::Mat affine=(cv::Mat_<double>(2,3)<<scale,0,dx,0,scale,dy);
    cv::warpAffine(in,out,affine,in.size());return out;
  };
  for(float dx:{16.f,32.f,-24.f}) {
    auto measured=std::vector<Detection>{car};const auto current=transform(source,dx,8);
    assert(motion.apply(current,at(300),measured)==1);
    assert(std::abs(measured[0].box.x-(180+dx))<3 && std::abs(measured[0].box.y-148)<3);
    assert(measured[0].track_id==8 && measured[0].class_id==2);
  }
  std::vector<Detection> predicted{car};predicted[0].box.x+=40;
  assert(motion.apply(source,at(400),predicted)==1);
  assert(std::abs(predicted[0].box.x-180)<2); // stopped object must not coast on stale velocity
  auto scaled=std::vector<Detection>{car};
  assert(motion.apply(transform(source,-12,-7,1.08f),at(350),scaled)==1);
  assert(std::abs(scaled[0].box.width-129.6f)<4);
  std::vector<Detection> missing;
  assert(motion.apply(source,at(350),missing)==0 && missing.empty()); // never resurrect old tracks
  auto expired=std::vector<Detection>{car};
  assert(motion.apply(source,at(651),expired)==0 && expired[0].box==car.box);
  assert(motion.apply(source,at(-1),expired)==0);
  auto replacement=std::vector<Detection>{car};replacement[0].track_id=9;
  assert(motion.apply(source,at(300),replacement)==0);
  auto occluded=std::vector<Detection>{car};
  cv::Mat blank(source.size(),source.type(),cv::Scalar(0));
  assert(motion.apply(blank,at(300),occluded)==0);
  motion.observe(motion.reference(source),{},at(100));
  assert(motion.apply(source,at(300),predicted)==0);
  car.class_id=7;motion.observe(motion.reference(source),{car},at(0));
  auto lamp=std::vector<Detection>{car};assert(motion.apply(source,at(300),lamp)==0);
  std::cout<<"PASS: pixel-aligned motion, turn direction, stopped velocity, scale, occlusion, no resurrection, expiry, identity and lamps\n";
}
