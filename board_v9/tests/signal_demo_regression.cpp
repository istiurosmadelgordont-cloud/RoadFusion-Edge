#include "adas/signal_demo.hpp"
#include <cassert>
#include <iostream>

// 使用固定 ROI 和合成检测验证状态机；不依赖视频、模型或 NPU 推理结果。
// 重点防止重复 UI 帧累计确认，以及方向切换后沿用上一方向的绿灯授权。
adas::Detection lamp(int id,float score=.99f,float x=.5f) {
  adas::Detection d;d.class_id=id;d.score=score;d.box=cv::Rect2f(x*100,50,8,8);return d;
}
adas::SignalDemo fresh(const char* file) { adas::SignalDemo d;assert(d.load(file));return d; }
void turn_green(adas::SignalDemo& d) {
  d.observe(0,{lamp(7)},100,100);d.observe(.4,{lamp(7)},100,100);
  assert(d.action()==adas::IntersectionAction::STOP);
  d.observe(.8,{lamp(11)},100,100);d.observe(1.2,{lamp(11)},100,100);
  assert(d.action()!=adas::IntersectionAction::GO);
  d.observe(1.6,{lamp(11)},100,100);assert(d.action()==adas::IntersectionAction::GO);
}
int main(int argc,char** argv) {
  assert(argc==2);
  auto d=fresh(argv[1]);
  d.observe(0,{lamp(7)},100,100);d.observe(0,{lamp(7)},100,100);
  for(int i=0;i<10;++i)d.tick(i*.01);
  assert(d.action()!=adas::IntersectionAction::STOP && d.confirmations()==1);
  d.observe(.4,{lamp(7)},100,100);assert(d.action()==adas::IntersectionAction::STOP);
  d.observe(.8,{},100,100);d.tick(2.3);assert(d.action()==adas::IntersectionAction::STOP);
  d.tick(2.5);assert(d.action()!=adas::IntersectionAction::STOP);
  d=fresh(argv[1]);turn_green(d);assert(d.moving());d.tick(2.5);assert(d.moving());
  d.tick(3.2);assert(!d.moving());
  d.observe(3.3,{lamp(11)},100,100);assert(d.action()!=adas::IntersectionAction::GO);
  d.observe(3.7,{lamp(11)},100,100);d.observe(4.1,{lamp(11)},100,100);assert(d.moving());
  d.observe(4.5,{lamp(7)},100,100);assert(!d.moving());
  d=fresh(argv[1]);d.observe(0,{lamp(7)},100,100);d.observe(.5,{},100,100);
  assert(d.action()!=adas::IntersectionAction::STOP && d.confirmations()==1);
  d.observe(1.1,{lamp(7)},100,100);assert(d.action()==adas::IntersectionAction::STOP);
  d=fresh(argv[1]);turn_green(d);d.observe(2,{lamp(7),lamp(11)},100,100);assert(!d.moving());
  d=fresh(argv[1]);d.observe(0,{lamp(11,.65f)},100,100);d.observe(.4,{lamp(11,.65f)},100,100);
  d.observe(.8,{lamp(11,.65f)},100,100);assert(!d.moving());
  d=fresh(argv[1]);d.observe(0,{lamp(7,.99f,.1f)},100,100);d.observe(.4,{lamp(7,.99f,.1f)},100,100);
  assert(d.action()!=adas::IntersectionAction::STOP);
  d.observe(.8,{lamp(7)},100,100);d.observe(1.2,{lamp(7)},100,100);d.set_motion(1);assert(d.alarm());
  d.set_direction(adas::ManeuverDirection::LEFT);assert(d.direction()==adas::ManeuverDirection::LEFT);
  d.set_direction(adas::ManeuverDirection::RIGHT);assert(d.direction()==adas::ManeuverDirection::RIGHT);
  d.set_motion(2);assert(!d.alarm());d.reset();assert(!d.moving() && d.confirmations()==0);
  d=fresh(argv[1]);d.set_direction(adas::ManeuverDirection::LEFT);
  d.observe(0,{lamp(13)},100,100);d.observe(.4,{lamp(13)},100,100);d.observe(.8,{lamp(13)},100,100);
  assert(d.action()!=adas::IntersectionAction::GO);  // right arrow cannot permit left
  d.observe(1.2,{lamp(12)},100,100);d.observe(1.6,{lamp(12)},100,100);d.observe(2,{lamp(12)},100,100);
  assert(d.action()==adas::IntersectionAction::GO);
  d.set_direction(adas::ManeuverDirection::RIGHT);assert(d.action()!=adas::IntersectionAction::GO);
  d.observe(2.4,{lamp(8)},100,100);d.observe(2.8,{lamp(8)},100,100);
  assert(d.action()!=adas::IntersectionAction::STOP); // left red doesn't govern right
  d.observe(3.2,{lamp(9)},100,100);d.observe(3.6,{lamp(9)},100,100);assert(d.action()==adas::IntersectionAction::STOP);
  d=fresh(argv[1]);turn_green(d);d.set_direction(adas::ManeuverDirection::LEFT);assert(!d.moving());
  d.observe(2,{lamp(11)},100,100);d.observe(2.4,{lamp(11)},100,100);d.observe(2.8,{lamp(11)},100,100);assert(d.moving());
  d.enable_full_view();d.observe(0,{lamp(12)},100,100);d.observe(.4,{lamp(12)},100,100);d.observe(.8,{lamp(12)},100,100);assert(d.moving());
  std::cout << "SIGNAL_DEMO_DIRECTION_FILTER_PASS" << std::endl;
  std::cout << "SIGNAL_DEMO_REGRESSION_PASS" << std::endl;
}
