#include "adas/navigation_demo.hpp"
#include "adas/lane_signal_simulator.hpp"
#include <cassert>
#include <iostream>
int main(){
  using N=adas::NavigationDemo;using D=adas::ManeuverDirection;using I=adas::IntersectionAction;
  N n;assert(n.advice(I::GO,false).action==N::Action::DISABLED);
  const auto press=[&](cv::Rect r){assert(n.click({r.x+3,r.y+3}));};
  press(N::toggle_button());assert(n.enabled() && n.distance()==120 && n.direction()==D::RIGHT);
  assert(n.current_route()==D::STRAIGHT && n.advice(I::GO,false).action==N::Action::STRAIGHT);
  assert(n.advice(I::STOP,false).action==N::Action::STOP);
  assert(n.advice(I::UNKNOWN,false).action==N::Action::WAIT);
  assert(n.advice(I::WAIT,false).action==N::Action::WAIT);
  assert(n.advice(I::GO,true).action==N::Action::STOP);
  for(int i=0;i<4;++i)press(N::distance_buttons()[0]);
  assert(n.distance()==0 && n.current_route()==D::RIGHT);
  assert(n.advice(I::GO,false).action==N::Action::RIGHT);
  press(N::direction_buttons()[0]);assert(n.advice(I::GO,false).action==N::Action::LEFT);
  press(N::direction_buttons()[1]);assert(n.advice(I::GO,false).action==N::Action::STRAIGHT);
  n.adjust_distance(-999);assert(n.distance()==0);n.adjust_distance(9999);assert(n.distance()==990);
  press(N::distance_buttons()[2]);assert(n.distance()==120);
  assert(!n.click({1415,815})); // Test Two controls must not edit navigation.
  assert(!n.click({1420,700})); // Test One controls remain independent.
  // Fresh directional light matching is supplied by the existing simulator.
  adas::LaneSignalSimulator gate;gate.set_lane(D::RIGHT);
  adas::Detection wrong;wrong.class_id=14;wrong.score=.99;wrong.box={300,80,20,20}; // straight green
  for(double t:{0.,.3,.65})gate.observe(t,{wrong},640,360);
  assert(gate.action()!=I::GO);
  wrong.class_id=13; // right arrow green
  for(double t:{1.,1.3,1.65})gate.observe(t,{wrong},640,360);
  assert(gate.action()==I::GO);
  gate.tick(5.);assert(gate.action()!=I::GO);
  std::cout<<"PASS: mock navigation controls, preparation/turning, matching fresh lights, stop priority, no fabricated recognition\n";
}
