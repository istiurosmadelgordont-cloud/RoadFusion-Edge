#include "adas/manual_avm_clock.hpp"
#include <cassert>
#include <iostream>
int main() {
  adas::ManualAvmClock clock(20);
  assert(!clock.enabled() && !clock.playing() && clock.position(100)==0);
  clock.adjust(.5,100);assert(clock.position(101)==.5 && !clock.enabled());
  clock.toggle(101);assert(clock.playing() && clock.enabled());
  assert(clock.position(102)==1.5);
  clock.toggle(102);assert(!clock.playing() && clock.position(110)==1.5);
  clock.adjust(-.5,110);assert(clock.position(120)==1);
  clock.toggle(120);assert(clock.position(141)==2);
  clock.adjust(-3,141);assert(clock.position(141)==19);
  clock.reset(142);assert(!clock.enabled() && clock.position(150)==0);
  bool invalid=false;try{adas::ManualAvmClock bad(0);}catch(const std::invalid_argument&){invalid=true;}
  assert(invalid);
  std::cout<<"PASS: manual supplementation starts disabled; start/pause/resume/reset; fine offsets and wrap; no FPGA sync assertion\n";
}
