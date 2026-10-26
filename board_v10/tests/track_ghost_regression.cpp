#include "adas/byte_tracker.hpp"
#include <cassert>
#include <iostream>
using adas::Detection;
static std::chrono::steady_clock::time_point at(int ms) {
  return std::chrono::steady_clock::time_point(std::chrono::seconds(1))+std::chrono::milliseconds(ms);
}
static Detection object(int cls,cv::Rect2f box) {
  Detection d;d.class_id=cls;d.score=.95f;d.box=box;return d;
}
int main() {
  for(int cls:{0,2,3,4,15,16,17,18,19,20}) {
    adas::ByteTracker tracker;const auto first=object(cls,{100,100,60,80});
    auto visible=tracker.update({first},at(0),at(250));assert(visible.size()==1);
    assert(tracker.predict(at(450)).size()==1); // bridge ordinary inter-result gap
    assert(tracker.update({},at(300),at(550)).empty()); // one real miss removes drawing
    assert(tracker.observations().empty());
    auto recovered=tracker.update({first},at(600),at(850));
    assert(recovered.size()==1 && recovered[0].track_id==visible[0].track_id);
  }
  {
    adas::ByteTracker tracker;auto car=object(2,{100,100,80,90});
    tracker.update({car},at(0),at(250));
    // A turn moves the target too far for association; only new geometry draws.
    car.box.x=240;
    auto visible=tracker.update({car},at(300),at(550));
    assert(visible.size()==1 && visible[0].box.x==240);
    assert(tracker.observations().size()==1);
  }
  {
    adas::ByteTracker tracker;auto car=object(2,{100,100,80,90});
    tracker.update({car},at(0),at(250));
    car.class_id=4;
    auto visible=tracker.update({car},at(300),at(550));
    assert(visible.size()==1 && visible[0].class_id==4); // no old car beneath new truck
  }
  {
    adas::ByteTracker tracker;auto sign=object(15,{100,100,30,40});
    tracker.update({sign},at(0),at(300));
    assert(tracker.predict(at(550)).size()==1);
    assert(tracker.predict(at(701)).empty()); // stopped inference is bounded by source age
  }
  {
    adas::ByteTracker tracker;auto car=object(2,{100,100,80,90});
    auto neighbor=car;neighbor.box.x=230;
    auto visible=tracker.update({car,neighbor},at(0),at(200));
    assert(visible.size()==2 && tracker.observations().size()==2); // real neighbours survive
  }
  std::cout<<"PASS: no ordinary ghost after real miss, sudden turn or class flip; bounded display gaps; distinct neighbours preserved\n";
}
