#include "adas/lane_visual_tracker_before.hpp"
#include "adas/lane_visual_tracker.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>

struct Sample {std::string scene;double seconds=0;adas::LaneResult lane;};
int main(int argc,char** argv){
  if(argc!=3)return 2;std::ifstream input(argv[1]);std::ofstream output(argv[2]);
  output<<"scene,case,seconds,before_left,before_right,after_left,after_right\n";
  std::vector<Sample> samples;std::string line;
  while(std::getline(input,line)){
    std::istringstream row(line);Sample s;int valid,partial;row>>s.scene>>s.seconds>>valid>>partial;s.lane.valid=valid;s.lane.partial=partial;
    for(int slot=0;slot<4;++slot){int count;row>>s.lane.boundary_coverage[slot]>>count;
      for(int i=0;i<count;++i){int x,y;row>>x>>y;s.lane.boundaries[slot].emplace_back(x,y);}}
    s.lane.left=s.lane.boundaries[1];s.lane.right.assign(s.lane.boundaries[2].rbegin(),s.lane.boundaries[2].rend());
    s.lane.polygon=s.lane.left;s.lane.polygon.insert(s.lane.polygon.end(),s.lane.right.begin(),s.lane.right.end());
    if(!row)return 3;samples.push_back(s);
  }
  using Clock=std::chrono::steady_clock;auto origin=Clock::time_point(std::chrono::seconds(1));
  auto at=[&](double s){return origin+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));};
  for(int mode=0;mode<2;++mode){
    adas::LaneVisualTrackerBefore before;adas::LaneVisualTracker after;
    size_t next=0;std::string scene;adas::LaneResult last;double started=0,end=0;
    while(next<samples.size()){
      scene=samples[next].scene;size_t begin=next;while(next<samples.size()&&samples[next].scene==scene)++next;size_t limit=next;
      before.reset();after.reset();last=adas::LaneResult();size_t update=begin;int seen=0;
      started=samples[begin].seconds;end=samples[limit-1].seconds+.55;
      for(double now=started;now<end;now+=1./11){
        while(update<limit && samples[update].seconds+.22<=now){
          auto measured=samples[update].lane;
          // Derive a one-update right outer dropout every five detector jobs.
          // The ego pair remains observed; no changed lane shape is invented.
          if(mode==1 && seen%5==3){measured.boundaries[3].clear();measured.boundary_coverage[3]=0;}
          last=measured;before.update(measured,measured,at(samples[update].seconds));
          after.update(measured,measured,at(samples[update].seconds),640,360,at(now));
          ++update;++seen;
        }
        auto a=before.render(last,at(now)),b=after.render(last,at(now));
        output<<scene<<','<<(mode==0?"original":"one_update_dropout")<<','<<std::fixed<<std::setprecision(3)<<now<<','
            <<!a.lane_polygons[0].empty()<<','<<!a.lane_polygons[2].empty()<<','<<!b.lane_polygons[0].empty()<<','<<!b.lane_polygons[2].empty()<<'\n';
      }
    }
  }
  std::cout<<"Timeline replay complete: fixed real geometry,220ms completion lag,3.33FPS updates,11FPS display\n";
}
