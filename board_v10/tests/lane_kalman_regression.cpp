#include "adas/lane_displacement_kalman.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
  adas::LaneDisplacementKalman filter;filter.reset();
  const float noise[]={-.03f,.025f,-.015f,.035f,-.025f,.01f};
  float position=0,ema=0;double mse=0,ema_mse=0;
  for(int i=0;i<160;++i){float observation=noise[i%6];auto update=filter.update(observation-position,.25f,.03f);
    assert(update.accepted);position+=update.displacement;ema=.67f*observation+.33f*ema;
    if(i>20){mse+=position*position;ema_mse+=ema*ema;}}
  assert(mse<ema_mse*.80);
  const auto outlier=filter.update(.35f,.25f,.03f);assert(!outlier.accepted);
  const auto recovery=filter.update(-position,.25f,.03f);assert(recovery.accepted);
  filter.reset();position=0;double moving_mse=0;float maximum_lag=0;
  for(int i=0;i<120;++i){float truth=i*.25f*.025f,observation=truth+noise[i%6]*.3f;
    auto update=filter.update(observation-position,.25f,.02f);assert(update.accepted);position+=update.displacement;
    if(i>20){moving_mse+=(truth-position)*(truth-position);maximum_lag=std::max(maximum_lag,std::abs(truth-position));}}
  assert(maximum_lag<.02f);
  assert(!filter.update(NAN,.25f,.03f).accepted);
  assert(!filter.update(0,0,.03f).accepted);
  filter.reset();assert(filter.update(.20f,.25f,.03f).accepted);
  std::cout<<"Kalman PASS: stationary RMS="<<std::sqrt(mse/139)<<" vs EMA="<<std::sqrt(ema_mse/139)
           <<"; moving max error="<<maximum_lag<<" lane widths; isolated jump rejection, recovery and reset\n";
}
