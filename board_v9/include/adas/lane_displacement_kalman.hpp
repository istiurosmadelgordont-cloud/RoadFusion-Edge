#pragma once
#include <algorithm>
#include <cmath>

namespace adas {
// Constant-velocity Kalman model in lane-width units. The position origin is
// rebased to the last displayed curve after each accepted update; covariance
// and velocity remain in the same coordinates. No missing paint is invented.
class LaneDisplacementKalman {
 public:
  struct Update {bool accepted=false;float displacement=0;};
  void reset(float sigma=.03f) {
    velocity_=0;position_variance_=sigma*sigma;cross_variance_=0;
    velocity_variance_=.0016f;accepted_=0;
  }
  Update update(float measured_displacement,float elapsed,float sigma) {
    Update result;
    if(!std::isfinite(measured_displacement) || !std::isfinite(elapsed) ||
       !std::isfinite(sigma) || elapsed<=0 || sigma<=0)return result;
    const float dt=std::min(.75f,elapsed),q=.0144f,r=sigma*sigma;
    const float p00=position_variance_+2*dt*cross_variance_+dt*dt*velocity_variance_+.25f*dt*dt*dt*dt*q;
    const float p01=cross_variance_+dt*velocity_variance_+.5f*dt*dt*dt*q;
    const float p11=velocity_variance_+dt*dt*q;
    const float predicted=velocity_*dt,innovation=measured_displacement-predicted,s=p00+r;
    // A large isolated jump is a candidate, not a new confirmed curve.
    if(accepted_>=3 && std::abs(innovation)>.10f && innovation*innovation>9*s)return result;
    const float k0=p00/s,k1=p01/s;
    result.accepted=true;result.displacement=predicted+k0*innovation;
    velocity_=std::max(-.45f,std::min(.45f,velocity_+k1*innovation));
    position_variance_=std::max(1e-8f,(1-k0)*p00);
    cross_variance_=(1-k0)*p01;
    velocity_variance_=std::max(1e-8f,p11-k1*p01);
    ++accepted_;return result;
  }
 private:
  float velocity_=0,position_variance_=.0009f,cross_variance_=0,velocity_variance_=.0016f;
  int accepted_=0;
};
}
