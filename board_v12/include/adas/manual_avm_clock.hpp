#pragma once
#include <cmath>
#include <stdexcept>
namespace adas {
// Presentation-only playback clock. It does not assert FPGA timestamp sync.
class ManualAvmClock {
 public:
  explicit ManualAvmClock(double duration):duration_(duration) {
    if(!std::isfinite(duration) || duration<1 || duration>600)throw std::invalid_argument("invalid AVM duration");
  }
  double position(double now) const { return wrap(offset_+(playing_?now-epoch_:0)); }
  bool enabled() const {return enabled_;}
  bool playing() const {return playing_;}
  void toggle(double now) {
    offset_=position(now);epoch_=now;enabled_=true;playing_=!playing_;
  }
  void reset(double now) {offset_=0;epoch_=now;enabled_=false;playing_=false;}
  void adjust(double delta,double now) {
    if(!std::isfinite(delta))return;
    offset_=wrap(position(now)+delta);epoch_=now;
  }
 private:
  double wrap(double value) const {value=std::fmod(value,duration_);return value<0?value+duration_:value;}
  double duration_,offset_=0,epoch_=0;
  bool playing_=false,enabled_=false;
};
}  // namespace adas
