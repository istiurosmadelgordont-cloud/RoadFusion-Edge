#pragma once
#include <array>
#include <string>
#include <stdexcept>
#include <opencv2/core.hpp>

namespace adas {
// Semantic indices stay front/rear/left/right throughout perception. This
// adapter describes input slots in row-major order, without moving pixels.
class CameraSlotOrder {
 public:
  explicit CameraSlotOrder(const std::string& order="front,rear,left,right") {
    if(order=="left,front,right,rear") slots_={{1,3,0,2}};
    else if(order=="front,rear,left,right") slots_={{0,1,2,3}};
    else throw std::invalid_argument("unsupported camera slot order");
  }
  int slot(int camera) const { return slots_.at(camera); }
  const std::array<int,4>& slots() const { return slots_; }
  template<class T> std::array<T,4> semantic(const std::array<T,4>& input) const {
    return {{input[slots_[0]],input[slots_[1]],input[slots_[2]],input[slots_[3]]}};
  }
  cv::Rect roi(int camera,const cv::Size& size) const {
    const int i=slot(camera);
    return {(i%2)*(size.width/2),(i/2)*(size.height/2),size.width/2,size.height/2};
  }
 private:
  std::array<int,4> slots_;
};
}  // namespace adas
