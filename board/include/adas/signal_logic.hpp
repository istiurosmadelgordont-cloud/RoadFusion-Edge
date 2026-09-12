#pragma once

#include "adas/types.hpp"
#include <deque>
#include <utility>
#include <vector>

namespace adas {

class SignalLogic {
 public:
  SignalResult update(const std::vector<Detection>& detections, int width, int height);

 private:
  std::deque<std::pair<SignalState, SignalDirection>> history_;
};

const char* signal_name(SignalState state);
const char* signal_direction_name(SignalDirection direction);

}  // namespace adas
