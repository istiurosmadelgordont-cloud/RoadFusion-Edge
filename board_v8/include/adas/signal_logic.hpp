#pragma once

#include "adas/types.hpp"
#include <deque>
#include <vector>

namespace adas {

class SignalLogic {
 public:
  SignalResult update(const std::vector<Detection>& detections, int width, int height);

 private:
  std::deque<SignalState> history_;
};

const char* signal_name(SignalState state);

}  // namespace adas
