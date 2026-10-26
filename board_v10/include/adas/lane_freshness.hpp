#pragma once
#include <chrono>
namespace adas {
using LaneClock=std::chrono::steady_clock;
inline bool lane_result_displayable(LaneClock::time_point captured,
                                    LaneClock::time_point completed,
                                    LaneClock::time_point now) {
  // A slow display loop must not discard a result that completed on time.
  // Keep the original 300ms acquisition-to-worker deadline and the existing
  // 1000ms display expiry. Capture timestamps are never changed to completion.
  return completed>=captured && now>=completed &&
      completed-captured<std::chrono::milliseconds(300) &&
      now-captured<std::chrono::milliseconds(1000);
}
inline bool lane_decision_fresh(LaneClock::time_point captured,LaneClock::time_point now) {
  return now>=captured && now-captured<std::chrono::milliseconds(300);
}
}
