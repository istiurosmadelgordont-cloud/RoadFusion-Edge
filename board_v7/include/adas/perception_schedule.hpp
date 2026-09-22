#pragma once
#include <array>
#include <chrono>

namespace adas {
struct PerceptionPeriods {
  int object_ms = 300;
  int front_lane_ms = 200;
  int rear_lane_ms = 450;
};

inline PerceptionPeriods lane_priority_periods(bool high_precision) {
  PerceptionPeriods periods;
  if (high_precision) {
    // A full-width UFLDv2 pass costs about 148 ms on this RK3568. Leave
    // enough NPU budget for object detection and a low-rate rear update.
    periods.object_ms = 700;
    periods.front_lane_ms = 160;
    periods.rear_lane_ms = 1500;
  } else {
    // The 800-wide student measures about 92 ms standalone. Prefer fresh
    // front geometry while retaining object and rear-view service.
    periods.object_ms = 350;
    periods.front_lane_ms = 180;
    periods.rear_lane_ms = 1000;
  }
  return periods;
}

// One RK3568 NPU queue: use the earliest next service deadline rather than
// waiting for another display-frame cadence after a worker becomes available.
// 0=objects, 1=front lane, 2=rear lane. These are service targets, not FPS
// promises.
inline int next_perception_task(
    const std::array<std::chrono::steady_clock::time_point, 3>& launched,
    bool rear_enabled, const PerceptionPeriods& periods = PerceptionPeriods()) {
  const int period_ms[] = {periods.object_ms, periods.front_lane_ms,
                           periods.rear_lane_ms};
  int selected = 0;
  for (int i = 1; i < (rear_enabled ? 3 : 2); ++i)
    if (launched[i] + std::chrono::milliseconds(period_ms[i]) <
        launched[selected] + std::chrono::milliseconds(period_ms[selected])) {
      selected = i;
    }
  return selected;
}
}  // namespace adas
