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
    // Full-width UFLDv2 occupies the shared NPU for roughly 200 ms. Service
    // objects between lane jobs: a 700 ms object target visibly lagged traffic.
    periods.object_ms = 120;
    periods.front_lane_ms = 320;
    periods.rear_lane_ms = 1500;
  } else {
    // The 800-wide student measures about 92 ms standalone. Prefer fresh
    // front geometry while retaining object and rear-view service.
    periods.object_ms = 120;
    periods.front_lane_ms = 240;
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
  // Two long lane jobs back-to-back would leave fast vehicles unobserved.
  // Insert one object update after any lane job before serving another lane.
  if (selected != 0 && (launched[1] > launched[0] ||
      (rear_enabled && launched[2] > launched[0]))) return 0;
  return selected;
}
}  // namespace adas
