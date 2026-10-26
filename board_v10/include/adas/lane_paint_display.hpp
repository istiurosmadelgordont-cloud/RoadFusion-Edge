#pragma once
#include "adas/adas_logic_v2.hpp"
#include <cmath>

namespace adas {
struct LanePaintBlend {
  float solid = 0.f, dashed = 0.f;
};
struct LanePaintDisplay {
  LanePaintBlend left, right;
};
// Animation only. Never modifies semantic evidence or lane-change permission.
class LanePaintDisplayTracker {
  using Clock = std::chrono::steady_clock;
 public:
  void reset() { state_ = LanePaintDisplay(); last_ = Clock::time_point(); }
  LanePaintDisplay update(const LaneSemantic& semantic, const LaneResult& lane,
                          Clock::time_point at) {
    if (!semantic.lane_valid || !lane.valid || lane.identity_uncertain ||
        (lane.partial && (lane.boundary_coverage[1] < .45f || lane.boundary_coverage[2] < .45f))) {
      reset(); return state_;
    }
    const bool initial = last_ == Clock::time_point() || at < last_ ||
                         at - last_ > std::chrono::milliseconds(650);
    const float dt = initial ? 0.f : std::chrono::duration<float>(at - last_).count();
    // 180 ms time constant: smooth across display redraws, independent of NPU cadence.
    const float gain = initial ? 1.f : 1.f - std::exp(-dt / .18f);
    auto step = [gain](LanePaintBlend& value, LaneMarking marking) {
      value.solid += gain * ((marking == LaneMarking::SOLID ? 1.f : 0.f) - value.solid);
      value.dashed += gain * ((marking == LaneMarking::DASHED ? 1.f : 0.f) - value.dashed);
    };
    step(state_.left, semantic.left); step(state_.right, semantic.right);
    last_ = at; return state_;
  }
 private:
  LanePaintDisplay state_;
  Clock::time_point last_{};
};
}
