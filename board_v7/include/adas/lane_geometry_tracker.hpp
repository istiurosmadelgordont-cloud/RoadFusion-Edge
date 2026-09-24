#pragma once

#include "adas/types.hpp"

#include <algorithm>
#include <cmath>

namespace adas {

// Preserves the physical ego-lane identity across short UFLD slot changes.
// A sudden one-lane jump is held briefly and reported to the LDW monitor. A
// persistent new pair is eventually accepted so the overlay cannot freeze.
class NeuralLaneGeometryTracker {
 public:
  LaneResult update(const LaneResult& measured) {
    if (!measured.valid) {
      ++missed_updates_;
      if (previous_.valid && missed_updates_ <= 3) {
        LaneResult held = previous_;
        held.partial = true;
        held.identity_uncertain = true;
        held.reassignment_side = LaneDepartureSide::NONE;
        return held;
      }
      previous_ = measured;
      return measured;
    }

    if (previous_.valid && has_boundaries(previous_) &&
        has_boundaries(measured)) {
      const float old_width = bottom_width(previous_);
      const float new_width = bottom_width(measured);
      const float shift = bottom_center(measured) - bottom_center(previous_);
      const bool plausible = old_width > 1.0f && new_width > 1.0f &&
          new_width > old_width * 0.68f && new_width < old_width * 1.42f &&
          std::abs(shift) < old_width * 0.32f;
      if (!plausible) {
        ++missed_updates_;
        // Slot replacement differs from physical motion: after a LEFT
        // crossing the newly selected lane centre jumps LEFT. Require prior
        // departure evidence; width collapse alone does not identify a side.
        const bool width_stable = old_width > 1.0f &&
            new_width > old_width * 0.68f && new_width < old_width * 1.42f;
        LaneDepartureSide side = LaneDepartureSide::NONE;
        if (width_stable && previous_.offset_ratio < -0.15f &&
            shift < -old_width * 0.32f)
          side = LaneDepartureSide::LEFT;
        if (width_stable && previous_.offset_ratio > 0.15f &&
            shift > old_width * 0.32f)
          side = LaneDepartureSide::RIGHT;
        if (missed_updates_ <= 4) {
          LaneResult held = previous_;
          held.partial = true;
          held.identity_uncertain = true;
          held.reassignment_side = side;
          return held;
        }
        LaneResult accepted = measured;
        accepted.identity_uncertain = true;
        accepted.reassignment_side = side;
        missed_updates_ = 0;
        previous_ = accepted;
        return accepted;
      }
    }

    missed_updates_ = 0;
    previous_ = measured;
    return measured;
  }

  void reset() {
    previous_ = LaneResult();
    missed_updates_ = 0;
  }

 private:
  static bool has_boundaries(const LaneResult& lane) {
    return !lane.left.empty() && !lane.right.empty();
  }

  static float bottom_width(const LaneResult& lane) {
    return static_cast<float>(lane.right.back().x - lane.left.front().x);
  }

  static float bottom_center(const LaneResult& lane) {
    return 0.5f * (lane.right.back().x + lane.left.front().x);
  }

  LaneResult previous_;
  int missed_updates_ = 0;
};

}  // namespace adas
