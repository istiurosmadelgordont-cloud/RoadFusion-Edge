#pragma once

#include <opencv2/core.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace adas {

struct UfldLaneSamples {
  std::vector<cv::Point2f> values;
  float confidence = 0.0f;  // Supported anchors / anchors in this head.
};

// CULane's native heads are NCHW: row [1,200,72,4], col [1,100,81,4].
// Official hybrid decoding uses rows for ego slots 1/2 and columns for 0/3.
// https://github.com/cfzd/Ultra-Fast-Lane-Detection-v2/blob/master/demo.py
inline UfldLaneSamples decode_ufld_v2_axis(const float* location,
    const float* existence, int slot, bool column, int width, int height) {
  UfldLaneSamples result;
  if (!location || !existence || width <= 0 || height <= 0) return result;
  const int anchors = column ? 81 : 72;
  const int grids = column ? 100 : 200;
  for (int anchor = 0; anchor < anchors; ++anchor) {
    const float absent = existence[anchor * 4 + slot];
    const float present = existence[(anchors + anchor) * 4 + slot];
    if (!std::isfinite(absent) || !std::isfinite(present) || present <= absent)
      continue;
    int best = -1;
    float maximum = -std::numeric_limits<float>::infinity();
    for (int grid = 0; grid < grids; ++grid) {
      const float value = location[(grid * anchors + anchor) * 4 + slot];
      if (std::isfinite(value) && value > maximum) {
        maximum = value;
        best = grid;
      }
    }
    if (best < 0) continue;
    float sum = 0.0f, weighted = 0.0f;
    for (int grid = std::max(0, best - 1); grid <= std::min(grids - 1, best + 1); ++grid) {
      const float value = location[(grid * anchors + anchor) * 4 + slot];
      if (!std::isfinite(value)) continue;
      const float probability = std::exp(value - maximum);
      sum += probability;
      weighted += probability * (grid + 0.5f);
    }
    const float position = weighted / sum / (grids - 1);
    const float x = column ? anchor / 80.0f * width : position * width;
    const float y = column ? position * height :
        (0.42f + 0.58f * anchor / 71.0f) * height;
    result.values.emplace_back(x, y);
  }
  result.confidence = result.values.size() / static_cast<float>(anchors);
  return result;
}

inline std::array<UfldLaneSamples, 4> decode_ufld_v2_slots(
    const float* row_location, const float* col_location,
    const float* row_existence, const float* col_existence,
    int width, int height) {
  return {{decode_ufld_v2_axis(col_location, col_existence, 0, true, width, height),
           decode_ufld_v2_axis(row_location, row_existence, 1, false, width, height),
           decode_ufld_v2_axis(row_location, row_existence, 2, false, width, height),
           decode_ufld_v2_axis(col_location, col_existence, 3, true, width, height)}};
}

}  // namespace adas
