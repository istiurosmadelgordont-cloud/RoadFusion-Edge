#include "adas/ufld_v2_decode.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

int main() {
  std::vector<float> row(200 * 72 * 4, -100.0f), col(100 * 81 * 4, -100.0f);
  std::vector<float> row_exists(2 * 72 * 4, 0.0f), col_exists(2 * 81 * 4, 0.0f);
  const auto put = [](std::vector<float>& loc, std::vector<float>& exists,
                     int anchors, int slot, int anchor, int grid) {
    loc[(grid * anchors + anchor) * 4 + slot] = 100.0f;
    exists[(anchors + anchor) * 4 + slot] = 1.0f;
  };
  put(col, col_exists, 81, 0, 10, 70);
  put(row, row_exists, 72, 1, 71, 40);
  put(row, row_exists, 72, 2, 0, 150);
  put(col, col_exists, 81, 3, 70, 80);
  // Contradictory predictions in the unused heads must never move a lane.
  put(row, row_exists, 72, 0, 0, 190);
  put(col, col_exists, 81, 1, 0, 0);
  put(row, row_exists, 72, 3, 0, 5);
  const auto lanes = adas::decode_ufld_v2_slots(row.data(), col.data(),
      row_exists.data(), col_exists.data(), 640, 360);
  for (const auto& lane : lanes) assert(lane.values.size() == 1);
  assert(std::abs(lanes[0].values[0].x - 80.0f) < .01f);
  assert(std::abs(lanes[0].values[0].y - 70.5f / 99 * 360) < .01f);
  assert(std::abs(lanes[1].values[0].x - 40.5f / 199 * 640) < .01f);
  assert(std::abs(lanes[1].values[0].y - 360.0f) < .01f);
  assert(std::abs(lanes[2].values[0].y - .42f * 360) < .01f);
  assert(std::abs(lanes[3].values[0].x - 560.0f) < .01f);
  assert(std::abs(lanes[0].confidence - 1.0f / 81) < .0001f);
  assert(std::abs(lanes[1].confidence - 1.0f / 72) < .0001f);
  // Absent column evidence must not fall back to the wrong row branch.
  col_exists[(81 + 70) * 4 + 3] = 0.0f;
  const auto absent = adas::decode_ufld_v2_slots(row.data(), col.data(),
      row_exists.data(), col_exists.data(), 640, 360);
  assert(absent[3].values.empty());
  col_exists[(81 + 10) * 4] = std::numeric_limits<float>::quiet_NaN();
  assert(adas::decode_ufld_v2_axis(col.data(), col_exists.data(), 0, true,
                                  640, 360).values.empty());
  std::cout << "PASS: UFLDv2 hybrid head selection, coordinates, coverage and absence\n";
}
