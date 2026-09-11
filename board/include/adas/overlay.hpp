#pragma once

#include "adas/types.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace adas {

void draw_overlay(cv::Mat& frame, const std::vector<Detection>& detections,
                  const LaneResult& lane, const SignalResult& signal,
                  const RiskResult& risk, double fps, double npu_ms);

}  // namespace adas
