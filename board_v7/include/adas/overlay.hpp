#pragma once

#include "adas/types.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace adas {

struct LaneSemantic;
cv::Scalar risk_overlay_color(const RiskResult& risk);
cv::Scalar normal_object_color(int class_id);
void draw_risk_halo(cv::Mat& frame, const cv::Rect2f& box,
                    const RiskResult& risk);
void draw_side_risk_cue(cv::Mat& frame, const cv::Rect2f& box,
                        bool blocked);
void draw_target_card(cv::Mat& frame, const std::vector<Detection>& detections,
                      const RiskResult& risk, const char* title);

void draw_overlay(cv::Mat& frame, const std::vector<Detection>& detections,
                  const LaneResult& lane, const SignalResult& signal,
                  const RiskResult& risk, const DriveResult& drive,
                  double fps, double npu_ms,
                  const LaneSemantic* semantics = nullptr);

}  // namespace adas
