#pragma once

#include "adas/types.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace adas {

struct LaneSemantic;
struct LanePaintDisplay;
struct FcwPaint;
// Current measured ego geometry only; no corridor, paint assertion or permission.
void draw_lane_geometry_reference(cv::Mat& frame, const LaneResult& observed);
// 返回绘图副本：选中灯的描边与 MO 框复用同一预测坐标，不修改灯态状态机。
SignalResult tracked_signal_for_display(const SignalResult& signal,
                                        const std::vector<Detection>& detections);
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
void draw_overlay(cv::Mat& frame, const std::vector<Detection>& detections,
                  const LaneResult& lane, const SignalResult& signal,
                  const RiskResult& risk, const DriveResult& drive,
                  double fps, double npu_ms, const LaneSemantic* semantics,
                  const LanePaintDisplay* paint);
void draw_overlay(cv::Mat& frame, const std::vector<Detection>& detections,
                  const LaneResult& lane, const SignalResult& signal,
                  const RiskResult& risk, const DriveResult& drive,
                  double fps, double npu_ms, const LaneSemantic* semantics,
                  const LanePaintDisplay* paint, const FcwPaint* fcw);

}  // namespace adas
