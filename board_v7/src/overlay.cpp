#include "adas/overlay.hpp"
#include "adas/signal_logic.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <opencv2/imgproc.hpp>

namespace adas {
namespace {

cv::Scalar color_for(int id) {
  if (id >= 7 && id <= 10) return cv::Scalar(0, 0, 255);
  if (id >= 11 && id <= 14) return cv::Scalar(0, 255, 0);
  if (id == 0) return cv::Scalar(255, 80, 255);
  return cv::Scalar(255, 180, 40);
}

void label(cv::Mat& image, const std::string& text, cv::Point origin,
           const cv::Scalar& color, double scale = 0.55, int thickness = 2) {
  int baseline = 0;
  const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
  origin.y = std::max(size.height + 6, origin.y);
  cv::rectangle(image, cv::Rect(origin.x, origin.y - size.height - 6, size.width + 8, size.height + 9),
                cv::Scalar(18, 22, 24), cv::FILLED);
  cv::putText(image, text, origin + cv::Point(4, -3), cv::FONT_HERSHEY_SIMPLEX,
              scale, color, thickness, cv::LINE_AA);
}

struct DecisionStyle {
  cv::Scalar background;
  cv::Scalar text;
};

DecisionStyle style_for(DriveDecision decision) {
  DecisionStyle style;
  if (decision == DriveDecision::STOP) {
    style.background = cv::Scalar(0, 0, 220);      // red
    style.text = cv::Scalar(255, 255, 255);
  } else if (decision == DriveDecision::SLOW) {
    style.background = cv::Scalar(0, 165, 235);    // amber
    style.text = cv::Scalar(25, 25, 25);
  } else {
    style.background = cv::Scalar(40, 140, 45);    // green
    style.text = cv::Scalar(255, 255, 255);
  }
  return style;
}

// Bottom-left pass/stop indicator. Nothing is drawn while the decision is
// UNKNOWN, so a scene without a traffic light keeps a clean frame.
void draw_decision_panel(cv::Mat& frame, const DriveResult& drive) {
  if (drive.decision == DriveDecision::UNKNOWN) return;

  const DecisionStyle style = style_for(drive.decision);
  const int margin = std::max(14, frame.cols / 64);
  const int width = std::max(180, frame.cols / 5);
  const int height = std::max(76, frame.rows / 8);
  const cv::Rect panel(margin, frame.rows - margin - height, width, height);

  cv::rectangle(frame, panel, style.background, cv::FILLED);
  cv::rectangle(frame, panel, style.text, 2, cv::LINE_AA);
  // Colour strip along the top edge, so the state is readable even in a glance
  // at a thumbnail-sized render.
  cv::rectangle(frame, cv::Rect(panel.x, panel.y, panel.width, std::max(5, height / 12)),
                style.text, cv::FILLED);

  int baseline = 0;
  const std::string title = decision_name(drive.decision);
  const double title_scale = std::max(0.9, height / 74.0);
  const cv::Size title_size = cv::getTextSize(title, cv::FONT_HERSHEY_SIMPLEX, title_scale, 2, &baseline);
  cv::putText(frame, title,
              cv::Point(panel.x + (panel.width - title_size.width) / 2,
                        panel.y + panel.height / 2 + title_size.height / 2 - 4),
              cv::FONT_HERSHEY_SIMPLEX, title_scale, style.text, 2, cv::LINE_AA);

  const std::string caption = decision_caption(drive.decision);
  const double caption_scale = std::max(0.45, height / 150.0);
  const cv::Size caption_size = cv::getTextSize(caption, cv::FONT_HERSHEY_SIMPLEX, caption_scale, 1, &baseline);
  cv::putText(frame, caption,
              cv::Point(panel.x + (panel.width - caption_size.width) / 2, panel.y + panel.height - 12),
              cv::FONT_HERSHEY_SIMPLEX, caption_scale, style.text, 1, cv::LINE_AA);
}

}  // namespace

void draw_overlay(cv::Mat& frame, const std::vector<Detection>& detections,
                  const LaneResult& lane, const SignalResult& signal,
                  const RiskResult& risk, const DriveResult& drive,
                  double fps, double npu_ms) {
  if (lane.valid) {
    cv::Mat layer = frame.clone();
    std::vector<std::vector<cv::Point>> polygons(1, lane.polygon);
    cv::fillPoly(layer, polygons, lane.departure ? cv::Scalar(0, 70, 255) : cv::Scalar(20, 190, 70));
    cv::addWeighted(layer, 0.27, frame, 0.73, 0, frame);
    cv::polylines(frame, lane.left, false, cv::Scalar(255, 230, 0), 5, cv::LINE_AA);
    cv::polylines(frame, lane.right, false, cv::Scalar(255, 230, 0), 5, cv::LINE_AA);
  }
  for (const Detection& detection : detections) {
    const cv::Scalar color = color_for(detection.class_id);
    cv::rectangle(frame, detection.box, color, 2);
    std::ostringstream text;
    text << detection.name;
    if (detection.track_id >= 0) text << " #" << detection.track_id;
    text << " " << std::fixed << std::setprecision(2) << detection.score;
    label(frame, text.str(), cv::Point(static_cast<int>(detection.box.x), static_cast<int>(detection.box.y)), color);
  }
  if (signal.state != SignalState::NONE) {
    std::string text = std::string("FORWARD SIGNAL: ") + signal_name(signal.state);
    if (!signal.stable) text += " (checking)";
    const cv::Scalar color = signal.state == SignalState::RED ? cv::Scalar(0, 0, 255) :
                             signal.state == SignalState::GREEN ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 220, 255);
    label(frame, text, cv::Point(20, 44), color, 0.78, 2);
  }
  if (risk.target) {
    std::ostringstream text;
    text << "FRONT " << std::fixed << std::setprecision(1) << risk.distance_m << "m  closing "
         << risk.relative_speed_kmh << "km/h";
    if (risk.ttc_s > 0.0f) text << "  TTC " << risk.ttc_s << "s";
    label(frame, text.str(), cv::Point(20, 84), risk.warning ? cv::Scalar(0, 0, 255) : cv::Scalar(70, 255, 110), 0.88, 3);
  }
  if (lane.valid) {
    std::ostringstream text;
    text << (lane.departure ? "LANE DEPARTURE" : (lane.partial ? "LANE PARTIAL" : "LANE KEEPING")) << "  offset "
         << std::fixed << std::setprecision(2) << lane.offset_ratio;
    label(frame, text.str(), cv::Point(20, 120), lane.departure ? cv::Scalar(0, 80, 255) : cv::Scalar(80, 255, 180), 0.67, 2);
  }
  std::ostringstream perf;
  perf << "FPS " << std::fixed << std::setprecision(1) << fps << "  NPU " << npu_ms << "ms";
  label(frame, perf.str(), cv::Point(std::max(0, frame.cols - 290), 35), cv::Scalar(255, 240, 100), 0.65, 2);

  // Drawn last so the indicator sits above every other annotation.
  draw_decision_panel(frame, drive);
}

}  // namespace adas
