#include "adas/overlay.hpp"
#include "adas/adas_logic_v2.hpp"
#include "adas/signal_logic.hpp"
#include "adas/text_renderer.hpp"

#include <algorithm>
#include <cmath>
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
  const bool utf8 = std::any_of(text.begin(), text.end(), [](char c) {
    return static_cast<unsigned char>(c) >= 0x80;
  });
  const int pixel_height = std::max(12, cvRound(scale * 28.0));
  const cv::Size size = utf8 ? ui::measure_text(text, pixel_height) :
      cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
  origin.y = std::max(size.height + 6, origin.y);
  cv::rectangle(image, cv::Rect(origin.x, origin.y - size.height - 6, size.width + 8, size.height + 9),
                cv::Scalar(18, 22, 24), cv::FILLED);
  if (utf8)
    ui::draw_text(image, text, origin + cv::Point(4, -3), pixel_height, color);
  else
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

void draw_lane_boundary(cv::Mat& frame, const std::vector<cv::Point>& curve,
                        LaneMarking marking, const cv::Scalar& color,
                        int thickness) {
  if (curve.size() < 2) return;
  if (marking == LaneMarking::SOLID) {
    cv::polylines(frame, curve, false, color, thickness, cv::LINE_AA);
    return;
  }
  if (marking == LaneMarking::DASHED) {
    for (size_t i = 1; i < curve.size(); ++i) {
      if ((i / 3) % 2 == 0)
        cv::line(frame, curve[i - 1], curve[i], color, thickness, cv::LINE_AA);
    }
    return;
  }
  for (size_t i = 0; i < curve.size(); i += 3)
    cv::circle(frame, curve[i], std::max(2, thickness / 2), color, cv::FILLED,
               cv::LINE_AA);
}

bool is_risk_target(const Detection& detection, const RiskResult& risk) {
  if (!risk.target) return false;
  if (risk.track_id >= 0 && detection.track_id >= 0)
    return risk.track_id == detection.track_id;
  const cv::Rect2f overlap = detection.box & cv::Rect2f(risk.box);
  const float united = detection.box.area() + risk.box.area() - overlap.area();
  return united > 1.0f && overlap.area() / united > 0.35f;
}

bool is_current_signal(const Detection& detection, const SignalResult& signal) {
  if (signal.state == SignalState::NONE || detection.class_id < 7 ||
      detection.class_id > 14 || signal.box.area() <= 0) return false;
  const cv::Rect overlap = cv::Rect(detection.box) & signal.box;
  const float united = detection.box.area() + signal.box.area() - overlap.area();
  return united > 1.0f && overlap.area() / united > 0.35f;
}

cv::Scalar risk_color(const RiskResult& risk) {
  if (risk.warning || (risk.ttc_s > 0.0f && risk.ttc_s < 1.5f))
    return cv::Scalar(35, 45, 245);
  if (risk.ttc_s > 0.0f && risk.ttc_s < 3.0f)
    return cv::Scalar(30, 150, 245);
  return cv::Scalar(50, 225, 245);
}

}  // namespace

void draw_overlay(cv::Mat& frame, const std::vector<Detection>& detections,
                  const LaneResult& lane, const SignalResult& signal,
                  const RiskResult& risk, const DriveResult& drive,
                  double fps, double npu_ms,
                  const LaneSemantic* semantics) {
  if (lane.valid) {
    cv::Mat layer = frame.clone();
    std::vector<std::vector<cv::Point>> polygons(1, lane.polygon);
    // A geometric offset alone is not a confirmed crossing.  Only the LDW
    // monitor may turn the lane red; uncertain geometry stays neutral.
    const bool uncertain = lane.partial || lane.identity_uncertain;
    const bool crossing = lane.departure;
    cv::fillPoly(layer, polygons, crossing ? cv::Scalar(20, 35, 215)
                                            : uncertain ? cv::Scalar(105, 125, 125)
                                                        : cv::Scalar(25, 150, 35));
    const double fill_alpha = crossing ? 0.22 : uncertain ? 0.05 : 0.10;
    cv::addWeighted(layer, fill_alpha, frame, 1.0 - fill_alpha, 0, frame);
    const cv::Scalar normal = uncertain ? cv::Scalar(125, 155, 155)
                                        : cv::Scalar(55, 255, 70);
    const cv::Scalar caution(35, 225, 245);
    const cv::Scalar urgent(30, 145, 245);
    const cv::Scalar danger(35, 45, 245);
    const bool left_danger = lane.departure &&
        lane.departure_side == LaneDepartureSide::LEFT;
    const bool right_danger = lane.departure &&
        lane.departure_side == LaneDepartureSide::RIGHT;
    const LaneMarking left_marking = semantics ? semantics->left : LaneMarking::UNKNOWN;
    const LaneMarking right_marking = semantics ? semantics->right : LaneMarking::UNKNOWN;
    cv::Scalar left_color = left_danger ? danger : normal;
    cv::Scalar right_color = right_danger ? danger : normal;
    CrossingSide approach_side = CrossingSide::NONE;
    cv::Scalar approach_color = normal;
    if (!uncertain && semantics && semantics->tlc_s > 0.0f && semantics->tlc_s < 2.5f) {
      approach_side = semantics->trend;
      approach_color = semantics->tlc_s < 1.5f && !lane.partial ? urgent : caution;
    } else if (!uncertain && std::abs(lane.offset_ratio) >= 0.10f) {
      approach_side = lane.offset_ratio < 0.0f ? CrossingSide::LEFT
                                               : CrossingSide::RIGHT;
      approach_color = std::abs(lane.offset_ratio) >= 0.15f && !lane.partial
          ? urgent : caution;
    }
    if (approach_side == CrossingSide::LEFT && !left_danger)
      left_color = approach_color;
    if (approach_side == CrossingSide::RIGHT && !right_danger)
      right_color = approach_color;
    draw_lane_boundary(frame, lane.left, left_marking,
                       left_color, left_danger ? 7 : 4);
    draw_lane_boundary(frame, lane.right, right_marking,
                       right_color, right_danger ? 7 : 4);
  }
  for (const Detection& detection : detections) {
    const bool selected = is_risk_target(detection, risk);
    const bool current_signal = is_current_signal(detection, signal);
    const cv::Scalar color = selected ? risk_color(risk) : color_for(detection.class_id);
    if (current_signal)
      cv::rectangle(frame, detection.box, cv::Scalar(245, 245, 245), 6,
                    cv::LINE_AA);
    cv::rectangle(frame, detection.box, color, selected || current_signal ? 4 : 2);
    std::ostringstream text;
    if (selected) text << "FCW TARGET ";
    text << detection.name;
    if (detection.track_id >= 0) text << " #" << detection.track_id;
    text << " " << std::fixed << std::setprecision(2) << detection.score;
    if (current_signal) text << " CURRENT";
    label(frame, text.str(), cv::Point(static_cast<int>(detection.box.x), static_cast<int>(detection.box.y)), color);
  }
  if (signal.state != SignalState::NONE) {
    const char* state = signal.state == SignalState::RED ? "红灯" :
                        signal.state == SignalState::GREEN ? "绿灯" : "未知";
    std::string text = std::string("前方信号灯：") + state;
    if (!signal.stable) text += "（确认中）";
    const cv::Scalar color = signal.state == SignalState::RED ? cv::Scalar(0, 0, 255) :
                             signal.state == SignalState::GREEN ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 220, 255);
    label(frame, text, cv::Point(20, 44), color, 0.78, 2);
  }
  if (risk.target) {
    std::ostringstream text;
    text << "前方 " << std::fixed << std::setprecision(1) << risk.distance_m << " m  接近 "
         << risk.relative_speed_kmh << " km/h";
    if (risk.ttc_s > 0.0f) text << "  TTC " << risk.ttc_s << " s";
    label(frame, text.str(), cv::Point(20, 84), risk.warning ? cv::Scalar(0, 0, 255) : cv::Scalar(70, 255, 110), 0.88, 3);
  }
  if (lane.valid) {
    std::ostringstream text;
    text << (lane.departure ? lane_departure_name(lane.departure_side) :
             ((lane.partial || lane.identity_uncertain) ? "车道确认中" : "车道保持")) << "  偏移 "
         << std::fixed << std::setprecision(2) << lane.offset_ratio;
    label(frame, text.str(), cv::Point(20, 120), lane.departure ? cv::Scalar(0, 80, 255) : cv::Scalar(80, 255, 180), 0.67, 2);
    if (!lane.partial && !lane.identity_uncertain && semantics &&
        semantics->tlc_s > 0.0f && semantics->tlc_s < 3.0f) {
      std::ostringstream tlc;
      tlc << (semantics->trend == CrossingSide::LEFT ? "← " :
              semantics->trend == CrossingSide::RIGHT ? "→ " : "")
          << "TLC " << std::fixed << std::setprecision(1) << semantics->tlc_s << " s";
      const cv::Scalar color = semantics->tlc_s < 1.5f ? cv::Scalar(30, 145, 245)
                                                        : cv::Scalar(35, 225, 245);
      label(frame, tlc.str(), cv::Point(20, 154), color, 0.64, 2);
    }
  }
  std::ostringstream perf;
  perf << "FPS " << std::fixed << std::setprecision(1) << fps << "  NPU " << npu_ms << "ms";
  label(frame, perf.str(), cv::Point(std::max(0, frame.cols - 290), 35), cv::Scalar(255, 240, 100), 0.65, 2);

  // Drawn last so the indicator sits above every other annotation.
  draw_decision_panel(frame, drive);
}

}  // namespace adas
