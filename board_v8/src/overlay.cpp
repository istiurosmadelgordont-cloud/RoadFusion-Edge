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
  origin.x = std::max(0, std::min(origin.x, image.cols - size.width - 8));
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

enum class RiskTier { UNCONFIRMED, NORMAL, CAUTION, WARNING, DANGER };

RiskTier risk_tier(const RiskResult& risk) {
  if (!risk.reliable) return RiskTier::UNCONFIRMED;
  if (!std::isfinite(risk.ttc_s) || risk.ttc_s <= 0.0f)
    return risk.warning ? RiskTier::WARNING : RiskTier::UNCONFIRMED;
  RiskTier tier = RiskTier::NORMAL;
  if (risk.ttc_s < 1.5f) tier = RiskTier::DANGER;
  else if (risk.ttc_s < 3.0f) tier = RiskTier::WARNING;
  else if (risk.ttc_s < 4.0f) tier = RiskTier::CAUTION;
  // The estimator can warn on distance even when closing speed is unavailable.
  if (risk.warning && tier < RiskTier::WARNING) tier = RiskTier::WARNING;
  return tier;
}

cv::Scalar tier_color(RiskTier tier) {
  switch (tier) {
    case RiskTier::NORMAL: return cv::Scalar(75, 220, 85);
    case RiskTier::CAUTION: return cv::Scalar(35, 225, 245);
    case RiskTier::WARNING: return cv::Scalar(30, 145, 245);
    case RiskTier::DANGER: return cv::Scalar(35, 45, 245);
    default: return cv::Scalar(160, 165, 165);
  }
}

void blend_vertical_polygon(cv::Mat& frame, const std::vector<cv::Point>& polygon,
                            const cv::Scalar& color, double base_alpha,
                            int near_y, int far_y) {
  if (polygon.size() < 3 || frame.empty()) return;
  const cv::Rect bounds = cv::boundingRect(polygon) &
                          cv::Rect(0, 0, frame.cols, frame.rows);
  if (bounds.area() <= 0) return;
  std::vector<cv::Point> local;
  local.reserve(polygon.size());
  for (const cv::Point& point : polygon) local.push_back(point - bounds.tl());
  cv::Mat mask(bounds.size(), CV_8UC1, cv::Scalar(0));
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{local},
               cv::Scalar(255), cv::LINE_AA);
  cv::Mat pixels = frame(bounds);
  const double span = std::max(1, near_y - far_y);
  for (int y = 0; y < bounds.height; ++y) {
    const double progress = std::max(0.0, std::min(1.0,
        (near_y - bounds.y - y) / span));
    const double row_alpha = base_alpha * (1.0 - .7 * progress);
    const uchar* coverage = mask.ptr<uchar>(y);
    cv::Vec3b* row = pixels.ptr<cv::Vec3b>(y);
    for (int x = 0; x < bounds.width; ++x) {
      if (!coverage[x]) continue;
      const double alpha = row_alpha * (coverage[x] / 255.0);
      for (int channel = 0; channel < 3; ++channel)
        row[x][channel] = cv::saturate_cast<uchar>(
            row[x][channel] * (1.0 - alpha) + color[channel] * alpha);
    }
  }
}

void draw_visual_corridor(cv::Mat& frame, const LaneResult& lane,
                          const LaneSemantic* semantics, bool fcw_warning) {
  if (!lane.valid || lane.left.size() < 8 || lane.right.size() < 8) return;
  const size_t count = std::min(lane.left.size(), lane.right.size());
  const size_t near_count = std::max<size_t>(5, count * 3 / 4);
  std::vector<cv::Point> left_band, right_band, center;
  left_band.reserve(near_count);
  right_band.reserve(near_count);
  center.reserve(near_count);
  for (size_t i = 0; i < near_count; ++i) {
    const cv::Point& left = lane.left[i];
    const cv::Point& right = lane.right[count - 1 - i];
    if (right.x - left.x < 12 || std::abs(right.y - left.y) > 15) return;
    const float width = static_cast<float>(right.x - left.x);
    // Use the observed lane width to create a visible corridor while keeping
    // both outer boundaries clear. This is not a projected ego trajectory.
    left_band.emplace_back(cvRound(left.x + width * 0.22f), left.y);
    right_band.emplace_back(cvRound(left.x + width * 0.78f), left.y);
    center.emplace_back(cvRound((left.x + right.x) * 0.5f), left.y);
  }
  if (!lane.identity_uncertain) {
    const bool approaching = semantics && semantics->trend != CrossingSide::NONE &&
        std::isfinite(semantics->tlc_s) && semantics->tlc_s > 0.0f &&
        semantics->tlc_s < 2.5f;
    const cv::Scalar band_color = lane.partial ? cv::Scalar(200, 150, 100) :
        fcw_warning ? cv::Scalar(35, 45, 245) : cv::Scalar(215, 175, 60);
    std::vector<cv::Point> corridor = left_band;
    corridor.insert(corridor.end(), right_band.rbegin(), right_band.rend());
    blend_vertical_polygon(frame, corridor, band_color,
                           lane.partial ? .20 : fcw_warning ? .35 : .30,
                           left_band.front().y, left_band.back().y);
    if (approaching && !lane.partial && !lane.departure) {
      const bool left_approach = semantics->trend == CrossingSide::LEFT;
      const cv::Scalar edge_color = semantics->tlc_s < 1.5f
          ? cv::Scalar(30, 145, 245) : cv::Scalar(35, 225, 245);
      std::vector<cv::Point> side;
      if (left_approach) {
        side.assign(lane.left.begin(), lane.left.begin() + near_count);
        side.insert(side.end(), left_band.rbegin(), left_band.rend());
      } else {
        side = right_band;
        for (size_t i = near_count; i-- > 0;)
          side.push_back(lane.right[count - 1 - i]);
      }
      blend_vertical_polygon(frame, side, edge_color, .24,
                             left_band.front().y, left_band.back().y);
    }
    // This is the observed lane centre, not a predicted vehicle trajectory.
    if (!lane.partial) {
      for (size_t i = 3; i < center.size(); i += 5)
        cv::circle(frame, center[i], 2, cv::Scalar(205, 235, 190), cv::FILLED,
                   cv::LINE_AA);
    }
  }

  if (!lane.departure || lane.departure_side == LaneDepartureSide::NONE) return;
  const bool left_crossing = lane.departure_side == LaneDepartureSide::LEFT;
  const size_t warning_count = std::max<size_t>(4, count * 2 / 3);
  std::vector<cv::Point> edge, inner;
  for (size_t i = 0; i < warning_count; ++i) {
    const cv::Point& left = lane.left[i];
    const cv::Point& right = lane.right[count - 1 - i];
    const float width = static_cast<float>(right.x - left.x);
    edge.push_back(left_crossing ? left : right);
    inner.emplace_back(left_crossing ? cvRound(left.x + width * 0.38f)
                                     : cvRound(right.x - width * 0.38f), left.y);
  }
  edge.insert(edge.end(), inner.rbegin(), inner.rend());
  blend_vertical_polygon(frame, edge, cv::Scalar(35, 45, 245), .25,
                         lane.left.front().y,
                         lane.left[warning_count - 1].y);
}

void draw_risk_card(cv::Mat& frame, const Detection& detection,
                    const RiskResult& risk, const char* title) {
  if (frame.cols < 190 || frame.rows < 90) return;
  const int width = std::min(180, frame.cols - 8);
  const int height = 75;
  const int x = std::max(4, std::min(cvRound(detection.box.x), frame.cols - width - 4));
  const int below = cvRound(detection.box.y + detection.box.height) + 5;
  const int above = cvRound(detection.box.y) - height - 5;
  // Prefer the sky above the target, keeping the road and lane warnings clear.
  const int y = above >= 4 ? above :
                below + height < frame.rows - 4 ? below :
                std::max(4, frame.rows - height - 4);
  const cv::Rect card(x, y, width, height);
  const RiskTier tier = risk_tier(risk);
  const cv::Scalar color = tier_color(tier);
  cv::rectangle(frame, card, cv::Scalar(20, 25, 28), cv::FILLED);
  cv::rectangle(frame, card, color, 2, cv::LINE_AA);
  std::ostringstream heading, closing, ttc;
  heading << title;
  if (detection.track_id >= 0) heading << " #" << detection.track_id;
  heading << "  ";
  if (std::isfinite(risk.distance_m) && risk.distance_m > 0.0f)
    heading << "~" << std::fixed << std::setprecision(1) << risk.distance_m << "m";
  else heading << "DIST --";
  if (std::isfinite(risk.relative_speed_kmh)) {
    const float speed = risk.relative_speed_kmh;
    closing << (speed > 1.0f ? "CLOSING " : speed < -1.0f ? "OPENING " : "REL SPEED ")
            << "~" << std::fixed << std::setprecision(0) << std::abs(speed) << "km/h";
  } else closing << "REL SPEED --";
  ttc << "TTC ";
  if (risk.reliable && std::isfinite(risk.ttc_s) && risk.ttc_s > 0.0f)
    ttc << "~" << std::fixed << std::setprecision(1) << risk.ttc_s << "s";
  else ttc << "--";
  cv::putText(frame, heading.str(), {x + 7, y + 17}, cv::FONT_HERSHEY_SIMPLEX,
              0.42, color, 1, cv::LINE_AA);
  cv::putText(frame, closing.str(), {x + 7, y + 36}, cv::FONT_HERSHEY_SIMPLEX,
              0.39, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
  cv::putText(frame, ttc.str(), {x + 7, y + 54}, cv::FONT_HERSHEY_SIMPLEX,
              0.42, color, 1, cv::LINE_AA);
  const cv::Scalar colors[] = {tier_color(RiskTier::NORMAL),
      tier_color(RiskTier::CAUTION), tier_color(RiskTier::WARNING),
      tier_color(RiskTier::DANGER)};
  const int active = static_cast<int>(tier);
  for (int i = 0; i < 4; ++i)
    cv::rectangle(frame, cv::Rect(x + 7 + i * 25, y + 63, 21, 5),
                  active > i ? colors[i] : cv::Scalar(65, 70, 72), cv::FILLED);
}

}  // namespace

cv::Scalar normal_object_color(int class_id) {
  if (class_id == 0 || class_id == 1 || class_id == 5 || class_id == 6)
    return cv::Scalar(205, 120, 185);
  if (class_id >= 2 && class_id <= 4) return cv::Scalar(210, 180, 75);
  return cv::Scalar(145, 155, 160);
}

cv::Scalar risk_overlay_color(const RiskResult& risk) {
  return tier_color(risk_tier(risk));
}

void draw_risk_halo(cv::Mat& frame, const cv::Rect2f& box,
                    const RiskResult& risk) {
  if (frame.empty() || !risk.reliable || risk_tier(risk) < RiskTier::WARNING)
    return;
  const cv::Rect image(0, 0, frame.cols, frame.rows);
  const cv::Rect target = cv::Rect(box) & image;
  if (target.area() <= 0) return;
  const cv::Rect expanded(target.x - 16, target.y - 16,
                          target.width + 32, target.height + 32);
  const cv::Rect roi = expanded & image;
  cv::Mat mask(roi.size(), CV_8UC1, cv::Scalar(0));
  cv::rectangle(mask, cv::Rect(target.x - roi.x, target.y - roi.y,
                               target.width, target.height),
                cv::Scalar(255), 4, cv::LINE_AA);
  cv::GaussianBlur(mask, mask, cv::Size(17, 17), 6.0);
  cv::Mat pixels = frame(roi);
  const cv::Scalar color = risk_overlay_color(risk);
  for (int y = 0; y < roi.height; ++y) {
    const uchar* strength = mask.ptr<uchar>(y);
    cv::Vec3b* row = pixels.ptr<cv::Vec3b>(y);
    for (int x = 0; x < roi.width; ++x) {
      if (!strength[x]) continue;
      const double alpha = .42 * (strength[x] / 255.0);
      for (int channel = 0; channel < 3; ++channel)
        row[x][channel] = cv::saturate_cast<uchar>(
            row[x][channel] * (1.0 - alpha) + color[channel] * alpha);
    }
  }
}

void draw_side_risk_cue(cv::Mat& frame, const cv::Rect2f& box,
                        bool blocked) {
  if (frame.empty()) return;
  const cv::Rect image(0, 0, frame.cols, frame.rows);
  const cv::Rect target = cv::Rect(box) & image;
  if (target.area() <= 0) return;
  const cv::Scalar color = blocked ? cv::Scalar(35, 45, 245)
                                   : cv::Scalar(50, 225, 245);
  const cv::Point contact(target.x + target.width / 2,
                          std::min(frame.rows - 1, target.y + target.height));
  const int radius_x = std::max(25, std::min(130, cvRound(target.width * .80)));
  const int radius_y = std::max(12, std::min(34, cvRound(target.height * .22)));
  const cv::Rect roi = cv::Rect(contact.x - radius_x - 20,
                                contact.y - radius_y - 20,
                                radius_x * 2 + 40, radius_y * 2 + 40) & image;
  if (roi.area() <= 0) return;
  cv::Mat mask(roi.size(), CV_8UC1, cv::Scalar(0));
  cv::ellipse(mask, contact - roi.tl(), {radius_x, radius_y}, 0,
              0, 360, cv::Scalar(255), cv::FILLED, cv::LINE_AA);
  cv::GaussianBlur(mask, mask, cv::Size(25, 25), 9.0);
  cv::Mat pixels = frame(roi);
  for (int y = 0; y < roi.height; ++y) {
    const uchar* strength = mask.ptr<uchar>(y);
    cv::Vec3b* row = pixels.ptr<cv::Vec3b>(y);
    for (int x = 0; x < roi.width; ++x) {
      if (!strength[x]) continue;
      const double alpha = (blocked ? .34 : .26) * strength[x] / 255.0;
      for (int channel = 0; channel < 3; ++channel)
        row[x][channel] = cv::saturate_cast<uchar>(
            row[x][channel] * (1.0 - alpha) + color[channel] * alpha);
    }
  }
}

void draw_target_card(cv::Mat& frame, const std::vector<Detection>& detections,
                      const RiskResult& risk, const char* title) {
  if (!risk.target || !risk.reliable || frame.empty()) return;
  for (const auto& detection : detections) {
    if (is_risk_target(detection, risk)) {
      draw_risk_card(frame, detection, risk, title);
      return;
    }
  }
}

void draw_overlay(cv::Mat& frame, const std::vector<Detection>& detections,
                  const LaneResult& lane, const SignalResult& signal,
                  const RiskResult& risk, const DriveResult& drive,
                  double fps, double npu_ms,
                  const LaneSemantic* semantics) {
  if (frame.empty()) return;
  draw_visual_corridor(frame, lane, semantics, risk.reliable &&
      (risk.warning || (std::isfinite(risk.ttc_s) && risk.ttc_s > 0.0f && risk.ttc_s < 1.5f)));
  if (lane.valid) {
    const bool uncertain = lane.partial || lane.identity_uncertain;
    const cv::Scalar normal = uncertain ? cv::Scalar(130, 145, 145)
                                        : cv::Scalar(55, 255, 70);
    const cv::Scalar caution(35, 225, 245), urgent(30, 145, 245);
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
    cv::Scalar approach_color = caution;
    if (!uncertain && semantics && semantics->tlc_s > 0.0f &&
        semantics->tlc_s < 2.5f) {
      approach_side = semantics->trend;
      approach_color = semantics->tlc_s < 1.5f ? urgent : caution;
    } else if (!uncertain && std::abs(lane.offset_ratio) >= 0.10f) {
      approach_side = lane.offset_ratio < 0.0f ? CrossingSide::LEFT
                                               : CrossingSide::RIGHT;
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
  const Detection* selected_target = nullptr;
  for (const Detection& detection : detections) {
    const bool selected = is_risk_target(detection, risk);
    const bool current_signal = is_current_signal(detection, signal);
    const bool confirmed_target = selected && risk.reliable;
    if (confirmed_target) selected_target = &detection;
    const cv::Scalar color = confirmed_target ? tier_color(risk_tier(risk)) :
        current_signal ? color_for(detection.class_id) : normal_object_color(detection.class_id);
    if (confirmed_target) draw_risk_halo(frame, detection.box, risk);
    cv::rectangle(frame, detection.box, color,
                  confirmed_target && risk.warning ? 5 : confirmed_target || current_signal ? 3 : 1,
                  cv::LINE_AA);
  }
  if (selected_target) draw_risk_card(frame, *selected_target, risk, "FCW");
  if (signal.state != SignalState::NONE && signal.box.area() > 0) {
    const cv::Scalar color = signal.state == SignalState::RED ? cv::Scalar(35, 45, 245) :
        signal.state == SignalState::GREEN ? cv::Scalar(75, 220, 85) :
        cv::Scalar(35, 225, 245);
    cv::rectangle(frame, signal.box, cv::Scalar(235, 235, 235), signal.stable ? 5 : 3, cv::LINE_AA);
    cv::rectangle(frame, signal.box, color, 2, cv::LINE_AA);
    const char* state = signal.state == SignalState::RED ? "RED" :
        signal.state == SignalState::GREEN ? "GREEN" :
        signal.state == SignalState::YELLOW ? "YELLOW" : "UNKNOWN";
    label(frame, std::string(signal.stable ? "CURRENT " : "CHECK ") + state,
          {signal.box.x, signal.box.y - 4}, color, 0.43, 1);
  }
  if (lane.valid) {
    if (lane.departure) {
      const char* side = lane.departure_side == LaneDepartureSide::LEFT ?
          "左边界" : "右边界";
      label(frame, std::string(lane_departure_name(lane.departure_side)) +
            "  越过" + side, {12, 45}, cv::Scalar(35, 45, 245), 0.60, 2);
    } else if (lane.partial || lane.identity_uncertain) {
      label(frame, "车道几何待确认", {12, 45}, cv::Scalar(180, 190, 190), 0.48, 1);
    } else if (semantics && semantics->trend != CrossingSide::NONE &&
               semantics->tlc_s > 0.0f && semantics->tlc_s < 2.5f) {
      std::ostringstream tlc;
      tlc << "TLC ~" << std::fixed << std::setprecision(1)
          << semantics->tlc_s << "s  "
          << (semantics->trend == CrossingSide::LEFT ? "接近左边界" : "接近右边界");
      label(frame, tlc.str(), {12, 45},
            semantics->tlc_s < 1.5f ? cv::Scalar(30, 145, 245)
                                     : cv::Scalar(35, 225, 245), 0.52, 1);
      const bool left = semantics->trend == CrossingSide::LEFT;
      cv::arrowedLine(frame, {left ? 52 : 16, 68}, {left ? 16 : 52, 68},
          semantics->tlc_s < 1.5f ? cv::Scalar(30, 145, 245) : cv::Scalar(35, 225, 245),
          3, cv::LINE_AA, 0, .35);
    }
  }
  std::ostringstream perf;
  perf << "FPS " << std::fixed << std::setprecision(1) << fps << "  NPU " << npu_ms << "ms";
  label(frame, perf.str(), cv::Point(std::max(0, frame.cols - 195), 18), cv::Scalar(140, 150, 155), 0.38, 1);

  // Drawn last so the indicator sits above every other annotation.
  draw_decision_panel(frame, drive);
}

}  // namespace adas
