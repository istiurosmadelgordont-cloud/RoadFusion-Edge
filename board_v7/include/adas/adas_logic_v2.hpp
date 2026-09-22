#pragma once

#include "adas/types.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

namespace adas {

// Compact vehicle-state interface modelled after the separation used by
// openpilot/Autoware.  A field is usable only when its own validity bit is set;
// missing CAN data must never silently become speed=0 or signal=off.
enum class TurnSignal { UNKNOWN, OFF, LEFT, RIGHT, HAZARD };
enum class GearState { UNKNOWN, PARK, REVERSE, NEUTRAL, DRIVE };

struct VehicleState {
  bool speed_valid = false;
  float speed_kmh = 0.0f;
  bool gear_valid = false;
  GearState gear = GearState::UNKNOWN;
  bool turn_valid = false;
  TurnSignal turn = TurnSignal::UNKNOWN;
  bool brake_valid = false;
  bool brake = false;
  bool demo = false;
};

class VehicleStateManager {
 public:
  void update(const VehicleState& state,
              std::chrono::steady_clock::time_point at) {
    state_ = state;
    updated_at_ = at;
  }

  void set_demo_turn(TurnSignal turn,
                     std::chrono::steady_clock::time_point at) {
    state_.turn_valid = true;
    state_.turn = turn;
    state_.demo = true;
    updated_at_ = at;
  }

  VehicleState snapshot(std::chrono::steady_clock::time_point now) const {
    VehicleState result = state_;
    const auto limit = result.demo ? std::chrono::seconds(10)
                                   : std::chrono::milliseconds(500);
    if (updated_at_ == std::chrono::steady_clock::time_point() ||
        now - updated_at_ > limit) {
      result = VehicleState();
    }
    return result;
  }

 private:
  VehicleState state_;
  std::chrono::steady_clock::time_point updated_at_{};
};

// Lanelet2 uses the same conservative rule: dashed permits crossing, solid or
// unknown does not.  UNKNOWN is intentional when image evidence is weak.
enum class LaneMarking { UNKNOWN, SOLID, DASHED };
enum class CrossingSide { NONE, LEFT, RIGHT };

inline const char* lane_marking_caption(LaneMarking marking) {
  switch (marking) {
    case LaneMarking::SOLID: return "实线";
    case LaneMarking::DASHED: return "虚线";
    default: return "未知";
  }
}

struct LaneMarkingObservation {
  LaneMarking left = LaneMarking::UNKNOWN;
  LaneMarking right = LaneMarking::UNKNOWN;
  float left_coverage = 0.0f;
  float right_coverage = 0.0f;
};

struct LaneSemantic {
  bool lane_valid = false;
  LaneMarking left = LaneMarking::UNKNOWN;
  LaneMarking right = LaneMarking::UNKNOWN;
  CrossingSide crossing = CrossingSide::NONE;
  float tlc_s = -1.0f;
  bool left_change_allowed = false;
  bool right_change_allowed = false;
  float left_evidence = 0.0f;
  float right_evidence = 0.0f;
};

inline LaneMarking classify_lane_pixels(const cv::Mat& paint,
                                        const std::vector<cv::Point>& curve,
                                        float* evidence = nullptr) {
  if (paint.empty() || curve.size() < 10) return LaneMarking::UNKNOWN;
  std::vector<unsigned char> supported;
  supported.reserve(curve.size());
  for (const cv::Point& point : curve) {
    const int radius = 6;
    const cv::Rect bounds(0, 0, paint.cols, paint.rows);
    const cv::Rect sample(point.x - radius, point.y - radius,
                          radius * 2 + 1, radius * 2 + 1);
    const cv::Rect roi = sample & bounds;
    if (roi.width <= 0 || roi.height <= 0) {
      supported.push_back(0);
      continue;
    }
    const float ratio = cv::countNonZero(paint(roi)) /
                        static_cast<float>(roi.area());
    supported.push_back(ratio >= 0.055f ? 1 : 0);
  }
  int hits = 0, runs = 0, longest_gap = 0, gap = 0;
  bool in_run = false;
  for (unsigned char value : supported) {
    hits += value;
    if (value) {
      if (!in_run) ++runs;
      in_run = true;
      gap = 0;
    } else {
      in_run = false;
      longest_gap = std::max(longest_gap, ++gap);
    }
  }
  const float coverage = hits / static_cast<float>(supported.size());
  if (evidence) *evidence = coverage;
  if (coverage >= 0.62f && longest_gap <= 5) return LaneMarking::SOLID;
  if (coverage >= 0.18f && coverage <= 0.78f && runs >= 2 && longest_gap >= 2)
    return LaneMarking::DASHED;
  return LaneMarking::UNKNOWN;
}

inline LaneMarkingObservation observe_lane_markings(const cv::Mat& bgr,
                                                    const LaneResult& lane) {
  LaneMarkingObservation result;
  if (bgr.empty() || !lane.valid || lane.left.empty() ||
      lane.right.empty()) return result;
  cv::Mat hls, hsv, white, yellow, paint;
  cv::cvtColor(bgr, hls, cv::COLOR_BGR2HLS);
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hls, cv::Scalar(0, 155, 0), cv::Scalar(180, 255, 125), white);
  cv::inRange(hsv, cv::Scalar(12, 65, 120), cv::Scalar(42, 255, 255), yellow);
  cv::bitwise_or(white, yellow, paint);
  result.left = classify_lane_pixels(paint, lane.left, &result.left_coverage);
  result.right = classify_lane_pixels(paint, lane.right, &result.right_coverage);
  return result;
}

class LaneSemanticTracker {
 public:
  LaneSemantic update(const LaneMarkingObservation& observed,
                      const LaneResult& lane,
                      std::chrono::steady_clock::time_point at) {
    LaneSemantic result = last_;
    const bool visual_evidence = observed.left != LaneMarking::UNKNOWN ||
                                 observed.right != LaneMarking::UNKNOWN;
    result.lane_valid = lane.valid && (!lane.partial || visual_evidence);
    result.left_evidence = observed.left_coverage;
    result.right_evidence = observed.right_coverage;
    result.crossing = CrossingSide::NONE;
    result.tlc_s = -1.0f;
    if (!result.lane_valid || !std::isfinite(lane.offset_ratio)) {
      result.left_change_allowed = result.right_change_allowed = false;
      last_ = result;
      return result;
    }

    result.left = stabilize(observed.left, left_candidate_, left_streak_, result.left);
    result.right = stabilize(observed.right, right_candidate_, right_streak_, result.right);
    result.left_change_allowed = result.left == LaneMarking::DASHED;
    result.right_change_allowed = result.right == LaneMarking::DASHED;
    if (lane.offset_ratio < -0.16f) result.crossing = CrossingSide::LEFT;
    if (lane.offset_ratio > 0.16f) result.crossing = CrossingSide::RIGHT;

    if (previous_at_ != std::chrono::steady_clock::time_point() && at > previous_at_) {
      const float dt = std::chrono::duration<float>(at - previous_at_).count();
      if (dt >= 0.04f && dt <= 1.0f) {
        const float raw_lateral = (lane.offset_ratio - previous_offset_) * 3.5f / dt;
        lateral_speed_mps_ = 0.65f * lateral_speed_mps_ + 0.35f * raw_lateral;
        const float clearance = std::max(0.0f, 0.85f - std::abs(lane.offset_ratio * 3.5f));
        const bool outward = (lane.offset_ratio > 0.0f && lateral_speed_mps_ > 0.05f) ||
                             (lane.offset_ratio < 0.0f && lateral_speed_mps_ < -0.05f);
        if (outward && clearance > 0.0f) {
          result.tlc_s = std::min(9.9f, clearance / std::abs(lateral_speed_mps_));
        }
      }
    }
    previous_offset_ = lane.offset_ratio;
    previous_at_ = at;
    last_ = result;
    return result;
  }

  void reset() { *this = LaneSemanticTracker(); }

 private:
  static LaneMarking stabilize(LaneMarking value, LaneMarking& candidate,
                               int& streak, LaneMarking current) {
    if (value == LaneMarking::UNKNOWN) return current;
    if (value != candidate) {
      candidate = value;
      streak = 1;
    } else {
      ++streak;
    }
    return streak >= 2 ? value : current;
  }

  LaneSemantic last_;
  LaneMarking left_candidate_ = LaneMarking::UNKNOWN;
  LaneMarking right_candidate_ = LaneMarking::UNKNOWN;
  int left_streak_ = 0;
  int right_streak_ = 0;
  float previous_offset_ = 0.0f;
  float lateral_speed_mps_ = 0.0f;
  std::chrono::steady_clock::time_point previous_at_{};
};

enum class ManeuverDirection { NONE, LEFT, STRAIGHT, RIGHT };
enum class LaneChangeState {
  IDLE, REQUEST, CHECK_RULE, CHECK_SAFE, ALLOW_CHANGE, CHANGING, DONE,
  BLOCKED, ABORT
};

struct LaneChangeResult {
  LaneChangeState state = LaneChangeState::IDLE;
  ManeuverDirection direction = ManeuverDirection::NONE;
  const char* reason = "未请求变道";
  bool permitted = false;
};

inline const char* lane_change_state_caption(LaneChangeState state) {
  switch (state) {
    case LaneChangeState::REQUEST: return "已请求";
    case LaneChangeState::CHECK_RULE: return "检查规则";
    case LaneChangeState::CHECK_SAFE: return "检查安全";
    case LaneChangeState::ALLOW_CHANGE: return "允许提示";
    case LaneChangeState::CHANGING: return "变道中";
    case LaneChangeState::DONE: return "已完成";
    case LaneChangeState::BLOCKED: return "已阻止";
    case LaneChangeState::ABORT: return "已中止";
    default: return "未请求";
  }
}

class LaneChangeFsm {
 public:
  void request(ManeuverDirection direction,
               std::chrono::steady_clock::time_point now) {
    direction_ = direction;
    state_ = direction == ManeuverDirection::LEFT ||
                     direction == ManeuverDirection::RIGHT
                 ? LaneChangeState::REQUEST : LaneChangeState::IDLE;
    since_ = state_since_ = now;
    clear_since_ = std::chrono::steady_clock::time_point();
  }

  LaneChangeResult update(const VehicleState& vehicle,
                          const LaneSemantic& lane,
                          bool left_blind, bool right_blind,
                          bool side_fresh, bool near_regulation,
                          std::chrono::steady_clock::time_point now) {
    if (state_ == LaneChangeState::IDLE || direction_ == ManeuverDirection::NONE)
      return make("未请求变道", false);
    if (state_ == LaneChangeState::ABORT)
      return make("变道已中止，请重新请求", false);
    if (state_ == LaneChangeState::DONE) {
      if (now - state_since_ < std::chrono::seconds(1))
        return make("变道完成", true);
      direction_ = ManeuverDirection::NONE;
      state_ = LaneChangeState::IDLE;
      return make("未请求变道", false);
    }
    if (now - since_ > std::chrono::seconds(10)) {
      state_ = LaneChangeState::ABORT;
      return make("变道请求超时，已取消", false);
    }
    const bool signal_matches = vehicle.turn_valid &&
        ((direction_ == ManeuverDirection::LEFT && vehicle.turn == TurnSignal::LEFT) ||
         (direction_ == ManeuverDirection::RIGHT && vehicle.turn == TurnSignal::RIGHT));
    const bool rule_allowed = lane.lane_valid &&
        (direction_ == ManeuverDirection::LEFT ? lane.left_change_allowed
                                               : lane.right_change_allowed);
    const bool blind = direction_ == ManeuverDirection::LEFT ? left_blind : right_blind;
    const bool crossed = direction_ == ManeuverDirection::LEFT
        ? lane.crossing == CrossingSide::LEFT
        : lane.crossing == CrossingSide::RIGHT;

    if (!signal_matches) {
      state_ = LaneChangeState::BLOCKED;
      return make("需开启目标方向转向灯", false);
    }
    if (near_regulation) {
      state_ = LaneChangeState::BLOCKED;
      return make("路口/斑马线附近禁止确认变道", false);
    }
    if (!rule_allowed) {
      state_ = LaneChangeState::CHECK_RULE;
      return make("边界非虚线或语义未知", false);
    }
    if (!side_fresh) {
      state_ = LaneChangeState::CHECK_SAFE;
      clear_since_ = std::chrono::steady_clock::time_point();
      return make("侧后方感知数据过期", false);
    }
    if (blind) {
      state_ = state_ == LaneChangeState::CHANGING ? LaneChangeState::ABORT
                                                   : LaneChangeState::BLOCKED;
      clear_since_ = std::chrono::steady_clock::time_point();
      return make(state_ == LaneChangeState::ABORT ? "变道中发现风险，立即中止"
                                                   : "目标侧盲区有车辆", false);
    }
    if (clear_since_ == std::chrono::steady_clock::time_point()) clear_since_ = now;
    if (now - clear_since_ < std::chrono::milliseconds(500)) {
      state_ = LaneChangeState::CHECK_SAFE;
      return make("持续确认目标侧安全", false);
    }
    if (crossed) {
      if (state_ != LaneChangeState::CHANGING) state_since_ = now;
      state_ = LaneChangeState::CHANGING;
      return make("正在变道，持续监测盲区", true);
    }
    if (state_ == LaneChangeState::CHANGING &&
        now - state_since_ > std::chrono::milliseconds(700)) {
      state_ = LaneChangeState::DONE;
      state_since_ = now;
      return make("变道完成", true);
    }
    state_ = LaneChangeState::ALLOW_CHANGE;
    return make("规则允许且盲区持续安全", true);
  }

  LaneChangeResult current() const { return make("未更新", false); }

 private:
  LaneChangeResult make(const char* reason, bool permitted) const {
    LaneChangeResult result;
    result.state = state_;
    result.direction = direction_;
    result.reason = reason;
    result.permitted = permitted;
    return result;
  }

  LaneChangeState state_ = LaneChangeState::IDLE;
  ManeuverDirection direction_ = ManeuverDirection::NONE;
  std::chrono::steady_clock::time_point since_{}, state_since_{}, clear_since_{};
};

enum class IntersectionAction { UNKNOWN, STOP, GO, WAIT };

struct IntersectionResult {
  IntersectionAction action = IntersectionAction::UNKNOWN;
  ManeuverDirection route = ManeuverDirection::STRAIGHT;
  bool route_valid = false;
  bool stable = false;
  const char* reason = "缺少路线意图";
};

inline const char* intersection_action_caption(IntersectionAction action) {
  switch (action) {
    case IntersectionAction::STOP: return "STOP 停车";
    case IntersectionAction::GO: return "GO 通行";
    case IntersectionAction::WAIT: return "WAIT 确认中";
    default: return "路线/灯态未知";
  }
}

class IntersectionLogic {
 public:
  IntersectionResult update(const std::vector<Detection>& lights,
                            const VehicleState& vehicle) {
    IntersectionResult result;
    result.route = vehicle.turn_valid && vehicle.turn == TurnSignal::LEFT
                       ? ManeuverDirection::LEFT
                       : vehicle.turn_valid && vehicle.turn == TurnSignal::RIGHT
                             ? ManeuverDirection::RIGHT
                             : ManeuverDirection::STRAIGHT;
    result.route_valid = vehicle.turn_valid;
    int red_exact = -1, green_exact = -1;
    if (result.route == ManeuverDirection::LEFT) { red_exact = 8; green_exact = 12; }
    if (result.route == ManeuverDirection::RIGHT) { red_exact = 9; green_exact = 13; }
    if (result.route == ManeuverDirection::STRAIGHT) { red_exact = 10; green_exact = 14; }
    float exact_score = 0.0f, circle_score = 0.0f;
    IntersectionAction exact = IntersectionAction::UNKNOWN;
    IntersectionAction circle = IntersectionAction::UNKNOWN;
    for (const Detection& d : lights) {
      if (d.score < 0.4f) continue;
      if ((d.class_id == red_exact || d.class_id == green_exact) &&
          d.score > exact_score) {
        exact_score = d.score;
        exact = d.class_id == red_exact ? IntersectionAction::STOP
                                        : IntersectionAction::GO;
      }
      if ((d.class_id == 7 || d.class_id == 11) && d.score > circle_score) {
        circle_score = d.score;
        circle = d.class_id == 7 ? IntersectionAction::STOP
                                 : IntersectionAction::GO;
      }
    }
    result.action = exact != IntersectionAction::UNKNOWN ? exact : circle;
    if (result.action == IntersectionAction::UNKNOWN) {
      history_.clear();
      result.reason = "未获得匹配方向灯";
      return result;
    }
    history_.push_back(result.action);
    if (history_.size() > 5) history_.pop_front();
    int same = 0;
    for (IntersectionAction value : history_) if (value == result.action) ++same;
    result.stable = same >= 3;
    if (!result.stable) {
      result.action = IntersectionAction::WAIT;
      result.reason = "方向灯结果确认中";
    } else {
      result.reason = result.action == IntersectionAction::STOP ? "匹配方向红灯"
                                                                 : "匹配方向绿灯";
    }
    return result;
  }

 private:
  std::deque<IntersectionAction> history_;
};

enum class WarningCode { NONE, FCW, RCW, BSD_LEFT, BSD_RIGHT, LDW, LANE_CHANGE, INTERSECTION };

struct WarningSummary {
  WarningCode primary = WarningCode::NONE;
  int priority = 0;
  const char* text = "系统正常";
  std::vector<WarningCode> active;
};

class WarningManager {
 public:
  WarningSummary update(bool fcw, bool rcw, bool left_bsd, bool right_bsd,
                        bool ldw, const LaneChangeResult& lane_change,
                        const IntersectionResult& intersection) const {
    WarningSummary result;
    add(result, fcw, WarningCode::FCW, 100, "前向碰撞预警 FCW");
    add(result, rcw, WarningCode::RCW, 90, "后车快速接近 RCW");
    add(result, lane_change.state == LaneChangeState::ABORT,
        WarningCode::LANE_CHANGE, 88, lane_change.reason);
    add(result, left_bsd, WarningCode::BSD_LEFT, 80, "左侧盲区有目标");
    add(result, right_bsd, WarningCode::BSD_RIGHT, 80, "右侧盲区有目标");
    add(result, ldw, WarningCode::LDW, 70, "车道偏离预警 LDW");
    add(result, intersection.stable &&
                    intersection.action == IntersectionAction::STOP,
        WarningCode::INTERSECTION, 60, "匹配方向红灯，停车");
    add(result, lane_change.state == LaneChangeState::BLOCKED ||
                    lane_change.state == LaneChangeState::CHECK_RULE,
        WarningCode::LANE_CHANGE, 50, lane_change.reason);
    return result;
  }

 private:
  static void add(WarningSummary& result, bool active, WarningCode code,
                  int priority, const char* text) {
    if (!active) return;
    result.active.push_back(code);
    if (priority > result.priority) {
      result.primary = code;
      result.priority = priority;
      result.text = text;
    }
  }
};

}  // namespace adas
