#pragma once
#include "adas/adas_logic_v2.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace adas {

// 用途车道选灯，单击方向按钮选择模拟动作；两种选择相互独立。
// 仅消费前视的新检测结果；不读取 signal_demo.txt，也不累计重复显示帧。
class LaneSignalSimulator {
 public:
  enum class AlarmType { NONE, RED_LIGHT, INVALID_DIRECTION };
  bool enabled() const { return true; }
  ManeuverDirection lane() const { return lane_; }
  void set_lane(ManeuverDirection lane) {
    if (lane != ManeuverDirection::LEFT && lane != ManeuverDirection::STRAIGHT &&
        lane != ManeuverDirection::RIGHT) return;
    if (lane_ != lane) { lane_ = lane; clear_signal(); }
  }
  void reset() { clear_signal(); stop(); }
  void toggle_direction(ManeuverDirection direction) {
    if (direction != ManeuverDirection::LEFT && direction != ManeuverDirection::STRAIGHT &&
        direction != ManeuverDirection::RIGHT) return;
    if (direction_ == direction && moving_) stop();
    else { direction_ = direction; moving_ = true; }
  }
  void stop() { moving_ = false; }
  bool moving() const { return moving_; }
  ManeuverDirection direction() const { return direction_; }
  bool direction_matches_lane() const { return direction_ == lane_; }
  const char* direction_text() const { return text(direction()); }
  const char* lane_text() const { return text(lane_); }
  static const char* text(ManeuverDirection d) {
    return d == ManeuverDirection::LEFT ? "左转" : d == ManeuverDirection::RIGHT ? "右转" : "直行";
  }
  SignalState raw() const { return raw_; }
  IntersectionAction action() const {
    if (confirmed_ == SignalState::RED) return IntersectionAction::STOP;
    if (confirmed_ == SignalState::GREEN) return IntersectionAction::GO;
    return raw_ == SignalState::UNKNOWN ? IntersectionAction::UNKNOWN : IntersectionAction::WAIT;
  }
  // 用途车道是固定约束：绿灯只允许该车道对应方向。
  // 确认红灯时任何行驶动作都优先报闯红灯，不被“方向不符”覆盖。
  bool operation_violation() const { return moving() && !direction_matches_lane(); }
  AlarmType alarm_type() const {
    if (moving() && confirmed_ == SignalState::RED && raw_ == SignalState::RED)
      return AlarmType::RED_LIGHT;
    if (operation_violation()) return AlarmType::INVALID_DIRECTION;
    return AlarmType::NONE;
  }
  bool alarm() const { return alarm_type() != AlarmType::NONE; }
  const char* alarm_title() const {
    return alarm_type() == AlarmType::INVALID_DIRECTION ? "违规操作" : "闯红灯报警";
  }
  float confidence() const { return selected_.score; }
  int confirmations() const { return count_; }
  bool held() const { return false; }
  const char* reason() const {
    if (alarm_type() == AlarmType::RED_LIGHT) return "模拟闯红灯：请立即停车";
    if (alarm_type() == AlarmType::INVALID_DIRECTION) return "违规操作：行驶方向与用途车道不符，请停车";
    if (!direction_matches_lane()) return "当前已停止，所选方向与用途车道不符";
    if (action() == IntersectionAction::STOP) return "红灯已确认，再点当前方向按钮停车";
    if (action() == IntersectionAction::GO) return "绿灯已确认，可以选择方向行驶";
    return "控制灯未确认，暂不判断闯红灯";
  }

  void observe(double time, const std::vector<Detection>& lights, int width, int height) {
    if (!std::isfinite(time) || time <= last_event_ || width <= 0 || height <= 0) return;
    tick(time);
    const double gap = time - last_event_;
    last_event_ = time;
    if (lane_ == ManeuverDirection::STRAIGHT)
      for (const auto& d : lights)
        if (valid(d,width,height) && (d.class_id == 10 || d.class_id == 14)) last_straight_arrow_ = time;
    // 直行箭头比圆灯更具体，优先使用。箭头短时漏检期间不立即换用圆灯，
    // 防止“直行红箭头漏一帧 + 圆绿灯”错误地变为允许通行。
    const bool use_straight_arrow = lane_ == ManeuverDirection::STRAIGHT && time-last_straight_arrow_ <= .8;
    const Detection* best = nullptr;
    float best_rank = std::numeric_limits<float>::max();
    for (const auto& d : lights) {
      if (!valid(d,width,height) ||
          (use_straight_arrow && d.class_id != 10 && d.class_id != 14)) continue;
      // 优先画面水平中间的灯；同等位置时优先置信度高的灯。
      // 已选目标有小幅优先权，避免检测框抖动导致两盏邻灯来回切换。
      float rank = std::abs(d.box.x + d.box.width * .5f - width * .5f) / width;
      if (have_target_ && same_target(d.box, target_)) rank -= .025f;
      rank -= .002f * d.score;
      if (rank < best_rank) { best_rank = rank; best = &d; }
    }
    if (!best) { clear_evidence(); return; }
    const int shape = (best->class_id-7) % 4;
    if (!have_target_ || shape != target_shape_ || !same_target(best->box, target_)) clear_evidence();
    target_ = best->box; have_target_ = true; last_target_ = time;
    target_shape_ = shape;

    // 同一位置的互相矛盾的颜色不作授权；不同位置的圆灯不混在一起投票。
    const SignalState next = best->class_id <= 10 ? SignalState::RED : SignalState::GREEN;
    for (const auto& d : lights) {
      if (&d != best && valid(d,width,height) &&
          (!use_straight_arrow || d.class_id == 10 || d.class_id == 14) &&
          (d.class_id <= 10) != (best->class_id <= 10) &&
          (d.box & best->box).area() > .3f * std::min(d.box.area(), best->box.area())) {
        clear_evidence(); return;
      }
    }
    selected_.box = cv::Rect(best->box); selected_.score = best->score;
    selected_.track_id = best->track_id;  // 只让高亮框跟随同一小灯，不参与授权投票。
    // 换色立即撤销原授权/报警，再对新颜色连续确认。
    if (raw_ != next || gap > .8) {
      confirmed_ = SignalState::UNKNOWN; count_ = 0; since_ = time;
    }
    raw_ = next; ++count_;
    const int needed = next == SignalState::RED ? 2 : 3;
    const double span = next == SignalState::RED ? .2 : .6;
    if (count_ >= needed && time - since_ + 1e-6 >= span) confirmed_ = next;
  }

  void tick(double time) {
    if (!std::isfinite(time)) return;
    // 断流/推理停滞不继续沿用旧红灯报警，也不沿用旧绿灯通行判断。
    if (time - last_event_ > .8) clear_evidence();
    if (time - last_target_ > 1.0) { have_target_ = false; selected_ = SignalResult(); }
  }
  IntersectionResult intersection() const {
    IntersectionResult result;
    result.route = lane_; result.route_valid = true; result.action = action();
    result.stable = confirmed_ != SignalState::UNKNOWN;
    result.selected_signal = selected_;
    result.selected_signal.state = raw_;
    result.selected_signal.stable = result.stable;
    result.reason = reason();
    return result;
  }
  // 识别/描边始终显示用途车道的灯；普通停车建议仅用于匹配方向。
  // 行驶报警另由 alarm_type() 分类：确认红灯优先报闯红灯，否则方向不符报违规操作。
  IntersectionResult motion_intersection() const {
    IntersectionResult result = intersection();
    if (!direction_matches_lane()) {
      result.action = IntersectionAction::UNKNOWN;
      result.stable = false; result.route_valid = false;
    }
    return result;
  }
 private:
  bool eligible(const Detection& d) const {
    const int red = lane_ == ManeuverDirection::LEFT ? 8 : lane_ == ManeuverDirection::RIGHT ? 9 : 7;
    return (d.class_id == red && d.score >= .65f) || (d.class_id == red + 4 && d.score >= .80f) ||
        (lane_ == ManeuverDirection::STRAIGHT &&
         ((d.class_id == 10 && d.score >= .65f) || (d.class_id == 14 && d.score >= .80f)));
  }
  bool valid(const Detection& d,int width,int height) const {
    return eligible(d) && std::isfinite(d.score) && std::isfinite(d.box.x) && std::isfinite(d.box.y) &&
        std::isfinite(d.box.width) && std::isfinite(d.box.height) &&
        d.box.width >= 2 && d.box.height >= 2 && d.box.x >= 0 && d.box.y >= 0 &&
        d.box.br().x <= width && d.box.br().y <= height;
  }
  static bool same_target(const cv::Rect2f& a, const cv::Rect2f& b) {
    const float overlap = (a & b).area();
    if (overlap > .2f * std::max(a.area(), b.area())) return true;
    const cv::Point2f delta = (a.tl() + a.br() - b.tl() - b.br()) * .5f;
    const float limit = std::max(5.f, .55f * std::min(std::max(a.width,a.height),std::max(b.width,b.height)));
    return delta.dot(delta) < limit * limit &&
        std::max(a.area(),b.area()) < 2.5f * std::min(a.area(),b.area());
  }
  void clear_evidence() {
    raw_ = confirmed_ = SignalState::UNKNOWN; count_ = 0; since_ = 0;
    selected_.state = SignalState::UNKNOWN; selected_.stable = false;
    selected_.score = 0;
  }
  void clear_signal() {
    clear_evidence(); have_target_ = false; target_ = cv::Rect2f(); selected_ = SignalResult();
    last_event_ = last_target_ = last_straight_arrow_ = -1e9; target_shape_ = -1;
  }
  ManeuverDirection lane_ = ManeuverDirection::STRAIGHT, direction_ = ManeuverDirection::STRAIGHT;
  bool moving_ = false;
  SignalState raw_ = SignalState::UNKNOWN, confirmed_ = SignalState::UNKNOWN;
  cv::Rect2f target_;
  SignalResult selected_;
  bool have_target_ = false;
  int target_shape_ = -1;
  int count_ = 0;
  double since_ = 0, last_event_ = -1e9, last_target_ = -1e9, last_straight_arrow_ = -1e9;
};
} // namespace adas
