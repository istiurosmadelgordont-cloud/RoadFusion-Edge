#include "adas/warning_logic.hpp"
#include "adas/signal_logic.hpp"
#include "adas/drive_decision.hpp"
#include "adas/risk_estimator.hpp"
#include "adas/byte_tracker.hpp"
#include "adas/perception_schedule.hpp"
#include "adas/experimental_adas.hpp"
#include "adas/adas_logic_v2.hpp"
#include <cassert>
#include <chrono>
#include <iostream>

int main() {
  using namespace adas;
  const auto base = std::chrono::steady_clock::now();
  const auto at = [&](int ms) { return base + std::chrono::milliseconds(ms); };
  std::array<std::chrono::steady_clock::time_point, 3> launched{{at(0),at(0),at(0)}};
  assert(next_perception_task(launched, true) == 1);
  launched[1] = at(200);
  assert(next_perception_task(launched, true) == 0);
  launched = {{at(600),at(600),at(0)}};
  assert(next_perception_task(launched, true) == 2);
  assert(next_perception_task(launched, false) == 1);
  const PerceptionPeriods precision = lane_priority_periods(true);
  assert(precision.object_ms == 700 && precision.front_lane_ms == 160 &&
         precision.rear_lane_ms == 1500);
  launched = {{at(0), at(0), at(0)}};
  assert(next_perception_task(launched, true, precision) == 1);
  LaneResult lane;
  lane.valid = true;
  lane.offset_ratio = 0.25f;
  LaneDepartureMonitor monitor;
  assert(!monitor.update(lane, at(0)).departure);
  assert(!monitor.update(lane, at(100)).departure);
  assert(monitor.update(lane, at(300)).departure);
  lane.partial = true;
  assert(!monitor.update(lane, at(400)).departure);
  lane.partial = false;
  assert(!monitor.update(lane, at(500)).departure);
  assert(monitor.update(lane, at(800)).departure);
  assert(!monitor.update(lane, at(2000)).departure); // gap must restart confirmation
  lane.offset_ratio = 0;
  for (int ms = 2200; ms <= 3000; ms += 200)
    assert(!monitor.update(lane, at(ms)).departure);

  VehicleWarningContext car;
  car.valid = true;
  car.speed_kmh = 20;
  lane.offset_ratio = .3f;
  monitor.reset();
  for (int ms = 0; ms < 1000; ms += 200) assert(!monitor.update(lane, at(ms), car).departure);
  car.speed_kmh = 70;
  car.left_blinker = true;
  assert(!monitor.update(lane, at(1000), car).departure);
  car.left_blinker = false;
  for (int ms = 1200; ms < 6000; ms += 200) assert(!monitor.update(lane, at(ms), car).departure);
  monitor.update(lane, at(6100), car);
  assert(monitor.update(lane, at(6400), car).departure);
  car.lateral_control_active = true;
  assert(!monitor.update(lane, at(6600), car).departure);

  SignalLogic signals;
  DriveDecisionLogic decisions{DriveConfig()};
  Detection green;
  green.class_id = 11;
  green.score = .9f;
  green.box = cv::Rect2f(300, 60, 25, 45);
  for (int i = 0; i < 3; ++i) decisions.update(signals.update({green}, 640, 360));
  assert(decisions.update(signals.update({green}, 640, 360)).decision == DriveDecision::GO);
  assert(decisions.update(signals.update({}, 640, 360)).decision == DriveDecision::UNKNOWN);
  Detection red = green;
  red.class_id = 7;
  for (int i = 0; i < 5; ++i) decisions.update(signals.update({red}, 640, 360));
  for (int i = 0; i < 12; ++i) decisions.update(signals.update({}, 640, 360));
  assert(decisions.update(signals.update({}, 640, 360)).decision == DriveDecision::UNKNOWN);
  signals = SignalLogic();
  green.score = .1f;
  for (int i = 0; i < 6; ++i) assert(!signals.update({green},640,360).stable);

  RiskConfig cfg;
  cfg.require_closing = true;
  cfg.focal_scale = .3f;
  RiskEstimator rear(cfg);
  Detection close;
  close.class_id = 2; close.track_id = 1; close.score = .9f;
  close.box = cv::Rect2f(250, 170, 140, 150);
  for (int i = 0; i < 10; ++i) {
    const auto risk = rear.update({close}, 640, 360);
    assert(risk.target && !risk.warning); // close stationary rear car is not RCW
  }
  ByteTracker tracker;
  tracker.update({close}, at(0), at(0));
  assert(tracker.observations().size() == 1);
  tracker.update({}, at(100), at(100));
  assert(tracker.observations().empty());
  assert(!tracker.predict(at(100)).empty()); // prediction is not a fresh measurement
  RiskEstimator replay(cfg);
  replay.update({close}, 640, 360, nullptr, true, 0.0);
  close.box.height += 20;
  assert(replay.update({close},640,360,nullptr,true,.2).relative_speed_kmh > 0.0f);

  cv::Mat roi_test(40, 120, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(roi_test, cv::Rect(5, 5, 25, 25), cv::Scalar(0, 0, 255), cv::FILLED);
  cv::rectangle(roi_test, cv::Rect(45, 5, 25, 25), cv::Scalar(0, 255, 0), cv::FILLED);
  cv::rectangle(roi_test, cv::Rect(85, 5, 25, 25), cv::Scalar(255, 255, 255), cv::FILLED);
  assert(roi_light_color(roi_test, cv::Rect2f(5, 5, 25, 25)) == SignalState::RED);
  assert(roi_light_color(roi_test, cv::Rect2f(45, 5, 25, 25)) == SignalState::GREEN);
  assert(roi_light_color(roi_test, cv::Rect2f(85, 5, 25, 25)) == SignalState::UNKNOWN);
  Detection red_roi = red;
  red_roi.box = cv::Rect2f(5, 5, 25, 25);
  Detection wrong_green = red_roi;
  wrong_green.class_id = 11;
  int checked = 0, rejected = 0;
  const auto verified = verify_light_colors(roi_test, {red_roi, wrong_green}, &checked, &rejected);
  assert(checked == 2 && rejected == 1 && verified.size() == 1);

  LaneChangeAdvisor advisor;
  advisor.set_intent(Intent::LEFT, at(0));
  assert(advisor.update(false, false, false, true, at(0)) == LaneChangeAdvice::UNKNOWN);
  assert(advisor.update(false, false, true, true, at(100)) == LaneChangeAdvice::CHECK);
  assert(advisor.update(true, false, true, true, at(200)) == LaneChangeAdvice::BLOCKED);
  assert(advisor.update(false, false, true, true, at(500)) == LaneChangeAdvice::BLOCKED);
  assert(advisor.update(false, false, true, true, at(1100)) == LaneChangeAdvice::CHECK);
  assert(advisor.update(false, false, true, true, at(11000)) == LaneChangeAdvice::IDLE);

  VehicleStateManager vehicle_manager;
  vehicle_manager.set_demo_turn(TurnSignal::LEFT, at(0));
  VehicleState vehicle = vehicle_manager.snapshot(at(100));
  assert(vehicle.turn_valid && vehicle.turn == TurnSignal::LEFT && vehicle.demo);
  assert(!vehicle_manager.snapshot(at(11000)).turn_valid);

  LaneSemanticTracker semantics;
  LaneMarkingObservation marking;
  marking.left = LaneMarking::DASHED;
  marking.right = LaneMarking::SOLID;
  lane.partial = false;
  lane.offset_ratio = 0.0f;
  LaneSemantic semantic = semantics.update(marking, lane, at(0));
  assert(!semantic.left_change_allowed);  // two observations are required
  semantic = semantics.update(marking, lane, at(100));
  assert(semantic.left_change_allowed && !semantic.right_change_allowed);

  LaneChangeFsm lane_change;
  lane_change.request(ManeuverDirection::LEFT, at(0));
  auto change = lane_change.update(vehicle, semantic, false, false, true, false, at(0));
  assert(change.state == LaneChangeState::CHECK_SAFE && !change.permitted);
  change = lane_change.update(vehicle, semantic, false, false, true, false, at(600));
  assert(change.state == LaneChangeState::ALLOW_CHANGE && change.permitted);
  change = lane_change.update(vehicle, semantic, true, false, true, false, at(700));
  assert(change.state == LaneChangeState::BLOCKED && !change.permitted);
  semantic.left = LaneMarking::UNKNOWN;
  semantic.left_change_allowed = false;
  change = lane_change.update(vehicle, semantic, false, false, true, false, at(800));
  assert(change.state == LaneChangeState::CHECK_RULE && !change.permitted);

  IntersectionLogic intersection_logic;
  Detection red_left = red;
  red_left.class_id = 8;
  for (int i = 0; i < 2; ++i)
    assert(!intersection_logic.update({red_left}, vehicle).stable);
  const auto intersection = intersection_logic.update({red_left}, vehicle);
  assert(intersection.stable && intersection.action == IntersectionAction::STOP);
  Detection green_straight = green;
  green_straight.class_id = 14;
  assert(intersection_logic.update({green_straight}, vehicle).action ==
         IntersectionAction::UNKNOWN);  // left intent cannot consume straight green

  WarningManager warning_manager;
  const auto warnings = warning_manager.update(true, true, true, false, true,
                                                change, intersection);
  assert(warnings.primary == WarningCode::FCW && warnings.priority == 100);

  std::cout << "PASS: perception scheduling, LDW gates, fresh observations, RCW, "
               "HSV light verification, ADAS Logic V2 state/rule/priority gates\n";
}
