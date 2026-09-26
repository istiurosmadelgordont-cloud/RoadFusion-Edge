#include "adas/warning_logic.hpp"
#include "adas/signal_logic.hpp"
#include "adas/drive_decision.hpp"
#include "adas/risk_estimator.hpp"
#include "adas/byte_tracker.hpp"
#include "adas/perception_schedule.hpp"
#include "adas/experimental_adas.hpp"
#include "adas/adas_logic_v2.hpp"
#include "adas/lane_geometry_tracker.hpp"
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
  assert(monitor.update(lane, at(100)).departure);
  assert(monitor.update(lane, at(300)).departure);
  lane.partial = true;
  assert(monitor.update(lane, at(400)).departure);  // one weak update must not flash off
  lane.partial = false;
  assert(monitor.update(lane, at(500)).departure);
  assert(monitor.update(lane, at(800)).departure);
  assert(!monitor.update(lane, at(2000)).departure); // gap must restart confirmation
  lane.offset_ratio = 0;
  for (int ms = 2200; ms <= 3000; ms += 200)
    assert(!monitor.update(lane, at(ms)).departure);

  const auto make_lane = [](int left_bottom, int right_bottom) {
    LaneResult value;
    value.valid = true;
    value.offset_ratio = (320.0f - (left_bottom + right_bottom) * 0.5f) /
                         (right_bottom - left_bottom);
    value.left = {{left_bottom, 350}, {left_bottom + 25, 170}};
    value.right = {{right_bottom - 25, 170}, {right_bottom, 350}};
    value.polygon = value.left;
    value.polygon.insert(value.polygon.end(), value.right.begin(), value.right.end());
    return value;
  };
  NeuralLaneGeometryTracker geometry_tracker;
  LaneResult stable_lane = geometry_tracker.update(make_lane(180, 460));
  assert(stable_lane.valid && !stable_lane.identity_uncertain);
  LaneResult adjacent_lane = geometry_tracker.update(make_lane(470, 750));
  assert(adjacent_lane.valid && adjacent_lane.partial && adjacent_lane.identity_uncertain);
  assert(adjacent_lane.reassignment_side == LaneDepartureSide::NONE);
  // Ego moves left: the SAME lane moves right in the image (negative offset).
  // Once the model selects the new left lane, its centre jumps back left.
  geometry_tracker.reset();
  geometry_tracker.update(make_lane(260, 540));
  adjacent_lane = geometry_tracker.update(make_lane(-20, 260));
  assert(adjacent_lane.reassignment_side == LaneDepartureSide::LEFT);
  monitor.reset();
  const LaneResult reassigned_warning = monitor.update(adjacent_lane, at(0));
  assert(reassigned_warning.departure &&
         reassigned_warning.departure_side == LaneDepartureSide::LEFT);

  LaneResult weak = make_lane(180, 460);
  weak.partial = true;
  assert(monitor.update(weak, at(200)).departure);
  for (int ms = 400; ms <= 1800; ms += 200) monitor.update(weak, at(ms));
  assert(!monitor.update(weak, at(2000)).departure);
  geometry_tracker.reset();
  geometry_tracker.update(make_lane(100, 380));
  adjacent_lane = geometry_tracker.update(make_lane(380, 660));
  assert(adjacent_lane.reassignment_side == LaneDepartureSide::RIGHT);
  monitor.reset();
  assert(monitor.update(adjacent_lane, at(0)).departure_side == LaneDepartureSide::RIGHT);
  LaneResult centred = make_lane(180, 460);
  for (int ms = 200; ms <= 1800; ms += 200) monitor.update(centred, at(ms));
  assert(!monitor.update(centred, at(2000)).departure);

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
  tracker.reset();
  tracker.update({close}, at(0), at(0));
  close.box.x += 5;
  tracker.update({close}, at(100), at(100));
  close.box.x += 5;
  const auto moving = tracker.update({close}, at(200), at(200));
  assert(moving.size() == 1 && moving.front().motion_valid && moving.front().image_velocity.x > 0);
  assert(!tracker.predict(at(1100)).front().motion_valid);
  BlindSpotMonitor side;
  Detection side_car = close;
  side_car.box = cv::Rect2f(200, 190, 160, 120);
  assert(!side.update({side_car}, 640, 360, true).occupied);
  side_car.track_id = 2;  // The zone is still occupied after an ID switch.
  const auto occupied_side = side.update({side_car}, 640, 360, true);
  assert(occupied_side.occupied && occupied_side.track_id == 2);
  assert(side.update({}, 640, 360, false).occupied);  // No fresh sample.
  assert(side.update({}, 640, 360, true).occupied);
  assert(side.update({}, 640, 360, true).occupied);
  assert(!side.update({}, 640, 360, true).occupied);
  side_car.track_id = -1;
  assert(!side.update({side_car}, 640, 360, true).occupied);
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

  LaneSemanticTracker tlc_tracker;
  lane.offset_ratio = 0.0f;
  tlc_tracker.update(marking, lane, at(0));
  lane.offset_ratio = 0.10f;
  semantic = tlc_tracker.update(marking, lane, at(100));
  assert(semantic.trend == CrossingSide::RIGHT && semantic.tlc_s > 0.0f);
  lane.departure = true;
  lane.departure_side = LaneDepartureSide::LEFT;
  semantic = tlc_tracker.update(marking, lane, at(200));
  assert(semantic.crossing == CrossingSide::LEFT);
  lane.departure = false;
  lane.departure_side = LaneDepartureSide::NONE;
  semantic.crossing = CrossingSide::NONE;
  semantic.trend = CrossingSide::NONE;

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
  green_straight.score = .9f;
  assert(intersection_logic.update({green_straight}, vehicle).action ==
         IntersectionAction::UNKNOWN);  // left intent cannot consume straight green

  VehicleState unknown_route;
  for (int i = 0; i < 5; ++i)
    assert(intersection_logic.update({green_straight}, unknown_route).action == IntersectionAction::UNKNOWN);
  unknown_route.turn_valid = true;
  unknown_route.turn = TurnSignal::OFF;
  assert(!intersection_logic.update({green_straight}, unknown_route).route_valid);
  unknown_route.route_valid = true;
  unknown_route.route = ManeuverDirection::STRAIGHT;
  for (int i = 0; i < 3; ++i) intersection_logic.update({green_straight}, unknown_route);
  assert(intersection_logic.update({green_straight}, unknown_route).action == IntersectionAction::GO);
  Detection green_left = green;
  green_left.class_id = 12;
  green_left.score = .9f;
  unknown_route.route = ManeuverDirection::LEFT;
  assert(intersection_logic.update({green_left}, unknown_route).action == IntersectionAction::WAIT);
  // A conflicting red must not lose to a higher-confidence green.
  red_left.score = .5f;
  green_left.score = .95f;
  for (int i = 0; i < 3; ++i) intersection_logic.update({green_left, red_left}, unknown_route);
  assert(intersection_logic.update({green_left, red_left}, unknown_route).action == IntersectionAction::STOP);

  WarningManager warning_manager;
  const auto warnings = warning_manager.update(true, true, true, false, true,
                                                change, intersection);
  assert(warnings.primary == WarningCode::FCW && warnings.priority == 100);

  std::cout << "PASS: perception scheduling, LDW gates, fresh observations, RCW, "
               "HSV light verification, ADAS Logic V2 state/rule/priority gates\n";
}
