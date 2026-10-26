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
  {
    // 小灯位移超过自身宽度时，仍应跟随同一个灯组，而非每次新建轨迹。
    ByteTracker tracker;
    Detection housing, bulb;
    housing.class_id=21; housing.name="交通灯"; housing.score=.95f;
    housing.box={100,20,40,100}; housing.source_housing_index=0;
    bulb.class_id=7; bulb.name="红色圆灯 MO 99%"; bulb.score=.99f;
    bulb.box={114,40,8,8}; bulb.source_housing_index=0;
    tracker.update({bulb,housing},at(0),at(0)); // 输入顺序故意先子后父。
    const int bulb_id=tracker.identify_measurements({bulb})[0].track_id;
    assert(bulb_id>=0);
    for(int step=1;step<=2;++step) {
      housing.box.x+=10; bulb.box.x+=10;
      // 每轮来源编号可变，不能直接作为跨帧身份。
      housing.source_housing_index=bulb.source_housing_index=step+10;
      tracker.update({bulb,housing},at(step*100),at(step*100));
      const auto identified=tracker.identify_measurements({bulb});
      assert(identified.size()==1 && identified[0].track_id==bulb_id);
      assert(identified[0].box==bulb.box && identified[0].score==bulb.score);
    }
    auto predicted=tracker.predict(at(350));
    bool followed=false;
    for(const auto& d:predicted) if(d.track_id==bulb_id) {
      assert(std::abs(d.box.x-149.f)<.1f); // 150ms 内移动 15px，不再被限制为 4px。
      followed=true;
    }
    assert(followed);
    // 漏掉一轮 MO 时只暂存描边；不能在 observations() 中伪造新灯态。
    housing.box.x=130;
    tracker.update({housing},at(300),at(300));
    for(const auto& d:tracker.observations()) assert(d.class_id==21);
    followed=false;
    for(const auto& d:tracker.predict(at(350))) if(d.track_id==bulb_id) {
      assert(std::abs(d.box.x-149.f)<.1f && d.name.find("暂存")!=std::string::npos);
      followed=true;
    }
    assert(followed);
    assert(tracker.identify_measurements({bulb})[0].track_id<0);
    assert(tracker.predict(at(1350)).empty());

    // 同一灯组放大后，小灯位置和尺寸也应随之变化。
    tracker.reset(); housing.box={100,20,40,100}; bulb.box={114,40,8,8};
    housing.source_housing_index=bulb.source_housing_index=0;
    tracker.update({housing,bulb},at(0),at(0));
    housing.box={110,20,44,110}; bulb.box={125.4f,42,8.8f,8.8f};
    tracker.update({housing,bulb},at(100),at(100));
    const auto raw=tracker.identify_measurements({bulb})[0];
    bool scaled=false;
    for(const auto& d:tracker.predict(at(200))) if(d.track_id==raw.track_id) {
      assert(d.box.x>raw.box.x+10.f && d.box.width>raw.box.width+.7f);
      scaled=true;
    }
    assert(scaled && raw.box==bulb.box);

    // 重叠灯组依靠明确来源建立父子关联；输入重排不能交换子灯身份。
    tracker.reset();
    Detection other_housing=housing, other_bulb=bulb;
    housing.box={100,20,60,100}; housing.source_housing_index=0;
    other_housing.box={130,20,60,100}; other_housing.source_housing_index=1;
    bulb.box={144,40,8,8}; bulb.source_housing_index=0;
    other_bulb.box={164,40,8,8}; other_bulb.source_housing_index=1;
    tracker.update({housing,other_housing,bulb,other_bulb},at(0),at(0));
    const auto originals=tracker.identify_measurements({bulb,other_bulb});
    housing.box.x+=10; other_housing.box.x+=10; bulb.box.x+=10; other_bulb.box.x+=10;
    housing.source_housing_index=bulb.source_housing_index=1;
    other_housing.source_housing_index=other_bulb.source_housing_index=0;
    tracker.update({other_bulb,other_housing,bulb,housing},at(100),at(100));
    const auto matched=tracker.identify_measurements({bulb,other_bulb});
    assert(matched[0].track_id==originals[0].track_id);
    assert(matched[1].track_id==originals[1].track_id && matched[0].track_id!=matched[1].track_id);
    // 换色只更新真实观测，不残留旧色轨迹。
    bulb.class_id=11;
    tracker.update({housing,other_housing,bulb,other_bulb},at(200),at(200));
    assert(tracker.identify_measurements({bulb})[0].track_id==originals[0].track_id);
    for(const auto& d:tracker.predict(at(220)))
      if(d.track_id==originals[0].track_id) assert(d.class_id==11);
    std::cout<<"PASS: lamp parent binding, translation/scale, missed evidence and changing source order\n";
  }
  {
    ByteTracker lamp_tracker;
    Detection red_bulb;
    red_bulb.class_id=7;red_bulb.name="red_circle";red_bulb.score=.95f;
    red_bulb.box=cv::Rect2f(100,40,20,20);
    Detection neighbor=red_bulb;neighbor.box.x=160;
    auto initial=lamp_tracker.update({red_bulb,neighbor},at(0),at(0));
    assert(initial.size()==2);
    const int bulb_id=initial[0].track_id;
    Detection green_bulb=red_bulb;green_bulb.class_id=11;green_bulb.name="green_circle";
    auto changed=lamp_tracker.update({green_bulb,neighbor},at(100),at(100));
    assert(changed.size()==2); // No lingering red track at the changed bulb.
    int changed_count=0;
    for(const auto& bulb:changed) if(bulb.box.x<130) {
      assert(bulb.class_id==11 && bulb.track_id==bulb_id);++changed_count;
    }
    assert(changed_count==1);
    Detection car=green_bulb;car.class_id=2;
    auto different_object=lamp_tracker.update({car,neighbor},at(200),at(200));
    bool new_car_id=false;
    for(const auto& object:different_object) if(object.class_id==2) new_car_id=object.track_id!=bulb_id;
    assert(new_car_id);
    auto expired=lamp_tracker.predict(at(1250));
    assert(expired.empty()); // No indefinite boxes when inference stops updating.
    ByteTracker vehicle_tracker;
    vehicle_tracker.update({car},at(0),at(0));
    assert(vehicle_tracker.predict(at(600)).size()==1);
    assert(vehicle_tracker.predict(at(700)).empty());
    vehicle_tracker.update({},at(200),at(200));
    assert(vehicle_tracker.predict(at(200)).empty());
    vehicle_tracker.update({},at(300),at(300));
    assert(vehicle_tracker.predict(at(300)).empty());
    // Ordinary objects bridge bounded inference gaps but never survive a fresh
    // detector miss. Slow results must not leave old geometry on a turning scene.
    ByteTracker delayed;
    delayed.update({car},at(0),at(450));
    assert(delayed.predict(at(600)).size()==1);
    assert(delayed.predict(at(700)).empty());
    assert(delayed.predict(at(950)).empty());
    assert(delayed.predict(at(1001)).empty()); // absolute source-age limit
    delayed.update({car},at(500),at(950));
    assert(delayed.predict(at(1150)).size()==1); // matched tracks refresh receipt time too
    delayed.update({},at(700),at(1150));
    assert(delayed.observations().empty() && delayed.predict(at(1150)).empty());
    delayed.update({},at(800),at(1250));
    assert(delayed.predict(at(1250)).empty());
    ByteTracker receipt_bound;
    receipt_bound.update({car},at(0),at(100));
    assert(receipt_bound.predict(at(751)).empty()); // receipt-age limit
    // The parent housing must bridge the same delayed shared-NPU interval as
    // its bulb. Expiring the parent 650ms after capture hides a fresh result
    // that only arrived 250ms ago, although no detector miss occurred.
    ByteTracker delayed_lamps;
    Detection housing;
    housing.class_id=21;housing.name="交通灯";housing.score=.95f;
    housing.box={90,20,50,90};housing.source_housing_index=4;
    Detection bulb=red_bulb;
    bulb.box={105,40,10,10};bulb.source_housing_index=4;
    delayed_lamps.update({housing,bulb},at(0),at(450));
    auto during_gap=delayed_lamps.predict(at(700));
    assert(during_gap.size()==2);
    bool held_bulb=false;
    for(const auto& d:during_gap)if(d.class_id==7) {
      held_bulb=true;assert(d.name.find("暂存")!=std::string::npos);
    }
    assert(held_bulb && delayed_lamps.predict(at(950)).size()==2);
    assert(delayed_lamps.predict(at(1001)).empty());
    assert(delayed_lamps.identify_measurements({bulb})[0].box==bulb.box);
    // New measured colour replaces the held one immediately, without turning
    // a carried-over parent/child into another measured lamp.
    bulb.class_id=11;
    delayed_lamps.update({housing,bulb},at(500),at(950));
    for(const auto& d:delayed_lamps.predict(at(1100)))assert(d.class_id!=7);
    delayed_lamps.update({},at(1000),at(1200));
    assert(delayed_lamps.observations().empty());
    delayed_lamps.update({},at(1100),at(1300));
    assert(delayed_lamps.predict(at(1300)).empty());
    delayed_lamps.reset();
    delayed_lamps.update({housing,bulb},at(0),at(50));
    assert(delayed_lamps.predict(at(701)).empty()); // bounded receipt gap
    std::cout<<"PASS: delayed lamp parent/child display; source/receipt expiry and fresh-colour replacement\n";
    std::cout<<"PASS: one lamp track across red/green transitions; adjacent lamps and cars remain distinct\n";
  }
  std::array<std::chrono::steady_clock::time_point, 3> launched{{at(0),at(0),at(0)}};
  assert(next_perception_task(launched, true) == 1);
  launched[1] = at(200);
  assert(next_perception_task(launched, true) == 0);
  launched = {{at(600),at(600),at(0)}};
  assert(next_perception_task(launched, true) == 2);
  assert(next_perception_task(launched, false) == 1);
  const PerceptionPeriods precision = lane_priority_periods(true);
  launched = {{at(0), at(0), at(0)}};
  assert(next_perception_task(launched, true, precision) == 0);
  const auto concurrent=lane_priority_periods(true,true);
  launched={{at(900),at(800),at(0)}};
  assert(next_perception_task(launched,false,precision)==0);
  assert(next_perception_task(launched,false,concurrent)==1);
  launched[1]=at(1000);
  assert(next_perception_task(launched,false,concurrent)==0);
  launched={{at(0),at(0),at(0)}};
  int balanced_counts[2]={0,0},balanced_time=1;
  for(int iteration=0;iteration<60;++iteration){
    int task=next_perception_task(launched,false,concurrent);
    if(balanced_counts[task])assert(balanced_time-std::chrono::duration_cast<std::chrono::milliseconds>(launched[task]-at(0)).count()<=400);
    launched[task]=at(balanced_time);++balanced_counts[task];balanced_time+=task==0?130:250;
  }
  assert(balanced_counts[0]==30 && balanced_counts[1]==30);
  // Model a shared NPU with 100 ms YOLO, 220 ms UFLD and 30 ms dispatch cost.
  // Neither task may be starved, including when rear inference is enabled.
  for (bool rear : {false, true}) {
    launched = {{at(0), at(0), at(0)}};
    int simulated_ms = 0;
    int count[3] = {0, 0, 0};
    for (int iteration = 0; iteration < 60; ++iteration) {
      const int task = next_perception_task(launched, rear, precision);
      const int gap = std::chrono::duration_cast<std::chrono::milliseconds>(
          at(simulated_ms) - launched[task]).count();
      if (count[task] > 0) assert(gap <= (task == 0 ? 550 : task == 1 ? 800 : 2100));
      launched[task] = at(simulated_ms);
      ++count[task];
      simulated_ms += task == 0 ? 130 : 250;
    }
    assert(count[0] > count[1] && count[1] > 5);
    if (rear) assert(count[2] > 0);
  }
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
  assert(tracker.predict(at(100)).empty()); // a fresh miss hides old ordinary geometry
  tracker.reset();
  tracker.update({close}, at(0), at(0));
  close.box.x += 5;
  tracker.update({close}, at(100), at(100));
  close.box.x += 5;
  const auto moving = tracker.update({close}, at(200), at(200));
  assert(moving.size() == 1 && moving.front().motion_valid && moving.front().image_velocity.x > 0);
  assert(tracker.predict(at(1100)).empty()); // Expired predictions must disappear.
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
