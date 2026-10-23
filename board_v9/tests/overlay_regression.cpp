#include "adas/adas_logic_v2.hpp"
#include "adas/overlay.hpp"
#include "adas/lane_paint_display.hpp"

#include <cassert>
#include <iostream>
#include <opencv2/core.hpp>

namespace {

cv::Mat render(const adas::LaneResult& lane, const adas::RiskResult& risk,
               const std::vector<adas::Detection>& detections,
               const adas::LaneSemantic* semantics = nullptr) {
  cv::Mat frame(360, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  adas::draw_overlay(frame, detections, lane, adas::SignalResult(), risk,
                     adas::DriveResult(), 20.0, 80.0, semantics);
  return frame;
}

}  // namespace

int main() {
  {
    // 高亮只能跟随已选 ID，不能继续画在旧位置或跳到占据旧位置的邻灯。
    adas::SignalResult signal;
    signal.state=adas::SignalState::RED; signal.stable=true;
    signal.box={100,40,8,8}; signal.track_id=41; signal.score=.99f;
    adas::Detection tracked,neighbor;
    tracked.class_id=7; tracked.track_id=41; tracked.box={160,45,10,10};
    neighbor=tracked; neighbor.track_id=42; neighbor.box=cv::Rect2f(signal.box);
    const auto displayed=adas::tracked_signal_for_display(signal,{neighbor,tracked});
    assert(displayed.box==cv::Rect(tracked.box));
    assert(signal.box.x==100 && signal.state==displayed.state && displayed.stable);
    assert(adas::tracked_signal_for_display(signal,{neighbor}).box.area()==0);
    tracked.class_id=11; // 新颜色不能借用旧红灯状态继续描边。
    const auto changed=adas::tracked_signal_for_display(signal,{tracked});
    assert(changed.box.area()==0 && changed.state==signal.state && changed.score==signal.score);
    signal.track_id=-1;
    assert(adas::tracked_signal_for_display(signal,{}).box==signal.box);
    std::cout<<"PASS: selected lamp outline follows ID without changing signal evidence\n";
  }
  adas::Detection car;
  car.class_id = 2;
  car.track_id = 23;
  car.box = cv::Rect2f(300, 170, 70, 70);
  adas::RiskResult risk;
  risk.target = true;
  risk.track_id = 23;
  risk.box = cv::Rect(car.box);
  risk.distance_m = 15.3f;
  risk.relative_speed_kmh = 22.0f;
  risk.ttc_s = 1.2f;

  // An unconfirmed detection must not claim an FCW risk on screen.
  cv::Mat frame = render(adas::LaneResult(), risk, {car});
  assert(frame.at<cv::Vec3b>(90, 310) == cv::Vec3b(40, 40, 40));

  risk.reliable = true;
  frame = render(adas::LaneResult(), risk, {car});
  const cv::Mat danger_frame = frame.clone();
  const cv::Vec3b danger = frame.at<cv::Vec3b>(90, 310);
  assert(danger[2] > 200 && danger[1] < 100);

  risk.ttc_s = 5.0f;
  frame = render(adas::LaneResult(), risk, {car});
  const cv::Vec3b normal = frame.at<cv::Vec3b>(90, 310);
  assert(normal[1] > 180 && normal[2] < 120);
  const cv::Rect outside_target(290, 184, 9, 42);
  assert(cv::mean(danger_frame(outside_target))[2] >
         cv::mean(frame(outside_target))[2] + 3.0);

  cv::Mat side_clear(360, 640, CV_8UC3, cv::Scalar(40, 40, 40));
  cv::Mat side_warning = side_clear.clone();
  cv::Mat side_blocked = side_clear.clone();
  adas::draw_side_risk_cue(side_warning, {250, 120, 110, 100}, false);
  adas::draw_side_risk_cue(side_blocked, {250, 120, 110, 100}, true);
  const cv::Vec3b yellow = side_warning.at<cv::Vec3b>(220, 305);
  const cv::Vec3b red = side_blocked.at<cv::Vec3b>(220, 305);
  assert(yellow[1] > side_clear.at<cv::Vec3b>(220, 305)[1] + 15);
  assert(red[2] > yellow[2] + 15);
  assert(side_warning.at<cv::Vec3b>(20, 20) ==
         side_clear.at<cv::Vec3b>(20, 20));

  adas::LaneResult lane;
  lane.valid = true;
  lane.departure = true;
  lane.departure_side = adas::LaneDepartureSide::RIGHT;
  for (int i = 0; i < 32; ++i) {
    lane.left.emplace_back(180 + i * 3, 350 - i * 7);
    lane.right.emplace_back(367 + i * 3, 133 + i * 7);
  }
  adas::LaneSemantic semantics;
  semantics.left = adas::LaneMarking::SOLID;
  semantics.right = adas::LaneMarking::SOLID;
  frame = render(lane, adas::RiskResult(), {}, &semantics);
  const cv::Vec3b left = frame.at<cv::Vec3b>(350, 180);
  const cv::Vec3b right = frame.at<cv::Vec3b>(350, 460);
  assert(left[1] > left[2] && right[2] > right[1]);

  adas::LaneResult display = lane;
  display.departure = false;
  display.departure_side = adas::LaneDepartureSide::NONE;
  display.partial = true;  // Exactly half of V2's anchors is safety-partial.
  display.boundary_coverage[1] = 0.50f;
  display.boundary_coverage[2] = 0.51f;
  frame = render(display, adas::RiskResult(), {});
  const cv::Vec3b dense_partial = frame.at<cv::Vec3b>(300, 320);
  assert(dense_partial[1] > dense_partial[0] + 10);
  display.boundary_coverage[1] = 0.20f;
  frame = render(display, adas::RiskResult(), {});
  const cv::Vec3b sparse_partial = frame.at<cv::Vec3b>(300, 320);
  assert(sparse_partial[1] < sparse_partial[0] + 5);
  frame = render(display, adas::RiskResult(), {}, &semantics);
  assert(frame.at<cv::Vec3b>(350,180)==cv::Vec3b(40,40,40));
  assert(frame.at<cv::Vec3b>(350,460)==cv::Vec3b(40,40,40));
  display.partial=false;display.identity_uncertain=true;
  frame = render(display, adas::RiskResult(), {}, &semantics);
  assert(frame.at<cv::Vec3b>(350,180)==cv::Vec3b(40,40,40));
  display.identity_uncertain=false;
  semantics.left=semantics.right=adas::LaneMarking::UNKNOWN;
  frame = render(display, adas::RiskResult(), {}, &semantics);
  const auto reference=frame.at<cv::Vec3b>(350,180);
  assert(reference[1]>reference[0]+50 && reference[1]>reference[2]+50);
  assert(semantics.left==adas::LaneMarking::UNKNOWN && !semantics.left_change_allowed);
  std::cout<<"PASS: weak/uncertain lanes hidden; unknown paint keeps green ego references, never white\n";

  {
    // Real paint transitions include an unclassified frame and the first
    // confirmation of the new type. Geometry must survive both while the
    // rule engine still rejects UNKNOWN and unconfirmed dashed evidence.
    adas::LaneSemanticTracker tracker;
    adas::LaneMarkingObservation observation;
    const auto base=std::chrono::steady_clock::time_point(std::chrono::seconds(1));
    int step=0;
    auto update=[&](adas::LaneMarking marking) {
      observation.left=observation.right=marking;
      observation.left_coverage=observation.right_coverage=.9f;
      return tracker.update(observation,display,base+std::chrono::milliseconds(330*step++));
    };
    auto confirmed=update(adas::LaneMarking::DASHED);
    confirmed=update(adas::LaneMarking::DASHED);
    assert(confirmed.left_change_allowed && confirmed.right_change_allowed);
    for(auto marking:{adas::LaneMarking::SOLID,adas::LaneMarking::UNKNOWN,
                      adas::LaneMarking::SOLID}) {
      const auto pending=update(marking);
      assert(pending.left==adas::LaneMarking::UNKNOWN && !pending.left_change_allowed);
      assert(pending.right==adas::LaneMarking::UNKNOWN && !pending.right_change_allowed);
      const auto transition=render(display,adas::RiskResult(),{},&pending);
      for(auto point:{cv::Point(180,350),cv::Point(460,350)}) {
        const auto pixel=transition.at<cv::Vec3b>(point);
        assert(pixel[1]>pixel[0]+50 && pixel[1]>pixel[2]+50);
      }
    }
    confirmed=update(adas::LaneMarking::SOLID);
    assert(confirmed.left==adas::LaneMarking::SOLID && !confirmed.left_change_allowed);
    const auto pending=update(adas::LaneMarking::DASHED);
    assert(pending.left==adas::LaneMarking::UNKNOWN && !pending.left_change_allowed);
    confirmed=update(adas::LaneMarking::DASHED);
    assert(confirmed.left_change_allowed && confirmed.right_change_allowed);
    auto absent=display;absent.valid=false;
    const auto hidden=render(absent,adas::RiskResult(),{},&pending);
    assert(hidden.at<cv::Vec3b>(350,180)==cv::Vec3b(40,40,40));
    assert(hidden.at<cv::Vec3b>(350,460)==cv::Vec3b(40,40,40));
    cv::Mat candidate(360,640,CV_8UC3,cv::Scalar(40,40,40));
    adas::draw_lane_geometry_reference(candidate,display);
    assert(candidate.at<cv::Vec3b>(350,180)[1]>100);
    assert(candidate.at<cv::Vec3b>(300,320)==cv::Vec3b(40,40,40)); // no confident corridor
    candidate.setTo(cv::Scalar(40,40,40));
    adas::draw_lane_geometry_reference(candidate,absent);
    assert(cv::norm(candidate,cv::Mat(candidate.size(),candidate.type(),cv::Scalar(40,40,40)))==0);
    auto uncertain=display;uncertain.identity_uncertain=true;
    adas::draw_lane_geometry_reference(candidate,uncertain); // held geometry is never a current observation
    assert(candidate.at<cv::Vec3b>(350,180)==cv::Vec3b(40,40,40));
    std::cout<<"PASS: dashed/solid/unknown transitions preserve current geometry without granting permission\n";
  }

  // Adjacent candidates require a supported outer slot and a reliable ego pair.
  {
    adas::LanePaintDisplayTracker animation;
    adas::LaneSemantic evidence;
    evidence.left=evidence.right=adas::LaneMarking::SOLID;
    const auto start=std::chrono::steady_clock::time_point(std::chrono::seconds(10));
    auto style=animation.update(evidence,display,start);
    assert(style.left.solid==1.f && style.left.dashed==0.f);
    evidence.left=evidence.right=adas::LaneMarking::UNKNOWN;
    style=animation.update(evidence,display,start+std::chrono::milliseconds(50));
    assert(style.left.solid>.7f && style.left.solid<1.f);
    assert(evidence.left==adas::LaneMarking::UNKNOWN && !evidence.left_change_allowed);
    const auto unchanged=animation.update(evidence,display,start+std::chrono::milliseconds(50));
    assert(unchanged.left.solid==style.left.solid); // redraw count cannot accelerate animation
    evidence.left=evidence.right=adas::LaneMarking::DASHED;
    style=animation.update(evidence,display,start+std::chrono::milliseconds(100));
    assert(style.left.dashed>0.f && style.left.dashed<.3f && style.left.solid>.5f);
    for(int ms=150;ms<=650;ms+=50)style=animation.update(evidence,display,start+std::chrono::milliseconds(ms));
    assert(style.left.dashed>.95f && style.left.solid<.03f);
    cv::Mat blended(360,640,CV_8UC3,cv::Scalar(40,40,40));
    adas::draw_overlay(blended,{},display,{}, {},{},3,250,&evidence,&style);
    int bright=0,faint=0;
    for(int y=150;y<340;++y){
      const int x=cvRound(180+(350-y)*3./7.);
      const auto value=blended.at<cv::Vec3b>(y,x);
      if(value[1]>170)++bright;
      if(value[1]>45 && value[1]<120)++faint;
    }
    assert(bright>40 && faint>20); // real gaps remain distinct, with a subdued reference
    auto absent=display;absent.valid=false;
    style=animation.update(evidence,absent,start+std::chrono::milliseconds(700));
    assert(style.left.solid==0.f && style.left.dashed==0.f);
    auto identity=display;identity.identity_uncertain=true;
    style=animation.update(evidence,identity,start+std::chrono::milliseconds(750));
    assert(style.right.solid==0.f && style.right.dashed==0.f);
    style=animation.update(evidence,display,start+std::chrono::milliseconds(800));
    assert(style.left.dashed==1.f); // new identity never inherits the old solid style
    std::cout<<"PASS: time-based paint animation preserves evidence, distinguishes dashes, resets on missing geometry\n";
  }

  // Adjacent candidates require a supported outer slot and a reliable ego pair.
  adas::LaneResult multi;
  multi.valid = true;
  multi.boundary_coverage[0] = 0.50f;
  multi.boundary_coverage[3] = 0.50f;
  multi.boundaries[0] = {{80, 350}, {85, 325}, {90, 300},
                         {95, 275}, {100, 250}, {105, 225}};
  multi.boundaries[3] = {{590, 350}, {575, 325}, {560, 300},
                         {545, 275}, {530, 250}, {515, 225}};
  multi.lane_polygons[0] = {{80, 225}, {150, 225}, {190, 350}, {50, 350}};
  multi.lane_polygons[2] = {{470, 225}, {550, 225}, {620, 350}, {520, 350}};
  adas::LaneSemantic multi_semantics;
  multi_semantics.lane_valid = true;
  multi_semantics.left = adas::LaneMarking::DASHED;
  multi_semantics.right = adas::LaneMarking::DASHED;
  multi_semantics.boundaries[0] = adas::LaneMarking::DASHED;
  multi_semantics.boundaries[3] = adas::LaneMarking::DASHED;
  frame = render(multi, adas::RiskResult(), {}, &multi_semantics);
  assert(frame.at<cv::Vec3b>(300, 110)[2] > 45);
  assert(frame.at<cv::Vec3b>(300, 560)[0] >
         frame.at<cv::Vec3b>(300, 560)[2] + 8);
  multi_semantics.boundaries[3] = adas::LaneMarking::UNKNOWN;
  frame = render(multi, adas::RiskResult(), {}, &multi_semantics);
  const auto unknown_edge=frame.at<cv::Vec3b>(300,560);
  // UNKNOWN keeps the subdued purple area, without a bright grey dot/stroke.
  assert(unknown_edge[0]>unknown_edge[2]+8);
  assert(std::max(unknown_edge[0],std::max(unknown_edge[1],unknown_edge[2]))<120);
  multi_semantics.boundaries[3] = adas::LaneMarking::DASHED;
  multi.identity_uncertain = true;
  frame = render(multi, adas::RiskResult(), {}, &multi_semantics);
  assert(frame.at<cv::Vec3b>(300, 110) == cv::Vec3b(40, 40, 40));
  multi.identity_uncertain = false;
  multi_semantics.right = adas::LaneMarking::SOLID;
  frame = render(multi, adas::RiskResult(), {}, &multi_semantics);
  assert(frame.at<cv::Vec3b>(300, 560)[0] > 40);
  multi_semantics.lane_valid = false;
  frame = render(multi, adas::RiskResult(), {}, &multi_semantics);
  assert(frame.at<cv::Vec3b>(300, 560)[0] > 40);
  multi.boundary_coverage[3] = 0.20f;
  frame = render(multi, adas::RiskResult(), {}, &multi_semantics);
  assert(frame.at<cv::Vec3b>(300, 560) == cv::Vec3b(40, 40, 40));

  std::cout << "PASS: FCW confirmation, risk tiers/halo, side cue, LDW color, adjacent lanes\n";
}
