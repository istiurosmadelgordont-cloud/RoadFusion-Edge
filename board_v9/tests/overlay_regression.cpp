#include "adas/adas_logic_v2.hpp"
#include "adas/overlay.hpp"

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
  frame = render(multi, adas::RiskResult(), {}, &multi_semantics);
  assert(frame.at<cv::Vec3b>(300, 110)[2] > 45);
  assert(frame.at<cv::Vec3b>(300, 560)[0] >
         frame.at<cv::Vec3b>(300, 560)[2] + 8);
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
