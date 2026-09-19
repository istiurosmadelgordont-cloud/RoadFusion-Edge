#pragma once

#include "adas/config.hpp"
#include "adas/types.hpp"
#include <opencv2/core.hpp>
#include <vector>

namespace adas {

class LaneDetector {
 public:
  explicit LaneDetector(const LaneConfig& config);
  LaneResult detect(const cv::Mat& frame);
  void reset();
  void set_roi(const cv::Point2f& tl, const cv::Point2f& tr,
               const cv::Point2f& br, const cv::Point2f& bl);

 private:
  cv::Mat threshold_lane_paint(const cv::Mat& frame);
  bool collect_sliding(const cv::Mat& warped, std::vector<cv::Point>& left,
                       std::vector<cv::Point>& right, cv::Mat& debug) const;
  bool collect_tracked(const cv::Mat& warped, std::vector<cv::Point>& left,
                       std::vector<cv::Point>& right, cv::Mat& debug) const;
  bool robust_fit(const std::vector<cv::Point>& points, int width, int height,
                  cv::Vec3d& fit) const;
  bool validate_fits(const cv::Vec3d& left, const cv::Vec3d& right,
                     int width, int height) const;
  LaneResult make_curve_result(const cv::Mat& binary, const cv::Mat& bird,
                               const cv::Mat& debug, const cv::Mat& inverse,
                               const cv::Vec3d& left, const cv::Vec3d& right,
                               int width, int height, float scale,
                               float reference_x, bool partial) const;
  LaneResult vanishing_fallback(const cv::Mat& binary, float scale);
  LaneResult make_direct_result(const cv::Mat& binary, const cv::Mat& road,
                                int y_top, int y_bottom, float scale,
                                bool partial) const;

  LaneConfig config_;
  cv::Vec3d previous_left_curve_{};
  cv::Vec3d previous_right_curve_{};
  bool has_curves_ = false;
  int missed_ = 0;
  float road_light_ = 0.0f;
  bool has_road_light_ = false;
  cv::Vec2d direct_left_{};
  cv::Vec2d direct_right_{};
  int direct_top_ = 0;
  bool has_direct_ = false;
  int direct_missed_ = 0;
};

}  // namespace adas
