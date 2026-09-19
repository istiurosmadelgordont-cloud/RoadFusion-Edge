#pragma once

#include "adas/config.hpp"
#include "adas/types.hpp"
#include <opencv2/core.hpp>

namespace adas {

class LaneDetector {
 public:
  explicit LaneDetector(const LaneConfig& config);
  LaneResult detect(const cv::Mat& frame);
  void reset();
  void set_roi(const cv::Point2f& tl, const cv::Point2f& tr,
               const cv::Point2f& br, const cv::Point2f& bl);

 private:
  bool fit_side(const std::vector<cv::Vec4i>& lines, bool left, int width, int height,
                cv::Vec4f& result) const;
  cv::Vec4f smooth(const cv::Vec4f& current, cv::Vec4f& previous, bool& has_previous) const;

  LaneConfig config_;
  cv::Vec4f previous_left_{};
  cv::Vec4f previous_right_{};
  cv::Vec3d previous_left_curve_{};
  cv::Vec3d previous_right_curve_{};
  bool has_left_ = false;
  bool has_right_ = false;
  bool has_left_curve_ = false;
  bool has_right_curve_ = false;
  int left_missing_ = 0;
  int right_missing_ = 0;
};

}  // namespace adas
