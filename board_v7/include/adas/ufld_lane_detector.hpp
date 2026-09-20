#pragma once

#include "adas/types.hpp"
#include "rknn_api.h"

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace adas {

// UFLD V1 CULane (800x288) and the experimental UFLDv2 CULane student
// (800x320) share this runtime.  Model type is inferred from its output count.
class UfldLaneDetector {
 public:
  explicit UfldLaneDetector(const std::string& model_path);
  ~UfldLaneDetector();
  UfldLaneDetector(const UfldLaneDetector&) = delete;
  UfldLaneDetector& operator=(const UfldLaneDetector&) = delete;

  bool ready() const { return ready_; }
  const std::string& error() const { return error_; }
  const std::string& profile() const { return profile_; }
  LaneResult detect(const cv::Mat& bgr, double* inference_ms = nullptr);
  void reset() {}

 private:
  struct Points {
    std::vector<cv::Point2f> values;
    float confidence = 0.0f;
  };

  LaneResult decode_v1(const std::vector<rknn_output>& outputs,
                       int width, int height) const;
  LaneResult decode_v2(const std::vector<rknn_output>& outputs,
                       int width, int height) const;
  LaneResult make_result(const Points& left, const Points& right,
                         int width, int height) const;
  cv::Mat preprocess(const cv::Mat& bgr) const;
  void fail(const std::string& message);

  rknn_context context_ = 0;
  rknn_input_output_num io_num_{};
  rknn_tensor_attr input_attr_{};
  std::vector<rknn_tensor_attr> output_attrs_;
  int input_width_ = 0;
  int input_height_ = 0;
  bool v2_ = false;
  bool ready_ = false;
  std::string profile_;
  std::string error_;
};

}  // namespace adas
