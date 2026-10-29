#pragma once

#include "adas/types.hpp"
#include "adas/pcie_rga.hpp"
#include "adas/ufld_v2_decode.hpp"
#include "rknn_api.h"

#include <opencv2/core.hpp>
#include <atomic>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace adas {

// UFLD V1 CULane (800x288) and the experimental UFLDv2 CULane student
// (800x320) share this runtime.  Model type is inferred from its output count.
class UfldLaneDetector {
 public:
  explicit UfldLaneDetector(const std::string& model_path, bool rga_required = false);
  ~UfldLaneDetector();
  UfldLaneDetector(const UfldLaneDetector&) = delete;
  UfldLaneDetector& operator=(const UfldLaneDetector&) = delete;

  bool ready() const { return ready_; }
  const std::string& error() const { return error_; }
  const std::string& profile() const { return profile_; }
  int input_width() const { return input_width_; }
  int input_height() const { return input_height_; }
  LaneResult detect(const cv::Mat& bgr, double* inference_ms = nullptr);
  void reset() {}

 private:
  using Points = UfldLaneSamples;

  LaneResult decode_v1(const std::vector<rknn_output>& outputs,
                       int width, int height) const;
  LaneResult decode_v2(const std::vector<rknn_output>& outputs,
                       int width, int height) const;
  LaneResult make_result(const std::array<Points, 4>& slots,
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
  bool rga_required_ = false;
  mutable std::atomic<bool> rga_failure_reported_{false};
  bool ready_ = false;
  std::string profile_;
  std::string error_;
  std::unique_ptr<PcieBgrPool> rga_resize_pool_;
  std::unique_ptr<PcieBgrPool> rga_rgb_pool_;
};

}  // namespace adas
