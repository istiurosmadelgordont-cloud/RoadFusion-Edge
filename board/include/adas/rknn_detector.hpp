#pragma once

#include "adas/config.hpp"
#include "adas/types.hpp"
#include "rknn_api.h"
#include <opencv2/core.hpp>
#include <vector>

namespace adas {

class RknnDetector {
 public:
  explicit RknnDetector(const DetectorConfig& config);
  ~RknnDetector();
  RknnDetector(const RknnDetector&) = delete;
  RknnDetector& operator=(const RknnDetector&) = delete;

  bool ready() const { return ready_; }
  const std::string& error() const { return error_; }
  const std::string& runtime_version() const { return runtime_version_; }
  std::vector<Detection> detect(const cv::Mat& bgr, double* inference_ms = nullptr);

 private:
  struct Letterbox {
    float scale = 1.0f;
    int pad_x = 0;
    int pad_y = 0;
    int source_w = 0;
    int source_h = 0;
  };

  cv::Mat preprocess(const cv::Mat& bgr, Letterbox& meta) const;
  std::vector<Detection> postprocess(const std::vector<rknn_output>& outputs,
                                     const Letterbox& meta) const;
  void fail(const std::string& message);

  DetectorConfig config_;
  rknn_context context_ = 0;
  rknn_input_output_num io_num_{};
  std::vector<rknn_tensor_attr> input_attrs_;
  std::vector<rknn_tensor_attr> output_attrs_;
  bool ready_ = false;
  std::string error_;
  std::string runtime_version_;
};

}  // namespace adas
