#pragma once
#include "adas/types.hpp"
#include "rknn_api.h"
#include <string>

namespace adas {
// Unquantized MobileNet128 on the NPU. Internal light IDs retain the
// original application's 7..14 semantics; an unconfirmed housing is ID 21.
class LampClassifier {
 public:
  explicit LampClassifier(const std::string& model_path);
  ~LampClassifier();
  bool ready() const { return context_ != 0 && error_.empty(); }
  const std::string& error() const { return error_; }
  std::vector<Detection> classify(const cv::Mat& bgr, const Detection& housing);
 private:
  bool predict(const cv::Mat& crop, int& category, float& confidence, float& margin);
  rknn_context context_ = 0;
  std::string error_;
};
}
