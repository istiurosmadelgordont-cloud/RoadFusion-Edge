#include "adas/rknn_detector.hpp"
#include "adas/model_file.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <numeric>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace adas {
namespace {

struct TensorView {
  const float* data = nullptr;
  int c = 0;
  int h = 0;
  int w = 0;
  bool nchw = true;

  float at(int channel, int y, int x) const {
    if (nchw) return data[(channel * h + y) * w + x];
    return data[(y * w + x) * c + channel];
  }
};

TensorView view_for(const rknn_tensor_attr& attr, const rknn_output& output) {
  TensorView view;
  view.data = static_cast<const float*>(output.buf);
  if (attr.n_dims != 4) return view;
  view.nchw = attr.fmt != RKNN_TENSOR_NHWC;
  if (view.nchw) {
    view.c = static_cast<int>(attr.dims[1]);
    view.h = static_cast<int>(attr.dims[2]);
    view.w = static_cast<int>(attr.dims[3]);
  } else {
    view.h = static_cast<int>(attr.dims[1]);
    view.w = static_cast<int>(attr.dims[2]);
    view.c = static_cast<int>(attr.dims[3]);
  }
  return view;
}

float iou(const Detection& a, const Detection& b) {
  const float x1 = std::max(a.box.x, b.box.x);
  const float y1 = std::max(a.box.y, b.box.y);
  const float x2 = std::min(a.box.x + a.box.width, b.box.x + b.box.width);
  const float y2 = std::min(a.box.y + a.box.height, b.box.y + b.box.height);
  const float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
  const float total = a.box.area() + b.box.area() - inter;
  return total > 0.0f ? inter / total : 0.0f;
}

float dfl_value(const TensorView& box, int side, int bins, int y, int x) {
  float max_value = -1e30f;
  for (int k = 0; k < bins; ++k) max_value = std::max(max_value, box.at(side * bins + k, y, x));
  float sum = 0.0f;
  float weighted = 0.0f;
  for (int k = 0; k < bins; ++k) {
    const float e = std::exp(box.at(side * bins + k, y, x) - max_value);
    sum += e;
    weighted += e * static_cast<float>(k);
  }
  return sum > 0.0f ? weighted / sum : 0.0f;
}

}  // namespace

RknnDetector::RknnDetector(const DetectorConfig& config) : config_(config) {
  std::string model_error;
  std::vector<unsigned char> model = read_model_file(config_.model_path, &model_error);
  if (model.empty()) {
    fail("cannot read RKNN model: " + model_error);
    return;
  }

  int ret = rknn_init(&context_, model.data(), static_cast<uint32_t>(model.size()), 0, nullptr);
  if (ret != RKNN_SUCC) {
    fail("rknn_init failed: " + std::to_string(ret));
    return;
  }

  rknn_sdk_version version{};
  if (rknn_query(context_, RKNN_QUERY_SDK_VERSION, &version, sizeof(version)) == RKNN_SUCC) {
    runtime_version_ = std::string(version.api_version) + " / driver " + version.drv_version;
  }
  ret = rknn_query(context_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
  if (ret != RKNN_SUCC || io_num_.n_input != 1) {
    fail("unexpected RKNN input/output metadata");
    return;
  }

  input_attrs_.resize(io_num_.n_input);
  for (uint32_t i = 0; i < io_num_.n_input; ++i) {
    std::memset(&input_attrs_[i], 0, sizeof(rknn_tensor_attr));
    input_attrs_[i].index = i;
    if (rknn_query(context_, RKNN_QUERY_INPUT_ATTR, &input_attrs_[i], sizeof(rknn_tensor_attr)) != RKNN_SUCC) {
      fail("failed to query RKNN input tensor");
      return;
    }
  }
  output_attrs_.resize(io_num_.n_output);
  int class_score_outputs = 0;
  for (uint32_t i = 0; i < io_num_.n_output; ++i) {
    std::memset(&output_attrs_[i], 0, sizeof(rknn_tensor_attr));
    output_attrs_[i].index = i;
    if (rknn_query(context_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i], sizeof(rknn_tensor_attr)) != RKNN_SUCC) {
      fail("failed to query RKNN output tensor");
      return;
    }
    const TensorView v = view_for(output_attrs_[i], rknn_output{});
    std::cout << "RKNN output " << i << ": c=" << v.c << " h=" << v.h << " w=" << v.w << std::endl;
    if (v.c == config_.class_count) ++class_score_outputs;
  }
  if (io_num_.n_output != 12 || class_score_outputs != 4) {
    fail("model output shape does not match the V7 P2 21-class detector");
    return;
  }
  ready_ = true;
}

RknnDetector::~RknnDetector() {
  if (context_) rknn_destroy(context_);
}

void RknnDetector::fail(const std::string& message) {
  error_ = message;
  ready_ = false;
  if (context_) {
    rknn_destroy(context_);
    context_ = 0;
  }
}

cv::Mat RknnDetector::preprocess(const cv::Mat& bgr, Letterbox& meta) const {
  meta.source_w = bgr.cols;
  meta.source_h = bgr.rows;
  meta.scale = std::min(static_cast<float>(config_.input_size) / bgr.cols,
                        static_cast<float>(config_.input_size) / bgr.rows);
  const int resized_w = static_cast<int>(std::round(bgr.cols * meta.scale));
  const int resized_h = static_cast<int>(std::round(bgr.rows * meta.scale));
  meta.pad_x = (config_.input_size - resized_w) / 2;
  meta.pad_y = (config_.input_size - resized_h) / 2;
  cv::Mat resized, canvas(config_.input_size, config_.input_size, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::resize(bgr, resized, cv::Size(resized_w, resized_h), 0, 0, cv::INTER_LINEAR);
  resized.copyTo(canvas(cv::Rect(meta.pad_x, meta.pad_y, resized_w, resized_h)));
  cv::cvtColor(canvas, canvas, cv::COLOR_BGR2RGB);
  return canvas;
}

std::vector<Detection> RknnDetector::detect(const cv::Mat& bgr, double* inference_ms) {
  if (!ready_ || bgr.empty() || bgr.type() != CV_8UC3) return {};
  Letterbox meta;
  cv::Mat input_image = preprocess(bgr, meta);
  rknn_input input{};
  input.index = 0;
  input.type = RKNN_TENSOR_UINT8;
  input.fmt = RKNN_TENSOR_NHWC;
  input.size = static_cast<uint32_t>(input_image.total() * input_image.elemSize());
  input.buf = input_image.data;
  input.pass_through = 0;
  if (rknn_inputs_set(context_, 1, &input) != RKNN_SUCC) return {};

  const auto start = std::chrono::steady_clock::now();
  if (rknn_run(context_, nullptr) != RKNN_SUCC) return {};
  std::vector<rknn_output> outputs(io_num_.n_output);
  for (uint32_t i = 0; i < io_num_.n_output; ++i) {
    std::memset(&outputs[i], 0, sizeof(rknn_output));
    outputs[i].index = i;
    outputs[i].want_float = 1;
  }
  if (rknn_outputs_get(context_, io_num_.n_output, outputs.data(), nullptr) != RKNN_SUCC) return {};
  for (const rknn_output& output : outputs) {
    if (!output.buf) {
      rknn_outputs_release(context_, io_num_.n_output, outputs.data());
      return {};
    }
  }
  const auto stop = std::chrono::steady_clock::now();
  if (inference_ms) *inference_ms = std::chrono::duration<double, std::milli>(stop - start).count();
  std::vector<Detection> result = postprocess(outputs, meta);
  rknn_outputs_release(context_, io_num_.n_output, outputs.data());
  return result;
}

std::vector<Detection> RknnDetector::postprocess(const std::vector<rknn_output>& outputs,
                                                 const Letterbox& meta) const {
  std::vector<TensorView> views;
  for (size_t i = 0; i < outputs.size(); ++i) views.push_back(view_for(output_attrs_[i], outputs[i]));
  std::vector<Detection> candidates;

  for (size_t bi = 0; bi < views.size(); ++bi) {
    const TensorView& box = views[bi];
    if (box.c < 8 || box.c % 4 != 0 || box.h <= 0 || box.w <= 0) continue;
    int score_index = -1;
    for (size_t si = 0; si < views.size(); ++si) {
      if (views[si].c == config_.class_count && views[si].h == box.h && views[si].w == box.w) {
        score_index = static_cast<int>(si);
        break;
      }
    }
    if (score_index < 0) continue;
    const TensorView& score = views[score_index];
    const int bins = box.c / 4;
    const float stride_x = static_cast<float>(config_.input_size) / box.w;
    const float stride_y = static_cast<float>(config_.input_size) / box.h;
    for (int y = 0; y < box.h; ++y) {
      for (int x = 0; x < box.w; ++x) {
        int best_class = -1;
        float best_score = config_.confidence;
        for (int c = 0; c < config_.class_count; ++c) {
          const float value = score.at(c, y, x);
          if (value > best_score) {
            best_score = value;
            best_class = c;
          }
        }
        if (best_class < 0) continue;
        const float left = dfl_value(box, 0, bins, y, x);
        const float top = dfl_value(box, 1, bins, y, x);
        const float right = dfl_value(box, 2, bins, y, x);
        const float bottom = dfl_value(box, 3, bins, y, x);
        float x1 = ((x + 0.5f - left) * stride_x - meta.pad_x) / meta.scale;
        float y1 = ((y + 0.5f - top) * stride_y - meta.pad_y) / meta.scale;
        float x2 = ((x + 0.5f + right) * stride_x - meta.pad_x) / meta.scale;
        float y2 = ((y + 0.5f + bottom) * stride_y - meta.pad_y) / meta.scale;
        x1 = std::max(0.0f, std::min(x1, static_cast<float>(meta.source_w - 1)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(meta.source_h - 1)));
        x2 = std::max(0.0f, std::min(x2, static_cast<float>(meta.source_w - 1)));
        y2 = std::max(0.0f, std::min(y2, static_cast<float>(meta.source_h - 1)));
        if (x2 <= x1 || y2 <= y1) continue;
        Detection detection;
        detection.class_id = best_class;
        detection.score = best_score;
        detection.box = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
        detection.name = class_name(best_class);
        candidates.push_back(detection);
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const Detection& a, const Detection& b) {
    return a.score > b.score;
  });
  std::vector<Detection> kept;
  for (const Detection& candidate : candidates) {
    bool suppressed = false;
    for (const Detection& selected : kept) {
      if (candidate.class_id == selected.class_id && iou(candidate, selected) > config_.nms) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) kept.push_back(candidate);
    if (kept.size() >= 100) break;
  }
  return kept;
}

}  // namespace adas
