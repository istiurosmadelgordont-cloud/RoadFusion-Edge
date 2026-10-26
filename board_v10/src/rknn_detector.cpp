#include "adas/rknn_detector.hpp"
#include "adas/npu_run_guard.hpp"
#include "adas/model_file.hpp"
#include "adas/lamp_classifier.hpp"

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
  const auto& input_attr = input_attrs_[0];
  if (input_attr.n_dims != 4) { fail("Expected a four-dimensional input"); return; }
  input_width_ = input_attr.fmt == RKNN_TENSOR_NHWC ? input_attr.dims[2] : input_attr.dims[3];
  input_height_ = input_attr.fmt == RKNN_TENSOR_NHWC ? input_attr.dims[1] : input_attr.dims[2];
  // 按 RKNN 张量契约识别新双阶段模型：960×544、9 个输出、14 个定位类别。
  // 历史 21 类 P2 模型仍走旧解码分支，不能混用它的类别 ID。
  two_stage_ = io_num_.n_output == 9 && input_width_ == 960 && input_height_ == 544;
  config_.class_count = two_stage_ ? 14 : 21;
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
  if ((!two_stage_ && (io_num_.n_output != 12 || class_score_outputs != 4)) ||
      (two_stage_ && class_score_outputs != 3)) {
    fail("Expected legacy P2/21 or 960x544 P3/14 output tensors");
    return;
  }
  if (two_stage_) {
    const auto slash=config_.model_path.find_last_of('/');
    const std::string directory=slash==std::string::npos ? "." : config_.model_path.substr(0,slash);
    lamp_classifier_.reset(new LampClassifier(directory+"/mo_fp16_baseline.rknn"));
    if (!lamp_classifier_->ready()) { fail(lamp_classifier_->error()); return; }
    std::cout<<"YOLO14 960x544 INT8 + MobileNet128 FP16 NPU"<<std::endl;
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
  meta.scale = std::min(static_cast<float>(input_width_) / bgr.cols,
                        static_cast<float>(input_height_) / bgr.rows);
  const int resized_w = static_cast<int>(std::round(bgr.cols * meta.scale));
  const int resized_h = static_cast<int>(std::round(bgr.rows * meta.scale));
  meta.pad_x = (input_width_ - resized_w) / 2;
  meta.pad_y = (input_height_ - resized_h) / 2;
  cv::Mat resized, canvas(input_height_, input_width_, CV_8UC3, cv::Scalar(114, 114, 114));
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
  NpuRunGuard npu_guard(npu_run_mutex());
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
  // Complete one YOLO + MO observation before another context uses the NPU.
  // Releasing here let a 200ms UFLD job enter before the lamp crops, consuming
  // the 500ms result deadline and dropping an otherwise valid observation.
  // A single-stage detector still releases before its CPU-only postprocess.
  if (!two_stage_) npu_guard.unlock();
  if (inference_ms) *inference_ms = std::chrono::duration<double, std::milli>(stop - start).count();
  std::vector<Detection> result = postprocess(outputs, meta);
  rknn_outputs_release(context_, io_num_.n_output, outputs.data());
  if (two_stage_) {
    std::vector<Detection> mapped;
    int classified_housings=0;
    static const char* names[]={"行人","骑行者","汽车","公交车","卡车","摩托车","自行车",
        "交通灯","交通标志","人行横道","地面导向箭头","交通锥","施工区域","道路导向柱"};
    for (Detection detection : result) {
      const int native=detection.class_id;
      // YOLO 原生交通灯 ID=7 只表示灯组定位，映射到内部 ID=21。
      // MO 的 8 个红绿灯类别占内部 ID=7..14；其余道路类别顺延到 15..20。
      detection.class_id=native<7 ? native : native==7 ? 21 : native+7;
      detection.name=names[native];
      if (native==7) {
        // 每次 YOLO 结果最多处理 8 个灯组，限制 MO 裁剪调用总量。
        // 超出预算的框标为“待识别”，不伪装成已确认的灯态。
        // classify() 从 housing 复制 Detection，子灯因此保留来源。
        // 此编号经四视图裁剪保留，进入跟踪器后再绑定持久的灯组 ID。
        detection.source_housing_index=classified_housings++;
        const bool classified=detection.source_housing_index<8;
        auto lamps=classified ? lamp_classifier_->classify(bgr,detection) : std::vector<Detection>{};
        detection.name=lamps.empty() ? (classified ? "交通灯（MO未确认）" : "交通灯（待识别）") : "交通灯";
        mapped.push_back(detection);
        mapped.insert(mapped.end(),lamps.begin(),lamps.end());
      } else { mapped.push_back(detection); }
    }
    // 相邻灯组框可能重复包含同一亮灯；灯态框按 IoU>0.45 跨类别去重。
    // 先保留置信度高的结果，避免同一灯泡重复画框或同时输出不同方向。
    std::stable_sort(mapped.begin(),mapped.end(),[](const Detection& a,const Detection& b){return a.score>b.score;});
    std::vector<Detection> unique;
    for (const auto& candidate : mapped) {
      bool duplicate=false;
      if (candidate.class_id>=7 && candidate.class_id<=14) {
        for (const auto& selected : unique) {
          if (selected.class_id>=7 && selected.class_id<=14 && iou(candidate,selected)>.45f) {duplicate=true;break;}
        }
      }
      if (!duplicate) unique.push_back(candidate);
    }
    result.swap(unique);
  }
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
    const float stride_x = static_cast<float>(input_width_) / box.w;
    const float stride_y = static_cast<float>(input_height_) / box.h;
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
