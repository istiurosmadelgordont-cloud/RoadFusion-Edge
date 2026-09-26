#include "adas/ufld_lane_detector.hpp"
#include "adas/model_file.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

#include <opencv2/imgproc.hpp>
#include <rga/im2d.h>
#include <rga/rga.h>

namespace adas {
namespace {

inline size_t index4(int a, int b, int c, int d,
                     int dim_b, int dim_c, int dim_d) {
  return ((static_cast<size_t>(a) * dim_b + b) * dim_c + c) * dim_d + d;
}

bool fit_quadratic(const std::vector<cv::Point2f>& points, cv::Vec3d& fit) {
  if (points.size() < 5) return false;
  cv::Mat design(static_cast<int>(points.size()), 3, CV_64F);
  cv::Mat values(static_cast<int>(points.size()), 1, CV_64F);
  for (size_t i = 0; i < points.size(); ++i) {
    const double y = points[i].y;
    design.at<double>(static_cast<int>(i), 0) = y * y;
    design.at<double>(static_cast<int>(i), 1) = y;
    design.at<double>(static_cast<int>(i), 2) = 1.0;
    values.at<double>(static_cast<int>(i), 0) = points[i].x;
  }
  cv::Mat solution;
  if (!cv::solve(design, values, solution, cv::DECOMP_SVD)) return false;
  fit = cv::Vec3d(solution.at<double>(0), solution.at<double>(1),
                  solution.at<double>(2));
  return true;
}

inline double curve_x(const cv::Vec3d& fit, double y) {
  return fit[0] * y * y + fit[1] * y + fit[2];
}

float stable_softmax_expectation(const float* values, int count, int center,
                                 int radius, float index_bias) {
  const int low = std::max(0, center - radius);
  const int high = std::min(count - 1, center + radius);
  float maximum = -std::numeric_limits<float>::infinity();
  for (int i = low; i <= high; ++i) maximum = std::max(maximum, values[i]);
  float sum = 0.0f, weighted = 0.0f;
  for (int i = low; i <= high; ++i) {
    const float probability = std::exp(values[i] - maximum);
    sum += probability;
    weighted += probability * (i + index_bias);
  }
  return sum > 0.0f ? weighted / sum : 0.0f;
}

}  // namespace

UfldLaneDetector::UfldLaneDetector(const std::string& model_path) {
  std::string model_error;
  const std::vector<unsigned char> model = read_model_file(model_path, &model_error);
  if (model.empty()) {
    fail("cannot read UFLD RKNN model: " + model_error);
    return;
  }
  if (rknn_init(&context_, const_cast<unsigned char*>(model.data()),
                static_cast<uint32_t>(model.size()), 0, nullptr) != RKNN_SUCC) {
    fail("UFLD rknn_init failed");
    return;
  }
  if (rknn_query(context_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_)) != RKNN_SUCC ||
      io_num_.n_input != 1 || (io_num_.n_output != 1 && io_num_.n_output != 4)) {
    fail("unsupported UFLD tensor count");
    return;
  }
  input_attr_.index = 0;
  if (rknn_query(context_, RKNN_QUERY_INPUT_ATTR, &input_attr_, sizeof(input_attr_)) != RKNN_SUCC) {
    fail("cannot query UFLD input");
    return;
  }
  if (input_attr_.n_dims != 4) {
    fail("UFLD input tensor must have four dimensions");
    return;
  }
  if (input_attr_.fmt == RKNN_TENSOR_NHWC) {
    input_height_ = static_cast<int>(input_attr_.dims[1]);
    input_width_ = static_cast<int>(input_attr_.dims[2]);
  } else {
    input_height_ = static_cast<int>(input_attr_.dims[2]);
    input_width_ = static_cast<int>(input_attr_.dims[3]);
  }
  output_attrs_.resize(io_num_.n_output);
  for (uint32_t i = 0; i < io_num_.n_output; ++i) {
    output_attrs_[i].index = i;
    if (rknn_query(context_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i],
                   sizeof(rknn_tensor_attr)) != RKNN_SUCC) {
      fail("cannot query UFLD output");
      return;
    }
  }
  v2_ = io_num_.n_output == 4;
  if (v2_) {
    bool has_loc_row = false;
    bool has_exist_row = false;
    for (const rknn_tensor_attr& attr : output_attrs_) {
      const std::string name = attr.name;
      if (name == "loc_row" && attr.n_elems >= 200U * 72U * 4U)
        has_loc_row = true;
      if (name == "exist_row" && attr.n_elems >= 2U * 72U * 4U)
        has_exist_row = true;
    }
    if (!has_loc_row || !has_exist_row) {
      fail("UFLDv2 output tensor schema is incompatible");
      return;
    }
  } else if (output_attrs_[0].n_elems < 201U * 18U * 4U) {
    fail("UFLD V1 output tensor schema is incompatible");
    return;
  }
  if (v2_) {
    profile_ = "UFLDv2 CULane " + std::to_string(input_width_) + "x" +
               std::to_string(input_height_);
  } else {
    profile_ = "UFLD V1 CULane 800x288";
  }
  ready_ = (v2_ && (input_width_ == 800 || input_width_ == 1600) &&
            input_height_ == 320) ||
           (!v2_ && input_width_ == 800 && input_height_ == 288);
  if (!ready_) fail("unexpected UFLD input dimensions");
}

UfldLaneDetector::~UfldLaneDetector() {
  if (context_) rknn_destroy(context_);
}

void UfldLaneDetector::fail(const std::string& message) {
  error_ = message;
  ready_ = false;
  if (context_) {
    rknn_destroy(context_);
    context_ = 0;
  }
}

cv::Mat UfldLaneDetector::preprocess(const cv::Mat& bgr) const {
  cv::Mat resized;
  if (v2_) {
    resized.create(533, input_width_, CV_8UC3);
    // RK3568 RGA rejects the 2.5x upscale used by the 640-wide simulation.
    // Native FPGA quadrants are 960 wide (1.67x), which RGA can accelerate.
    bool rga_ok = bgr.cols >= input_width_ / 2 && bgr.isContinuous() &&
                  resized.isContinuous();
    if (rga_ok) {
      const rga_buffer_t source = wrapbuffer_virtualaddr(
          const_cast<unsigned char*>(bgr.data), bgr.cols, bgr.rows,
          RK_FORMAT_BGR_888);
      const rga_buffer_t target = wrapbuffer_virtualaddr(
          resized.data, resized.cols, resized.rows, RK_FORMAT_BGR_888);
      rga_ok = imresize(source, target) == IM_STATUS_SUCCESS;
    }
    if (!rga_ok) {
      cv::resize(bgr, resized, cv::Size(input_width_, 533), 0, 0,
                 cv::INTER_LINEAR);
    }
    resized = resized.rowRange(resized.rows - input_height_, resized.rows).clone();
  } else {
    cv::resize(bgr, resized, cv::Size(input_width_, input_height_), 0, 0, cv::INTER_LINEAR);
  }
  cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
  return resized;
}

LaneResult UfldLaneDetector::detect(const cv::Mat& bgr, double* inference_ms) {
  if (!ready_ || bgr.empty() || bgr.type() != CV_8UC3) return LaneResult();
  cv::Mat image = preprocess(bgr);
  rknn_input input{};
  input.index = 0;
  input.type = RKNN_TENSOR_UINT8;
  input.fmt = RKNN_TENSOR_NHWC;
  input.size = static_cast<uint32_t>(image.total() * image.elemSize());
  input.buf = image.data;
  if (rknn_inputs_set(context_, 1, &input) != RKNN_SUCC) return LaneResult();
  const auto started = std::chrono::steady_clock::now();
  if (rknn_run(context_, nullptr) != RKNN_SUCC) return LaneResult();
  std::vector<rknn_output> outputs(io_num_.n_output);
  for (uint32_t i = 0; i < io_num_.n_output; ++i) {
    outputs[i].index = i;
    outputs[i].want_float = 1;
  }
  if (rknn_outputs_get(context_, io_num_.n_output, outputs.data(), nullptr) != RKNN_SUCC)
    return LaneResult();
  for (const rknn_output& output : outputs) {
    if (!output.buf) {
      rknn_outputs_release(context_, io_num_.n_output, outputs.data());
      return LaneResult();
    }
  }
  if (inference_ms) {
    *inference_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
  }
  LaneResult result = v2_ ? decode_v2(outputs, bgr.cols, bgr.rows)
                          : decode_v1(outputs, bgr.cols, bgr.rows);
  rknn_outputs_release(context_, io_num_.n_output, outputs.data());
  return result;
}

LaneResult UfldLaneDetector::decode_v1(const std::vector<rknn_output>& outputs,
                                       int width, int height) const {
  static const int anchors[18] = {121, 131, 141, 150, 160, 170, 180, 189, 199,
                                  209, 219, 228, 238, 248, 258, 267, 277, 287};
  const float* output = static_cast<const float*>(outputs[0].buf);
  auto lane_points = [&](int lane) {
    Points decoded;
    for (int row = 0; row < 18; ++row) {
      const int source_row = 17 - row;
      std::vector<float> logits(201);
      int best = 0;
      for (int grid = 0; grid < 201; ++grid) {
        logits[grid] = output[index4(0, grid, source_row, lane, 201, 18, 4)];
        if (logits[grid] > logits[best]) best = grid;
      }
      if (best == 200) continue;
      const float position = stable_softmax_expectation(logits.data(), 200, best, 199, 1.0f);
      const float x = (position * (799.0f / 199.0f) - 1.0f) * width / 800.0f;
      const float y = anchors[source_row] * height / 288.0f;
      decoded.values.emplace_back(x, y);
    }
    decoded.confidence = decoded.values.size() / 18.0f;
    return decoded;
  };
  return make_result(lane_points(1), lane_points(2), width, height);
}

LaneResult UfldLaneDetector::decode_v2(const std::vector<rknn_output>& outputs,
                                       int width, int height) const {
  int loc_index = -1, exist_index = -1;
  for (size_t i = 0; i < output_attrs_.size(); ++i) {
    const std::string name = output_attrs_[i].name;
    if (name == "loc_row") loc_index = static_cast<int>(i);
    if (name == "exist_row") exist_index = static_cast<int>(i);
  }
  if (loc_index < 0 || exist_index < 0) return LaneResult();
  const float* loc = static_cast<const float*>(outputs[loc_index].buf);
  const float* exists = static_cast<const float*>(outputs[exist_index].buf);
  auto lane_points = [&](int lane) {
    Points decoded;
    for (int row = 0; row < 72; ++row) {
      const float absent = exists[index4(0, 0, row, lane, 2, 72, 4)];
      const float present = exists[index4(0, 1, row, lane, 2, 72, 4)];
      if (present <= absent) continue;
      std::vector<float> logits(200);
      int best = 0;
      for (int grid = 0; grid < 200; ++grid) {
        logits[grid] = loc[index4(0, grid, row, lane, 200, 72, 4)];
        if (logits[grid] > logits[best]) best = grid;
      }
      const float position = stable_softmax_expectation(logits.data(), 200, best, 1, 0.5f);
      const float x = position / 199.0f * width;
      const float y = (0.42f + 0.58f * row / 71.0f) * height;
      decoded.values.emplace_back(x, y);
    }
    decoded.confidence = decoded.values.size() / 72.0f;
    return decoded;
  };
  return make_result(lane_points(1), lane_points(2), width, height);
}

LaneResult UfldLaneDetector::make_result(const Points& left, const Points& right,
                                         int width, int height) const {
  LaneResult result;
  if (left.values.size() < 5 || right.values.size() < 5) return result;
  float first_y = std::max(
      std::min_element(left.values.begin(), left.values.end(),
          [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; })->y,
      std::min_element(right.values.begin(), right.values.end(),
          [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; })->y);
  float last_y = std::min(
      std::max_element(left.values.begin(), left.values.end(),
          [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; })->y,
      std::max_element(right.values.begin(), right.values.end(),
          [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; })->y);
  last_y = std::min(last_y, height * 0.98f);
  if (last_y - first_y < height * 0.16f) return result;
  cv::Vec3d left_fit, right_fit;
  if (!fit_quadratic(left.values, left_fit) || !fit_quadratic(right.values, right_fit)) return result;
  // Fit only spatially consistent anchors; do not add temporal lag here.
  const auto refine = [width](const std::vector<cv::Point2f>& points, cv::Vec3d& fit) {
    std::vector<double> residuals;
    for (const auto& p : points) residuals.push_back(std::abs(p.x - curve_x(fit, p.y)));
    std::sort(residuals.begin(), residuals.end());
    const double limit = std::max(width * 0.008, 2.5 * residuals[residuals.size() / 2]);
    std::vector<cv::Point2f> inliers;
    for (const auto& p : points)
      if (std::abs(p.x - curve_x(fit, p.y)) <= limit) inliers.push_back(p);
    if (inliers.size() < 5 || inliers.size() * 2 < points.size()) return false;
    return fit_quadratic(inliers, fit);
  };
  if (!refine(left.values, left_fit) || !refine(right.values, right_fit)) return result;
  for (int i = 0; i < 32; ++i) {
    const float y = last_y - (last_y - first_y) * i / 31.0f;
    const int lx = cvRound(std::max(0.0, std::min(curve_x(left_fit, y),
                                                  static_cast<double>(width - 1))));
    result.left.emplace_back(lx, cvRound(y));
  }
  for (int i = 0; i < 32; ++i) {
    const float y = first_y + (last_y - first_y) * i / 31.0f;
    const int rx = cvRound(std::max(0.0, std::min(curve_x(right_fit, y),
                                                  static_cast<double>(width - 1))));
    result.right.emplace_back(rx, cvRound(y));
  }
  for (int i = 0; i < 32; ++i) {
    const int lx = result.left[31 - i].x;
    const int rx = result.right[i].x;
    if (rx - lx <= 5) return LaneResult();
  }
  const float left_bottom = result.left.front().x;
  const float right_bottom = result.right.back().x;
  const float bottom_width = right_bottom - left_bottom;
  if (bottom_width < width * 0.15f || bottom_width >= width * 0.75f ||
      left_bottom >= width * 0.62f || right_bottom <= width * 0.38f)
    return LaneResult();
  result.polygon = result.left;
  result.polygon.insert(result.polygon.end(), result.right.begin(), result.right.end());
  result.offset_ratio = (width * 0.5f - (left_bottom + right_bottom) * 0.5f) /
                        std::max(1.0f, bottom_width);
  result.departure = std::abs(result.offset_ratio) > 0.16f;
  result.valid = true;
  // Official V2 requires > half the row anchors. Sparse curves remain visible
  // for diagnostics but cannot initiate LDW (coverage is not a probability).
  result.partial = v2_ && (left.confidence <= 0.5f || right.confidence <= 0.5f);
  return result;
}

}  // namespace adas
