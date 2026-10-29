#include "adas/ufld_lane_detector.hpp"
#include "adas/rga_image_ops.hpp"
#include "adas/npu_run_guard.hpp"
#include "adas/model_file.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
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

UfldLaneDetector::UfldLaneDetector(const std::string& model_path, bool rga_required)
    : rga_required_(rga_required) {
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
    bool has_loc_col = false;
    bool has_exist_col = false;
    for (const rknn_tensor_attr& attr : output_attrs_) {
      const std::string name = attr.name;
      if (name == "loc_row" && attr.n_elems == 200U * 72U * 4U)
        has_loc_row = true;
      if (name == "exist_row" && attr.n_elems == 2U * 72U * 4U)
        has_exist_row = true;
      if (name == "loc_col" && attr.n_elems == 100U * 81U * 4U)
        has_loc_col = true;
      if (name == "exist_col" && attr.n_elems == 2U * 81U * 4U)
        has_exist_col = true;
    }
    if (!has_loc_row || !has_exist_row || !has_loc_col || !has_exist_col) {
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
  if (ready_ && v2_) {
    rga_resize_pool_.reset(new PcieBgrPool(3, input_width_, 533));
    rga_rgb_pool_.reset(new PcieBgrPool(3, input_width_, input_height_));
  }
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
    if (rga_resize_pool_) resized = rga_resize_pool_->acquire();
    const bool rga_resize_target = !resized.empty();
    if (resized.empty() && rga_required_) return {};
    if (resized.empty()) resized.create(533, input_width_, CV_8UC3);
    bool rga_ok = rga_resize_target && bgr.cols >= input_width_ / 2 && bgr.step % 3 == 0 &&
                  resized.isContinuous();
    if (rga_ok) {
      try {
        rga_image_ops::resize_bgr(bgr, resized);
      } catch (const std::exception& error) {
        rga_ok = false;
        if (rga_required_ && !rga_failure_reported_.exchange(true))
          std::cerr << "UFLD RGA preprocessing failed: " << error.what() << std::endl;
      }
    }
    if (!rga_ok) {
      if (rga_required_) {
        if (!rga_failure_reported_.exchange(true))
          std::cerr << "UFLD RGA preprocessing cannot handle this input size" << std::endl;
        return {};
      }
      cv::resize(bgr, resized, cv::Size(input_width_, 533), 0, 0,
                 cv::INTER_LINEAR);
    }
    const cv::Mat crop = resized.rowRange(resized.rows - input_height_, resized.rows);
    cv::Mat rgb;
    if (rga_rgb_pool_) rgb = rga_rgb_pool_->acquire();
    const bool rga_rgb_target = !rgb.empty();
    if (rgb.empty() && !rga_required_) rgb.create(input_height_, input_width_, CV_8UC3);
    try {
      if (!rga_rgb_target) throw std::runtime_error("RGA UFLD RGB pool is full");
      rga_image_ops::bgr_to_rgb(crop, rgb);
      return rgb;
    } catch (const std::exception& error) {
      if (rga_required_) {
        if (!rga_failure_reported_.exchange(true))
          std::cerr << "UFLD RGA color conversion failed: " << error.what() << std::endl;
        return {};
      }
      cv::Mat copied = crop.clone();
      cv::cvtColor(copied, copied, cv::COLOR_BGR2RGB);
      return copied;
    }
  } else {
    cv::resize(bgr, resized, cv::Size(input_width_, input_height_), 0, 0, cv::INTER_LINEAR);
  }
  cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
  return resized;
}

LaneResult UfldLaneDetector::detect(const cv::Mat& bgr, double* inference_ms) {
  if (!ready_ || bgr.empty() || bgr.type() != CV_8UC3) return LaneResult();
  cv::Mat image = preprocess(bgr);
  if (image.empty()) return LaneResult();
  rknn_input input{};
  input.index = 0;
  input.type = RKNN_TENSOR_UINT8;
  input.fmt = RKNN_TENSOR_NHWC;
  input.size = static_cast<uint32_t>(image.total() * image.elemSize());
  input.buf = image.data;
  NpuRunGuard npu_guard(npu_run_mutex());
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
  npu_guard.unlock();
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
  return make_result({{lane_points(0), lane_points(1),
                       lane_points(2), lane_points(3)}}, width, height);
}

LaneResult UfldLaneDetector::decode_v2(const std::vector<rknn_output>& outputs,
                                       int width, int height) const {
  const float* loc_row = nullptr;
  const float* loc_col = nullptr;
  const float* exist_row = nullptr;
  const float* exist_col = nullptr;
  for (size_t i = 0; i < output_attrs_.size(); ++i) {
    const std::string name = output_attrs_[i].name;
    const float* data = static_cast<const float*>(outputs[i].buf);
    if (name == "loc_row") loc_row = data;
    if (name == "loc_col") loc_col = data;
    if (name == "exist_row") exist_row = data;
    if (name == "exist_col") exist_col = data;
  }
  if (!loc_row || !loc_col || !exist_row || !exist_col) return LaneResult();
  return make_result(decode_ufld_v2_slots(loc_row, loc_col, exist_row,
                                        exist_col, width, height), width, height);
}

LaneResult UfldLaneDetector::make_result(const std::array<Points, 4>& slots,
                                         int width, int height) const {
  LaneResult result;
  for (size_t slot = 0; slot < slots.size(); ++slot) {
    result.boundary_coverage[slot] = slots[slot].confidence;
    for (const cv::Point2f& point : slots[slot].values)
      result.boundaries[slot].emplace_back(cvRound(point.x), cvRound(point.y));
    std::sort(result.boundaries[slot].begin(), result.boundaries[slot].end(),
        [](const cv::Point& a, const cv::Point& b) { return a.y > b.y; });
  }
  const Points& left = slots[1];
  const Points& right = slots[2];
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
    if (rx - lx <= 5) return result;
  }
  const float left_bottom = result.left.front().x;
  const float right_bottom = result.right.back().x;
  const float bottom_width = right_bottom - left_bottom;
  if (bottom_width < width * 0.15f || bottom_width >= width * 0.75f ||
      left_bottom >= width * 0.62f || right_bottom <= width * 0.38f)
    return result;
  result.polygon = result.left;
  result.polygon.insert(result.polygon.end(), result.right.begin(), result.right.end());
  result.offset_ratio = (width * 0.5f - (left_bottom + right_bottom) * 0.5f) /
                        std::max(1.0f, bottom_width);
  result.departure = std::abs(result.offset_ratio) > 0.16f;
  result.valid = true;
  // Official V2 requires > half the row anchors. Sparse curves remain visible
  // for diagnostics but cannot initiate LDW (coverage is not a probability).
  result.partial = v2_ && (left.confidence <= 0.5f || right.confidence <= 0.5f);
  result.boundaries[1] = result.left;
  result.boundaries[2].assign(result.right.rbegin(), result.right.rend());
  result.boundary_coverage[1] = left.confidence;
  result.boundary_coverage[2] = right.confidence;
  result.lane_polygons[1] = result.polygon;

  // Adjacent slots are independent observations. Never extend an outer curve
  // beyond its supported anchors or assume that it describes another lane.
  for (int slot : {0, 3}) {
    const Points& outer = slots[slot];
    // The official column head accepts > 1/4 of its 81 anchors. Its coverage
    // is not comparable to the row head's > 1/2 of 72 anchors.
    if (outer.values.size() < 5 || outer.confidence <= (v2_ ? 0.25f : 0.30f)) continue;
    cv::Vec3d fit;
    if (!fit_quadratic(outer.values, fit) || !refine(outer.values, fit)) continue;
    const auto limits = std::minmax_element(outer.values.begin(), outer.values.end(),
        [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
    const float top = std::max(first_y, limits.first->y);
    const float bottom = std::min(last_y, limits.second->y);
    if (bottom - top < height * 0.15f) continue;
    const cv::Vec3d& inner_fit = slot == 0 ? left_fit : right_fit;
    bool ordered = true;
    for (int i = 0; i < 7; ++i) {
      const double y = top + (bottom - top) * i / 6.0;
      const double outside = curve_x(fit, y);
      const double inside = curve_x(inner_fit, y);
      const double ego_width = curve_x(right_fit, y) - curve_x(left_fit, y);
      const double gap = slot == 0 ? inside - outside : outside - inside;
      if (outside < 0 || outside >= width || ego_width < 10 ||
          gap < std::max(8.0, ego_width * 0.35) || gap > ego_width * 2.2) {
        ordered = false;
        break;
      }
    }
    if (!ordered) continue;
    std::vector<cv::Point> outer_curve, inner_curve;
    for (int i = 0; i < 24; ++i) {
      const double y = bottom - (bottom - top) * i / 23.0;
      outer_curve.emplace_back(cvRound(curve_x(fit, y)), cvRound(y));
      inner_curve.emplace_back(cvRound(curve_x(inner_fit, y)), cvRound(y));
    }
    result.boundaries[slot] = outer_curve;
    result.boundary_coverage[slot] = outer.confidence;
    std::vector<cv::Point>& area = result.lane_polygons[slot == 0 ? 0 : 2];
    if (slot == 0) {
      area = outer_curve;
      area.insert(area.end(), inner_curve.rbegin(), inner_curve.rend());
    } else {
      area = inner_curve;
      area.insert(area.end(), outer_curve.rbegin(), outer_curve.rend());
    }
  }
  return result;
}

}  // namespace adas
