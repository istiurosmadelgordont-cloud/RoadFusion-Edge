#include "adas/lane_detector.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace adas {
namespace {

struct BirdCurves {
  bool left_valid = false;
  bool right_valid = false;
  bool left_inferred = false;
  bool right_inferred = false;
  cv::Vec3d left{};
  cv::Vec3d right{};
};

bool fit_quadratic(const std::vector<cv::Point>& points, cv::Vec3d& coefficients) {
  if (points.size() < 30) return false;
  int min_y = points.front().y;
  int max_y = points.front().y;
  for (const cv::Point& point : points) {
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  if (max_y - min_y < 90) return false;
  cv::Mat design(static_cast<int>(points.size()), 3, CV_64F);
  cv::Mat values(static_cast<int>(points.size()), 1, CV_64F);
  for (size_t i = 0; i < points.size(); ++i) {
    const double y = points[i].y;
    design.at<double>(static_cast<int>(i), 0) = y * y;
    design.at<double>(static_cast<int>(i), 1) = y;
    design.at<double>(static_cast<int>(i), 2) = 1.0;
    values.at<double>(static_cast<int>(i), 0) = points[i].x;
  }
  cv::Mat solved;
  if (!cv::solve(design, values, solved, cv::DECOMP_SVD)) return false;
  coefficients = cv::Vec3d(solved.at<double>(0), solved.at<double>(1), solved.at<double>(2));
  return true;
}

void search_side(const std::vector<cv::Point>& pixels, bool left, int width, int height,
                 cv::Mat& visualization, std::vector<cv::Point>& selected) {
  std::vector<int> histogram(width, 0);
  for (const cv::Point& point : pixels) {
    if (point.y >= height / 2) ++histogram[point.x];
  }
  const int begin = left ? 0 : width / 2;
  const int end = left ? width / 2 : width;
  int current_x = left ? width / 4 : width * 3 / 4;
  for (int x = begin; x < end; ++x) {
    if (histogram[x] > histogram[current_x]) current_x = x;
  }

  const int windows = 9;
  const int window_height = std::max(1, height / windows);
  const int margin = std::max(28, width / 14);
  for (int window = 0; window < windows; ++window) {
    const int y_high = height - window * window_height;
    const int y_low = std::max(0, y_high - window_height);
    const int x_low = std::max(0, current_x - margin);
    const int x_high = std::min(width - 1, current_x + margin);
    cv::rectangle(visualization, cv::Rect(x_low, y_low, std::max(1, x_high - x_low),
                  std::max(1, y_high - y_low)), cv::Scalar(0, 190, 0), 1);
    long x_sum = 0;
    int count = 0;
    for (const cv::Point& point : pixels) {
      if (point.y >= y_low && point.y < y_high && point.x >= x_low && point.x < x_high) {
        selected.push_back(point);
        x_sum += point.x;
        ++count;
      }
    }
    if (count >= 18) current_x = static_cast<int>(x_sum / count);
  }
}

BirdCurves make_curve_fit(const cv::Mat& bird_eye, cv::Mat& visualization) {
  BirdCurves model;
  cv::cvtColor(bird_eye, visualization, cv::COLOR_GRAY2BGR);
  std::vector<cv::Point> pixels;
  cv::findNonZero(bird_eye, pixels);
  std::vector<cv::Point> left_points;
  std::vector<cv::Point> right_points;
  search_side(pixels, true, bird_eye.cols, bird_eye.rows, visualization, left_points);
  search_side(pixels, false, bird_eye.cols, bird_eye.rows, visualization, right_points);
  for (const cv::Point& point : left_points) visualization.at<cv::Vec3b>(point) = cv::Vec3b(255, 60, 30);
  for (const cv::Point& point : right_points) visualization.at<cv::Vec3b>(point) = cv::Vec3b(30, 40, 255);
  cv::Vec3d left_curve, right_curve;
  const bool left_ok = fit_quadratic(left_points, left_curve);
  const bool right_ok = fit_quadratic(right_points, right_curve);
  bool left_inferred = false;
  bool right_inferred = false;
  if (left_ok && !right_ok) {
    right_curve = left_curve;
    right_curve[2] += bird_eye.cols * 0.50;
    right_inferred = true;
  } else if (!left_ok && right_ok) {
    left_curve = right_curve;
    left_curve[2] -= bird_eye.cols * 0.50;
    left_inferred = true;
  }
  model.left_valid = left_ok || left_inferred;
  model.right_valid = right_ok || right_inferred;
  model.left_inferred = left_inferred;
  model.right_inferred = right_inferred;
  model.left = left_curve;
  model.right = right_curve;
  for (int y = 0; y < bird_eye.rows; ++y) {
    if (left_ok || left_inferred) {
      const int x = static_cast<int>(left_curve[0] * y * y + left_curve[1] * y + left_curve[2]);
      if (x >= 0 && x < bird_eye.cols && (!left_inferred || y % 8 < 4))
        cv::circle(visualization, cv::Point(x, y), 2,
                   left_inferred ? cv::Scalar(120, 120, 120) : cv::Scalar(255, 255, 0), cv::FILLED);
    }
    if (right_ok || right_inferred) {
      const int x = static_cast<int>(right_curve[0] * y * y + right_curve[1] * y + right_curve[2]);
      if (x >= 0 && x < bird_eye.cols && (!right_inferred || y % 8 < 4))
        cv::circle(visualization, cv::Point(x, y), 2,
                   right_inferred ? cv::Scalar(120, 120, 120) : cv::Scalar(0, 255, 255), cv::FILLED);
    }
  }
  return model;
}

}  // namespace

LaneDetector::LaneDetector(const LaneConfig& config) : config_(config) {}

void LaneDetector::reset() {
  has_left_ = has_right_ = false;
  has_left_curve_ = has_right_curve_ = false;
  left_missing_ = right_missing_ = 0;
}

void LaneDetector::set_roi(const cv::Point2f& tl, const cv::Point2f& tr,
                           const cv::Point2f& br, const cv::Point2f& bl) {
  config_.top_left = tl;
  config_.top_right = tr;
  config_.bottom_right = br;
  config_.bottom_left = bl;
  reset();
}

bool LaneDetector::fit_side(const std::vector<cv::Vec4i>& lines, bool left, int width, int height,
                            cv::Vec4f& result) const {
  std::vector<cv::Point2f> points;
  for (const cv::Vec4i& line : lines) {
    const float dx = static_cast<float>(line[2] - line[0]);
    const float dy = static_cast<float>(line[3] - line[1]);
    if (std::abs(dx) < 2.0f || std::sqrt(dx * dx + dy * dy) < height * 0.035f) continue;
    const float slope = dy / dx;
    if ((left && slope > -0.40f) || (!left && slope < 0.40f) || std::abs(slope) > 5.0f) continue;
    const float mid_x = 0.5f * (line[0] + line[2]);
    if ((left && mid_x > width * 0.62f) || (!left && mid_x < width * 0.38f)) continue;
    points.emplace_back(static_cast<float>(line[0]), static_cast<float>(line[1]));
    points.emplace_back(static_cast<float>(line[2]), static_cast<float>(line[3]));
  }
  if (points.size() < 4) return false;
  cv::Vec4f fit;
  cv::fitLine(points, fit, cv::DIST_L2, 0, 0.01, 0.01);
  if (std::abs(fit[1]) < 1e-4f) return false;
  const float y_top = height * std::min(config_.top_left.y, config_.top_right.y);
  const float y_bottom = height * std::max(config_.bottom_left.y, config_.bottom_right.y);
  const float x_top = fit[2] + (y_top - fit[3]) * fit[0] / fit[1];
  const float x_bottom = fit[2] + (y_bottom - fit[3]) * fit[0] / fit[1];
  result = cv::Vec4f(x_top, y_top, x_bottom, y_bottom);
  const bool direction_ok = left ? (x_bottom < x_top - width * 0.03f)
                                 : (x_bottom > x_top + width * 0.03f);
  return direction_ok && x_top > width * 0.18f && x_top < width * 0.82f &&
         x_bottom > -width * 0.05f && x_bottom < width * 1.05f;
}

cv::Vec4f LaneDetector::smooth(const cv::Vec4f& current, cv::Vec4f& previous,
                               bool& has_previous) const {
  if (!has_previous) {
    previous = current;
    has_previous = true;
    return current;
  }
  previous = previous * config_.smoothing + current * (1.0f - config_.smoothing);
  return previous;
}

LaneResult LaneDetector::detect(const cv::Mat& frame) {
  LaneResult result;
  if (frame.empty()) return result;
  cv::Mat work;
  const float processing_scale = frame.cols > config_.processing_width
      ? config_.processing_width / static_cast<float>(frame.cols) : 1.0f;
  if (processing_scale < 1.0f) {
    cv::resize(frame, work, cv::Size(config_.processing_width,
               static_cast<int>(std::round(frame.rows * processing_scale))), 0, 0, cv::INTER_AREA);
  } else {
    work = frame;
  }
  const int w = work.cols;
  const int h = work.rows;
  cv::Mat hls, white, yellow, binary, blurred, edges;
  cv::cvtColor(work, hls, cv::COLOR_BGR2HLS);
  cv::inRange(hls, cv::Scalar(0, 145, 0), cv::Scalar(180, 255, 255), white);
  cv::inRange(hls, cv::Scalar(12, 70, 65), cv::Scalar(42, 230, 255), yellow);
  cv::bitwise_or(white, yellow, binary);

  std::vector<cv::Point> roi = {
      cv::Point(static_cast<int>(config_.top_left.x * w), static_cast<int>(config_.top_left.y * h)),
      cv::Point(static_cast<int>(config_.top_right.x * w), static_cast<int>(config_.top_right.y * h)),
      cv::Point(static_cast<int>(config_.bottom_right.x * w), static_cast<int>(config_.bottom_right.y * h)),
      cv::Point(static_cast<int>(config_.bottom_left.x * w), static_cast<int>(config_.bottom_left.y * h))};
  cv::Mat mask = cv::Mat::zeros(binary.size(), CV_8UC1);
  cv::fillConvexPoly(mask, roi, cv::Scalar(255));
  cv::bitwise_and(binary, mask, binary);
  cv::GaussianBlur(binary, blurred, cv::Size(5, 5), 0);
  cv::Canny(blurred, edges, 45, 130);
  cv::bitwise_and(edges, mask, result.binary);

  const cv::Point2f source[] = {
      cv::Point2f(config_.top_left.x * w, config_.top_left.y * h),
      cv::Point2f(config_.top_right.x * w, config_.top_right.y * h),
      cv::Point2f(config_.bottom_right.x * w, config_.bottom_right.y * h),
      cv::Point2f(config_.bottom_left.x * w, config_.bottom_left.y * h)};
  const cv::Point2f target[] = {
      cv::Point2f(w * 0.25f, 0.0f), cv::Point2f(w * 0.75f, 0.0f),
      cv::Point2f(w * 0.75f, static_cast<float>(h - 1)),
      cv::Point2f(w * 0.25f, static_cast<float>(h - 1))};
  const cv::Mat perspective = cv::getPerspectiveTransform(source, target);
  cv::warpPerspective(result.binary, result.bird_eye, perspective, result.binary.size(),
                      cv::INTER_NEAREST, cv::BORDER_CONSTANT);
  BirdCurves curves = make_curve_fit(result.bird_eye, result.curve_fit);
  if (curves.left_valid && curves.right_valid) {
    const double curve_smoothing = 0.64;
    if (has_left_curve_) curves.left = previous_left_curve_ * curve_smoothing + curves.left * (1.0 - curve_smoothing);
    if (has_right_curve_) curves.right = previous_right_curve_ * curve_smoothing + curves.right * (1.0 - curve_smoothing);
    previous_left_curve_ = curves.left;
    previous_right_curve_ = curves.right;
    has_left_curve_ = has_right_curve_ = true;

    std::vector<cv::Point2f> left_bird;
    std::vector<cv::Point2f> right_bird;
    const int curve_samples = 28;
    for (int i = 0; i < curve_samples; ++i) {
      const float y_left = (h - 1) * (1.0f - i / static_cast<float>(curve_samples - 1));
      const float y_right = (h - 1) * (i / static_cast<float>(curve_samples - 1));
      const float left_x = static_cast<float>(curves.left[0] * y_left * y_left + curves.left[1] * y_left + curves.left[2]);
      const float right_x = static_cast<float>(curves.right[0] * y_right * y_right + curves.right[1] * y_right + curves.right[2]);
      left_bird.emplace_back(left_x, y_left);
      right_bird.emplace_back(right_x, y_right);
    }
    const cv::Mat inverse_perspective = cv::getPerspectiveTransform(target, source);
    std::vector<cv::Point2f> left_projected;
    std::vector<cv::Point2f> right_projected;
    cv::perspectiveTransform(left_bird, left_projected, inverse_perspective);
    cv::perspectiveTransform(right_bird, right_projected, inverse_perspective);
    bool projected_ok = left_projected.size() == static_cast<size_t>(curve_samples) &&
                        right_projected.size() == static_cast<size_t>(curve_samples);
    for (int i = 0; projected_ok && i < curve_samples; ++i) {
      projected_ok = std::isfinite(left_projected[i].x) && std::isfinite(left_projected[i].y) &&
                     std::isfinite(right_projected[i].x) && std::isfinite(right_projected[i].y) &&
                     left_projected[i].x > -w * 0.15f && left_projected[i].x < w * 1.15f &&
                     right_projected[i].x > -w * 0.15f && right_projected[i].x < w * 1.15f;
    }
    if (projected_ok) {
      for (const cv::Point2f& point : left_projected)
        result.left.emplace_back(cvRound(point.x), cvRound(point.y));
      for (const cv::Point2f& point : right_projected)
        result.right.emplace_back(cvRound(point.x), cvRound(point.y));
      result.polygon = result.left;
      result.polygon.insert(result.polygon.end(), result.right.begin(), result.right.end());
      const int bottom_width = result.right.back().x - result.left.front().x;
      const int top_width = result.right.front().x - result.left.back().x;
      if (bottom_width > w * 0.22f && bottom_width < w * 1.10f &&
          top_width > w * 0.02f && top_width < bottom_width) {
        result.valid = true;
        result.partial = curves.left_inferred || curves.right_inferred;
        const float lane_center = 0.5f * (result.left.front().x + result.right.back().x);
        result.offset_ratio = (w * 0.5f - lane_center) / std::max(1.0f, static_cast<float>(bottom_width));
        result.departure = std::abs(result.offset_ratio) > config_.departure_ratio;
        if (processing_scale < 1.0f) {
          const float inverse = 1.0f / processing_scale;
          for (cv::Point& point : result.left) { point.x = cvRound(point.x * inverse); point.y = cvRound(point.y * inverse); }
          for (cv::Point& point : result.right) { point.x = cvRound(point.x * inverse); point.y = cvRound(point.y * inverse); }
          for (cv::Point& point : result.polygon) { point.x = cvRound(point.x * inverse); point.y = cvRound(point.y * inverse); }
        }
        return result;
      }
      result.left.clear();
      result.right.clear();
      result.polygon.clear();
    }
  }

  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(result.binary, lines, 1, CV_PI / 180.0, 22, h * 0.045, h * 0.055);
  cv::Vec4f left, right;
  const bool left_ok = fit_side(lines, true, w, h, left);
  const bool right_ok = fit_side(lines, false, w, h, right);
  if (left_ok) {
    left = smooth(left, previous_left_, has_left_);
    left_missing_ = 0;
  } else if (++left_missing_ > config_.missing_hold_updates) {
    has_left_ = false;
  }
  if (right_ok) {
    right = smooth(right, previous_right_, has_right_);
    right_missing_ = 0;
  } else if (++right_missing_ > config_.missing_hold_updates) {
    has_right_ = false;
  }
  if (!left_ok && has_left_) left = previous_left_;
  if (!right_ok && has_right_) right = previous_right_;
  const bool have_left = left_ok || has_left_;
  const bool have_right = right_ok || has_right_;
  if (!have_left && !have_right) return result;
  result.partial = !left_ok || !right_ok;
  if (!have_left) {
    left = cv::Vec4f(config_.top_left.x * w, config_.top_left.y * h,
                     config_.bottom_left.x * w, config_.bottom_left.y * h);
    result.partial = true;
  }
  if (!have_right) {
    right = cv::Vec4f(config_.top_right.x * w, config_.top_right.y * h,
                      config_.bottom_right.x * w, config_.bottom_right.y * h);
    result.partial = true;
  }

  const cv::Point lt(static_cast<int>(left[0]), static_cast<int>(left[1]));
  const cv::Point lb(static_cast<int>(left[2]), static_cast<int>(left[3]));
  const cv::Point rt(static_cast<int>(right[0]), static_cast<int>(right[1]));
  const cv::Point rb(static_cast<int>(right[2]), static_cast<int>(right[3]));
  const int top_width = rt.x - lt.x;
  const int bottom_width = rb.x - lb.x;
  if (bottom_width < w * 0.24f || bottom_width > w * 1.05f ||
      top_width < w * 0.025f || top_width > w * 0.50f ||
      top_width >= bottom_width) return result;
  result.left = {lb, lt};
  result.right = {rt, rb};
  result.polygon = {lb, lt, rt, rb};
  result.valid = true;
  const float lane_center = 0.5f * (lb.x + rb.x);
  result.offset_ratio = (w * 0.5f - lane_center) / std::max(1.0f, static_cast<float>(rb.x - lb.x));
  result.departure = std::abs(result.offset_ratio) > config_.departure_ratio;
  if (processing_scale < 1.0f) {
    const float inverse = 1.0f / processing_scale;
    for (cv::Point& point : result.left) { point.x = static_cast<int>(point.x * inverse); point.y = static_cast<int>(point.y * inverse); }
    for (cv::Point& point : result.right) { point.x = static_cast<int>(point.x * inverse); point.y = static_cast<int>(point.y * inverse); }
    for (cv::Point& point : result.polygon) { point.x = static_cast<int>(point.x * inverse); point.y = static_cast<int>(point.y * inverse); }
  }
  return result;
}

}  // namespace adas
