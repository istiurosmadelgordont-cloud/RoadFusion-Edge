#include "adas/lane_detector.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace adas {
namespace {

double curve_x(const cv::Vec3d& fit, double y) {
  return fit[0] * y * y + fit[1] * y + fit[2];
}

float percentile(std::vector<float> values, float q) {
  if (values.empty()) return 0.0f;
  const size_t index = std::min(values.size() - 1,
      static_cast<size_t>(q * static_cast<float>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}

cv::Mat width_feature_mask(const cv::Mat& frame) {
  cv::Mat hls, light, blurred;
  cv::cvtColor(frame, hls, cv::COLOR_BGR2HLS);
  cv::extractChannel(hls, light, 1);
  cv::GaussianBlur(light, blurred, cv::Size(3, 3), 0);
  cv::Mat source;
  blurred.convertTo(source, CV_32F);
  cv::Mat best = cv::Mat::zeros(source.size(), CV_32F);
  const int widths[] = {3, 5, 7, 9, 11};
  for (int stripe : widths) {
    const int flank = std::max(2, stripe);
    cv::Mat kernel(1, flank * 2 + stripe, CV_32F);
    for (int x = 0; x < kernel.cols; ++x) {
      kernel.at<float>(0, x) = x >= flank && x < flank + stripe
          ? 1.0f / stripe : -0.5f / flank;
    }
    cv::Mat response;
    cv::filter2D(source, response, CV_32F, kernel, cv::Point(-1, -1), 0,
                 cv::BORDER_REPLICATE);
    cv::max(best, response, best);
  }
  std::vector<float> positive;
  positive.reserve(best.total() / 6);
  for (int y = 0; y < best.rows; y += 2) {
    const float* row = best.ptr<float>(y);
    for (int x = 0; x < best.cols; x += 2)
      if (row[x] > 0.0f) positive.push_back(row[x]);
  }
  const float threshold = std::max(7.0f, std::min(22.0f, percentile(positive, 0.68f)));
  cv::Mat mask;
  cv::compare(best, threshold, mask, cv::CMP_GE);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 3)));
  return mask;
}

void scale_points(std::vector<cv::Point>& points, float inverse) {
  if (std::abs(inverse - 1.0f) < 1e-5f) return;
  for (cv::Point& point : points) {
    point.x = cvRound(point.x * inverse);
    point.y = cvRound(point.y * inverse);
  }
}

}  // namespace

LaneDetector::LaneDetector(const LaneConfig& config) : config_(config) {}

void LaneDetector::reset() {
  has_curves_ = false;
  missed_ = 0;
  has_road_light_ = false;
  has_direct_ = false;
  direct_missed_ = 0;
}

void LaneDetector::set_roi(const cv::Point2f& tl, const cv::Point2f& tr,
                           const cv::Point2f& br, const cv::Point2f& bl) {
  config_.top_left = tl;
  config_.top_right = tr;
  config_.bottom_right = br;
  config_.bottom_left = bl;
  reset();
}

cv::Mat LaneDetector::threshold_lane_paint(const cv::Mat& frame) {
  cv::Mat gray, equalized, blurred, hls, hsv;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
  clahe->apply(gray, equalized);
  cv::GaussianBlur(equalized, blurred, cv::Size(5, 5), 0);
  cv::cvtColor(frame, hls, cv::COLOR_BGR2HLS);
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  cv::Mat light;
  cv::extractChannel(hls, light, 1);
  std::vector<unsigned char> samples;
  samples.reserve(light.total() / 16);
  for (int y = 0; y < light.rows; y += 4) {
    const unsigned char* row = light.ptr<unsigned char>(y);
    for (int x = 0; x < light.cols; x += 4)
      if (row[x] > 15) samples.push_back(row[x]);
  }
  float measured = 0.0f;
  if (!samples.empty()) {
    const size_t middle = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + middle, samples.end());
    measured = samples[middle];
  }
  if (!has_road_light_) {
    road_light_ = measured;
    has_road_light_ = true;
  } else {
    road_light_ = road_light_ * 0.85f + measured * 0.15f;
  }
  const int white_min = cvRound(std::max(55.0f, std::min(180.0f, road_light_ + 20.0f)));
  const int yellow_min = cvRound(std::max(45.0f, std::min(85.0f, road_light_ * 0.65f)));

  cv::Mat white, yellow;
  cv::inRange(hls, cv::Scalar(0, white_min, 0), cv::Scalar(180, 255, 150), white);
  cv::inRange(hsv, cv::Scalar(12, 65, yellow_min), cv::Scalar(42, 255, 255), yellow);
  cv::Mat width_mask = width_feature_mask(frame);
  cv::Mat nearby_width;
  cv::dilate(width_mask, nearby_width, cv::Mat::ones(3, 3, CV_8U));
  cv::bitwise_and(white, nearby_width, white);

  cv::Mat sobel, magnitude, gradient;
  cv::Sobel(blurred, sobel, CV_32F, 1, 0, 3);
  cv::absdiff(sobel, cv::Scalar(0), magnitude);
  double max_magnitude = 0.0;
  cv::minMaxLoc(magnitude, nullptr, &max_magnitude);
  magnitude.convertTo(gradient, CV_8U, 255.0 / std::max(1.0, max_magnitude));
  cv::inRange(gradient, cv::Scalar(28), cv::Scalar(255), gradient);
  cv::Mat paint, paint_near;
  cv::bitwise_or(white, yellow, paint);
  cv::dilate(paint, paint_near, cv::Mat::ones(3, 3, CV_8U));
  cv::bitwise_and(gradient, paint_near, gradient);
  cv::Mat yellow_support;
  cv::bitwise_or(nearby_width, gradient, yellow_support);
  cv::bitwise_and(yellow, yellow_support, yellow);
  cv::Mat binary;
  cv::bitwise_or(white, yellow, binary);
  cv::bitwise_or(binary, gradient, binary);
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5)));
  return binary;
}

bool LaneDetector::collect_sliding(const cv::Mat& warped, std::vector<cv::Point>& left,
                                   std::vector<cv::Point>& right, cv::Mat& debug) const {
  const int w = warped.cols, h = warped.rows;
  std::vector<int> histogram(w, 0);
  for (int y = h / 2; y < h; ++y) {
    const unsigned char* row = warped.ptr<unsigned char>(y);
    for (int x = 0; x < w; ++x) if (row[x]) ++histogram[x];
  }
  const int left_begin = cvRound(w * 0.08), right_end = cvRound(w * 0.92);
  int left_x = left_begin, right_x = w / 2;
  for (int x = left_begin; x < w / 2; ++x)
    if (histogram[x] > histogram[left_x]) left_x = x;
  for (int x = w / 2; x < right_end; ++x)
    if (histogram[x] > histogram[right_x]) right_x = x;
  if (histogram[left_x] < 3 || histogram[right_x] < 3) return false;

  std::vector<cv::Point> pixels;
  cv::findNonZero(warped, pixels);
  const int windows = 9, window_h = std::max(1, h / windows);
  const int margin = std::max(28, cvRound(w * 0.075));
  const int min_pixels = std::max(10, w / 90);
  for (int window = 0; window < windows; ++window) {
    const int high = h - window * window_h, low = std::max(0, high - window_h);
    std::vector<int> lx, rx;
    for (const cv::Point& point : pixels) {
      if (point.y < low || point.y >= high) continue;
      if (std::abs(point.x - left_x) < margin) { left.push_back(point); lx.push_back(point.x); }
      if (std::abs(point.x - right_x) < margin) { right.push_back(point); rx.push_back(point.x); }
    }
    auto recenter = [min_pixels](std::vector<int>& xs, int& center) {
      if (static_cast<int>(xs.size()) < min_pixels) return;
      const size_t middle = xs.size() / 2;
      std::nth_element(xs.begin(), xs.begin() + middle, xs.end());
      center = xs[middle];
    };
    recenter(lx, left_x); recenter(rx, right_x);
    const int left_low = std::max(0, left_x - margin), left_high = std::min(w, left_x + margin);
    const int right_low = std::max(0, right_x - margin), right_high = std::min(w, right_x + margin);
    cv::rectangle(debug, cv::Rect(left_low, low, std::max(1, left_high - left_low),
                  std::min(window_h, h - low)), cv::Scalar(0, 210, 0), 1);
    cv::rectangle(debug, cv::Rect(right_low, low, std::max(1, right_high - right_low),
                  std::min(window_h, h - low)), cv::Scalar(0, 210, 0), 1);
  }
  const int minimum = std::max(65, cvRound(w * h * 0.00009));
  return static_cast<int>(left.size()) >= minimum && static_cast<int>(right.size()) >= minimum;
}

bool LaneDetector::collect_tracked(const cv::Mat& warped, std::vector<cv::Point>& left,
                                   std::vector<cv::Point>& right, cv::Mat& debug) const {
  const int w = warped.cols, h = warped.rows;
  const int margin = std::max(18, cvRound(w * 0.045));
  std::vector<cv::Point> pixels;
  cv::findNonZero(warped, pixels);
  for (const cv::Point& point : pixels) {
    const double dl = std::abs(point.x - curve_x(previous_left_curve_, point.y));
    const double dr = std::abs(point.x - curve_x(previous_right_curve_, point.y));
    if (dl < margin && dl <= dr) left.push_back(point);
    else if (dr < margin && dr < dl) right.push_back(point);
  }
  const int minimum = std::max(55, cvRound(w * h * 0.000075));
  auto span = [](const std::vector<cv::Point>& points) {
    if (points.empty()) return 0;
    int low = points.front().y, high = low;
    for (const cv::Point& point : points) { low = std::min(low, point.y); high = std::max(high, point.y); }
    return high - low;
  };
  if (static_cast<int>(left.size()) < minimum || static_cast<int>(right.size()) < minimum ||
      span(left) < h * 0.48 || span(right) < h * 0.48) return false;
  for (int y = 0; y < h; y += std::max(1, h / 9)) {
    const int lx = cvRound(curve_x(previous_left_curve_, y));
    const int rx = cvRound(curve_x(previous_right_curve_, y));
    cv::rectangle(debug, cv::Point(lx - margin, y),
                  cv::Point(lx + margin, std::min(h - 1, y + h / 9)), cv::Scalar(0, 210, 0), 1);
    cv::rectangle(debug, cv::Point(rx - margin, y),
                  cv::Point(rx + margin, std::min(h - 1, y + h / 9)), cv::Scalar(0, 210, 0), 1);
  }
  return true;
}

bool LaneDetector::robust_fit(const std::vector<cv::Point>& input, int width, int height,
                              cv::Vec3d& fit) const {
  std::vector<cv::Point> points = input;
  for (int iteration = 0; iteration < 3; ++iteration) {
    if (points.size() < 35) return false;
    cv::Mat design(static_cast<int>(points.size()), 3, CV_64F);
    cv::Mat values(static_cast<int>(points.size()), 1, CV_64F);
    int min_y = height, max_y = 0;
    for (size_t i = 0; i < points.size(); ++i) {
      const double y = points[i].y;
      design.at<double>(static_cast<int>(i), 0) = y * y;
      design.at<double>(static_cast<int>(i), 1) = y;
      design.at<double>(static_cast<int>(i), 2) = 1.0;
      values.at<double>(static_cast<int>(i), 0) = points[i].x;
      min_y = std::min(min_y, points[i].y); max_y = std::max(max_y, points[i].y);
    }
    if (max_y - min_y < height * 0.35) return false;
    cv::Mat solved;
    if (!cv::solve(design, values, solved, cv::DECOMP_SVD)) return false;
    fit = cv::Vec3d(solved.at<double>(0), solved.at<double>(1), solved.at<double>(2));
    if (iteration == 2) break;
    std::vector<float> residuals;
    residuals.reserve(points.size());
    for (const cv::Point& point : points)
      residuals.push_back(static_cast<float>(std::abs(point.x - curve_x(fit, point.y))));
    const float limit = std::max(width * 0.012f, percentile(residuals, 0.5f) * 2.5f);
    std::vector<cv::Point> kept;
    kept.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i)
      if (residuals[i] < limit) kept.push_back(points[i]);
    points.swap(kept);
  }
  return true;
}

bool LaneDetector::validate_fits(const cv::Vec3d& left, const cv::Vec3d& right,
                                 int width, int height) const {
  double min_width = 1e9, max_width = 0.0, sum = 0.0;
  for (int i = 0; i < 7; ++i) {
    const double y = i * (height - 1) / 6.0;
    const double lane_width = curve_x(right, y) - curve_x(left, y);
    if (!std::isfinite(lane_width) || lane_width <= width * 0.16 || lane_width >= width * 0.72)
      return false;
    min_width = std::min(min_width, lane_width); max_width = std::max(max_width, lane_width); sum += lane_width;
  }
  const double variation = (max_width - min_width) / std::max(1.0, sum / 7.0);
  return variation <= 0.48 && curve_x(left, height - 1) < width * 0.56 &&
         curve_x(right, height - 1) > width * 0.44;
}

LaneResult LaneDetector::make_curve_result(const cv::Mat& binary, const cv::Mat& bird,
                                           const cv::Mat& debug, const cv::Mat& inverse,
                                           const cv::Vec3d& left, const cv::Vec3d& right,
                                           int width, int height, float scale,
                                           float reference_x, bool partial) const {
  LaneResult result;
  result.binary = binary; result.bird_eye = bird; result.curve_fit = debug;
  std::vector<cv::Point2f> left_bird, right_bird;
  for (int i = 0; i < 32; ++i) {
    const float yl = (height - 1) * (1.0f - i / 31.0f);
    const float yr = (height - 1) * (i / 31.0f);
    left_bird.emplace_back(static_cast<float>(curve_x(left, yl)), yl);
    right_bird.emplace_back(static_cast<float>(curve_x(right, yr)), yr);
  }
  std::vector<cv::Point2f> left_image, right_image;
  cv::perspectiveTransform(left_bird, left_image, inverse);
  cv::perspectiveTransform(right_bird, right_image, inverse);
  for (const cv::Point2f& point : left_image)
    result.left.emplace_back(cvRound(std::max(0.0f, std::min(point.x, width - 1.0f))),
                             cvRound(std::max(0.0f, std::min(point.y, height - 1.0f))));
  for (const cv::Point2f& point : right_image)
    result.right.emplace_back(cvRound(std::max(0.0f, std::min(point.x, width - 1.0f))),
                              cvRound(std::max(0.0f, std::min(point.y, height - 1.0f))));
  result.polygon = result.left;
  result.polygon.insert(result.polygon.end(), result.right.begin(), result.right.end());
  const double left_bottom = curve_x(left, height - 1), right_bottom = curve_x(right, height - 1);
  result.offset_ratio = static_cast<float>((reference_x - (left_bottom + right_bottom) * 0.5) /
                                           std::max(1.0, right_bottom - left_bottom));
  result.departure = std::abs(result.offset_ratio) > config_.departure_ratio;
  result.valid = true;
  result.partial = partial;
  const float inverse_scale = 1.0f / scale;
  scale_points(result.left, inverse_scale); scale_points(result.right, inverse_scale);
  scale_points(result.polygon, inverse_scale);
  return result;
}

LaneResult LaneDetector::make_direct_result(const cv::Mat& binary, const cv::Mat& road,
                                            int y_top, int y_bottom, float scale,
                                            bool partial) const {
  LaneResult result;
  result.binary = binary;
  result.bird_eye = road;
  cv::cvtColor(road, result.curve_fit, cv::COLOR_GRAY2BGR);
  for (int i = 0; i < 32; ++i) {
    const double yl = y_bottom - i * (y_bottom - y_top) / 31.0;
    const double yr = y_top + i * (y_bottom - y_top) / 31.0;
    result.left.emplace_back(cvRound(direct_left_[0] * yl + direct_left_[1]), cvRound(yl));
    result.right.emplace_back(cvRound(direct_right_[0] * yr + direct_right_[1]), cvRound(yr));
  }
  const double left_bottom = direct_left_[0] * y_bottom + direct_left_[1];
  const double right_bottom = direct_right_[0] * y_bottom + direct_right_[1];
  const double lane_width = right_bottom - left_bottom;
  if (lane_width <= binary.cols * 0.04) return LaneResult();
  result.polygon = result.left;
  result.polygon.insert(result.polygon.end(), result.right.begin(), result.right.end());
  result.offset_ratio = static_cast<float>((binary.cols * 0.5 -
      (left_bottom + right_bottom) * 0.5) / lane_width);
  result.departure = std::abs(result.offset_ratio) > config_.departure_ratio;
  result.valid = true;
  result.partial = partial;
  cv::polylines(result.curve_fit, result.left, false, cv::Scalar(255, 80, 30), 3, cv::LINE_AA);
  cv::polylines(result.curve_fit, result.right, false, cv::Scalar(20, 40, 255), 3, cv::LINE_AA);
  const float inverse_scale = 1.0f / scale;
  scale_points(result.left, inverse_scale); scale_points(result.right, inverse_scale);
  scale_points(result.polygon, inverse_scale);
  return result;
}

LaneResult LaneDetector::vanishing_fallback(const cv::Mat& binary, float scale) {
  const int w = binary.cols, h = binary.rows;
  const int y_min = cvRound(h * 0.43), y_bottom = cvRound(h * 0.90);
  cv::Mat mask = cv::Mat::zeros(binary.size(), CV_8U);
  const std::vector<cv::Point> area = {
      cv::Point(cvRound(w * 0.05), y_bottom), cv::Point(cvRound(w * 0.30), y_min),
      cv::Point(cvRound(w * 0.70), y_min), cv::Point(cvRound(w * 0.95), y_bottom)};
  cv::fillConvexPoly(mask, area, cv::Scalar(255));
  cv::Mat road;
  cv::bitwise_and(binary, mask, road);
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(road, lines, 1, CV_PI / 360.0, 14,
                  std::max(12, w / 55), std::max(28, w / 16));
  struct Candidate { double a, b, length; };
  std::vector<Candidate> left, right;
  for (const cv::Vec4i& line : lines) {
    const double dy = line[3] - line[1];
    if (std::abs(dy) < 5.0) continue;
    const double a = (line[2] - line[0]) / dy;
    if (std::abs(a) < 0.28 || std::abs(a) > 4.5) continue;
    const double b = line[0] - a * line[1];
    const double length = std::hypot(line[2] - line[0], line[3] - line[1]);
    (a < 0.0 ? left : right).push_back({a, b, length});
  }
  bool found = false;
  double best_score = -1e30, best_la = 0.0, best_lb = 0.0;
  double best_ra = 0.0, best_rb = 0.0, best_vy = 0.0;
  for (const Candidate& l : left) for (const Candidate& r : right) {
    const double denominator = l.a - r.a;
    if (std::abs(denominator) < 1e-4) continue;
    const double vanish_y = (r.b - l.b) / denominator;
    const double vanish_x = l.a * vanish_y + l.b;
    const double lx = l.a * y_bottom + l.b, rx = r.a * y_bottom + r.b;
    const double lane_width = rx - lx;
    if (vanish_y <= h * 0.28 || vanish_y >= h * 0.68 ||
        vanish_x <= w * 0.28 || vanish_x >= w * 0.72 ||
        lx >= w * 0.50 || rx <= w * 0.50 ||
        lane_width <= w * 0.16 || lane_width >= w * 0.68) continue;
    const double center = (lx + rx) * 0.5;
    const double score = l.length + r.length - std::abs(vanish_x - w * 0.5) * 0.9 -
                         std::abs(center - w * 0.5) * 0.45 -
                         std::abs(lane_width - w * 0.36) * 0.25;
    if (!found || score > best_score) {
      found = true; best_score = score; best_la = l.a; best_lb = l.b;
      best_ra = r.a; best_rb = r.b; best_vy = vanish_y;
    }
  }
  if (!found) {
    if (!has_direct_ || direct_missed_ >= 3) {
      has_direct_ = false; direct_missed_ = 0;
      return LaneResult();
    }
    ++direct_missed_;
    return make_direct_result(binary, road, direct_top_, y_bottom, scale, true);
  }
  int top = cvRound(std::max(h * 0.48, std::min(h * 0.68, best_vy + h * 0.055)));
  if (has_direct_) {
    double displacement = 0.0;
    for (int y : {top, y_bottom}) {
      displacement = std::max(displacement, std::abs(best_la * y + best_lb -
          (direct_left_[0] * y + direct_left_[1])));
      displacement = std::max(displacement, std::abs(best_ra * y + best_rb -
          (direct_right_[0] * y + direct_right_[1])));
    }
    if (displacement > w * 0.18 && direct_missed_ < 3) {
      ++direct_missed_;
      return make_direct_result(binary, road, direct_top_, y_bottom, scale, true);
    }
    const double alpha = std::max(0.12, std::min(0.28, 0.12 + displacement / (w * 0.60)));
    best_la = direct_left_[0] * (1.0 - alpha) + best_la * alpha;
    best_lb = direct_left_[1] * (1.0 - alpha) + best_lb * alpha;
    best_ra = direct_right_[0] * (1.0 - alpha) + best_ra * alpha;
    best_rb = direct_right_[1] * (1.0 - alpha) + best_rb * alpha;
    top = cvRound(direct_top_ * (1.0 - alpha) + top * alpha);
  }
  direct_left_ = cv::Vec2d(best_la, best_lb);
  direct_right_ = cv::Vec2d(best_ra, best_rb);
  direct_top_ = top; has_direct_ = true; direct_missed_ = 0;
  return make_direct_result(binary, road, top, y_bottom, scale, false);
}

LaneResult LaneDetector::detect(const cv::Mat& frame) {
  LaneResult empty;
  if (frame.empty()) return empty;
  const float scale = frame.cols > config_.processing_width
      ? config_.processing_width / static_cast<float>(frame.cols) : 1.0f;
  cv::Mat work;
  if (scale < 1.0f)
    cv::resize(frame, work, cv::Size(config_.processing_width, cvRound(frame.rows * scale)), 0, 0, cv::INTER_AREA);
  else work = frame;
  const int w = work.cols, h = work.rows;
  const int road_top = std::max(0, cvRound(std::min(config_.top_left.y, config_.top_right.y) * h) - 4);
  cv::Mat binary = cv::Mat::zeros(h, w, CV_8U);
  threshold_lane_paint(work.rowRange(road_top, h)).copyTo(binary.rowRange(road_top, h));

  // The UI stores TL,TR,BR,BL. The PC estimator uses LN,LF,RF,RN.
  const cv::Point2f source[] = {
      cv::Point2f(config_.bottom_left.x * w, config_.bottom_left.y * h),
      cv::Point2f(config_.top_left.x * w, config_.top_left.y * h),
      cv::Point2f(config_.top_right.x * w, config_.top_right.y * h),
      cv::Point2f(config_.bottom_right.x * w, config_.bottom_right.y * h)};
  const cv::Point2f target[] = {
      cv::Point2f(w * (290.0f / 1280.0f), h - 1.0f),
      cv::Point2f(w * (290.0f / 1280.0f), 0.0f),
      cv::Point2f(w * (990.0f / 1280.0f), 0.0f),
      cv::Point2f(w * (990.0f / 1280.0f), h - 1.0f)};
  const cv::Mat perspective = cv::getPerspectiveTransform(source, target);
  const cv::Mat inverse = cv::getPerspectiveTransform(target, source);
  std::vector<cv::Point2f> reference_source(1, cv::Point2f(w * 0.5f, source[0].y));
  std::vector<cv::Point2f> reference_bird;
  cv::perspectiveTransform(reference_source, reference_bird, perspective);
  const float reference_x = reference_bird.front().x;
  cv::Mat bird;
  cv::warpPerspective(binary, bird, perspective, binary.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT);
  cv::Mat debug;
  cv::cvtColor(bird, debug, cv::COLOR_GRAY2BGR);

  std::vector<cv::Point> left_points, right_points;
  bool found = false;
  if (has_curves_) {
    found = collect_tracked(bird, left_points, right_points, debug);
    if (!found && missed_ >= 5) {
      left_points.clear(); right_points.clear();
      found = collect_sliding(bird, left_points, right_points, debug);
    }
  } else {
    found = collect_sliding(bird, left_points, right_points, debug);
  }

  cv::Vec3d left_fit, right_fit;
  if (found) found = robust_fit(left_points, w, h, left_fit) &&
                     robust_fit(right_points, w, h, right_fit) &&
                     validate_fits(left_fit, right_fit, w, h);
  if (found) {
    for (const cv::Point& point : left_points) debug.at<cv::Vec3b>(point) = cv::Vec3b(255, 80, 30);
    for (const cv::Point& point : right_points) debug.at<cv::Vec3b>(point) = cv::Vec3b(20, 40, 255);
    if (has_curves_) {
      double displacement = 0.0;
      for (int i = 0; i < 5; ++i) {
        const double y = i * (h - 1) / 4.0;
        displacement = std::max(displacement, std::abs(curve_x(left_fit, y) - curve_x(previous_left_curve_, y)));
        displacement = std::max(displacement, std::abs(curve_x(right_fit, y) - curve_x(previous_right_curve_, y)));
      }
      if (displacement > w * 0.09 && missed_ < 2) found = false;
      else {
        const double alpha = std::max(0.16, std::min(0.34, 0.16 + displacement / (w * 0.30)));
        left_fit = previous_left_curve_ * (1.0 - alpha) + left_fit * alpha;
        right_fit = previous_right_curve_ * (1.0 - alpha) + right_fit * alpha;
      }
    }
  }
  if (found) {
    previous_left_curve_ = left_fit; previous_right_curve_ = right_fit;
    has_curves_ = true; missed_ = 0;
    has_direct_ = false; direct_missed_ = 0;
    for (int y = 0; y < h; ++y) {
      const cv::Point lp(cvRound(curve_x(left_fit, y)), y), rp(cvRound(curve_x(right_fit, y)), y);
      if (lp.x >= 0 && lp.x < w) cv::circle(debug, lp, 1, cv::Scalar(255, 120, 0), -1);
      if (rp.x >= 0 && rp.x < w) cv::circle(debug, rp, 1, cv::Scalar(0, 80, 255), -1);
    }
    return make_curve_result(binary, bird, debug, inverse, left_fit, right_fit,
                             w, h, scale, reference_x, false);
  }

  ++missed_;
  if (has_curves_ && missed_ <= 5)
    return make_curve_result(binary, bird, debug, inverse, previous_left_curve_, previous_right_curve_,
                             w, h, scale, reference_x, true);
  has_curves_ = false;
  LaneResult direct = vanishing_fallback(binary, scale);
  if (direct.valid) return direct;
  empty.binary = binary; empty.bird_eye = bird; empty.curve_fit = debug;
  return empty;
}

}  // namespace adas
