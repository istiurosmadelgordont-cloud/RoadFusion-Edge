#include "adas/preview_resize.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>

int main() {
  cv::setNumThreads(1);
  // Non-contiguous input exercises row strides as well as RGB interpolation.
  cv::Mat padded(1082, 1924, CV_8UC3);
  cv::randu(padded, 0, 256);
  cv::Mat source = padded(cv::Rect(1, 1, 1920, 1080));
  cv::Mat actual, reference;
  adas::resize_preview(source, actual);
  cv::resize(source, reference, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
  const double error = cv::norm(actual, reference, cv::NORM_INF);
  if (error > 1.0) { std::cerr << "Preview interpolation error: " << error << '\n'; return 1; }
  std::cout << "PASS: preview bilinear max channel error=" << error << '\n';
}
