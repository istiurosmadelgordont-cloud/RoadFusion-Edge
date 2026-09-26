#pragma once
#include <opencv2/core.hpp>

namespace adas {
// Exact 3:2 bilinear sampling: source positions are .25 and 1.75 in
// each three-pixel block. Integer weights avoid generic resize overhead
// in the board's old OpenCV build. The native inference image is untouched.
inline void resize_preview(const cv::Mat& source, cv::Mat& target) {
  CV_Assert(source.type() == CV_8UC3 && source.cols % 3 == 0 && source.rows % 3 == 0);
  target.create(source.rows * 2 / 3, source.cols * 2 / 3, CV_8UC3);
  for (int y = 0; y < source.rows / 3; ++y) {
    const unsigned char* a = source.ptr<unsigned char>(y * 3);
    const unsigned char* b = source.ptr<unsigned char>(y * 3 + 1);
    const unsigned char* c = source.ptr<unsigned char>(y * 3 + 2);
    unsigned char* top = target.ptr<unsigned char>(y * 2);
    unsigned char* bottom = target.ptr<unsigned char>(y * 2 + 1);
    for (int x = 0; x < source.cols / 3; ++x) {
      for (int channel = 0; channel < 3; ++channel) {
        const int i = x * 9 + channel, o = x * 6 + channel;
        top[o] = (9 * a[i] + 3 * a[i+3] + 3 * b[i] + b[i+3] + 8) >> 4;
        top[o+3] = (3 * a[i+3] + 9 * a[i+6] + b[i+3] + 3 * b[i+6] + 8) >> 4;
        bottom[o] = (3 * b[i] + b[i+3] + 9 * c[i] + 3 * c[i+3] + 8) >> 4;
        bottom[o+3] = (b[i+3] + 3 * b[i+6] + 3 * c[i+3] + 9 * c[i+6] + 8) >> 4;
      }
    }
  }
}
}
