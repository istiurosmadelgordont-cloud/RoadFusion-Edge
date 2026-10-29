#pragma once

#include <opencv2/core.hpp>
#include <rga/im2d.h>
#include <rga/rga.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace adas {
namespace rga_image_ops {

inline void require_page_aligned_output(const cv::Mat& destination) {
  const long page_size = ::sysconf(_SC_PAGESIZE);
  if (page_size <= 0 || reinterpret_cast<uintptr_t>(destination.data) %
                            static_cast<size_t>(page_size) != 0)
    throw std::runtime_error("RGA output must start at a page-aligned pool address");
}

inline void resize_bgr(const cv::Mat& source, cv::Mat& destination) {
  if (source.empty() || source.type() != CV_8UC3 || destination.empty() ||
      destination.type() != CV_8UC3 || source.step % 3 != 0 || destination.step % 3 != 0 ||
      source.step < static_cast<size_t>(source.cols) * 3 ||
      destination.step < static_cast<size_t>(destination.cols) * 3)
    throw std::runtime_error("RGA BGR resize requires valid CV_8UC3 image strides");
  require_page_aligned_output(destination);
  const auto input = wrapbuffer_virtualaddr_t(
      const_cast<unsigned char*>(source.data), source.cols, source.rows,
      static_cast<int>(source.step / 3), source.rows, RK_FORMAT_BGR_888);
  const auto output = wrapbuffer_virtualaddr_t(
      destination.data, destination.cols, destination.rows,
      static_cast<int>(destination.step / 3), destination.rows, RK_FORMAT_BGR_888);
  const IM_STATUS status = imresize_t(input, output, 0, 0, IM_INTERP_DEFAULT, 1);
  if (status != IM_STATUS_SUCCESS)
    throw std::runtime_error(std::string("RGA BGR resize failed: ") + imStrError_t(status));
}

inline void bgr_to_rgb(const cv::Mat& source, cv::Mat& destination) {
  if (source.empty() || source.type() != CV_8UC3 || destination.empty() ||
      destination.type() != CV_8UC3 || source.size() != destination.size() ||
      source.step % 3 != 0 || destination.step % 3 != 0 ||
      source.step < static_cast<size_t>(source.cols) * 3 ||
      destination.step < static_cast<size_t>(destination.cols) * 3)
    throw std::runtime_error("RGA BGR-to-RGB requires same-size CV_8UC3 images with valid strides");
  require_page_aligned_output(destination);
  const auto input = wrapbuffer_virtualaddr_t(
      const_cast<unsigned char*>(source.data), source.cols, source.rows,
      static_cast<int>(source.step / 3), source.rows, RK_FORMAT_BGR_888);
  const auto output = wrapbuffer_virtualaddr_t(
      destination.data, destination.cols, destination.rows,
      static_cast<int>(destination.step / 3), destination.rows, RK_FORMAT_RGB_888);
  const IM_STATUS status = imcvtcolor_t(
      input, output, RK_FORMAT_BGR_888, RK_FORMAT_RGB_888, 0, 1);
  if (status != IM_STATUS_SUCCESS)
    throw std::runtime_error(std::string("RGA BGR-to-RGB failed: ") + imStrError_t(status));
}

}  // namespace rga_image_ops
}  // namespace adas
