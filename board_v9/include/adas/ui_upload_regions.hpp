#pragma once

#include <array>
#include <vector>
#include <opencv2/core.hpp>

namespace adas {
// Camera textures cover these pixels after the UI texture is drawn. Upload
// only its visible remainder; missing cameras and AVM still use the full UI.
inline std::vector<cv::Rect> visible_ui_regions(
    const cv::Size& size, const std::array<cv::Rect, 4>& views,
    unsigned camera_mask) {
  std::vector<cv::Rect> regions{cv::Rect(0, 0, size.width, size.height)};
  for (int i = 0; i < 4; ++i) {
    if (!(camera_mask & (1U << i))) continue;
    std::vector<cv::Rect> next;
    for (const cv::Rect& region : regions) {
      const cv::Rect overlap = region & views[i];
      if (overlap.area() <= 0) { next.push_back(region); continue; }
      if (overlap.y > region.y)
        next.emplace_back(region.x, region.y, region.width, overlap.y-region.y);
      if (overlap.br().y < region.br().y)
        next.emplace_back(region.x, overlap.br().y, region.width,
                          region.br().y-overlap.br().y);
      if (overlap.x > region.x)
        next.emplace_back(region.x, overlap.y, overlap.x-region.x, overlap.height);
      if (overlap.br().x < region.br().x)
        next.emplace_back(overlap.br().x, overlap.y,
                          region.br().x-overlap.br().x, overlap.height);
    }
    regions.swap(next);
  }
  return regions;
}
}  // namespace adas
