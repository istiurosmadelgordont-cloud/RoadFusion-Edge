#pragma once

#include <opencv2/core.hpp>
#include <string>

namespace adas {
namespace ui {

bool initialize_text(const std::string& font_path);
bool text_ready();
cv::Size measure_text(const std::string& text, int pixel_height);
void draw_text(cv::Mat& image, const std::string& text, const cv::Point& baseline,
               int pixel_height, const cv::Scalar& color);

}  // namespace ui
}  // namespace adas
