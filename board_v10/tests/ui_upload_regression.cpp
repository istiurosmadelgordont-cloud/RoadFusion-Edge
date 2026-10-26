#include "adas/ui_upload_regions.hpp"
#include <cassert>
#include <iostream>

int main() {
  const cv::Size size(1920,1080);
  const std::array<cv::Rect,4> views{{{16,124,672,378},{700,124,672,378},
                                   {16,568,672,378},{700,568,672,378}}};
  const auto check = [&](const std::array<cv::Rect,4>& rectangles, unsigned mask) {
    cv::Mat expected(size,CV_8UC1,cv::Scalar(1));
    for (int i=0;i<4;++i) if (mask & (1U<<i))
      expected(rectangles[i] & cv::Rect(0,0,size.width,size.height)).setTo(0);
    cv::Mat actual(size,CV_8UC1,cv::Scalar(0));
    for (const auto& r : adas::visible_ui_regions(size,rectangles,mask)) {
      assert((r & cv::Rect(0,0,size.width,size.height)) == r);
      actual(r) += 1;
    }
    cv::Mat difference;
    cv::compare(actual,expected,difference,cv::CMP_NE);
    assert(cv::countNonZero(difference)==0); // no holes or overlapping uploads
    return cv::countNonZero(actual);
  };
  for (unsigned mask=0;mask<16;++mask) check(views,mask);
  assert(check(views,15) == size.area()-4*672*378);
  // Clipping and overlapping cameras must also preserve all visible UI.
  check({{{-10,-10,500,400},{250,250,500,500},{900,900,1500,500},{0,0,0,0}}},15);
  std::cout << "PASS: UI visible pixels, missing cameras, clipping and overlap\n";
}
