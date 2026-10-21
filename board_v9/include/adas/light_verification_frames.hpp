#pragma once
#include "adas/types.hpp"
#include "adas/preview_resize.hpp"
namespace adas {
// HSV only consumes already classified lamp candidates. Skip four-view
// downsampling when there are none, and otherwise resize only needed views.
inline void prepare_light_verification_frames(const cv::Mat& composite,
    const std::array<std::vector<Detection>,4>& detections,std::array<cv::Mat,4>& frames) {
  for(int view=0;view<4;++view){
    bool required=false;
    for(const auto& d:detections[view])if(d.class_id>=7 && d.class_id<=14 && d.score>=.4f){required=true;break;}
    if(!required){frames[view].release();continue;}
    const cv::Rect quadrant((view%2)*(composite.cols/2),(view/2)*(composite.rows/2),composite.cols/2,composite.rows/2);
    resize_preview(composite(quadrant),frames[view]);
  }
}
}
