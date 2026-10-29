#pragma once
#include "adas/types.hpp"
#include "adas/text_renderer.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>
#include <opencv2/imgproc.hpp>

namespace adas { namespace ui {

inline int annotation_bound(int value,int low,int high) {
  return std::max(low,std::min(value,high));
}

inline bool is_lamp_annotation_target(const Detection& d) {
  return d.class_id == 21 || (d.class_id >= 7 && d.class_id <= 14);
}

inline std::vector<cv::Rect> protected_lamps(cv::Size size,
                                          const std::vector<Detection>& detections) {
  std::vector<cv::Rect> result;
  for (const auto& d : detections) {
    if (!is_lamp_annotation_target(d)) continue;
    cv::Rect box(cvFloor(d.box.x)-4, cvFloor(d.box.y)-4,
                 cvCeil(d.box.width)+8, cvCeil(d.box.height)+8);
    box &= cv::Rect(0,0,size.width,size.height);
    if (box.area()>0) result.push_back(box);
  }
  return result;
}

inline cv::Rect compact_signal_hud(cv::Size size,
                                  const std::vector<Detection>& detections) {
  const int width=std::min(168,size.width-16), height=42;
  const auto lamps=protected_lamps(size,detections);
  const std::vector<cv::Rect> candidates={
    {8,8,width,height}, {8,size.height-height-8,width,height},
    {size.width-width-8,size.height-height-8,width,height},
    {size.width-width-8,8,width,height}};
  cv::Rect best=candidates.front(); int best_overlap=std::numeric_limits<int>::max();
  for (const auto& candidate:candidates) {
    int overlap=0;
    for (const auto& lamp:lamps) overlap+=(candidate&lamp).area();
    if (overlap<best_overlap) {best=candidate;best_overlap=overlap;}
    if (!overlap) break;
  }
  return best;
}

class LampAnnotationLayout {
 public:
  LampAnnotationLayout(cv::Size size,const std::vector<Detection>& detections)
      : size_(size), blocked_(protected_lamps(size,detections)) {
    blocked_.push_back(compact_signal_hud(size,detections));
  }
  bool place(cv::Size text_size,const cv::Rect2f& lamp,cv::Rect& result) {
    const int w=text_size.width+8,h=text_size.height+8;
    if (w>size_.width-8 || h>size_.height-8) return false;
    const int x=cvRound(lamp.x), y=cvRound(lamp.y), bottom=cvCeil(lamp.y+lamp.height);
    const int anchor_x=annotation_bound(x,4,size_.width-w-4);
    std::vector<cv::Point> candidates={{anchor_x,bottom+8},{anchor_x,y-h-8},
      {cvCeil(lamp.x+lamp.width)+8,y}, {x-w-8,y}};
    for(int row=1;row<=8;++row) {
      const int lower=bottom+8+row*(h+4), upper=y-8-row*(h+4);
      for(int col=0;col<=4;++col) {
        candidates.push_back(cv::Point(anchor_x-col*(w+6),lower));
        candidates.push_back(cv::Point(anchor_x+col*(w+6),lower));
        candidates.push_back(cv::Point(anchor_x-col*(w+6),upper));
        candidates.push_back(cv::Point(anchor_x+col*(w+6),upper));
      }
    }
    double best=std::numeric_limits<double>::infinity();bool found=false;
    for(const auto& point:candidates) {
      cv::Rect rect(point.x,point.y,w,h);
      if(rect.x<4 || rect.y<4 || rect.br().x>size_.width-4 || rect.br().y>size_.height-4) continue;
      bool overlap=false;
      for(const auto& block:blocked_) if((rect&block).area()>0) {overlap=true;break;}
      if(overlap) continue;
      const double score=std::hypot(rect.x+w*.5-lamp.x-lamp.width*.5,
                                    rect.y+h*.5-lamp.y-lamp.height*.5);
      if(score<best) {best=score;result=rect;found=true;}
    }
    if(found) blocked_.push_back(result);
    return found;
  }
 private:
  cv::Size size_;
  std::vector<cv::Rect> blocked_;
};

inline void draw_compact_lamp_caption(cv::Mat& frame,const Detection& detection,
                                      LampAnnotationLayout& layout) {
  if(detection.class_id<7 || detection.class_id>14) return;
  static const char* names[]={"圆红灯","左转红灯","右转红灯","直行红灯",
                              "圆绿灯","左转绿灯","右转绿灯","直行绿灯"};
  std::ostringstream text;text<<names[detection.class_id-7]<<" MO "
                               <<int(std::round(detection.score*100))<<'%';
  const int font=16;const cv::Size size=measure_text(text.str(),font);cv::Rect rect;
  if(!layout.place(size,detection.box,rect)) return;
  const cv::Scalar color=detection.class_id<=10?cv::Scalar(45,65,245):cv::Scalar(80,235,95);
  const cv::Point center(cvRound(detection.box.x+detection.box.width*.5),
                         cvRound(detection.box.y+detection.box.height*.5));
  const cv::Point end(annotation_bound(center.x,rect.x,rect.br().x-1),
                      annotation_bound(center.y,rect.y,rect.br().y-1));
  cv::line(frame,center,end,color,1,cv::LINE_AA);
  cv::Mat shade(rect.size(),frame.type(),cv::Scalar(18,22,24));
  cv::addWeighted(frame(rect),.25,shade,.75,0,frame(rect));
  draw_text(frame,text.str(),{rect.x+4,rect.y+size.height+2},font,color);
}

}}  // namespace adas::ui
