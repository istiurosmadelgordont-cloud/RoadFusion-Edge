#pragma once
#include "adas/types.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace adas {
// Rendering only: match pixels from the exact detector source image to the
// displayed image. Never creates tracks, extends lifetimes or supplies evidence.
class BoxMotionCompensator {
 public:
  using Time=std::chrono::steady_clock::time_point;
  static bool ordinary(int cls) { return !(cls>=7 && cls<=14) && cls!=21; }
  static cv::Mat reference(const cv::Mat& image) {
    if(image.empty())return {};
    cv::Mat tile,small,gray;
    // Match the live 960x540 -> 640x360 preview sampling before reducing it.
    if(image.size()!=cv::Size(640,360))cv::resize(image,tile,{640,360},0,0,cv::INTER_LINEAR);
    else tile=image;
    cv::resize(tile,small,{320,180},0,0,cv::INTER_LINEAR);
    if(small.channels()==1)return small;
    cv::cvtColor(small,gray,cv::COLOR_BGR2GRAY);return gray;
  }
  void reset() { gray_.release();pyramid_.clear();points_.clear();entries_.clear(); }
  void observe(const cv::Mat& gray,const std::vector<Detection>& measured,Time source) {
    reset();source_=source;
    if(gray.empty() || gray.type()!=CV_8UC1)return;
    gray_=gray; // Immutable, owned result image; source detections use 640x360.
    for(const auto& d:measured) {
      if(!ordinary(d.class_id) || d.track_id<0 || entries_.size()>=12)continue;
      const cv::Rect2f b=d.box;
      cv::Rect roi(cvFloor(b.x*.5f),cvFloor(b.y*.5f),cvCeil(b.width*.5f),cvCeil(b.height*.5f));
      roi &= cv::Rect(0,0,gray.cols,gray.rows);
      if(roi.width<6 || roi.height<6)continue;
      std::vector<cv::Point2f> corners;
      cv::goodFeaturesToTrack(gray(roi),corners,8,.02,3);
      if(corners.size()<3)continue;
      Entry e;e.id=d.track_id;e.cls=d.class_id;e.box=d.box;e.first=points_.size();
      for(auto p:corners)points_.push_back(p+cv::Point2f(float(roi.x),float(roi.y)));
      e.count=corners.size();entries_.push_back(e);
    }
    if(!points_.empty())cv::buildOpticalFlowPyramid(gray_,pyramid_,{15,15},3);
  }
  int apply(const cv::Mat& image,Time displayed,std::vector<Detection>& visible) const {
    if(gray_.empty() || points_.empty() || visible.empty() || displayed<source_ ||
        displayed-source_>std::chrono::milliseconds(650))return 0;
    bool needed=false;
    for(const auto& d:visible)for(const auto& e:entries_)
      if(d.track_id==e.id && d.class_id==e.cls)needed=true;
    if(!needed)return 0;
    const cv::Mat current=reference(image);
    if(current.empty() || current.size()!=gray_.size())return 0;
    // Reuse the detector-source pyramid and share the current pyramid across
    // forward/backward checks; otherwise LK builds these images four times.
    std::vector<cv::Mat> current_pyramid;
    cv::buildOpticalFlowPyramid(current,current_pyramid,{15,15},3);
    std::vector<cv::Point2f> moved,back;
    std::vector<unsigned char> valid,back_valid;std::vector<float> error,back_error;
    const cv::TermCriteria stop(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,12,.03);
    cv::calcOpticalFlowPyrLK(pyramid_,current_pyramid,points_,moved,valid,error,{15,15},3,stop);
    cv::calcOpticalFlowPyrLK(current_pyramid,pyramid_,moved,back,back_valid,back_error,{15,15},3,stop);
    int corrected=0;
    for(auto& d:visible)for(const auto& e:entries_) {
      if(d.track_id!=e.id || d.class_id!=e.cls)continue;
      std::vector<float> dx,dy;std::vector<size_t> accepted;
      for(size_t j=e.first;j<e.first+e.count;++j) {
        if(!valid[j] || !back_valid[j] || error[j]>25 ||
            cv::norm(back[j]-points_[j])>1.25 || moved[j].x<0 || moved[j].y<0 ||
            moved[j].x>=current.cols || moved[j].y>=current.rows)continue;
        dx.push_back(moved[j].x-points_[j].x);dy.push_back(moved[j].y-points_[j].y);accepted.push_back(j);
      }
      if(accepted.size()<3 || accepted.size()*2<e.count)continue;
      const float tx=median(dx),ty=median(dy);
      std::vector<float> spread;
      for(size_t j=0;j<dx.size();++j)spread.push_back(std::hypot(dx[j]-tx,dy[j]-ty));
      if(median(spread)>3.f || std::hypot(tx,ty)>90.f)continue;
      std::vector<float> scales;
      for(size_t a=0;a<accepted.size();++a)for(size_t b=a+1;b<accepted.size();++b) {
        const size_t i=accepted[a],j=accepted[b];const float before=cv::norm(points_[i]-points_[j]);
        if(before>6)scales.push_back(cv::norm(moved[i]-moved[j])/before);
      }
      const float scale=scales.size()>=3?std::max(.7f,std::min(1.5f,median(scales))):1.f;
      const cv::Point2f center(e.box.x+e.box.width*.5f+tx*2,e.box.y+e.box.height*.5f+ty*2);
      const float width=e.box.width*scale,height=e.box.height*scale;
      const cv::Rect2f box(center.x-width*.5f,center.y-height*.5f,width,height);
      const cv::Rect2f clipped=box & cv::Rect2f(0,0,640,360);
      if(clipped.width<3 || clipped.height<3)continue;
      d.box=clipped;++corrected;
    }
    return corrected;
  }
 private:
  struct Entry { int id=-1,cls=-1;cv::Rect2f box;size_t first=0,count=0; };
  static float median(std::vector<float> values) {
    const size_t mid=values.size()/2;
    std::nth_element(values.begin(),values.begin()+mid,values.end());return values[mid];
  }
  cv::Mat gray_;Time source_{};
  std::vector<cv::Mat> pyramid_;
  std::vector<cv::Point2f> points_;std::vector<Entry> entries_;
};
} // namespace adas
