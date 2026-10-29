#pragma once
#include <array>
#include <opencv2/core.hpp>
namespace adas {
// Semantic indices never change: front=0, rear=1, left=2, right=3.
class CameraViewLayout {
 public:
  static std::array<cv::Rect,4> grid_rects() {
    return {{{704,218,656,369},{704,666,656,369},
             {32,218,656,369},{32,666,656,369}}};
  }
  static cv::Rect area(){return {24,176,1344,884};}
  static cv::Rect return_button(){return {1242,120,118,36};}
  static cv::Rect priority_button(){return {1106,120,128,36};}
  static cv::Rect grid_zoom_button(int camera){
    if(camera<0 || camera>=4)return {};
    return zoom_button(grid_rects()[camera]);
  }
  static cv::Rect zoom_button(const cv::Rect& r){return {r.br().x-48,r.y-32,40,28};}
  static cv::Rect select_button(int camera){
    const int x[]={832,968,764,900};
    if(camera<0 || camera>=4)return {};
    return {x[camera],176,62,28};
  }
  int focused() const{return focused_;}
  bool priority() const{return priority_;}
  int action_at(const cv::Point& p) const{
    if(return_button().contains(p))return -1;
    if(priority_button().contains(p))return -3;
    if(focused_>=0)for(int i=0;i<4;++i)if(select_button(i).contains(p))return i;
    const auto views=rects();
    for(int i=0;i<4;++i)if(views[i].area()>0){
      const auto r=views[i];
      if(cv::Rect(r.x,r.y-32,r.width,30).contains(p))
        return focused_==i?(focus_from_priority_?-3:-1):i;
      if(focused_<0 && r.contains(p))return i;
    }
    return -2;
  }
  bool select(int camera){
    if(camera==-3){const bool changed=!priority_||focused_>=0;priority_=true;focused_=-1;return changed;}
    if(camera < -1 || camera>=4)return false;
    const bool changed=camera!=focused_ || priority_;
    if(camera>=0 && focused_<0)focus_from_priority_=priority_;
    priority_=false;focused_=camera;if(camera>=0)last_focused_=camera;return changed;
  }
  bool toggle(){return select(focused_<0?last_focused_:-1);}
  std::array<cv::Rect,4> rects() const{
    if(focused_>=0){std::array<cv::Rect,4> r{};r[focused_]={32,248,1328,747};return r;}
    if(priority_)return {{{216,214,960,540},{928,808,432,243},
                         {32,808,432,243},{480,808,432,243}}};
    return grid_rects();
  }
  std::array<cv::Mat,4> visible_frames(const std::array<cv::Mat,4>& frames) const{
    auto r=frames;if(focused_>=0)for(int i=0;i<4;++i)if(i!=focused_)r[i].release();return r;
  }
 private:
  int focused_=-1,last_focused_=0;bool priority_=false,focus_from_priority_=false;
};
}
