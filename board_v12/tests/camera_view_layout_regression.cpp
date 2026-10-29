#include "adas/camera_view_layout.hpp"
#include "adas/ui_upload_regions.hpp"
#include <cassert>
#include <iostream>

int main() {
  adas::CameraViewLayout layout;
  const auto grid=layout.rects();
  assert(grid[2].tl()==cv::Point(32,218));
  assert(grid[0].tl()==cv::Point(704,218));
  assert(grid[3].tl()==cv::Point(32,666));
  assert(grid[1].tl()==cv::Point(704,666));
  std::array<cv::Mat,4> frames;
  for(int i=0;i<4;++i)frames[i]=cv::Mat(360,640,CV_8UC3,cv::Scalar(i*40+10,20,30));
  const auto verify_upload=[](const std::array<cv::Rect,4>& rects,unsigned mask) {
    cv::Mat coverage(1080,1920,CV_8U,cv::Scalar());
    for(auto r:adas::visible_ui_regions(coverage.size(),rects,mask))coverage(r)+=1;
    for(int i=0;i<4;++i)if(mask&(1U<<i))coverage(rects[i])+=1;
    double lo,hi;cv::minMaxLoc(coverage,&lo,&hi);assert(lo==1 && hi==1);
  };
  verify_upload(grid,15);
  assert(!layout.select(-2) && !layout.select(4));
  for(int i=0;i<4;++i) {
    assert(layout.select(-1) || layout.focused()==-1);
    assert(layout.action_at(adas::CameraViewLayout::grid_zoom_button(i).tl()+cv::Point(3,3))==i);
    assert(layout.select(i));
    const auto rects=layout.rects();const auto visible=layout.visible_frames(frames);
    for(int j=0;j<4;++j) {
      assert((rects[j].area()==0)==(i!=j));assert(visible[j].empty()==(i!=j));
      assert(!frames[j].empty()); // Presentation must not release source camera data.
      assert(layout.action_at(adas::CameraViewLayout::select_button(j).tl()+cv::Point(3,3))==j);
    }
    verify_upload(rects,1U<<i);
    assert(rects[i].x+rects[i].width<1390 && rects[i].y+rects[i].height<1060);
    const auto mid=rects[i].tl()+cv::Point(rects[i].width/2,rects[i].height/2);
    assert(std::abs((mid.x-rects[i].x)/double(rects[i].width)-.5)<.002);
    assert(layout.action_at(adas::CameraViewLayout::return_button().tl()+cv::Point(3,3))==-1);
    assert(layout.toggle() && layout.focused()==-1);
    assert(layout.toggle() && layout.focused()==i);
  }
  std::cout<<"PASS: semantic camera order, zoom/select/return, source ownership, UI coverage, calibration coordinates\n";
}
