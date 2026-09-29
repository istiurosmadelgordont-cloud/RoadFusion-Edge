#include "adas/avm_renderer.hpp"
#include <cassert>
#include <iostream>
int main() {
  const std::string path="/tmp/roadfusion_avm_test.yml";
  std::vector<cv::Point2f> image{{0,0},{1,0},{1,1},{0,1}};
  std::vector<cv::Point2f> ground{{-8,8},{-4,8},{-4,4},{-8,4}};
  { cv::FileStorage fs(path,cv::FileStorage::WRITE); fs<<"version"<<1;
    for(auto name:{"front","left"}) fs<<name<<"{"<<"image_points"<<image<<"ground_points"<<ground<<"}";
  }
  adas::AvmRenderer renderer; assert(renderer.load(path)); assert(renderer.count()==2);
  std::array<cv::Mat,4> frames;
  frames[0]=cv::Mat(100,100,CV_8UC3,cv::Scalar(0,0,200));
  frames[2]=cv::Mat(100,100,CV_8UC3,cv::Scalar(200,0,0));
  auto out=renderer.render(frames);
  auto p=out.at<cv::Vec3b>(170,170); assert(p[0]>=99&&p[0]<=101&&p[2]>=99&&p[2]<=101);
  assert(out.at<cv::Vec3b>(470,470)[2]<60); // no extrapolation into unseen rear/right
  frames[0]=cv::Mat(200,200,CV_8UC3,cv::Scalar(0,0,200));
  assert(cv::norm(renderer.render(frames),out,cv::NORM_INF)<2);
  assert(!renderer.load("/tmp/nonexistent_avm.yml")); assert(renderer.count()==0);
  std::remove(path.c_str()); std::cout<<"AVM projection / overlap / blind area / resize / reset PASS\n";
}
