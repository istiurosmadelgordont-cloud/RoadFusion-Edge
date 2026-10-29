#include "adas/preview_resize.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>

int main() {
  cv::setNumThreads(1);
  // Non-contiguous input exercises row strides as well as RGB interpolation.
  cv::Mat padded(1082, 1924, CV_8UC3);
  cv::randu(padded, 0, 256);
  cv::Mat source = padded(cv::Rect(1, 1, 1920, 1080));
  cv::Mat actual, reference;
  adas::resize_preview(source, actual);
  cv::resize(source, reference, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
  const double error = cv::norm(actual, reference, cv::NORM_INF);
  if (error > 1.0) { std::cerr << "Preview interpolation error: " << error << '\n'; return 1; }
  std::cout << "PASS: preview bilinear max channel error=" << error << '\n';
  cv::Mat scalar_full;adas::resize_preview_impl(source,scalar_full,false);
  if(cv::norm(actual,scalar_full,cv::NORM_INF)!=0){std::cerr<<"Parallel band mismatch\n";return 3;}
  for(int width:{3,6,15,24,27,33,48,99,960,1920}) {
    cv::Mat storage(110,width+7,CV_8UC3);cv::randu(storage,0,256);
    const cv::Mat roi=storage(cv::Rect(2,1,width,108));cv::Mat scalar,fast;
    adas::resize_preview_impl(roi,scalar,false);adas::resize_preview(roi,fast);
    if(cv::norm(scalar,fast,cv::NORM_INF)!=0){std::cerr<<"Vector/scalar mismatch width="<<width<<'\n';return 2;}
  }
  double scalar_ms=0,fast_ms=0;
  for(int i=0;i<12;++i){auto start=cv::getTickCount();adas::resize_preview_impl(source,actual,false);
    scalar_ms+=(cv::getTickCount()-start)*1000/cv::getTickFrequency();
    start=cv::getTickCount();adas::resize_preview(source,actual);fast_ms+=(cv::getTickCount()-start)*1000/cv::getTickFrequency();}
  std::cout<<"PASS: strided ROIs and tails byte-identical; scalar_ms="<<scalar_ms/12<<" fast_ms="<<fast_ms/12<<'\n';
}
