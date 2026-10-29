#pragma once
#include "adas/text_renderer.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <string>
namespace adas { namespace dashboard {
inline int& theme(){static int value=0;return value;}
inline cv::Scalar rgb(unsigned c){return {double(c&255),double((c>>8)&255),double((c>>16)&255)};}
inline const char* theme_name(int i){
  static const char* n[]={"黑金 · 曜石","深海 · 冰蓝","石墨 · 薄荷","石墨 · 电橙","冷灰 · 银白","雾白 · 松绿","曜黑 · 赛道红"};
  return n[std::max(0,std::min(6,i))];
}
inline cv::Scalar color(int role){
  static const unsigned p[7][6]={
    {0x100f0c,0x201d16,0x29251b,0x4a402b,0xe2b96d,0xece8df},
    {0x0b1422,0x142337,0x1c3048,0x344b65,0x8fc8f5,0xe6edf5},
    {0x0b1016,0x141c25,0x202c35,0x34433f,0x70e1bc,0xedf3f8},
    {0x121313,0x1d1e1d,0x292a28,0x383a36,0xff985d,0xe8eae5},
    {0x101113,0x1b1d20,0x292c31,0x393d43,0xe1e5e9,0xf0f2f4},
    {0xe9eae3,0xf5f5ef,0xe6e9df,0xcdd2c6,0x38624e,0x25342b},
    {0x121113,0x201d20,0x30292d,0x41373c,0xef858e,0xf0eaed}};
  return rgb(p[theme()][role]);
}
inline cv::Scalar bg(){return color(0);} inline cv::Scalar panel(){return color(1);}
inline cv::Scalar button(){return color(2);} inline cv::Scalar line(){return color(3);}
inline cv::Scalar accent(){return color(4);} inline cv::Scalar text(){return color(5);}
inline cv::Scalar muted(){return theme()==5?rgb(0x677365):rgb(0xa59f91);}
inline void rounded(cv::Mat& im,cv::Rect r,cv::Scalar fill,int radius=10){
  if(r.area()<=0)return;
  radius=std::min(radius,std::min(r.width,r.height)/2);
  cv::rectangle(im,{r.x+radius,r.y,r.width-2*radius,r.height},fill,-1);
  cv::rectangle(im,{r.x,r.y+radius,r.width,r.height-2*radius},fill,-1);
  for(int x:{r.x+radius,r.br().x-radius-1})for(int y:{r.y+radius,r.br().y-radius-1})
    cv::circle(im,{x,y},radius,fill,-1,cv::LINE_AA);
}
inline void card(cv::Mat& im,cv::Rect r){rounded(im,r,line(),14);rounded(im,{r.x+1,r.y+1,r.width-2,r.height-2},panel(),13);}
inline void label(cv::Mat& im,const std::string& s,cv::Point p,int size=16,cv::Scalar c=text()){
  ui::draw_text(im,s,p,size,c);
}
inline void control(cv::Mat& im,cv::Rect r,const std::string& s,bool selected=false,int size=15){
  rounded(im,r,selected?accent():line(),6);
  rounded(im,{r.x+1,r.y+1,r.width-2,r.height-2},button(),5);
  while(size>10 && ui::measure_text(s,size).width>r.width-16)--size;
  const auto extent=ui::measure_text(s,size);
  label(im,s,{r.x+(r.width-extent.width)/2,r.y+(r.height+size)/2-3},size,selected?accent():text());
}
}}
