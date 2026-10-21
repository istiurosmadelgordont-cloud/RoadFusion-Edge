#pragma once
#include "adas/adas_logic_v2.hpp"

namespace adas {
// Direct port of pc_lane_markings.py's road-only local-contrast classifier.
// UNKNOWN remains explicit; this is not a neural paint-type prediction.
struct PaintProfile {
  LaneMarking label=LaneMarking::UNKNOWN;
  float coverage=0;
};
inline std::vector<std::pair<int,int>> paint_runs(const std::vector<unsigned char>& p,bool on) {
  std::vector<std::pair<int,int>> runs;int first=-1;
  for(size_t i=0;i<=p.size();++i){const bool hit=i<p.size() && bool(p[i])==on;
    if(hit && first<0)first=i;if(!hit && first>=0){runs.emplace_back(first,i);first=-1;}}
  return runs;
}
inline PaintProfile classify_contrast_profile(std::vector<unsigned char> support,
                                             bool allow_far_endpoint_occlusion=false) {
  PaintProfile result;if(support.size()<36)return result;
  for(auto r:paint_runs(support,true))if(r.second-r.first<3)
    std::fill(support.begin()+r.first,support.begin()+r.second,0);
  for(auto r:paint_runs(support,false))if(r.first>0 && r.second<int(support.size()) && r.second-r.first<=2)
    std::fill(support.begin()+r.first,support.begin()+r.second,1);
  int hits=0,longest=0,internal=0,internal_longest=0;for(auto p:support)hits+=p;
  auto patches=paint_runs(support,true);
  for(auto r:paint_runs(support,false)){
    longest=std::max(longest,r.second-r.first);
    if(r.first>0 && r.second<int(support.size())){
      internal_longest=std::max(internal_longest,r.second-r.first);
      if(r.second-r.first>=5)++internal;
    }
  }
  result.coverage=float(hits)/support.size();
  if(result.coverage>=.83f && longest<=std::max(4,int(std::nearbyint(support.size()*.09))))result.label=LaneMarking::SOLID;
  // An outer edge can begin behind a distant car. Keep the same strong
  // coverage requirement, but distinguish its far endpoint from an internal
  // dashed-line gap. This exception never applies to ego change permissions.
  else if(allow_far_endpoint_occlusion && result.coverage>=.83f && !patches.empty() &&
      patches.front().first>0 && int(support.size())-patches.back().second<=2 &&
      int(support.size())-patches.front().first>=36 &&
      internal_longest<=std::max(4,int(std::nearbyint(support.size()*.09))))result.label=LaneMarking::SOLID;
  else if(result.coverage>=.10f && result.coverage<=.78f){
    const bool bounded=patches.size()==1 && patches[0].first>=5 && int(support.size())-patches[0].second>=5;
    if((patches.size()>=2 && internal) || bounded)result.label=LaneMarking::DASHED;
  }
  return result;
}
class LanePaintFrame {
 public:
  explicit LanePaintFrame(const cv::Mat& source) : sx_(640.f/source.cols),sy_(360.f/source.rows) {
    cv::Mat image,hls,hsv;cv::resize(source,image,{640,360});
    cv::cvtColor(image,gray_,cv::COLOR_BGR2GRAY);cv::cvtColor(image,hls,cv::COLOR_BGR2HLS);cv::cvtColor(image,hsv,cv::COLOR_BGR2HSV);
    colour_.create(360,640,CV_8U);
    for(int y=0;y<360;++y)for(int x=0;x<640;++x){auto h=hls.at<cv::Vec3b>(y,x),v=hsv.at<cv::Vec3b>(y,x);
      colour_.at<unsigned char>(y,x)=h[2]<=125 || (v[0]>=12 && v[0]<=42 && v[1]>=65);}
  }
  PaintProfile observe(const std::vector<cv::Point>& curve,bool outer=false) const {
    if(curve.size()<10)return PaintProfile();
    std::vector<cv::Point2f> points;for(auto p:curve)points.emplace_back(p.x*sx_,p.y*sy_);
    std::sort(points.begin(),points.end(),[](const cv::Point2f&a,const cv::Point2f&b){return a.y<b.y;});
    const float top=std::max(points.front().y,360*.49f),bottom=std::min(points.back().y,360*.69f);
    if(bottom-top<36)return PaintProfile();
    std::vector<unsigned char> supported;std::vector<float> ridges,ys;
    int clipped=0;size_t index=1;
    for(int y=int(std::ceil(top));y<=int(std::floor(bottom));++y){
      while(index+1<points.size() && points[index].y<y)++index;
      auto a=points[index-1],b=points[index];float t=(y-a.y)/std::max(.0001f,b.y-a.y),centre=a.x+(b.x-a.x)*t;
      if(centre<11 || centre>=629)++clipped;
      unsigned char samples[49];for(int d=-24;d<=24;++d)samples[d+24]=gray_.at<unsigned char>(y,std::max(0,std::min(639,cvRound(centre+d))));
      std::nth_element(samples,samples+24,samples+49);float background=samples[24],peak=-1000;int offset=-24;
      for(int d=-10;d<=10;++d){int x=cvRound(centre+d);if(x<0 || x>=640 || !colour_.at<unsigned char>(y,x))continue;
        int value=gray_.at<unsigned char>(y,x);if(value<50)continue;
        if(value-background>peak){peak=value-background;offset=d;}}
      supported.push_back(peak>=std::max(12.f,background*.12f));ridges.push_back(offset);ys.push_back(y);
    }
    if(float(clipped)/supported.size()>.10f)return PaintProfile();
    double sx=0,sy=0,sxx=0,sxy=0;int n=0;
    for(size_t i=0;i<supported.size();++i)if(supported[i]){double x=ys[i],y=ridges[i];sx+=x;sy+=y;sxx+=x*x;sxy+=x*y;++n;}
    if(n>=6){double den=n*sxx-sx*sx,slope=den?((n*sxy-sx*sy)/den):0,intercept=(sy-slope*sx)/n;
      for(size_t i=0;i<supported.size();++i)if(std::abs(ridges[i]-(slope*ys[i]+intercept))>5)supported[i]=0;}
    return classify_contrast_profile(supported,outer);
  }
 private:
  cv::Mat gray_,colour_;float sx_,sy_;
};
inline LaneMarkingObservation observe_lane_markings_contrast(const cv::Mat& image,const LaneResult& lane) {
  LaneMarkingObservation result;if(image.empty() || !lane.valid)return result;
  LanePaintFrame paint(image);
  for(int slot=0;slot<4;++slot){
    const auto& line=slot==1?lane.left:slot==2?lane.right:lane.boundaries[slot];
    const auto observation=paint.observe(line,slot==0 || slot==3);result.boundaries[slot]=observation.label;
    result.boundary_coverage[slot]=observation.coverage;
  }
  result.left=result.boundaries[1];result.right=result.boundaries[2];
  result.left_coverage=result.boundary_coverage[1];result.right_coverage=result.boundary_coverage[2];return result;
}
}
