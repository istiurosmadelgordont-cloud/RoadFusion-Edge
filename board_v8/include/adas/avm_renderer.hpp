#pragma once
#include <array>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace adas {
// Image points normalized to [0,1]; ground coordinates: x right, y forward, metres.
class AvmRenderer {
 public:
  bool load(const std::string& path) {
    if (car_asset_.empty()) car_asset_=cv::imread("assets/avm_car_top.png",cv::IMREAD_UNCHANGED);
    ready_.fill(false); error_.clear(); approximate_=false; calibrated_maps_=false;
    pixels_per_metre_=25; vehicle_width_=2; vehicle_length_=4.72;
    try {
      cv::FileStorage fs(path, cv::FileStorage::READ);
      if (!fs.isOpened()) { error_ = "Calibration required"; return false; }
      int version = 0; fs["version"] >> version;
      int approximate=0; fs["approximate"] >> approximate; approximate_=approximate!=0;
      if (version == 2) {
        fs["pixels_per_metre"] >> pixels_per_metre_;
        fs["vehicle_width"] >> vehicle_width_; fs["vehicle_length"] >> vehicle_length_;
        if (!std::isfinite(pixels_per_metre_) || pixels_per_metre_<5 || pixels_per_metre_>100 ||
            !std::isfinite(vehicle_width_) || vehicle_width_<1 || vehicle_width_>4 ||
            !std::isfinite(vehicle_length_) || vehicle_length_<2 || vehicle_length_>12) return false;
        const char* names[] = {"front","rear","left","right","rear_left","rear_right","front_tele"};
        for (int i=0;i<7;++i) {
          if (fs[names[i]].empty()) continue;
          fs[names[i]]["map_x"] >> normalized_x_[i];
          fs[names[i]]["map_y"] >> normalized_y_[i];
          fs[names[i]]["mask"] >> valid_mask_[i];
          if (normalized_x_[i].size()!=cv::Size(256,256) || normalized_y_[i].size()!=cv::Size(256,256) ||
              normalized_x_[i].type()!=CV_32F || normalized_y_[i].type()!=CV_32F ||
              valid_mask_[i].size()!=cv::Size(256,256) || valid_mask_[i].type()!=CV_8U ||
              !cv::checkRange(normalized_x_[i]) || !cv::checkRange(normalized_y_[i])) continue;
          ready_[i]=true; sizes_[i]=cv::Size();
        }
        calibrated_maps_=true;
        return count()>0;
      }
      if (version != 1) { error_ = "Unsupported calibration version"; return false; }
      const char* names[] = {"front", "rear", "left", "right"};
      for (int i=0; i<4; ++i) {
        auto node = fs[names[i]];
        if (node.empty()) continue;
        std::vector<cv::Point2f> image, ground;
        node["image_points"] >> image; node["ground_points"] >> ground;
        if (image.size()!=4 || ground.size()!=4 || !cv::isContourConvex(image) ||
            !cv::isContourConvex(ground) || std::abs(cv::contourArea(image)) < .001 ||
            std::abs(cv::contourArea(ground)) < .01) continue;
        bool valid = true;
        for (const auto& p:image) valid &= std::isfinite(p.x) && std::isfinite(p.y) && p.x>=0 && p.x<=1 && p.y>=0 && p.y<=1;
        for (auto& p:ground) {
          valid &= std::isfinite(p.x) && std::isfinite(p.y) && std::abs(p.x)<=20 && std::abs(p.y)<=20;
          p = cv::Point2f(320 + p.x*25, 320 - p.y*25);
        }
        if (!valid) continue;
        source_[i]=image;
        homography_[i]=cv::getPerspectiveTransform(image, ground);
        if (!cv::checkRange(homography_[i]) || std::abs(cv::determinant(homography_[i]))<1e-9) continue;
        ready_[i]=true; sizes_[i]=cv::Size();
      }
    } catch (const cv::Exception&) { ready_.fill(false); error_="Invalid calibration"; }
    return count()>0;
  }
  bool approximate() const { return approximate_; }
  int count() const { return std::count(ready_.begin(),ready_.end(),true); }
  const std::string& error() const { return error_; }
  template<size_t N>
  cv::Mat render(const std::array<cv::Mat,N>& frames) {
    cv::Mat total(256,256,CV_32FC3,cv::Scalar()), weights(256,256,CV_32F,cv::Scalar());
    for (size_t i=0;i<std::min(N,size_t(7));++i) {
      if (!ready_[i] || frames[i].empty()) continue;
      if (sizes_[i]!=frames[i].size()) prepare(i,frames[i].size());
      cv::Mat warped, f, channels[3];
      if (calibrated_maps_) cv::remap(frames[i],warped,map_x_[i],map_y_[i],cv::INTER_LINEAR);
      else cv::warpPerspective(frames[i],warped,transforms_[i],cv::Size(256,256),cv::INTER_LINEAR);
      warped.convertTo(f,CV_32FC3); cv::split(f,channels);
      for(auto& c:channels) c=c.mul(feather_[i]);
      cv::merge(channels,3,f); total+=f; weights+=feather_[i];
    }
    cv::Mat out(256,256,CV_8UC3,cv::Scalar(22,29,35));
    for(int y=0;y<256;++y) for(int x=0;x<256;++x) {
      float w=weights.at<float>(y,x);
      if(w>1e-5) out.at<cv::Vec3b>(y,x)=total.at<cv::Vec3f>(y,x)/w;
      else if(((x+y)/12)%2==0) out.at<cv::Vec3b>(y,x)=cv::Vec3b(30,38,45);
    }
    cv::resize(out,out,cv::Size(640,640),0,0,cv::INTER_LINEAR);
    // Top-down body with measured footprint; no change to the ground geometry.
    const int halfw=cvRound(vehicle_width_*pixels_per_metre_*.5);
    const int halfh=cvRound(vehicle_length_*pixels_per_metre_*.5);
    cv::Rect body(320-halfw,320-halfh,2*halfw,2*halfh);
    if (car_asset_.empty() || car_asset_.type()!=CV_8UC4) {
    cv::ellipse(out,{320,322},{halfw+7,halfh+8},0,0,360,{8,12,16},-1,cv::LINE_AA);
    cv::ellipse(out,{320,320},{halfw,halfh},0,0,360,{185,195,205},-1,cv::LINE_AA);
    cv::rectangle(out,cv::Rect(320-halfw+3,320-halfh/2,2*halfw-6,halfh),{155,165,175},-1);
    cv::rectangle(out,cv::Rect(320-halfw+6,320-halfh/2,2*halfw-12,halfh/3),{28,40,50},-1);
    cv::rectangle(out,cv::Rect(320-halfw+6,320+halfh/3,2*halfw-12,halfh/4),{28,40,50},-1);
    cv::line(out,{320-halfw+5,320+halfh-8},{320+halfw-5,320+halfh-8},{70,75,245},3,cv::LINE_AA);
    }
    if (!car_asset_.empty() && car_asset_.type()==CV_8UC4) {
      cv::Mat sprite;
      const int height=std::max(20,2*halfh);
      const int width=std::max(10,cvRound(car_asset_.cols*height/double(car_asset_.rows)));
      cv::resize(car_asset_,sprite,cv::Size(width,height),0,0,cv::INTER_AREA);
      const int left=320-width/2, top=320-height/2;
      for(int y=0;y<height;++y) for(int x=0;x<width;++x) {
        if(left+x<0 || left+x>=out.cols || top+y<0 || top+y>=out.rows) continue;
        const cv::Vec4b p=sprite.at<cv::Vec4b>(y,x);
        cv::Vec3b& dst=out.at<cv::Vec3b>(top+y,left+x);
        for(int c=0;c<3;++c) dst[c]=cv::saturate_cast<uchar>((p[c]*p[3]+dst[c]*(255-p[3]))/255.0);
      }
    }
    cv::putText(out,"FRONT",{290,24},cv::FONT_HERSHEY_SIMPLEX,.6,{230,225,215},1,cv::LINE_AA);
    cv::putText(out,"REAR",{295,622},cv::FONT_HERSHEY_SIMPLEX,.6,{230,225,215},1,cv::LINE_AA);
    cv::putText(out,"LEFT",{12,320},cv::FONT_HERSHEY_SIMPLEX,.6,{230,225,215},1,cv::LINE_AA);
    cv::putText(out,"RIGHT",{565,320},cv::FONT_HERSHEY_SIMPLEX,.6,{230,225,215},1,cv::LINE_AA);
    return out;
  }
 private:
  void prepare(int i, cv::Size size) {
    sizes_[i]=size;
    if (calibrated_maps_) {
      map_x_[i]=normalized_x_[i]*(size.width-1);
      map_y_[i]=normalized_y_[i]*(size.height-1);
      cv::Mat mask=valid_mask_[i].clone(), padded, distance;
      mask.setTo(0,normalized_x_[i]<0); mask.setTo(0,normalized_x_[i]>1);
      mask.setTo(0,normalized_y_[i]<0); mask.setTo(0,normalized_y_[i]>1);
      cv::copyMakeBorder(mask,padded,1,1,1,1,cv::BORDER_CONSTANT,0);
      cv::distanceTransform(padded,distance,cv::DIST_L2,3);
      feather_[i]=distance(cv::Rect(1,1,256,256)).clone();cv::min(feather_[i],8,feather_[i]);
      return;
    }
    cv::Mat scale=(cv::Mat_<double>(3,3)<<1.0/(size.width-1),0,0,0,1.0/(size.height-1),0,0,0,1);
    cv::Mat outputScale=(cv::Mat_<double>(3,3)<<.4,0,0,0,.4,0,0,0,1);
    transforms_[i]=outputScale*homography_[i]*scale;
    cv::Mat sourceMask(size,CV_8U,cv::Scalar()); std::vector<cv::Point> points;
    for(auto p:source_[i]) points.emplace_back(cvRound(p.x*(size.width-1)),cvRound(p.y*(size.height-1)));
    cv::fillConvexPoly(sourceMask,points,cv::Scalar(255));
    cv::Mat mask; cv::warpPerspective(sourceMask,mask,transforms_[i],{256,256},cv::INTER_NEAREST);
    // Pad to guarantee an exterior zero boundary for distanceTransform.
    cv::Mat padded, distance; cv::copyMakeBorder(mask,padded,1,1,1,1,cv::BORDER_CONSTANT,0);
    cv::distanceTransform(padded,distance,cv::DIST_L2,3);
    feather_[i]=distance(cv::Rect(1,1,256,256)).clone(); cv::min(feather_[i],8,feather_[i]);
  }
  std::array<bool,7> ready_{{false,false,false,false,false,false,false}};
  std::array<cv::Size,7> sizes_;
  std::array<std::vector<cv::Point2f>,7> source_;
  std::array<cv::Mat,7> homography_,transforms_,feather_;
  std::string error_;
  cv::Mat car_asset_;
  bool approximate_=false, calibrated_maps_=false;
  double pixels_per_metre_=25, vehicle_width_=2, vehicle_length_=4.72;
  std::array<cv::Mat,7> normalized_x_,normalized_y_,valid_mask_,map_x_,map_y_;
};
}
