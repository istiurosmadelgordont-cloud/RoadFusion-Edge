#pragma once
#include "adas/image_processing_dialog.hpp"

namespace adas {
// 主界面快捷入口。预设、详细窗口、恢复默认、应用设置共用 dialog 的草稿。
// 菜单绘制在侧栏上方；保存底图并在下一帧恢复，不要求整块侧栏每帧重画。
class ImageProcessingToolbar {
 public:
  explicit ImageProcessingToolbar(ImageProcessingDialog& dialog):dialog_(dialog){}
  // 为板端 Xfce 顶栏留出 30 像素，控件和提示仍位于原侧栏标题上方。
  static cv::Rect header_rect(){return {1412,0,508,84};}
  static cv::Rect select_rect(){return {1418,30,230,30};}
  static cv::Rect reset_rect(){return {1656,30,112,30};}
  static cv::Rect apply_rect(){return {1776,30,128,30};}
  static cv::Rect menu_rect(){return {1418,66,486,380};}
  static cv::Rect preset_rect(int i){return {1426,96+i*30,470,29};}
  static cv::Rect details_rect(){return {1426,404,470,34};}
  bool expanded() const{return expanded_;}
  bool dismiss(){const bool was=expanded_;expanded_=false;return was;}

  enum class Click { PASS, CONSUMED, DETAILS };
  Click click(cv::Point point,const ImageTransaction& status,bool live,
              const ImageProcessingDialog::Submit& submit) {
    if(select_rect().contains(point)) {
      expanded_=!expanded_;return Click::CONSUMED;
    }
    if(reset_rect().contains(point)) {
      expanded_=false;dialog_.restore_defaults();return Click::CONSUMED;
    }
    if(apply_rect().contains(point)) {
      expanded_=false;dialog_.apply(status,live,submit);return Click::CONSUMED;
    }
    if(!expanded_)return Click::PASS;
    for(int i=0;i<10;++i)if(preset_rect(i).contains(point)) {
      dialog_.select_preset(i);expanded_=false;return Click::CONSUMED;
    }
    if(details_rect().contains(point)){expanded_=false;return Click::DETAILS;}
    // 点击菜单外只收起，不能同时触发被覆盖的车辆/导航按钮。
    if(!menu_rect().contains(point))expanded_=false;
    return Click::CONSUMED;
  }

  // 同时过滤按下事件队列及旧版拖动坐标，防止一次点击穿透到原侧栏。
  // 返回 true 仅表示请求打开详细窗口，便于在无图形环境中测试路由。
  bool route_input(MomentaryInput::Snapshot& input,int& click_x,int& click_y,
                   const ImageTransaction& status,bool live,
                   const ImageProcessingDialog::Submit& submit) {
    const bool physically_down=input.down;
    bool details=false;
    std::vector<cv::Point> remaining;
    for(const auto& point:input.clicks) {
      const auto action=click(point,status,live,submit);
      owns_press_=action!=Click::PASS;
      if(action==Click::PASS)remaining.push_back(point);
      else {
        if(action==Click::DETAILS)details=true;
        if(input.click==point)input.click={-1,-1};
        if(cv::Point(click_x,click_y)==point)click_x=click_y=-1;
      }
    }
    input.clicks.swap(remaining);
    if(owns_press_) {
      input.down=false;input.origin=input.pointer=input.click={-1,-1};
      click_x=click_y=-1;
    }
    if(!physically_down)owns_press_=false;
    return details;
  }

  void restore_overlay(cv::Mat& canvas) {
    if(!underlay_.empty()) {
      underlay_.copyTo(canvas(menu_rect()));underlay_.release();
    }
  }
  void draw(cv::Mat& canvas,const ImageTransaction& status,bool live) {
    const bool ready=live && status.available && !status.busy;
    const auto selected=std::string(ImageParameters::preset_name(dialog_.parameters().preset));
    const auto message=std::string(live?"PCIe | ":"视频回放 | ")+dialog_.feedback(status,live);
    const auto key=selected+message+(expanded_?"/open":"/closed")+(ready?"/ready":"/disabled");
    if(header_.empty() || key!=header_key_) {
      header_=cv::Mat(header_rect().size(),CV_8UC3,cv::Scalar(37,24,10));header_key_=key;
      const auto r=select_rect()-header_rect().tl();
      button(header_,r,"",normal());
      label(header_,{r.x,r.y,r.width-30,r.height},
            std::string("图像处理：")+short_name(dialog_.parameters().preset),18,normal());
      const cv::Point arrow(r.x+r.width-19,r.y+18);
      cv::line(header_,{arrow.x-5,arrow.y-3},{arrow.x,arrow.y+3},normal(),1);
      cv::line(header_,{arrow.x,arrow.y+3},{arrow.x+5,arrow.y-3},normal(),1);
      button(header_,reset_rect()-header_rect().tl(),"恢复默认",normal());
      button(header_,apply_rect()-header_rect().tl(),"应用设置",ready?green():muted());
      label(header_,{6,64,490,19},message,14,status.busy?amber():muted());
    }
    header_.copyTo(canvas(header_rect()));
    if(!expanded_)return;
    canvas(menu_rect()).copyTo(underlay_);
    const int selected_index=dialog_.parameters().preset;
    if(menu_.empty() || menu_selection_!=selected_index) {
      menu_selection_=selected_index;
      menu_=cv::Mat(menu_rect().size(),CV_8UC3,cv::Scalar(32,30,25));
      cv::rectangle(menu_,{0,0,menu_.cols,menu_.rows},muted(),1);
      label(menu_,{10,4,466,24},"选择预设（点击应用设置后生效）",16,muted());
      for(int i=0;i<10;++i) {
        const auto row=preset_rect(i)-menu_rect().tl();
        if(i==selected_index)cv::rectangle(menu_,row,cv::Scalar(45,63,35),cv::FILLED);
        label(menu_,row,std::string(i==selected_index?"> ":"   ")+ImageParameters::preset_name(i),
              18,i==selected_index?green():normal());
      }
      button(menu_,details_rect()-menu_rect().tl(),"详细处理...",normal());
    }
    menu_.copyTo(canvas(menu_rect()));
  }
 private:
  static const char* short_name(int preset) {
    static const char* names[]={"全旁路","温和对比","背光提亮","仅去雾","雾天增强",
        "夜间/隧道","轻度降噪","暗光增强","高光压制","暗光+高光"};
    return preset>=0 && preset<10?names[preset]:"自定义";
  }
  static cv::Scalar normal(){return {220,220,220};}
  static cv::Scalar muted(){return {160,160,160};}
  static cv::Scalar green(){return {110,225,100};}
  static cv::Scalar amber(){return {80,205,240};}
  static void label(cv::Mat& image,cv::Rect row,const std::string& text,int size,cv::Scalar color) {
    // 用 ROI 限制长回执提示的绘制范围，避免盖到相邻按钮。
    cv::Mat area=image(row);
    while(size>12 && ui::measure_text(text,size).width>row.width-12)--size;
    ui::draw_text(area,text,{6,(row.height+size)/2-2},size,color);
  }
  static void button(cv::Mat& image,cv::Rect row,const std::string& text,cv::Scalar color) {
    cv::rectangle(image,row,cv::Scalar(39,34,26),cv::FILLED);
    cv::rectangle(image,row,color,1);label(image,row,text,18,color);
  }
  ImageProcessingDialog& dialog_;
  bool expanded_=false,owns_press_=false;
  int menu_selection_=-2;
  cv::Mat underlay_,header_,menu_;
  std::string header_key_;
};
} // namespace adas
