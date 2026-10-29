#pragma once
#include "adas/image_parameters.hpp"
#include "adas/dashboard_style.hpp"
#include "adas/image_transaction.hpp"
#include "adas/text_renderer.hpp"
#include "adas/momentary_input.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc.hpp>
#include <functional>

namespace adas {
// 独立参数窗口，复用 OpenCV/FreeType；主界面在窗口打开时屏蔽驾驶操作。
// 鼠标回调只排队，UI 线程绘制，参数 ioctl 由 PcieImageControl 的线程执行。
class ImageProcessingDialog {
 public:
  using Submit=std::function<bool(const fpga_image_submit&)>;
  ~ImageProcessingDialog(){ close(); }
  static const char* name(){ return "FPGA Image Processing"; }
  static int width(){return 960;}
  static int height(){return 824;}
  static cv::Rect check_rect(int i){return {24+(i%4)*232,106+(i/4)*32,222,28};}
  static cv::Rect value_rect(int i){return {24+(i%2)*464,232+(i/2)*42,440,34};}
  static cv::Rect apply_rect(){return {24,540,172,38};}
  static cv::Rect bypass_rect(){return {212,540,196,38};}
  static cv::Rect close_rect(){return {780,540,156,38};}
  static cv::Rect previous_rect(){return {24,57,72,32};}
  static cv::Rect next_rect(){return {656,57,72,32};}
  static cv::Rect reset_rect(){return {746,57,190,32};}
  // 主界面快捷菜单与详细窗口共用这一份草稿，不各自保存一套参数。
  // 选择预设/恢复默认只编辑；显式应用时才交给控制线程发送。
  void select_preset(int index) {
    if(index<0 || index>=10)return;
    params_.load_preset(index); dirty_=true;
    local_message_=std::string("已选择：")+ImageParameters::preset_name(index)+"；点击应用设置生效";
  }
  void restore_defaults(){ select_preset(0); }
  bool apply(const ImageTransaction& status,bool live,const Submit& submit) {
    dirty_=true;
    if(!live){local_message_="视频回放不发送 FPGA 参数，请使用 PCIe 实时输入";return false;}
    if(!status.available){local_message_=status.message;return false;}
    if(status.busy){local_message_="上次参数仍在等待确认，不可覆盖；可继续编辑";return false;}
    const bool queued=submit(params_.words());
    local_message_=queued?"参数已排队；编辑值不会自动发送":"当前无法提交，请查看状态后重试";
    return queued;
  }
  std::string feedback(const ImageTransaction& status,bool live) const {
    if(!live)return "当前为视频回放，仅 PCIe 实时输入支持 FPGA 调参";
    // 详细窗口关闭时也会在主循环中 tick，以便清除已被新回执替代的提示。
    return local_message_.empty()?status.message:local_message_;
  }
  bool visible() const {return visible_;}
  void open() {
    if(visible_)return;
    cv::namedWindow(name(),cv::WINDOW_AUTOSIZE);
    // GTK 会先按空窗口选择位置，再随首帧放大，容易把底部移出屏幕。
    // 给参数窗口设置初始位置，确保本机 1080p 界面能完整显示日志区。
    cv::moveWindow(name(),480,120);
    cv::setMouseCallback(name(),mouse,this);
    visible_=true; dirty_=true; input_.cancel();
  }
  void close() noexcept {
    if(!visible_)return;
    visible_=false; input_.cancel();
    // 用户点系统关闭按钮后，部分 HighGUI 后端已提前销毁了窗口。
    try { cv::setMouseCallback(name(),nullptr,nullptr); cv::destroyWindow(name()); }
    catch(const cv::Exception&) {}
  }
  bool tick(const ImageTransaction& status,bool live,const Submit& submit) {
    if(status.message!=last_status_message_)local_message_.clear();
    last_status_message_=status.message;
    if(!visible_)return false;
    // 板子使用 OpenCV 3.2 的 GTK 后端，WND_PROP_VISIBLE 不受支持，
    // 即使窗口正常存在也返回 -1，不能据此判断用户关闭了窗口。
    // 改为检查 HighGUI 登记的原生窗口句柄：创建后非空，销毁后为空；
    // 首次 imshow() 前也有效，避免弹窗还没画出来就被误关闭。
    try { if(cvGetWindowHandle(name())==nullptr){close();return false;} }
    catch(const cv::Exception&) { close();return false; }
    const auto input=input_.take();
    for(const auto& point:input.clicks)handle_click(point,status,live,submit);
    if(input.down)for(int i=0;i<13;++i){
      const auto row=value_rect(i);
      if(row.contains(input.origin) && input.origin.x>=row.x+308){
        const cv::Point point(std::max(row.x+308,std::min(row.x+438,input.pointer.x)),row.y+16);
        handle_click(point,status,live,submit);break;
      }
    }
    if(!visible_)return false;
    const auto now=std::chrono::steady_clock::now();
    if(dirty_ || status.message!=last_message_ || now-last_draw_>std::chrono::milliseconds(250)) {
      cv::imshow(name(),render(status,live));
      dirty_=false;last_message_=status.message;last_draw_=now;
    }
    return true;
  }
  void handle_click(cv::Point point,const ImageTransaction& status,bool live,const Submit& submit) {
    dirty_=true; local_message_.clear();
    if(close_rect().contains(point)){close();return;}
    if(previous_rect().contains(point))select_preset(params_.preset<0?0:(params_.preset+9)%10);
    if(next_rect().contains(point))select_preset(params_.preset<0?0:(params_.preset+1)%10);
    if(reset_rect().contains(point))restore_defaults();
    for(int i=0;i<11;++i)if(check_rect(i).contains(point)) {
      params_.switches[i]=!params_.switches[i];params_.preset=-1;
    }
    for(int i=0;i<13;++i) {
      const auto row=value_rect(i); const int x=point.x-row.x;
      if(!row.contains(point))continue;
      const int step=i==4?16:i==7?10:1;
      if(x>=136 && x<166)params_.set_value(i,params_.values[i]-step);
      else if(x>=268 && x<298)params_.set_value(i,params_.values[i]+step);
      else if(x>=308) {
        const double ratio=std::max(0.,std::min(1.,(x-308)/130.));
        params_.set_value(i,cvRound(ImageParameters::minimum(i)+ratio*
                            (ImageParameters::maximum(i)-ImageParameters::minimum(i))));
      }
    }
    const bool bypass=bypass_rect().contains(point);
    if(!apply_rect().contains(point) && !bypass)return;
    // 旁路按钮保持原有的“恢复并应用”行为，也复用同一提交入口。
    if(bypass && (!live || !status.available || status.busy)){apply(status,live,submit);return;}
    if(bypass)params_.load_preset(0);
    apply(status,live,submit);
  }
  cv::Mat render(const ImageTransaction& status,bool live) const {
    cv::Mat image(height(),width(),CV_8UC3,dashboard::panel());
    const cv::Scalar normal=dashboard::text(),muted=dashboard::muted(),green=dashboard::accent(),amber(80,205,240);
    text(image,"FPGA 图像处理",24,34,24,normal);
    text(image,"参数作用于整张四宫格图像，不改变识别与报警规则",280,34,17,muted);
    button(image,previous_rect(),"上一项",normal);
    text(image,std::string("预设：")+ImageParameters::preset_name(params_.preset),112,79,20,green);
    button(image,next_rect(),"下一项",normal);button(image,reset_rect(),"恢复默认（仅编辑）",normal);
    for(int i=0;i<11;++i) {
      const auto r=check_rect(i); const bool on=params_.switches[i];
      dashboard::control(image,r,"",on);
      text(image,ImageParameters::switch_name(i),r.x+9,r.y+20,16,on?green:muted);
      cv::Rect check(r.br().x-27,r.y+7,14,14);
      cv::rectangle(image,check,on?green:muted,on?cv::FILLED:1);
      if(on){cv::line(image,{check.x+2,check.y+7},{check.x+6,check.y+11},dashboard::bg(),2);
        cv::line(image,{check.x+6,check.y+11},{check.x+12,check.y+2},dashboard::bg(),2);}
    }
    text(image,"数值：点击 - / + 微调，或点击横条设置；sigma 仅 16 / 32 / 48",24,221,17,muted);
    for(int i=0;i<13;++i) {
      const auto r=value_rect(i);
      text(image,ImageParameters::value_name(i),r.x,r.y+23,17,normal);
      button(image,{r.x+136,r.y,30,32},"-",normal);
      text(image,std::to_string(params_.values[i]),r.x+178,r.y+23,18,green);
      button(image,{r.x+268,r.y,30,32},"+",normal);
      cv::rectangle(image,{r.x+308,r.y+13,130,7},cv::Scalar(75,75,75),cv::FILLED);
      const float ratio=float(params_.values[i]-ImageParameters::minimum(i))/
          (ImageParameters::maximum(i)-ImageParameters::minimum(i));
      cv::circle(image,{r.x+308+cvRound(ratio*130),r.y+16},6,green,cv::FILLED);
    }
    const bool ready=live && status.available && !status.busy;
    button(image,apply_rect(),"应用当前参数",ready?green:muted);
    button(image,bypass_rect(),"恢复旁路并应用",ready?amber:muted);
    button(image,close_rect(),"关闭窗口",normal);
    text(image,"关闭窗口不撤销已提交参数；生效回执不等同于已显示对应视频帧。",24,604,17,muted);
    const std::string message=feedback(status,live);
    text(image,message,24,634,18,status.busy?amber:green);
    text(image,"控制日志",24,667,18,normal);
    // 显示最近六条，长句缩小到窗口可用宽度，避免盖住边界。
    const size_t begin=status.logs.size()>6?status.logs.size()-6:0;
    int y=693;
    for(size_t i=begin;i<status.logs.size();++i,y+=23)text(image,status.logs[i],24,y,16,muted);
    return image;
  }
  const ImageParameters& parameters() const{return params_;}
 private:
  static void text(cv::Mat& image,const std::string& s,int x,int y,int size,const cv::Scalar& color) {
    while(size>11 && ui::measure_text(s,size).width>width()-x-24)--size;
    ui::draw_text(image,s,{x,y},size,color);
  }
  static void button(cv::Mat& image,const cv::Rect& r,const std::string& label,const cv::Scalar& color) {
    dashboard::control(image,r,label,color==dashboard::accent(),16);
  }
  static void mouse(int event,int x,int y,int flags,void* data) {
    auto& self=*static_cast<ImageProcessingDialog*>(data);
    if(event==cv::EVENT_LBUTTONDOWN)self.input_.press(x,y);
    if(event==cv::EVENT_LBUTTONUP)self.input_.release();
    if(event==cv::EVENT_MOUSEMOVE)self.input_.move(x,y,(flags & cv::EVENT_FLAG_LBUTTON)!=0);
  }
  ImageParameters params_;
  MomentaryInput input_;
  bool visible_=false,dirty_=true;
  std::string local_message_,last_message_,last_status_message_;
  std::chrono::steady_clock::time_point last_draw_{};
};
} // namespace adas

