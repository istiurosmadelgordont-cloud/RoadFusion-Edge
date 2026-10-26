#include "adas/pcie_image_control.hpp"
#include "adas/image_processing_toolbar.hpp"
#include <atomic>
#include <cassert>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
using namespace adas;
int main(int argc,char** argv) {
  using Clock=ImageTransaction::Clock;
  const auto now=Clock::now();
  ImageParameters params;
  const auto config=params.words();
  assert(config.words[0]==0x1c0 && config.words[1]==0x00f38034);
  for(int i=0;i<10;++i) {
    params.load_preset(i);const auto packed=params.words();__u32 clean[5]{};
    fpga_cfg_sanitize(packed.words,clean);assert(!std::memcmp(clean,packed.words,sizeof(clean)));
    for(auto n:packed.reserved)assert(n==0);
  }
  params.set_value(0,255);assert(params.values[1]==255);
  params.set_value(4,24);assert(params.values[4]==32);
  params.set_value(8,100);assert(params.values[8]==4);
  ImageTransaction state;state.available=state.ack_mode=true;
  assert(state.queue(config));assert(!state.queue(config));state.sent(now);
  state.check_timeout(now+std::chrono::seconds(4));
  assert(state.timed_out && state.busy && !state.queue(config));
  fpga_image_ack_status ack{};ack.cookie=17;
  std::memcpy(ack.words,config.words,sizeof(ack.words));
  ack.flags=FPGA_IMAGE_RECEIVED;state.observe(ack);
  assert(state.busy && state.phase==ImageTransaction::Phase::RECEIVED);
  ack.flags|=FPGA_IMAGE_APPLIED;state.observe(ack);
  assert(!state.busy && state.phase==ImageTransaction::Phase::APPLIED);
  assert(state.message.find("迟到")!=std::string::npos);
  assert(state.queue(config));state.sent(now);ack.cookie=18;ack.words[0]^=1;
  state.observe(ack);assert(state.busy && state.phase==ImageTransaction::Phase::UNCERTAIN);
  ImageTransaction one_way;one_way.available=true;
  assert(one_way.queue(config));one_way.sent(now);
  assert(!one_way.busy && one_way.phase==ImageTransaction::Phase::SUBMITTED);
  ImageProcessingDialog dialog;
  ImageTransaction ready;ready.available=true;
  int submissions=0;
  const auto submit=[&](const fpga_image_submit& c){++submissions;assert(c.reserved[0]==0);return true;};
  dialog.handle_click({30,114},ready,true,submit);assert(submissions==0);
  auto row=ImageProcessingDialog::value_rect(3);
  const int old=dialog.parameters().values[3];
  dialog.handle_click({row.x+275,row.y+5},ready,true,submit);
  assert(dialog.parameters().values[3]==std::min(256,old+1) && submissions==0);
  const auto button=ImageProcessingDialog::apply_rect();
  const cv::Point apply(button.x+5,button.y+5);
  dialog.handle_click(apply,ready,false,submit);assert(submissions==0);
  ready.busy=true;dialog.handle_click(apply,ready,true,submit);assert(submissions==0);
  ready.busy=false;dialog.handle_click(apply,ready,true,submit);assert(submissions==1);
  // 快捷预设与详细窗口共用草稿，点击选择或恢复默认绝不能隐式发参数。
  ImageProcessingDialog shared;
  ImageProcessingToolbar toolbar(shared);
  int quick_submissions=0;fpga_image_submit quick_actual{};
  const auto quick_submit=[&](const fpga_image_submit& c){++quick_submissions;quick_actual=c;return true;};
  const auto center=[](cv::Rect r){return cv::Point(r.x+r.width/2,r.y+r.height/2);};
  using Click=ImageProcessingToolbar::Click;
  for(int i=0;i<10;++i) {
    assert(toolbar.click(center(toolbar.select_rect()),ready,true,quick_submit)==Click::CONSUMED);
    assert(toolbar.expanded());
    assert(toolbar.click(center(toolbar.preset_rect(i)),ready,true,quick_submit)==Click::CONSUMED);
    assert(!toolbar.expanded() && shared.parameters().preset==i && quick_submissions==0);
  }
  toolbar.click(center(toolbar.apply_rect()),ready,true,quick_submit);
  assert(quick_submissions==1);
  auto selected_words=shared.parameters().words();
  assert(!std::memcmp(quick_actual.words,selected_words.words,sizeof(quick_actual.words)));
  toolbar.click(center(toolbar.reset_rect()),ready,true,quick_submit);
  assert(shared.parameters().preset==0 && quick_submissions==1);
  toolbar.click(center(toolbar.apply_rect()),ready,true,quick_submit);
  assert(quick_submissions==2 && quick_actual.words[0]==0x1c0);
  // 在详细窗口修改后，快捷应用发出的也必须是修改后的同一份参数。
  const auto custom_row=shared.value_rect(2);
  shared.handle_click({custom_row.x+275,custom_row.y+5},ready,true,quick_submit);
  assert(shared.parameters().preset==-1 && quick_submissions==2);
  toolbar.click(center(toolbar.apply_rect()),ready,true,quick_submit);
  selected_words=shared.parameters().words();
  assert(quick_submissions==3 && !std::memcmp(quick_actual.words,selected_words.words,sizeof(quick_actual.words)));
  ready.busy=true;toolbar.click(center(toolbar.apply_rect()),ready,true,quick_submit);
  ready.busy=false;toolbar.click(center(toolbar.apply_rect()),ready,false,quick_submit);
  ready.available=false;toolbar.click(center(toolbar.apply_rect()),ready,true,quick_submit);
  assert(quick_submissions==3);ready.available=true;
  // 展开菜单、选择“详细处理”、点外侧及按住拖动均不得穿透到其他按钮。
  MomentaryInput quick_input;int x=-1,y=-1;
  const auto route=[&](cv::Point p,bool release) {
    quick_input.press(p.x,p.y);if(release)quick_input.release();x=p.x;y=p.y;
    auto snapshot=quick_input.take();
    const bool details=toolbar.route_input(snapshot,x,y,ready,true,quick_submit);
    assert(snapshot.clicks.empty() && snapshot.click==cv::Point(-1,-1) && x==-1 && y==-1);
    return details;
  };
  assert(!route(center(toolbar.select_rect()),true) && toolbar.expanded());
  assert(route(center(toolbar.details_rect()),true) && !toolbar.expanded());
  assert(!shared.visible()); // 路由只请求打开，回归检查不需要桌面窗口。
  route(center(toolbar.select_rect()),true);
  assert(!route({1500,500},true) && !toolbar.expanded());
  route(center(toolbar.select_rect()),false);
  quick_input.move(1450,900,true);auto held=quick_input.take();x=1450;y=900;
  toolbar.route_input(held,x,y,ready,true,quick_submit);
  assert(!held.down && x==-1 && y==-1);quick_input.release();toolbar.dismiss();
  auto released=quick_input.take();toolbar.route_input(released,x,y,ready,true,quick_submit);
  quick_input.press(450,920);quick_input.release();auto other=quick_input.take();x=450;y=920;
  toolbar.route_input(other,x,y,ready,true,quick_submit);
  assert(other.clicks.size()==1 && x==450 && y==920); // 正常点击仍交给原逻辑。
  assert(!toolbar.dismiss());toolbar.click(center(toolbar.select_rect()),ready,true,quick_submit);
  assert(toolbar.dismiss() && !toolbar.expanded());
  // 菜单收起时恢复原侧栏，不能留下文字或边框残影。
  cv::Mat toolbar_canvas(1080,1920,CV_8UC3,cv::Scalar(24,15,7));
  toolbar.draw(toolbar_canvas,ready,true);const auto original=toolbar_canvas.clone();
  toolbar.click(center(toolbar.select_rect()),ready,true,quick_submit);
  toolbar.draw(toolbar_canvas,ready,true);
  toolbar.restore_overlay(toolbar_canvas);toolbar.dismiss();toolbar.draw(toolbar_canvas,ready,true);
  assert(cv::norm(toolbar_canvas,original,cv::NORM_INF)==0);
  // 同一个控制线程先 PREPARE 再 SUBMIT；最多一个在途事务，不自动重发。
  PcieImageControl controller;
  std::atomic<int> prepared{0},submitted{0},queries{0},flags{0};
  fpga_image_submit actual{};
  controller.attach_transport([&](unsigned long cmd,void* data) {
    if(cmd==FPGA_IOC_GET_CAPS) {
      auto* caps=static_cast<fpga_caps*>(data);*caps={};
      caps->version=1;caps->flags=FPGA_CAP_IMAGE_ACK|FPGA_CAP_IMAGE_SUBMIT;return 0;
    }
    if(cmd==FPGA_IOC_PREPARE_IMAGE_ACK){++prepared;return 0;}
    if(cmd==FPGA_IOC_SUBMIT_IMAGE_ACK){assert(prepared==1);actual=*static_cast<fpga_image_submit*>(data);++submitted;return 0;}
    if(cmd==FPGA_IOC_GET_IMAGE_ACK) {
      assert(submitted==1);auto* status=static_cast<fpga_image_ack_status*>(data);*status={};
      status->cookie=32;status->flags=flags.load();
      std::memcpy(status->words,actual.words,sizeof(status->words));++queries;return 0;
    }
    assert(false);return -1;
  });
  assert(controller.requires_stop() && prepared==1 && submitted==0);
  assert(controller.submit(config));assert(!controller.submit(config));
  for(int i=0;i<100 && queries==0;++i)std::this_thread::sleep_for(std::chrono::milliseconds(5));
  assert(submitted==1 && queries>0 && controller.snapshot().busy);
  flags=FPGA_IMAGE_RECEIVED;
  for(int i=0;i<100 && controller.snapshot().phase!=ImageTransaction::Phase::RECEIVED;++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  assert(controller.snapshot().busy);
  flags=FPGA_IMAGE_RECEIVED|FPGA_IMAGE_APPLIED;
  for(int i=0;i<100 && controller.snapshot().busy;++i)std::this_thread::sleep_for(std::chrono::milliseconds(5));
  assert(!controller.snapshot().busy && submitted==1);controller.stop();
  assert(!controller.submit(config));
  // 无参数 ioctl 的旧驱动不阻断视频；PREPARE 失败不能假报可提交。
  controller.attach_transport([](unsigned long,void*){errno=ENOTTY;return -1;});
  assert(!controller.snapshot().available);controller.stop();
  controller.attach_transport([](unsigned long cmd,void* data){
    if(cmd==FPGA_IOC_GET_CAPS){auto* c=static_cast<fpga_caps*>(data);*c={};c->version=1;c->flags=3;return 0;}
    errno=EBUSY;return -1;
  });
  assert(!controller.snapshot().available && controller.requires_stop());controller.stop();
  if(argc>1) {
    assert(ui::initialize_text("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc") ||
           ui::initialize_text("/mnt/c/Windows/Fonts/msyh.ttc"));
    assert(cv::imwrite(std::string(argv[1])+"/image-controls.png",dialog.render(ready,true)));
    assert(cv::imwrite(std::string(argv[1])+"/image-controls-timeout.png",dialog.render(state,true)));
    // 绘图缓存应在字体初始化后创建，与主程序的初始化顺序保持一致。
    shared.select_preset(4);
    ImageProcessingToolbar preview_toolbar(shared);
    toolbar.restore_overlay(toolbar_canvas);preview_toolbar.draw(toolbar_canvas,ready,true);
    assert(cv::imwrite(std::string(argv[1])+"/image-quick-toolbar.png",toolbar_canvas));
    preview_toolbar.click(center(toolbar.select_rect()),ready,true,quick_submit);
    preview_toolbar.draw(toolbar_canvas,ready,true);
    assert(cv::imwrite(std::string(argv[1])+"/image-quick-menu.png",toolbar_canvas));
  }
  std::cout<<"IMAGE_TOOLBAR_PASS: presets/detail share draft, explicit apply, default reset, busy/replay gates, input isolation, overlay restore\n";
  std::cout<<"IMAGE_CONTROL_REGRESSION_PASS: packing, UI apply gate, received/applied, late ACK, no overwrite, capabilities\n";
}

