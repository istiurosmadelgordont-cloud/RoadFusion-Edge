// 真实预览类/Mat 引用管理；RGA 边界模拟，不连接 FPGA 或模型。
#include "adas/pcie_rga_preview.hpp"
#include <cassert>
#include <iostream>
#include <opencv2/imgproc.hpp>

static bool reject=false;
static unsigned requests=0;
extern "C" rga_buffer_t wrapbuffer_virtualaddr_t(void* address,int w,int h,int ws,int hs,int format) {
  rga_buffer_t buffer{};buffer.vir_addr=address;buffer.width=w;buffer.height=h;
  buffer.wstride=ws;buffer.hstride=hs;buffer.format=format;return buffer;
}
extern "C" const char* imStrError_t(IM_STATUS) {return "injected resize failure";}
extern "C" IM_STATUS imresize_t(rga_buffer_t source,rga_buffer_t target,
    double fx,double fy,int interpolation,int sync) {
  assert(source.format==RK_FORMAT_BGR_888 && target.format==RK_FORMAT_BGR_888);
  assert(fx==0 && fy==0 && interpolation==IM_INTERP_DEFAULT && sync==1);
  const size_t page=static_cast<size_t>(sysconf(_SC_PAGESIZE));
  assert(reinterpret_cast<uintptr_t>(target.vir_addr)%page==0);
  if(source.width==1920) {
    assert(source.height==1080 && source.wstride==1920 && source.hstride==1080);
    assert(target.width==1280 && target.height==720 && target.wstride==1280 && target.hstride==720);
    assert(reinterpret_cast<uintptr_t>(source.vir_addr)%page==0);
  } else {
    assert(source.width==640 && source.height==360 && source.wstride==640 && source.hstride==360);
    assert((target.width==672 && target.height==378) || (target.width==1356 && target.height==762));
    assert(target.wstride==target.width && target.hstride==target.height);
  }
  ++requests;
  if(reject)return IM_STATUS_FAILED;
  cv::Mat input(source.height,source.width,CV_8UC3,source.vir_addr,source.wstride*3);
  cv::Mat output(target.height,target.width,CV_8UC3,target.vir_addr,target.wstride*3);
  cv::resize(input,output,output.size(),0,0,cv::INTER_NEAREST);
  return IM_STATUS_SUCCESS;
}
extern "C" IM_STATUS imcvtcolor_t(rga_buffer_t source,rga_buffer_t target,
    int source_format,int target_format,int mode,int sync) {
  assert(source_format==RK_FORMAT_BGR_888 && target_format==RK_FORMAT_RGB_888);
  assert(source.format==source_format && target.format==target_format && mode==0 && sync==1);
  assert(source.width==target.width && source.height==target.height);
  assert(source.wstride>=source.width && target.wstride>=target.width);
  const size_t page=static_cast<size_t>(sysconf(_SC_PAGESIZE));
  assert(reinterpret_cast<uintptr_t>(target.vir_addr)%page==0);
  cv::Mat input(source.height,source.width,CV_8UC3,source.vir_addr,source.wstride*3);
  cv::Mat output(target.height,target.width,CV_8UC3,target.vir_addr,target.wstride*3);
  cv::cvtColor(input,output,cv::COLOR_BGR2RGB);
  return IM_STATUS_SUCCESS;
}
int main() {
  cv::setNumThreads(1);
  adas::PcieBgrPool inputPool(3,1920,1080);
  cv::Mat input=inputPool.acquire();
  const cv::Scalar colors[]={{10,20,30},{40,50,60},{70,80,90},{100,110,120}};
  for(int view=0;view<4;++view)
    input(cv::Rect((view%2)*960,(view/2)*540,960,540)).setTo(colors[view]);
  const cv::Mat original=input.clone();
  cv::Mat first,second,third,next;
  std::array<cv::Mat,4> views;
  {
    adas::PcieRgaPreview preview;
    assert(preview.resize(input,first));
    adas::PcieRgaPreview::split(first,views);
    for(int view=0;view<4;++view) {
      assert(views[view].size()==cv::Size(640,360));
      assert(views[view].u==first.u && views[view].step==1280U*3);
      const auto color=views[view].at<cv::Vec3b>(180,320);
      assert(color==cv::Vec3b(colors[view][0],colors[view][1],colors[view][2]));
      assert(views[view].clone().isContinuous()); // GLES 仍使用原来的独立绘制副本。
    }
    adas::PcieBgrPool tile_source_pool(3,640,360);
    cv::Mat tile_source=tile_source_pool.acquire();tile_source.setTo(colors[0]);
    cv::Mat tile;
    assert(preview.resize_tile(tile_source,tile));
    assert(tile.size()==cv::Size(672,378));
    assert(tile.at<cv::Vec3b>(189,336)==cv::Vec3b(10,20,30));
    cv::Mat focused, focused2, focused3, focused4;
    assert(preview.resize_tile(tile_source,focused,cv::Size(1356,762)));
    assert(focused.size()==cv::Size(1356,762));
    assert(focused.at<cv::Vec3b>(381,678)==cv::Vec3b(10,20,30));
    assert(preview.resize_tile(tile_source,focused2,cv::Size(1356,762)));
    assert(preview.resize_tile(tile_source,focused3,cv::Size(1356,762)));
    const auto focus_requests=requests;
    assert(!preview.resize_tile(tile_source,focused4,cv::Size(1356,762)) && requests==focus_requests);
    auto focus_memory=focused2.data;focused2.release();
    assert(preview.resize_tile(tile_source,focused4,cv::Size(1356,762)) && focused4.data==focus_memory);
    assert(focused.at<cv::Vec3b>(381,678)==cv::Vec3b(10,20,30));
    adas::PcieBgrPool rgb_pool(3,672,378);
    cv::Mat rgb=rgb_pool.acquire();
    adas::rga_image_ops::bgr_to_rgb(tile,rgb);
    assert(rgb.at<cv::Vec3b>(189,336)==cv::Vec3b(30,20,10));
    // ROI 自己保护预览，即使 first 头部释放，也不能覆盖其像素。
    first.release();assert(preview.resize(input,second));assert(preview.resize(input,third));
    const auto before=requests;assert(!preview.resize(input,next) && requests==before);
    auto reusable=second.data;second.release();
    assert(preview.resize(input,next) && next.data==reusable);
    assert(cv::norm(input,original,cv::NORM_INF)==0); // 不修改识别原图。
    // 绘图只改 clone，不能污染任何池内原图或其它 ROI。
    auto paint=views[0].clone();paint.setTo(cv::Scalar());
    assert(views[0].at<cv::Vec3b>(0,0)==cv::Vec3b(10,20,30));
    for(auto& view:views)view.release();
    reject=true;cv::Mat failed;bool threw=false;
    try{preview.resize(input,failed);}catch(const std::runtime_error&){threw=true;}
    assert(threw && failed.empty());reject=false;
    cv::Mat invalid(720,1280,CV_8UC3);threw=false;
    try{preview.resize(invalid,failed);}catch(const std::runtime_error&){threw=true;}
    assert(threw && failed.empty());
  }
  assert(third.at<cv::Vec3b>(0,0)==cv::Vec3b(10,20,30)); // 池析构后外部图仍有效。
  assert(next.at<cv::Vec3b>(719,1279)==cv::Vec3b(100,110,120));
  std::cout<<"PCIE_RGA_PREVIEW_PASS: sync resize, dimensions, four-view ROI, source unchanged, bounded pool, retained ROI and error handling\n";
}
