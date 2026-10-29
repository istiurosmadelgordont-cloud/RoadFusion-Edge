// 真实 RGA 色彩回归：只用内存测试图，不打开 PCIe、不修改 FPGA 参数。
// 普通 mock 无法验证硬件格式枚举对应的真实颜色，因此保留此独立检查。
#include "adas/pcie_rga_preview.hpp"
#include "adas/rga_image_ops.hpp"
#include <opencv2/imgproc.hpp>
#include <cstdlib>
#include <iostream>
#include <memory>

int main() {
  try {
    cv::setNumThreads(1);
    constexpr int width=1920,height=1080;
    void* memory=nullptr;
    if(posix_memalign(&memory,4096,width*height*2))
      throw std::runtime_error("Cannot allocate page-aligned RGB565 test image");
    std::unique_ptr<void,decltype(&free)> owner(memory,&free);
    cv::Mat raw(height,width,CV_8UC2,memory);
    // 低字节在前，遍历所有 16 位像素值，同时覆盖红/绿/蓝/灰及混合颜色。
    for(int y=0;y<height;++y)for(int x=0;x<width;++x) {
      const unsigned value=(y*width+x)&65535U;
      raw.ptr<unsigned char>(y)[x*2]=value&255;
      raw.ptr<unsigned char>(y)[x*2+1]=value>>8;
    }
    const cv::Mat original=raw.clone();
    cv::Mat reference;
    cv::cvtColor(raw,reference,cv::COLOR_BGR5652BGR);
    adas::PcieBgrPool pool(3,width,height);
    cv::Mat converted=pool.acquire();
    const auto source=wrapbuffer_virtualaddr_t(memory,width,height,width,height,
        adas::kPcieRgb565RgaFormat);
    const auto target=wrapbuffer_virtualaddr_t(converted.data,width,height,width,height,
        RK_FORMAT_BGR_888);
    const auto status=imcvtcolor_t(source,target,adas::kPcieRgb565RgaFormat,
        RK_FORMAT_BGR_888,0,1);
    if(status!=IM_STATUS_SUCCESS)throw std::runtime_error(imStrError_t(status));
    const double error=cv::norm(reference,converted,cv::NORM_INF);
    std::cout<<"RGA_RGB565_ALL_65536_COLORS max_channel_error="<<error<<std::endl;
    if(error!=0 || cv::norm(original,raw,cv::NORM_INF)!=0)return 2;
    cv::Mat rgb=pool.acquire(),reference_rgb;
    adas::rga_image_ops::bgr_to_rgb(converted,rgb);
    cv::cvtColor(converted,reference_rgb,cv::COLOR_BGR2RGB);
    const double rgb_error=cv::norm(reference_rgb,rgb,cv::NORM_INF);
    std::cout<<"RGA_BGR_TO_RGB max_channel_error="<<rgb_error<<std::endl;
    if(rgb_error!=0)return 5;
    // 缩放前后再验证纯色中心值，排除预览阶段再次交换通道。
    const cv::Scalar colors[]={{0,0,248},{0,252,0},{248,0,0},{128,128,128}};
    for(int i=0;i<4;++i)
      converted(cv::Rect((i%2)*960,(i/2)*540,960,540)).setTo(colors[i]);
    // Production UFLD consumes a strided 960x540 ROI from the 1920x1080
    // capture. Verify RGA accepts that source stride without a CPU resize.
    adas::PcieBgrPool resize_pool(3,1600,533);
    cv::Mat resize_output=resize_pool.acquire();
    const cv::Mat source_roi=converted(cv::Rect(960,0,960,540));
    cv::Mat resize_target=resize_output(cv::Rect(0,0,1600,533));
    adas::rga_image_ops::resize_bgr(source_roi,resize_target);
    const auto resized_center=resize_output.at<cv::Vec3b>(266,800);
    for(int c=0;c<3;++c)if(resized_center[c]!=colors[1][c])return 6;
    adas::PcieRgaPreview preview;
    cv::Mat small;
    if(!preview.resize(converted,small))return 3;
    for(int i=0;i<4;++i) {
      const auto value=small.at<cv::Vec3b>((i/2)*360+180,(i%2)*640+320);
      for(int c=0;c<3;++c)if(value[c]!=colors[i][c])return 4;
    }
    for (const cv::Size size : {cv::Size(672,378),cv::Size(1356,762),cv::Size(656,369),cv::Size(1328,747),cv::Size(960,540),cv::Size(432,243)}) {
      cv::Mat displayed;
      if(!preview.resize_tile(small(cv::Rect(640,0,640,360)),displayed,size))return 7;
      if(displayed.size()!=size)return 8;
      const auto center=displayed.at<cv::Vec3b>(size.height/2,size.width/2);
      for(int c=0;c<3;++c)if(center[c]!=colors[1][c])return 9;
    }
    std::cout<<"RGA_COLOR_HARDWARE_PASS: RGB565 conversion, BGR/RGB conversion and preview channel order\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<std::endl;return 1;}
}
