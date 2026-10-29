#pragma once
#include <opencv2/core.hpp>
#include <thread>
#include <exception>
#if defined(__aarch64__)
#include <arm_neon.h>
#include <pthread.h>
#endif

namespace adas {
// Exact 3:2 bilinear sampling: source positions are .25 and 1.75 in
// each three-pixel block. Integer weights avoid generic resize overhead
// in the board's old OpenCV build. The native inference image is untouched.
inline void resize_preview_impl(const cv::Mat& source, cv::Mat& target,bool vectorized) {
  CV_Assert(source.type() == CV_8UC3 && source.cols % 3 == 0 && source.rows % 3 == 0);
  target.create(source.rows * 2 / 3, source.cols * 2 / 3, CV_8UC3);
  for (int y = 0; y < source.rows / 3; ++y) {
    const unsigned char* a = source.ptr<unsigned char>(y * 3);
    const unsigned char* b = source.ptr<unsigned char>(y * 3 + 1);
    const unsigned char* c = source.ptr<unsigned char>(y * 3 + 2);
    unsigned char* top = target.ptr<unsigned char>(y * 2);
    unsigned char* bottom = target.ptr<unsigned char>(y * 2 + 1);
    int first_group=0;
#if defined(__aarch64__)
    if(vectorized) {
      const uint8_t gather_values[16]={0,1,3,4,6,7,9,10,12,13,15,16,18,19,21,22};
      const uint8_t wa_values[16]={9,3,9,3,9,3,9,3,9,3,9,3,9,3,9,3};
      const uint8_t wb_values[16]={3,9,3,9,3,9,3,9,3,9,3,9,3,9,3,9};
      const uint8_t wc_values[16]={3,1,3,1,3,1,3,1,3,1,3,1,3,1,3,1};
      const uint8_t wd_values[16]={1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3};
      const auto ix=vld1q_u8(gather_values),iy=vaddq_u8(ix,vdupq_n_u8(1));
      const auto wa=vld1q_u8(wa_values),wb=vld1q_u8(wb_values),wc=vld1q_u8(wc_values),wd=vld1q_u8(wd_values);
      auto blend=[](uint8x16_t ax,uint8x16_t ay,uint8x16_t bx,uint8x16_t by,
                    uint8x16_t aweight,uint8x16_t bweight,uint8x16_t cweight,uint8x16_t dweight){
        auto lo=vmull_u8(vget_low_u8(ax),vget_low_u8(aweight));
        lo=vmlal_u8(lo,vget_low_u8(ay),vget_low_u8(bweight));
        lo=vmlal_u8(lo,vget_low_u8(bx),vget_low_u8(cweight));
        lo=vmlal_u8(lo,vget_low_u8(by),vget_low_u8(dweight));
        auto hi=vmull_u8(vget_high_u8(ax),vget_high_u8(aweight));
        hi=vmlal_u8(hi,vget_high_u8(ay),vget_high_u8(bweight));
        hi=vmlal_u8(hi,vget_high_u8(bx),vget_high_u8(cweight));
        hi=vmlal_u8(hi,vget_high_u8(by),vget_high_u8(dweight));
        return vcombine_u8(vrshrn_n_u16(lo,4),vrshrn_n_u16(hi,4));
      };
      // Load 32 source pixels, consume 24, emit 16. The tail stays scalar so
      // no vector read crosses an ROI row or the end of the source storage.
      for(int x=0;x+32<=source.cols;x+=24,first_group+=8) {
        const auto a0=vld3q_u8(a+x*3),a1=vld3q_u8(a+x*3+48);
        const auto b0=vld3q_u8(b+x*3),b1=vld3q_u8(b+x*3+48);
        const auto c0=vld3q_u8(c+x*3),c1=vld3q_u8(c+x*3+48);
        uint8x16x3_t upper,lower;
        for(int channel=0;channel<3;++channel) {
          const uint8x16x2_t at={{a0.val[channel],a1.val[channel]}},
                              bt={{b0.val[channel],b1.val[channel]}},
                              ct={{c0.val[channel],c1.val[channel]}};
          const auto ax=vqtbl2q_u8(at,ix),ay=vqtbl2q_u8(at,iy),
                     bx=vqtbl2q_u8(bt,ix),by=vqtbl2q_u8(bt,iy),
                     cx=vqtbl2q_u8(ct,ix),cy=vqtbl2q_u8(ct,iy);
          upper.val[channel]=blend(ax,ay,bx,by,wa,wb,wc,wd);
          lower.val[channel]=blend(bx,by,cx,cy,wc,wd,wa,wb);
        }
        vst3q_u8(top+x*2,upper);vst3q_u8(bottom+x*2,lower);
      }
    }
#else
    (void)vectorized;
#endif
    for (int x = first_group; x < source.cols / 3; ++x) {
      for (int channel = 0; channel < 3; ++channel) {
        const int i = x * 9 + channel, o = x * 6 + channel;
        top[o] = (9 * a[i] + 3 * a[i+3] + 3 * b[i] + b[i+3] + 8) >> 4;
        top[o+3] = (3 * a[i+3] + 9 * a[i+6] + b[i+3] + 3 * b[i+6] + 8) >> 4;
        bottom[o] = (3 * b[i] + b[i+3] + 9 * c[i] + 3 * c[i+3] + 8) >> 4;
        bottom[o+3] = (b[i+3] + 3 * b[i+6] + 3 * c[i+3] + 9 * c[i+6] + 8) >> 4;
      }
    }
  }
}
inline void resize_preview(const cv::Mat& source,cv::Mat& target){
#if defined(__aarch64__)
  // Two independent row bands use spare CPU capacity without touching the
  // board's shared OpenCV pool. Their pixels and rounding remain identical.
  if(source.cols>=960 && source.rows>=540 && source.rows%6==0) {
    CV_Assert(source.type()==CV_8UC3 && source.cols%3==0);
    target.create(source.rows*2/3,source.cols*2/3,CV_8UC3);
    const int middle=source.rows/2,out_middle=target.rows/2;
    cv::Mat upper=target.rowRange(0,out_middle),lower=target.rowRange(out_middle,target.rows);
    std::exception_ptr failure;
    std::thread helper([&](){
      cpu_set_t cpus;CPU_ZERO(&cpus);CPU_SET(0,&cpus);CPU_SET(1,&cpus);
      pthread_setaffinity_np(pthread_self(),sizeof(cpus),&cpus);
      try{resize_preview_impl(source.rowRange(0,middle),upper,true);}catch(...){failure=std::current_exception();}
    });
    try{resize_preview_impl(source.rowRange(middle,source.rows),lower,true);}
    catch(...){helper.join();throw;}
    helper.join();if(failure)std::rethrow_exception(failure);return;
  }
#endif
  resize_preview_impl(source,target,true);
}
}
