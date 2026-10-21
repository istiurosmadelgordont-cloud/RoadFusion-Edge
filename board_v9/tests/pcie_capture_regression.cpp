// 使用真实 OpenCV 和真实 PcieCapture，仅替换设备系统调用；不访问 FPGA。
// 检查 ABI、颜色、归还后数据生命周期、短读、STOP 唤醒、异常清理。
#include "adas/fpga_uapi.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <cstdarg>
#include <future>
#include <vector>
#include <algorithm>

namespace mock {
std::mutex lock;
std::condition_variable ready;
bool stopped = false, bad_abi = false, timeout = false, bad_frame = false;
int frames = 0, puts = 0, starts = 0, stops = 0, closes = 0, unmaps = 0, maps = 0;
int read_calls = 0, stop_timeouts = 0, fail_map = -1;
size_t cursor = 0;
std::vector<unsigned char> pixels(4149248);
bool held = false, get_waiting = false;
void reset() {
 stopped = bad_abi = timeout = bad_frame = held = get_waiting = false;
 frames = puts = starts = stops = closes = unmaps = maps = read_calls = stop_timeouts = 0;
 fail_map = -1; cursor = 0;
 // 重复红、绿、蓝、白的 RGB565 像素，显式设置小端字节。
 const uint16_t colors[] = {0xf800, 0x07e0, 0x001f, 0xffff};
 for (size_t i = 0; i < pixels.size()/2; ++i) {
  pixels[2*i] = colors[i%4] & 255; pixels[2*i+1] = colors[i%4] >> 8;
 }
}
}
extern "C" int __wrap_open(const char*, int, ...) { return 123; }
extern "C" int __wrap_close(int fd) { assert(fd == 123); ++mock::closes; return 0; }
extern "C" void* __wrap_mmap(void*, size_t, int prot, int flags, int fd, off_t) {
 assert(fd==123 && prot==PROT_READ && flags==MAP_SHARED);
 if (mock::maps++ == mock::fail_map) { errno=ENOMEM; return MAP_FAILED; }
 return mock::pixels.data();
}
extern "C" int __wrap_munmap(void*, size_t) { assert(!mock::held); ++mock::unmaps; return 0; }
extern "C" int __wrap_ioctl(int fd, unsigned long cmd, ...) {
 va_list args; va_start(args,cmd); void* arg=va_arg(args,void*); va_end(args);
 assert(fd==123);
 std::unique_lock<std::mutex> l(mock::lock);
 if(cmd==FPGA_IOC_GET_INFO) {
  fpga_info& i=*static_cast<fpga_info*>(arg); i={};
  i.abi_version=mock::bad_abi?1:2; i.width=1920; i.height=1080; i.stride=3840;
  i.image_bytes=4147200; i.slot_count=4; i.pixel_format=1; i.map_bytes=4149248;
  for(int n=0;n<4;++n)i.offset[n]=n*i.map_bytes;
  return 0;
 }
 if(cmd==FPGA_IOC_START) { ++mock::starts; return 0; }
 if(cmd==FPGA_IOC_STOP) {
  ++mock::stops; mock::stopped=true; mock::ready.notify_all();
  if(mock::stop_timeouts>0) { --mock::stop_timeouts; errno=ETIMEDOUT; return -1; }
  return 0;
 }
 if(cmd==FPGA_IOC_GET_FRAME) {
  if(mock::timeout) { errno=ETIMEDOUT; return -1; }
  if(mock::frames>0) {
   mock::get_waiting=true;mock::ready.notify_all();
   mock::ready.wait(l,[]{return mock::stopped;});
  }
  if(mock::stopped) {errno=EPIPE;return -1;}
  ++mock::frames; mock::held=true;
  fpga_frame& f=*static_cast<fpga_frame*>(arg); f={};
  f.session=1;f.slot=0;f.token=4;f.sequence=1;
  f.bytes_used=mock::bad_frame?1:4147200;
  return 0;
 }
 if(cmd==FPGA_IOC_PUT_FRAME) {
  assert(mock::held);mock::held=false;++mock::puts;
  // 模拟归还后 FPGA 立刻覆盖 DMA 内存，BGR 发布帧必须不受影响。
  std::fill(mock::pixels.begin(),mock::pixels.end(),0);
  return 0;
 }
 assert(false);return -1;
}
extern "C" ssize_t __wrap_read(int, void* data, size_t len) {
 std::unique_lock<std::mutex> l(mock::lock);
 ++mock::read_calls;
 if(mock::cursor>=4147200) mock::ready.wait(l,[]{return mock::stopped;});
 if(mock::stopped)return 0;
 if(mock::read_calls==1){errno=EINTR;return -1;}
 const size_t n=std::min<size_t>(len,700001);
 std::memcpy(data,mock::pixels.data()+mock::cursor,n);mock::cursor+=n;return n;
}
#include "adas/pcie_capture.hpp"

void colors(const cv::Mat& m) {
 assert(m.rows==1080 && m.cols==1920 && m.type()==CV_8UC3);
 assert(m.at<cv::Vec3b>(0,0)==cv::Vec3b(0,0,248));
 assert(m.at<cv::Vec3b>(0,1)==cv::Vec3b(0,252,0));
 assert(m.at<cv::Vec3b>(0,2)==cv::Vec3b(248,0,0));
}
int main() {
 static_assert(sizeof(fpga_info)==72,"info ABI");
 static_assert(sizeof(fpga_frame)==32,"frame ABI");
 volatile sig_atomic_t quit=0;
 cv::setNumThreads(1);
 {
  mock::reset();adas::PcieCapture c(quit);c.open("mock","mmap");
  cv::Mat m;std::chrono::steady_clock::time_point t;
  assert(c.read(m,t));colors(m);assert(mock::puts==1);
  // 确实进入下一次阻塞 GET 后才关闭，检查 STOP 能解除等待。
  { std::unique_lock<std::mutex> l(mock::lock);
    assert(mock::ready.wait_for(l,std::chrono::seconds(2),[]{return mock::get_waiting;})); }
  const auto begin=std::chrono::steady_clock::now();c.close();
  assert(std::chrono::steady_clock::now()-begin<std::chrono::seconds(1));
  colors(m);assert(mock::stops==1 && mock::unmaps==4 && mock::closes==1);
  c.close();assert(mock::closes==1);
 }
 {
  mock::reset();adas::PcieCapture c(quit);c.open("mock"); // 默认启动必须走 read。
  cv::Mat m;std::chrono::steady_clock::time_point t;
  assert(c.read(m,t));colors(m);c.close();
  assert(mock::read_calls>=7 && mock::maps==0 && mock::puts==0);
 }
 for(int scenario=0;scenario<2;++scenario) {
  mock::reset();mock::timeout=scenario==0;mock::bad_frame=scenario==1;
  adas::PcieCapture c(quit);c.open("mock","mmap");
  cv::Mat m;std::chrono::steady_clock::time_point t;
  assert(!c.read(m,t) && !c.error().empty());c.close();
  assert(mock::stops==1 && mock::puts==scenario && mock::unmaps==4);
 }
 {
  mock::reset();mock::bad_abi=true;
  adas::PcieCapture c(quit);bool rejected=false;
  try{c.open("mock","mmap");}catch(const std::exception&){rejected=true;}
  assert(rejected && mock::closes==1 && mock::starts==0);
 }
 {
  mock::reset();mock::fail_map=2;
  adas::PcieCapture c(quit);bool rejected=false;
  try{c.open("mock","mmap");}catch(const std::exception&){rejected=true;}
  assert(rejected && mock::unmaps==2 && mock::closes==1);
 }
 {
  mock::reset();mock::stop_timeouts=2;
  adas::PcieCapture c(quit);c.open("mock","mmap");
  cv::Mat m;std::chrono::steady_clock::time_point t;
  assert(c.read(m,t));c.close();assert(mock::stops==3 && mock::puts==1);
 }
 std::cout<<"PCIe capture regression PASS: ABI, RGB565 colors, DMA lease, short reads, STOP, errors\n";
}
