#include "adas/fpga_uapi.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <sys/ioctl.h>
#include <unistd.h>
#include <opencv2/imgproc.hpp>

namespace mock {
const size_t bytes = 1920U * 1080U * 2U;
std::mutex lock;
std::condition_variable ready;
bool stopped, waiting, reading, start_failure, info_failure, timeout, partial_eof;
int bad_field, opens, closes, starts, stops, reads, stop_timeouts, allowed_frames, completed;
int info_interrupts;
bool ack_capable=false; int prepared=0;
size_t cursor;
std::vector<unsigned char> pixels(bytes);
void reset() {
  std::lock_guard<std::mutex> guard(lock);
  stopped = waiting = reading = start_failure = info_failure = timeout = partial_eof = false;
  bad_field = -1;
  opens = closes = starts = stops = reads = stop_timeouts = completed = info_interrupts = 0;
  allowed_frames = 1; cursor = 0; ack_capable=false; prepared=0;
  const uint16_t colors[] = {0xf800, 0x07e0, 0x001f, 0xffff};
  for (size_t i=0; i<bytes/2; ++i) {
    pixels[2*i] = colors[i%4] & 255; pixels[2*i+1] = colors[i%4] >> 8;
  }
}
void wait_blocked() {
  std::unique_lock<std::mutex> guard(lock);
  assert(ready.wait_for(guard, std::chrono::seconds(3), [] { return waiting; }));
}
}

extern "C" int __wrap_open(const char*, int flags, ...) {
  assert(flags == (O_RDONLY | O_CLOEXEC));
  std::lock_guard<std::mutex> guard(mock::lock);
  ++mock::opens; return 123;
}
extern "C" int __wrap_close(int fd) {
  assert(fd == 123);
  std::lock_guard<std::mutex> guard(mock::lock);
  assert(!mock::reading); ++mock::closes; return 0;
}
extern "C" int __wrap_ioctl(int fd, unsigned long command, ...) {
  assert(fd == 123);
  va_list args; va_start(args,command); void* argument=va_arg(args,void*); va_end(args);
  std::lock_guard<std::mutex> guard(mock::lock);
  if (command == FPGA_IOC_GET_INFO) {
    if (mock::info_interrupts-- > 0) { errno=EINTR; return -1; }
    if (mock::info_failure) { errno=ENOTTY; return -1; }
    fpga_info& info=*static_cast<fpga_info*>(argument); info={};
    info.abi_version=2; info.width=1920; info.height=1080; info.stride=3840;
    info.image_bytes=mock::bytes; info.slot_count=4; info.pixel_format=FPGA_FORMAT_RGB565;
    switch (mock::bad_field) {
      case 0: info.abi_version=1; break;
      case 1: info.width=1280; break;
      case 2: info.height=720; break;
      case 3: info.stride=4000; break;
      case 4: info.image_bytes=4096; break;
      case 5: info.slot_count=2; break;
      case 6: info.pixel_format=0; break;
    }
    return 0;
  }
  if (command == FPGA_IOC_GET_CAPS) {
    if(!mock::ack_capable){errno=ENOTTY;return -1;}
    auto* caps=static_cast<fpga_caps*>(argument);*caps={};caps->version=1;caps->flags=3;return 0;
  }
  if(command == FPGA_IOC_PREPARE_IMAGE_ACK){assert(mock::starts==0);++mock::prepared;return 0;}
  if (command == FPGA_IOC_START) {
    ++mock::starts;
    if (mock::start_failure) { errno=EBUSY; return -1; }
    return 0;
  }
  if (command == FPGA_IOC_STOP) {
    ++mock::stops; mock::stopped=true; mock::ready.notify_all();
    if (mock::stop_timeouts-- > 0) { errno=ETIMEDOUT; return -1; }
    return 0;
  }
  assert(false && "Unexpected ioctl: only read-mode video commands are allowed");
  return -1;
}
extern "C" ssize_t __wrap_read(int fd, void* buffer, size_t length) {
  assert(fd==123);
  std::unique_lock<std::mutex> guard(mock::lock);
  ++mock::reads;
  if (mock::reads==1) { errno=EINTR; return -1; }
  if (mock::timeout) { errno=ETIMEDOUT; return -1; }
  if (mock::partial_eof && mock::cursor) return 0;
  if (mock::completed>=mock::allowed_frames) {
    mock::reading=mock::waiting=true; mock::ready.notify_all();
    mock::ready.wait(guard, [] { return mock::stopped || mock::completed<mock::allowed_frames; });
    mock::reading=mock::waiting=false;
  }
  if (mock::stopped) return 0;
  assert(length==mock::bytes-mock::cursor);
  const size_t size=std::min<size_t>(length,700001);
  std::memcpy(buffer,mock::pixels.data()+mock::cursor,size);
  mock::cursor+=size;
  if (mock::cursor==mock::bytes) { mock::cursor=0; ++mock::completed; }
  return static_cast<ssize_t>(size);
}

#include "adas/pcie_capture.hpp"

void colors(const cv::Mat& image) {
  assert(image.rows==1080 && image.cols==1920 && image.type()==CV_8UC3);
  assert(image.at<cv::Vec3b>(0,0)==cv::Vec3b(0,0,248));
  assert(image.at<cv::Vec3b>(0,1)==cv::Vec3b(0,252,0));
  assert(image.at<cv::Vec3b>(0,2)==cv::Vec3b(248,0,0));
}

int main() {
  static_assert(sizeof(fpga_info)==72,"pcie_v2 info ABI");
  static_assert(FPGA_IOC_START==0x5001,"START ioctl");
  static_assert(FPGA_IOC_STOP==0x5002,"STOP ioctl");
  volatile sig_atomic_t quit=0;
  cv::setNumThreads(1);
  {
    mock::reset(); mock::info_interrupts=1;
    adas::PcieCapture capture(quit); capture.open("mock");
    cv::Mat image; std::chrono::steady_clock::time_point stamp;
    assert(capture.read(image,stamp)); colors(image); mock::wait_blocked();
    assert(stamp!=std::chrono::steady_clock::time_point());
    const auto begin=std::chrono::steady_clock::now(); capture.close();
    assert(std::chrono::steady_clock::now()-begin<std::chrono::seconds(1));
    assert(mock::starts==1 && mock::stops==1 && mock::closes==1 && mock::reads>=8);
    capture.close(); assert(mock::closes==1); colors(image);
  }
  {
    mock::reset(); adas::PcieCapture capture(quit); capture.open("mock");
    cv::Mat first, second; std::chrono::steady_clock::time_point stamp;
    assert(capture.read(first,stamp)); mock::wait_blocked();
    {
      std::lock_guard<std::mutex> guard(mock::lock);
      std::fill(mock::pixels.begin(),mock::pixels.end(),0);
      mock::allowed_frames=2; mock::ready.notify_all();
    }
    assert(capture.read(second,stamp));
    assert(second.at<cv::Vec3b>(0,0)==cv::Vec3b(0,0,0));
    colors(first); capture.close(); colors(first);
  }
  for (int field=0; field<8; ++field) {
    mock::reset(); mock::bad_field=field; mock::info_failure=field==7;
    adas::PcieCapture capture(quit); bool rejected=false;
    try { capture.open("mock"); } catch(const std::exception&) { rejected=true; }
    assert(rejected && mock::closes==1 && mock::starts==0 && mock::stops==0);
  }
  for (int failure=0; failure<3; ++failure) {
    mock::reset(); mock::timeout=failure==0; mock::partial_eof=failure==1;
    mock::start_failure=failure==2;
    adas::PcieCapture capture(quit); capture.open("mock");
    cv::Mat image; std::chrono::steady_clock::time_point stamp;
    assert(!capture.read(image,stamp) && image.empty() && !capture.error().empty());
    capture.close();
    assert(mock::closes==1 && mock::stops==(failure==2 ? 0 : 1));
  }
  {
    mock::reset(); mock::stop_timeouts=2;
    adas::PcieCapture capture(quit); capture.open("mock");
    cv::Mat image; std::chrono::steady_clock::time_point stamp;
    assert(capture.read(image,stamp)); capture.close();
    assert(mock::stops==3 && mock::closes==1);
  }
  {
    mock::reset(); adas::PcieCapture capture(quit); capture.open("mock"); capture.close();
    assert(mock::starts==0 && mock::stops==0 && mock::closes==1);
    mock::reset(); capture.open("mock");
    cv::Mat image; std::chrono::steady_clock::time_point stamp;
    assert(capture.read(image,stamp)); capture.close();
    assert(mock::starts==1 && mock::stops==1 && mock::closes==1);
  }
  {
    mock::reset(); quit=1; adas::PcieCapture capture(quit); capture.open("mock");
    cv::Mat image; std::chrono::steady_clock::time_point stamp;
    assert(!capture.read(image,stamp)); capture.close();
    assert(mock::starts==0 && mock::stops==0 && mock::closes==1); quit=0;
  }
  {
    mock::reset(); mock::allowed_frames=0;
    adas::PcieCapture capture(quit); capture.open("mock");
    cv::Mat image; std::chrono::steady_clock::time_point stamp;
    const auto begin=std::chrono::steady_clock::now();
    using R=adas::PcieCapture::ReadResult;
    assert(capture.poll_read(image,stamp,16)==R::WAITING);
    assert(image.empty() && stamp==std::chrono::steady_clock::time_point());
    assert(std::chrono::steady_clock::now()-begin<std::chrono::milliseconds(500));
    mock::wait_blocked();
    { std::lock_guard<std::mutex> guard(mock::lock); mock::allowed_frames=1; mock::ready.notify_all(); }
    R result=R::WAITING;
    for(int i=0;i<100 && result==R::WAITING;++i) result=capture.poll_read(image,stamp,20);
    assert(result==R::FRAME); colors(image); mock::wait_blocked();
    assert(capture.poll_read(image,stamp,16)==R::WAITING); colors(image);
    quit=1; assert(capture.poll_read(image,stamp,16)==R::CLOSED);
    capture.close(); quit=0;
    assert(mock::starts==1 && mock::stops==1 && mock::closes==1);
  }
  {
    // 已登记回执邮箱、但未 START，也必须在 close 时 STOP 排空。
    mock::reset();mock::ack_capable=true;
    adas::PcieCapture capture(quit);capture.open("mock");
    assert(mock::prepared==1 && mock::starts==0);
    capture.close();assert(mock::stops==1 && mock::closes==1);
    mock::reset();mock::ack_capable=true;capture.open("mock");
    cv::Mat image;std::chrono::steady_clock::time_point stamp;
    assert(capture.read(image,stamp));colors(image);capture.close();
    assert(mock::prepared==1 && mock::starts==1 && mock::stops==1);
  }
  std::cout << "PASS: bounded PCIe wait, first frame, no duplicate frame, stop while waiting\n";
  std::cout << "PASS: read ABI, dimensions, RGB565, short reads/EINTR, immutable frames, "
               "STOP wakeup/retry, timeout/EOF/START errors, reopen, pre-start exit\n";
}
