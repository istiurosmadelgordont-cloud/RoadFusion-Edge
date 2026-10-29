#pragma once
#include "adas/image_transaction.hpp"
#include <cerrno>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <sys/ioctl.h>

namespace adas {
class PcieImageControl {
 public:
  using Transport=std::function<int(unsigned long,void*)>;
  ~PcieImageControl(){ stop(); }
  void attach(int fd) {
    attach_transport([fd](unsigned long command,void* data) {
      int result; do { result=::ioctl(fd,command,data); }while(result<0 && errno==EINTR);
      return result;
    });
  }
  // 可注入传输层，用假设备检查 ACK、超时与退出；生产路径始终使用同一个采集 fd。
  void attach_transport(Transport transport) {
    stop(); transport_=std::move(transport);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_=ImageTransaction(); stop_=false; requires_stop_=false; query_failed_=false;
      fpga_caps caps{};
      if(transport_(FPGA_IOC_GET_CAPS,&caps)<0 || caps.version!=FPGA_CAPS_VERSION ||
         caps.reserved[0] || caps.reserved[1] || !(caps.flags & FPGA_CAP_IMAGE_SUBMIT)) {
        state_.log("驱动无图像参数能力；视频仍可使用"); return;
      }
      state_.ack_mode=(caps.flags & FPGA_CAP_IMAGE_ACK)!=0;
      if(state_.ack_mode) {
        // 必须在 START 之前登记邮箱。即使没有启动视频，邮箱也可能 DMA，退出须 STOP。
        requires_stop_=true;
        if(transport_(FPGA_IOC_PREPARE_IMAGE_ACK,nullptr)<0) {
          state_.log("回执邮箱登记失败："+std::string(std::strerror(errno))); return;
        }
      }
      state_.available=true;
      state_.log(state_.ack_mode?"回执邮箱已登记；等待真实 RECEIVED/APPLIED":"兼容单向提交；不能查询 FPGA 生效");
    }
    worker_=std::thread(&PcieImageControl::run,this);
  }
  bool submit(const fpga_image_submit& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stop_ || !state_.queue(config))return false;
    changed_.notify_all(); return true;
  }
  ImageTransaction snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_); return state_;
  }
  bool requires_stop() const { return requires_stop_; } // attach/close 线程使用，不在 worker 中改变。
  void stop() {
    { std::lock_guard<std::mutex> lock(mutex_); stop_=true; }
    changed_.notify_all(); if(worker_.joinable())worker_.join();
  }
 private:
  void run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while(!stop_) {
      if(state_.phase==ImageTransaction::Phase::QUEUED) {
        query_failed_=false;
        auto config=state_.submitted; const bool ack=state_.ack_mode;
        lock.unlock();
        const int result=transport_(ack?FPGA_IOC_SUBMIT_IMAGE_ACK:FPGA_IOC_SUBMIT_IMAGE,&config);
        const int error=errno;
        lock.lock();
        if(result==0)state_.sent(ImageTransaction::Clock::now());
        else state_.submit_failed("参数提交失败："+std::string(std::strerror(error)),
                                  error!=EINVAL && error!=ENOTTY && error!=EOPNOTSUPP);
        std::cout<<"PCIe image: "<<state_.message<<std::endl;
      }
      if(state_.busy && state_.ack_mode &&
         state_.phase!=ImageTransaction::Phase::QUEUED &&
         state_.phase!=ImageTransaction::Phase::UNCERTAIN) {
        fpga_image_ack_status status{};
        lock.unlock(); const int result=transport_(FPGA_IOC_GET_IMAGE_ACK,&status);
        const int error=errno; lock.lock();
        const auto old=state_.message;
        if(result==0)state_.observe(status);
        else if(!query_failed_) {
          state_.log("回执查询失败："+std::string(std::strerror(error))+"；继续等待，不自动重发");
          query_failed_=true;
        }
        state_.check_timeout(ImageTransaction::Clock::now());
        if(old!=state_.message)std::cout<<"PCIe image: "<<state_.message<<std::endl;
      }
      changed_.wait_for(lock,std::chrono::milliseconds(state_.busy?20:200),
          [this](){return stop_ || state_.phase==ImageTransaction::Phase::QUEUED;});
    }
  }
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::thread worker_;
  Transport transport_;
  ImageTransaction state_;
  bool stop_=true, requires_stop_=false, query_failed_=false;
};
} // namespace adas

