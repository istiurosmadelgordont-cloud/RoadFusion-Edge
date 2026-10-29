#pragma once
#include "adas/fpga_uapi.h"
#include "adas/pcie_image_control.hpp"
#include "adas/pcie_rga.hpp"
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
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <pthread.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <opencv2/imgproc.hpp>

namespace adas {
inline void bind_cpu_pair(int first, const char* name) {
  cpu_set_t cpus;
  CPU_ZERO(&cpus);
  CPU_SET(first, &cpus); CPU_SET(first + 1, &cpus);
  const int error = pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus);
  if (error) throw std::runtime_error(std::string("CPU affinity: ") + std::strerror(error));
  pthread_setname_np(pthread_self(), name);
}

// PCIe 视频固定使用 DMA-BUF + 同步 RGA，输出完整 BGR Mat 供原识别链消费。
// 图像参数仍使用同一控制 fd；RGA/驱动不匹配或转换失败时明确报错。
class PcieCapture {
 public:
  explicit PcieCapture(const volatile sig_atomic_t& quit) : quit_(quit) {}
  ~PcieCapture() { close(); }

  void open(const std::string& path, unsigned buffers = 6) {
    if (fd_ >= 0 || worker_.joinable())
      throw std::runtime_error("PCIe capture is already open");
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) throw std::runtime_error(path + ": " + std::strerror(errno));

    // 先确认驱动的视频接口。GET_INFO 不启动 DMA，也不修改 FPGA 图像参数。
    fpga_info info{};
    const int result = device_ioctl(fd, FPGA_IOC_GET_INFO, &info);
    const int error = errno;
    if (result < 0 || info.abi_version != FPGA_ABI_VERSION ||
        info.width != 1920 || info.height != 1080 || info.stride != 3840 ||
        info.image_bytes != 1920U * 1080U * 2U ||
        info.slot_count != FPGA_FRAME_SLOTS ||
        info.pixel_format != FPGA_FORMAT_RGB565) {
      ::close(fd);
      if (result < 0)
        throw std::runtime_error("PCIe GET_INFO: " + std::string(std::strerror(error)));
      throw std::runtime_error("PCIe ABI mismatch (requires pcie_v3 ABI 2, 1080p RGB565)");
    }
    fd_ = fd;
    // 所有导出和池分配都在 START 前完成；随后登记图像回执，沿用同一 fd。
    try {
      rga_.reset(new PcieRgaReceiver(fd_, info, buffers));
      image_control_.attach(fd_);
    }
    catch (...) { close(); throw; }
    stop_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    failed_ = false; error_.clear(); latest_.release();
    sequence_ = consumed_ = 0;
    std::cout << "PCIe capture mode=RGA BGR_pool=" << rga_->pool_size();
    std::cout << std::endl;
  }

  bool read(cv::Mat& frame, std::chrono::steady_clock::time_point& stamp) {
    if (fd_ < 0) throw std::runtime_error("PCIe capture is not open");
    if (!worker_.joinable()) worker_ = std::thread(&PcieCapture::capture, this);
    std::unique_lock<std::mutex> lock(mutex_);
    while (sequence_ == consumed_ && !failed_ && !stop_ && !quit_)
      ready_.wait_for(lock, std::chrono::milliseconds(100));
    if (failed_ || stop_ || quit_) return false;
    frame = latest_; stamp = stamp_; consumed_ = sequence_;
    return true;
  }

  enum class ReadResult { FRAME, WAITING, CLOSED };
  // UI 使用有界等待：FPGA 暂时不出帧时仍能处理鼠标松开/关闭窗口。
  // 这里只等采集线程发布，不改变内核 read、DMA 或 START/STOP 协议。
  ReadResult poll_read(cv::Mat& frame, std::chrono::steady_clock::time_point& stamp,
                       int timeout_ms = 16) {
    if (fd_ < 0) throw std::runtime_error("PCIe capture is not open");
    if (!worker_.joinable()) worker_ = std::thread(&PcieCapture::capture, this);
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this]() {
      return sequence_ != consumed_ || failed_ || stop_ || quit_;
    });
    if (failed_ || stop_ || quit_) return ReadResult::CLOSED;
    if (sequence_ == consumed_) return ReadResult::WAITING;
    frame = latest_; stamp = stamp_; consumed_ = sequence_;
    return ReadResult::FRAME;
  }

  std::string error() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
  }

  bool submit_image(const fpga_image_submit& config) { return image_control_.submit(config); }
  ImageTransaction image_status() const { return image_control_.snapshot(); }

  // Read-only snapshot for a queued inference job at execution time. It does
  // not consume the UI sequence and never touches the device or DMA protocol.
  bool latest_frame(cv::Mat& frame, std::chrono::steady_clock::time_point& stamp) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if(latest_.empty() || failed_ || stop_ || quit_)return false;
    frame=latest_; stamp=stamp_; return true;
  }

  void close() {
    stop_ = true;
    ready_.notify_all();
    // 先结束参数 ioctl，再 STOP 排空视频及回执 DMA，最后关闭共享 fd。
    image_control_.stop();
    {
      // START 和 STOP 串行，避免退出时出现“先 STOP、后 START”的竞态。
      std::lock_guard<std::mutex> control(control_mutex_);
      if (fd_ >= 0 && (started_ || image_control_.requires_stop())) {
        // 必须先 STOP 唤醒内核中等待帧的 read，再 join，最后关闭 fd。
        // 超时最多尝试三次；驱动继续等同一个 STOP cookie，不生成新请求。
        int result = 0;
        for (int attempt = 0; attempt < 3; ++attempt) {
          result = device_ioctl(fd_, FPGA_IOC_STOP);
          if (result == 0 || errno != ETIMEDOUT) break;
        }
        if (result < 0)
          std::cerr << "PCIe STOP failed: " << std::strerror(errno) << '\n';
        started_ = false;
      }
    }
    if (worker_.joinable()) worker_.join();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_.release();
    }
    // STOP + join 后才关输入导出 fd。池的普通 Mat 数据由最后一个消费者释放。
    rga_.reset();
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

 private:
  static int device_ioctl(int fd, unsigned long command, void* argument = nullptr) {
    int result;
    do { result = ::ioctl(fd, command, argument); }
    while (result < 0 && errno == EINTR);
    return result;
  }

  void capture() {
    try {
      bind_cpu_pair(0, "pcie-capture");
      {
        std::lock_guard<std::mutex> control(control_mutex_);
        if (stop_ || quit_) return;
        // pcie_v3 不会在 open/read 时自动启动，必须显式调用 START。
        if (device_ioctl(fd_, FPGA_IOC_START) < 0)
          throw std::runtime_error("PCIe START: " + std::string(std::strerror(errno)));
        started_ = true;
      }
      auto window = std::chrono::steady_clock::now();
      unsigned count = 0, attempts = 0, drops = 0;
      double get_ms = 0, rga_ms = 0;
      while (!stop_ && !quit_) {
        cv::Mat bgr;
        std::chrono::steady_clock::time_point captured;
        PcieRgaReceiver::Timing timing;
        const auto result = rga_->receive(bgr, captured, timing, stop_, quit_);
        if (result == PcieRgaReceiver::Result::STOPPED) break;
        ++attempts; get_ms += timing.get_ms; rga_ms += timing.rga_ms;
        if (result == PcieRgaReceiver::Result::DROPPED) ++drops;
        const auto converted = std::chrono::steady_clock::now();
        if (stop_ || quit_) break;
        if (!bgr.empty()) {
          std::lock_guard<std::mutex> lock(mutex_);
          // RGA 模式是池中的强引用，不会覆盖仍由消费者持有的图像。
          latest_ = bgr; stamp_ = captured; ++sequence_;
          ++count;
          ready_.notify_one();
        }
        const double seconds = std::chrono::duration<double>(converted - window).count();
        if (seconds >= 5) {
          std::cout << "pcie_perf mode=rga fps=" << count / seconds
                    << " get_ms=" << get_ms / attempts
                    << " rga_ms=" << (count ? rga_ms / count : 0)
                    << " pool_drops=" << drops << " cpu=0,1" << std::endl;
          count = attempts = drops = 0; get_ms = rga_ms = 0; window = converted;
        }
      }
    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> lock(mutex_);
      failed_ = true; error_ = e.what(); ready_.notify_all();
    }
  }

  const volatile sig_atomic_t& quit_;
  PcieImageControl image_control_;
  std::unique_ptr<PcieRgaReceiver> rga_;
  int fd_ = -1;
  bool started_ = false;  // 由 control_mutex_ 保护，采集线程 START、主线程 STOP。
  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::mutex control_mutex_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  cv::Mat latest_;
  std::chrono::steady_clock::time_point stamp_{};
  uint64_t sequence_ = 0, consumed_ = 0;
  bool failed_ = false;
  std::string error_;
};
} // namespace adas
