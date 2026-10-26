#pragma once
#include "adas/fpga_uapi.h"
#include "adas/pcie_image_control.hpp"
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

// pcie_v3 保持视频 ABI 2：read 完整 RGB565 帧 -> BGR。
// 对外保留 board_v8 的 open/read/error/close 接口和“只保留最新帧”行为，
// 图像参数使用同一 fd 的独立控制线程，不阻塞采集与 GUI；仍默认 read。
class PcieCapture {
 public:
  explicit PcieCapture(const volatile sig_atomic_t& quit) : quit_(quit) {}
  ~PcieCapture() { close(); }

  void open(const std::string& path) {
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
    // 参数邮箱登记发生在 START 之前；能力缺失只关闭调参，不中断视频输入。
    try { image_control_.attach(fd_); }
    catch (...) { close(); throw; }
    stop_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    failed_ = false; error_.clear(); latest_.release();
    sequence_ = consumed_ = 0;
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
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

 private:
  static int device_ioctl(int fd, unsigned long command, void* argument = nullptr) {
    int result;
    do { result = ::ioctl(fd, command, argument); }
    while (result < 0 && errno == EINTR);
    return result;
  }

  // 驱动允许短读：每次返回当前帧剩余数据的一部分，不会跨帧。
  // 因此累积到整帧后才交给 OpenCV；帧尾 token 不属于图像，不读取。
  bool read_frame(cv::Mat& raw) {
    const size_t size = raw.total() * raw.elemSize();
    size_t received = 0;
    while (received < size && !stop_ && !quit_) {
      const ssize_t bytes = ::read(fd_, raw.data + received, size - received);
      const int error = errno;
      if (stop_ || quit_) return false;
      if (bytes < 0) {
        if (error == EINTR) continue;
        throw std::runtime_error("PCIe read: " + std::string(std::strerror(error)));
      }
      if (bytes == 0) throw std::runtime_error("PCIe stopped before a complete frame");
      received += static_cast<size_t>(bytes);
    }
    return received == size && !stop_ && !quit_;
  }

  void capture() {
    try {
      bind_cpu_pair(0, "pcie-capture");
      cv::Mat raw(1080, 1920, CV_8UC2);
      {
        std::lock_guard<std::mutex> control(control_mutex_);
        if (stop_ || quit_) return;
        // pcie_v3 不会在 open/read 时自动启动，必须显式调用 START。
        if (device_ioctl(fd_, FPGA_IOC_START) < 0)
          throw std::runtime_error("PCIe START: " + std::string(std::strerror(errno)));
        started_ = true;
      }
      auto window = std::chrono::steady_clock::now();
      int count = 0; double read_ms = 0, rgb_ms = 0;
      while (!stop_ && !quit_) {
        const auto begin = std::chrono::steady_clock::now();
        if (!read_frame(raw)) break;
        const auto captured = std::chrono::steady_clock::now();
        cv::Mat bgr;
        // FPGA 输出小端 RGB565；OpenCV 的这个枚举转换后得到 BGR 三通道。
        cv::cvtColor(raw, bgr, cv::COLOR_BGR5652BGR);
        const auto converted = std::chrono::steady_clock::now();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          // 每帧使用独立 BGR 存储。消费者的 Mat 引用仍有效时不能覆盖数据。
          latest_ = bgr; stamp_ = captured; ++sequence_;
        }
        ready_.notify_one();
        ++count;
        read_ms += std::chrono::duration<double, std::milli>(captured - begin).count();
        rgb_ms += std::chrono::duration<double, std::milli>(converted - captured).count();
        const double seconds = std::chrono::duration<double>(converted - window).count();
        if (seconds >= 5) {
          std::cout << "pcie_perf fps=" << count / seconds << " read_ms=" << read_ms / count
                    << " rgb_ms=" << rgb_ms / count << " cpu=0,1" << std::endl;
          count = 0; read_ms = rgb_ms = 0; window = converted;
        }
      }
    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> lock(mutex_);
      failed_ = true; error_ = e.what(); ready_.notify_all();
    }
  }

  const volatile sig_atomic_t& quit_;
  PcieImageControl image_control_;
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
