#pragma once
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

// HS2 read() returns one complete RGB565 frame and recycles its DMA slot.
// Published BGR buffers are immutable; refcounts protect consumers while the
// producer replaces the single latest-frame slot. There is no growing queue.
class PcieCapture {
 public:
  explicit PcieCapture(const volatile sig_atomic_t& quit) : quit_(quit) {}
  ~PcieCapture() { close(); }
  void open(const std::string& path) {
    fd_ = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd_ < 0) throw std::runtime_error(path + ": " + std::strerror(errno));
    uint32_t info[4] = {};
    if (::ioctl(fd_, _IOR('H', 0, uint32_t[4]), info) < 0 ||
        info[0] != 0x32534850U || info[1] != 2 ||
        info[2] != 1920U * 1080U * 2U || info[3] != 4) {
      ::close(fd_); fd_ = -1;
      throw std::runtime_error("PCIe HS2 ABI mismatch (requires 1080p RGB565, four buffers)");
    }
  }
  bool read(cv::Mat& frame, std::chrono::steady_clock::time_point& stamp) {
    if (!worker_.joinable()) worker_ = std::thread(&PcieCapture::capture, this);
    std::unique_lock<std::mutex> lock(mutex_);
    while (sequence_ == consumed_ && !failed_ && !stop_ && !quit_)
      ready_.wait_for(lock, std::chrono::milliseconds(100));
    if (failed_ || stop_ || quit_) return false;
    frame = latest_; stamp = stamp_; consumed_ = sequence_;
    return true;
  }
  std::string error() const { std::lock_guard<std::mutex> lock(mutex_); return error_; }
  void close() {
    stop_ = true; ready_.notify_all();
    // Driver read timeout bounds the join. Never close an fd used by read().
    if (worker_.joinable()) worker_.join();
    if (fd_ >= 0) {
      if (read_started_) {
        int result;
        do { result = ::ioctl(fd_, _IO('H', 1)); } while (result < 0 && errno == EINTR);
        if (result < 0) std::cerr << "PCIe STOP failed: " << std::strerror(errno) << '\n';
      }
      ::close(fd_); fd_ = -1;
    }
  }
 private:
  void capture() {
    try {
      bind_cpu_pair(0, "pcie-capture");
      cv::Mat raw(1080, 1920, CV_8UC2);
      auto window = std::chrono::steady_clock::now();
      int count = 0; double read_ms = 0, rgb_ms = 0;
      while (!stop_ && !quit_) {
        auto begin = std::chrono::steady_clock::now();
        read_started_ = true;
        ssize_t bytes;
        do { bytes = ::read(fd_, raw.data, raw.total() * raw.elemSize()); }
        while (bytes < 0 && errno == EINTR && !stop_ && !quit_);
        if (stop_ || quit_) break;
        if (bytes != static_cast<ssize_t>(raw.total() * raw.elemSize()))
          throw std::runtime_error(bytes < 0 ? std::strerror(errno) : "short PCIe frame");
        auto captured = std::chrono::steady_clock::now();
        cv::Mat bgr;
        cv::cvtColor(raw, bgr, cv::COLOR_BGR5652BGR);
        auto converted = std::chrono::steady_clock::now();
        {
          std::lock_guard<std::mutex> lock(mutex_);
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
  int fd_ = -1;
  bool read_started_ = false;
  std::atomic<bool> stop_{false};
  std::thread worker_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  cv::Mat latest_;
  std::chrono::steady_clock::time_point stamp_{};
  uint64_t sequence_ = 0, consumed_ = 0;
  bool failed_ = false;
  std::string error_;
};
} // namespace adas
