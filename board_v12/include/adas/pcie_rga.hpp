#pragma once
#include "adas/fpga_uapi.h"
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <csignal>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <rga/im2d.h>
#include <rga/rga.h>

namespace adas {

// FPGA/Qt/OpenCV 的 RGB565：R 在位 15..11，G 在 10..5，B 在 4..0。
// 在本板配套 librga 1.10.6 的格式定义中，该布局应声明为 BGR_565。
// 不能按同名直接选 RGB_565，否则转换到 OpenCV BGR 时红蓝互换。
// 已用真实 RGA 穷举全部 65536 种像素，与 COLOR_BGR5652BGR 对照一致。
constexpr int kPcieRgb565RgaFormat = RK_FORMAT_BGR_565;

/*
 * 固定 BGR 池：启动时分配，采集线程是唯一写入者。
 * 池自己保留每块 Mat 的一个引用；refcount==1 且没有 UMat 引用时才允许复用。
 * UI、推理任务和 ROI 都使用普通 Mat 强引用，不改变现有识别函数的参数。
 * 读取引用计数必须用原子操作，不能直接与另一个线程的 release() 竞争。
 * close 销毁池头部后，仍被外部 Mat 引用的数据也会保留到最后一个使用者结束。
 */
class PcieBgrPool {
 public:
  PcieBgrPool(unsigned count, int width, int height) {
    if (count < 3 || count > 16) throw std::runtime_error("PCIe BGR pool requires 3..16 buffers");
    if (width <= 0 || height <= 0 || width > 1920 || height > 1080)
      throw std::runtime_error("PCIe BGR pool dimensions exceed the supported frame size");
    const long page_size = ::sysconf(_SC_PAGESIZE);
    if (page_size <= 0) throw std::runtime_error("Cannot query page size for RGA BGR pool");
    const size_t page = static_cast<size_t>(page_size);
    const size_t bytes = static_cast<size_t>(width) * height * 3;
    images_.reserve(count);
    for (unsigned i = 0; i < count; ++i) {
      /*
       * 预留前后页边界，将像素起点按页对齐。RGA 导入用户页/维护缓存时，
       * 覆盖的页面全部落在这块存储内部，不与 malloc 元数据或其他对象共页。
       * colRange + reshape 保留 OpenCV 原生 UMatData 引用，不能改用无所有权的裸指针 Mat。
       * 清零也提前触碰页面，减少采集期间的缺页分配。
       */
      cv::Mat storage(1, static_cast<int>(bytes + 2 * page), CV_8UC1, cv::Scalar::all(0));
      const size_t padding = (page - reinterpret_cast<uintptr_t>(storage.data) % page) % page;
      images_.push_back(storage.colRange(static_cast<int>(padding), static_cast<int>(padding + bytes)).reshape(3, height));
      if (!images_.back().isContinuous() || images_.back().step != static_cast<size_t>(width) * 3)
        throw std::runtime_error("PCIe BGR pool requires packed BGR888 storage");
    }
  }
  PcieBgrPool(const PcieBgrPool&) = delete;
  PcieBgrPool& operator=(const PcieBgrPool&) = delete;
  cv::Mat acquire() {
    // 只有采集线程调用 acquire；其他线程只操作各自 Mat 头部的共享引用计数。
    for (size_t i = 0; i < images_.size(); ++i) {
      const size_t slot = (next_ + i) % images_.size();
      cv::Mat& image = images_[slot];
      if (image.u && CV_XADD(&image.u->refcount, 0) == 1 && CV_XADD(&image.u->urefcount, 0) == 0) {
        next_ = (slot + 1) % images_.size();
        return image; // 拷贝 Mat 头部并增加引用；不复制像素。
      }
    }
    return {}; // 不覆盖旧帧；由采集端跳过本次输入，保持实时性。
  }
  size_t capacity() const { return images_.size(); }
 private:
  std::vector<cv::Mat> images_;
  size_t next_ = 0;
};

/*
 * 输入：driver 8.2 导出的四槽 DMA-BUF；输出：普通可缓存 BGR 池。
 * RGA 通过虚拟地址导入输出页，由 RGA 驱动完成映射/缓存维护。
 * 选择这个输出方式是为沿用 CPU/OpenCV 消费链，避免直接读取未确认缓存属性的 DRM 内存。
 * 输入侧不 read、不 mmap、不 CPU 转色；输出导入仍有开销，不宣称全链路零复制。
 */
class PcieRgaReceiver {
 public:
  using Clock = std::chrono::steady_clock;
  enum class Result { FRAME, DROPPED, STOPPED };
  struct Timing { double get_ms = 0, rga_ms = 0; };

  PcieRgaReceiver(int fd, const fpga_info& info, unsigned buffers)
      : control_(fd), info_(info), pool_(buffers, info.width, info.height) {
    sources_.fill(-1);
    try {
      fpga_caps caps{};
      if (call(FPGA_IOC_GET_CAPS, &caps) < 0)
        fail("GET_CAPS");
      if (caps.version != FPGA_CAPS_VERSION || caps.reserved[0] || caps.reserved[1] ||
          !(caps.flags & FPGA_CAP_DMABUF))
        throw std::runtime_error("board_v12 requires driver 8.2 with DMA-BUF export for RGA capture");
      for (unsigned slot = 0; slot < FPGA_FRAME_SLOTS; ++slot) {
        fpga_dmabuf_export request{};
        request.slot = slot; request.fd = -1;
        if (call(FPGA_IOC_EXPORT_DMABUF, &request) < 0)
          fail("EXPORT_DMABUF slot " + std::to_string(slot));
        if (request.fd < 0) throw std::runtime_error("EXPORT_DMABUF returned invalid fd");
        sources_[slot] = request.fd;
      }
    } catch (...) {
      close_sources(); // 构造未完成时析构函数不会运行，要主动清理已导出的 fd。
      throw;
    }
  }
  ~PcieRgaReceiver() { close_sources(); }
  PcieRgaReceiver(const PcieRgaReceiver&) = delete;
  PcieRgaReceiver& operator=(const PcieRgaReceiver&) = delete;
  size_t pool_size() const { return pool_.capacity(); }

  Result receive(cv::Mat& image, Clock::time_point& captured, Timing& timing,
                 const std::atomic<bool>& stop, const volatile sig_atomic_t& quit) {
    image.release(); timing = {};
    fpga_frame frame{};
    const auto begin = Clock::now();
    const int result = call(FPGA_IOC_GET_FRAME, &frame, &stop, &quit);
    const auto acquired = Clock::now();
    timing.get_ms = std::chrono::duration<double, std::milli>(acquired - begin).count();
    if (result < 0) {
      if (stop || quit) return Result::STOPPED;
      fail("GET_FRAME");
    }
    // 不能使用或归还无法验证的 token；失败由 STOP/关闭流程处理。
    if (frame.slot >= FPGA_FRAME_SLOTS || frame.bytes_used != info_.image_bytes ||
        !frame.session || frame.reserved || !(frame.token >> 2) ||
        (frame.token & 3U) != frame.slot || frame.sequence != (frame.token >> 2))
      throw std::runtime_error("RGA mode received invalid frame metadata");
    captured = acquired;
    // 取到帧后再选择输出，避免等待 FPGA 时占着空闲池，也能及时利用刚释放的缓冲区。
    cv::Mat output = pool_.acquire();
    if (stop || quit || output.empty()) {
      // 池满时不等待/覆盖已有 BGR 帧，直接归还本次完整输入，避免积压旧视频。
      put(frame, stop, quit);
      return (stop || quit) ? Result::STOPPED : Result::DROPPED;
    }

    const auto source = wrapbuffer_fd_t(sources_[frame.slot], info_.width, info_.height,
                                       info_.stride / 2, info_.height, kPcieRgb565RgaFormat);
    const auto target = wrapbuffer_virtualaddr_t(output.data, output.cols, output.rows,
                                                output.step / 3, output.rows, RK_FORMAT_BGR_888);
    // sync=1：成功返回前不发布输出、不归还源槽，也不让池复用目标内存。
    const IM_STATUS status = imcvtcolor_t(source, target, kPcieRgb565RgaFormat, RK_FORMAT_BGR_888, 0, 1);
    timing.rga_ms = std::chrono::duration<double, std::milli>(Clock::now() - acquired).count();
    if (status != IM_STATUS_SUCCESS)
      throw std::runtime_error(std::string("PCIe RGA conversion failed: ") + imStrError(status));
    // RGA 错误不会到这里，不把未知完成状态的源槽再次交给 FPGA 覆盖。
    put(frame, stop, quit);
    if (stop || quit) return Result::STOPPED;
    image = output;
    return Result::FRAME;
  }

 private:
  int call(unsigned long command, void* value,
           const std::atomic<bool>* stop = nullptr, const volatile sig_atomic_t* quit = nullptr) const {
    int result;
    do { result = ::ioctl(control_, command, value); }
    while (result < 0 && errno == EINTR && (!stop || !*stop) && (!quit || !*quit));
    return result;
  }
  static void fail(const std::string& operation) {
    throw std::runtime_error("PCIe RGA " + operation + ": " + std::strerror(errno));
  }
  void put(fpga_frame& frame, const std::atomic<bool>& stop, const volatile sig_atomic_t& quit) {
    // STOP 允许与同步转换并行；STOP 已确认后 PUT 只结束软件持有，不重发 RELEASE。
    if (call(FPGA_IOC_PUT_FRAME, &frame, &stop, &quit) < 0 && !stop && !quit) fail("PUT_FRAME");
  }
  void close_sources() {
    for (int& fd : sources_) if (fd >= 0) { ::close(fd); fd = -1; }
  }
  int control_;
  fpga_info info_;
  std::array<int, FPGA_FRAME_SLOTS> sources_;
  PcieBgrPool pool_;
};
} // namespace adas
