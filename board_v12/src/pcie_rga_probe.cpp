/*
 * 第一阶段联通工具：PCIe 原槽 DMA-BUF -> 同步 RGA -> 一个可复用的 BGR 输出。
 * 不启动 GUI/NPU，不修改 FPGA 图像参数；它是独立诊断工具。
 * board_v12 的集成入口在 PcieCapture：DMA-BUF 输入 + 普通 BGR 池虚拟地址输出。
 * 此工具仍保留第一阶段 DRM 输出，不能替代集成入口的端到端验证。
 * --verify 用首帧做 CPU 像素对照，--output 保存首帧 PPM，仅验证时访问像素。
 * 正常循环不 mmap 原槽、不 read 图像，也不做 CPU RGB565 -> BGR 转换。
 */
#include "adas/fpga_uapi.h"
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <rga/im2d.h>
#include <rga/rga.h>
#include <drm.h>
#include <drm_mode.h>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace pcie_rga {
volatile sig_atomic_t interrupted = 0;
void signal_handler(int) { interrupted = 1; }

int call_ioctl(int fd, unsigned long command, void* value = nullptr) {
  int result;
  do { result = ::ioctl(fd, command, value); }
  while (result < 0 && errno == EINTR && !interrupted);
  return result;
}
void check(bool okay, const std::string& operation) {
  if (!okay) throw std::runtime_error(operation + ": " + std::strerror(errno));
}

// fd 只负责引用，不负责判断 DMA 是否结束；STOP 和 RGA 完成必须显式处理。
class Fd {
 public:
  ~Fd() { reset(); }
  int get() const { return value_; }
  void reset(int value = -1) {
    if (value_ >= 0) ::close(value_);
    value_ = value;
  }
  Fd() = default;
  Fd(const Fd&) = delete;
  Fd& operator=(const Fd&) = delete;
 private:
  int value_ = -1;
};

// 仅调试/保存时使用。SYNC 是缓存访问边界，不能代替 GET_FRAME 或 RGA 完成。
class CpuAccess {
 public:
  CpuAccess(int fd, size_t bytes, uint64_t access = DMA_BUF_SYNC_READ)
      : fd_(fd), bytes_(bytes), access_(access) {
    dma_buf_sync sync{}; sync.flags = DMA_BUF_SYNC_START | access_;
    check(call_ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync) == 0, "DMA_BUF_SYNC_START");
    const int protection = (access_ & DMA_BUF_SYNC_WRITE) ? PROT_READ | PROT_WRITE : PROT_READ;
    data_ = ::mmap(nullptr, bytes_, protection, MAP_SHARED, fd_, 0);
    if (data_ == MAP_FAILED) {
      const int saved = errno;
      sync.flags = DMA_BUF_SYNC_END | access_;
      call_ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync);
      errno = saved;
      check(false, "DMA-BUF mmap");
    }
  }
  ~CpuAccess() {
    if (data_ != MAP_FAILED) {
      ::munmap(data_, bytes_);
      dma_buf_sync sync{}; sync.flags = DMA_BUF_SYNC_END | access_;
      if (call_ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync) != 0)
        std::cerr << "DMA_BUF_SYNC_END failed: " << std::strerror(errno) << '\n';
    }
  }
  uint8_t* data() { return static_cast<uint8_t*>(data_); }
  void finish() {
    dma_buf_sync sync{}; sync.flags = DMA_BUF_SYNC_END | access_;
    check(call_ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync) == 0, "DMA_BUF_SYNC_END");
    ::munmap(data_, bytes_); data_ = MAP_FAILED;
  }
  CpuAccess(const CpuAccess&) = delete;
  CpuAccess& operator=(const CpuAccess&) = delete;
 private:
  int fd_;
  size_t bytes_;
  uint64_t access_;
  void* data_ = MAP_FAILED;
};

// 使用 Rockchip DRM dumb buffer 分配输出，再导出 PRIME DMA-BUF fd。
// 不设置显示模式、不取得 DRM master，不操作当前桌面的 framebuffer。
class BgrOutput {
 public:
  BgrOutput(const std::string& device, unsigned width, unsigned height) {
    drm_.reset(::open(device.c_str(), O_RDWR | O_CLOEXEC));
    check(drm_.get() >= 0, device);
    drm_mode_create_dumb create{};
    create.width = width; create.height = height; create.bpp = 24;
    check(call_ioctl(drm_.get(), DRM_IOCTL_MODE_CREATE_DUMB, &create) == 0,
          "DRM CREATE_DUMB (BGR888)");
    handle_ = create.handle; pitch = create.pitch; bytes = create.size;
    // RGA 的 wstride 以像素计，不能把字节 pitch 直接传进去。
    if (pitch < width * 3U || pitch % 3 || bytes < static_cast<uint64_t>(pitch) * height)
      throw std::runtime_error("DRM BGR pitch cannot be represented as RGA pixel stride");
    drm_prime_handle prime{};
    prime.handle = handle_; prime.flags = DRM_CLOEXEC | DRM_RDWR;
    check(call_ioctl(drm_.get(), DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime) == 0,
          "DRM PRIME_HANDLE_TO_FD");
    dma_.reset(prime.fd);
  }
  ~BgrOutput() {
    dma_.reset();
    if (handle_) {
      drm_mode_destroy_dumb destroy{}; destroy.handle = handle_;
      call_ioctl(drm_.get(), DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
    }
  }
  int fd() const { return dma_.get(); }
  unsigned pitch = 0;
  size_t bytes = 0;
 private:
  Fd drm_, dma_;
  uint32_t handle_ = 0;
};

class Input {
 public:
  explicit Input(const std::string& device) {
    control_.reset(::open(device.c_str(), O_RDONLY | O_CLOEXEC));
    check(control_.get() >= 0, device);
    check(call_ioctl(control_.get(), FPGA_IOC_GET_INFO, &info) == 0, "GET_INFO");
    if (info.abi_version != FPGA_ABI_VERSION || info.width != 1920 || info.height != 1080 ||
        info.stride != 3840 || info.image_bytes != 4147200 || info.map_bytes < info.image_bytes ||
        info.slot_count != FPGA_FRAME_SLOTS || info.pixel_format != FPGA_FORMAT_RGB565)
      throw std::runtime_error("Requires ABI 2, four slots, 1920x1080 RGB565");
    fpga_caps caps{};
    check(call_ioctl(control_.get(), FPGA_IOC_GET_CAPS, &caps) == 0, "GET_CAPS");
    if (caps.version != FPGA_CAPS_VERSION || !(caps.flags & FPGA_CAP_DMABUF))
      throw std::runtime_error("Driver has no DMA-BUF export capability; use driver 8.2 or newer");
    for (unsigned slot = 0; slot < FPGA_FRAME_SLOTS; ++slot) {
      fpga_dmabuf_export request{};
      request.slot = slot; request.fd = -1;
      check(call_ioctl(control_.get(), FPGA_IOC_EXPORT_DMABUF, &request) == 0,
            "EXPORT_DMABUF slot " + std::to_string(slot));
      if (request.fd < 0) throw std::runtime_error("EXPORT_DMABUF returned an invalid fd");
      source_[slot].reset(request.fd);
    }
  }
  ~Input() {
    if (!stop()) std::cerr << "STOP not confirmed; driver must retain DMA memory\n";
    // source_ 先析构，control_ 后析构；导出对象自己也持有 control file 的引用。
  }
  void start() {
    running_ = true;  // 失败路径也尝试 STOP，不能仅靠 close 隐式停止。
    check(call_ioctl(control_.get(), FPGA_IOC_START) == 0, "START");
  }
  bool stop() {
    if (!running_) return true;
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (call_ioctl(control_.get(), FPGA_IOC_STOP) == 0) { running_ = false; return true; }
      if (errno != ETIMEDOUT && errno != EINTR) break;
    }
    return false;
  }
  bool get(fpga_frame& frame) {
    frame = {};
    const int result = call_ioctl(control_.get(), FPGA_IOC_GET_FRAME, &frame);
    if (result < 0 && errno == EINTR && interrupted) return false;
    check(result == 0, "GET_FRAME");
    if (frame.slot >= FPGA_FRAME_SLOTS || frame.bytes_used != info.image_bytes ||
        (frame.token & 3U) != frame.slot || !(frame.token >> 2))
      throw std::runtime_error("Invalid frame metadata; stop instead of recycling an unknown slot");
    return true;
  }
  void put(fpga_frame& frame) {
    check(call_ioctl(control_.get(), FPGA_IOC_PUT_FRAME, &frame) == 0, "PUT_FRAME");
  }
  int fd(unsigned slot) const { return source_[slot].get(); }
  fpga_info info{};
 private:
  Fd control_;
  std::array<Fd, FPGA_FRAME_SLOTS> source_;
  bool running_ = false;
};

// 只对首帧验证：RGB565 扩展低位可能采用补零或复制高位，容许相应舍入差。
void verify(const uint8_t* rgb565, const uint8_t* bgr, unsigned width,
            unsigned height, unsigned source_pitch, unsigned target_pitch) {
  size_t bad = 0;
  for (unsigned y = 0; y < height; ++y) {
    for (unsigned x = 0; x < width; ++x) {
      const uint8_t* p = rgb565 + y * source_pitch + x * 2U;
      const unsigned pixel = p[0] | (static_cast<unsigned>(p[1]) << 8);
      const uint8_t* actual = bgr + y * target_pitch + x * 3U;
      const unsigned expected[3] = {(pixel & 31U) << 3, ((pixel >> 5) & 63U) << 2,
                                    ((pixel >> 11) & 31U) << 3};
      const int tolerance[3] = {7, 3, 7};
      bool wrong = false;
      for (unsigned c = 0; c < 3; ++c)
        wrong |= std::abs(static_cast<int>(actual[c]) - static_cast<int>(expected[c])) > tolerance[c];
      bad += wrong;
    }
  }
  std::cout << "verify: pixels=" << static_cast<size_t>(width) * height
            << " mismatched=" << bad << '\n';
  if (bad) throw std::runtime_error("RGA conversion mismatch (channel order/stride/data lifetime)");
}

void save_ppm(const std::string& path, const uint8_t* bgr,
              unsigned width, unsigned height, unsigned pitch) {
  std::ofstream file(path.c_str(), std::ios::binary);
  if (!file) throw std::runtime_error("Cannot create " + path);
  file << "P6\n" << width << ' ' << height << "\n255\n";
  std::vector<uint8_t> row(width * 3U);
  for (unsigned y = 0; y < height; ++y) {
    for (unsigned x = 0; x < width; ++x) {
      const uint8_t* pixel = bgr + y * pitch + x * 3U;
      row[x * 3U] = pixel[2]; row[x * 3U + 1] = pixel[1]; row[x * 3U + 2] = pixel[0];
    }
    file.write(reinterpret_cast<const char*>(row.data()), row.size());
  }
  file.close();
  if (!file) throw std::runtime_error("Failed writing " + path);
}

struct Options {
  std::string device = "/dev/fpga_frame_0000:01:00.0", drm = "/dev/dri/card0", output;
  unsigned frames = 60;
  bool verify = false, help = false;
};
Options parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "--help") options.help = true;
    else if (option == "--verify") options.verify = true;
    else if (option == "--device" || option == "--drm" || option == "--frames" || option == "--output") {
      if (++i == argc) throw std::runtime_error("Missing value for " + option);
      if (option == "--device") options.device = argv[i];
      else if (option == "--drm") options.drm = argv[i];
      else if (option == "--output") options.output = argv[i];
      else {
        char* end = nullptr;
        const unsigned long frames = std::strtoul(argv[i], &end, 10);
        if (!*argv[i] || *end || frames < 1 || frames > 1000000)
          throw std::runtime_error("--frames must be 1..1000000");
        options.frames = static_cast<unsigned>(frames);
      }
    } else throw std::runtime_error("Unknown option: " + option);
  }
  return options;
}

int run(const Options& options) {
  Input input(options.device);
  // 输出先准备好，确保 START 前就发现 DRM 权限/格式等错误。
  BgrOutput output(options.drm, input.info.width, input.info.height);
  if (options.verify) {
    CpuAccess cpu(output.fd(), output.bytes, DMA_BUF_SYNC_WRITE);
    std::memset(cpu.data(), 0xa5, output.bytes);  // 防止“未写入的零内存”伪通过验证。
    cpu.finish();
  }
  const auto target = wrapbuffer_fd_t(output.fd(), input.info.width, input.info.height,
                                     output.pitch / 3, input.info.height, RK_FORMAT_BGR_888);
  std::cout << "DMA-BUF -> RGA: " << input.info.width << 'x' << input.info.height
            << " RGB565 -> BGR888, output_pitch=" << output.pitch << '\n';
  double wait_ms = 0, rga_ms = 0;
  unsigned completed = 0;
  const auto begin = std::chrono::steady_clock::now();
  input.start();
  while (completed < options.frames && !interrupted) {
    const auto before_get = std::chrono::steady_clock::now();
    fpga_frame frame{};
    if (!input.get(frame)) break;
    const auto acquired = std::chrono::steady_clock::now();
    // FPGA RGB565 的 R 在高 5 位，配套 librga 应使用 BGR_565，避免红蓝互换。
    const auto source = wrapbuffer_fd_t(input.fd(frame.slot), input.info.width, input.info.height,
                                       input.info.stride / 2, input.info.height, RK_FORMAT_BGR_565);
    // 最后一个参数 1：同步调用。成功返回后才可归还源槽、访问输出、复用目标。
    const IM_STATUS status = imcvtcolor_t(source, target, RK_FORMAT_BGR_565, RK_FORMAT_BGR_888, 0, 1);
    if (status != IM_STATUS_SUCCESS)
      throw std::runtime_error(std::string("RGA imcvtcolor: ") + imStrError(status));
    const auto converted = std::chrono::steady_clock::now();
    if (!completed && (options.verify || !options.output.empty())) {
      CpuAccess bgr(output.fd(), output.bytes);
      if (options.verify) {
        CpuAccess raw(input.fd(frame.slot), input.info.map_bytes);
        verify(raw.data(), bgr.data(), input.info.width, input.info.height,
               input.info.stride, output.pitch);
        raw.finish();
      }
      if (!options.output.empty())
        save_ppm(options.output, bgr.data(), input.info.width, input.info.height, output.pitch);
      bgr.finish();
    }
    input.put(frame); // RGA 和可选的 CPU 验证均已结束。
    ++completed;
    wait_ms += std::chrono::duration<double, std::milli>(acquired - before_get).count();
    rga_ms += std::chrono::duration<double, std::milli>(converted - acquired).count();
    if (completed == 1)
      std::cout << "first_frame: slot=" << frame.slot << " sequence=" << frame.sequence << " converted and released\n";
  }
  check(input.stop(), "STOP");
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
  std::cout << "frames=" << completed << " fps=" << (seconds > 0 ? completed / seconds : 0)
            << " get_ms=" << (completed ? wait_ms / completed : 0)
            << " rga_ms=" << (completed ? rga_ms / completed : 0) << '\n';
  return interrupted ? 130 : 0;
}
} // namespace pcie_rga

#ifndef PCIE_RGA_PROBE_NO_MAIN
int main(int argc, char** argv) {
  try {
    const auto options = pcie_rga::parse(argc, argv);
    if (options.help) {
      std::cout << "pcie_rga_probe [--device /dev/fpga_frame_BDF] [--drm /dev/dri/card0]\n"
                   "               [--frames 60] [--verify] [--output first.ppm]\n"
                   "Exit the viewer first. Requires driver 8.2 and Rockchip RGA/DRM.\n";
      return 0;
    }
    std::signal(SIGINT, pcie_rga::signal_handler);
    std::signal(SIGTERM, pcie_rga::signal_handler);
    return pcie_rga::run(options);
  } catch (const std::exception& error) {
    std::cerr << "pcie_rga_probe: " << error.what() << '\n';
    return 1;
  }
}
#endif
