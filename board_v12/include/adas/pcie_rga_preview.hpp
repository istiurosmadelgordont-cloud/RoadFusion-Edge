#pragma once
#include "adas/pcie_rga.hpp"
#include "adas/rga_image_ops.hpp"
#include <array>
#include <map>
#include <memory>
#include <opencv2/core.hpp>

namespace adas {

/*
 * PCIe 实时预览和显示缩放使用独立的同步 RGA 输出池。
 * 识别仍读取原始 1920x1080 BGR；预览生成 1280x720 四路 ROI，显示阶段再缩放到各视图尺寸。
 * 同步完成后才发布输出。预览不会持有 FPGA 槽，也不修改识别原图。
 * 只有界面线程调用本类；Mat/ROI 的引用计数防止池覆盖仍在使用的图像。
 */
class PcieRgaPreview {
 public:
  PcieRgaPreview() : pool_(3, 1280, 720) {}
  PcieRgaPreview(const PcieRgaPreview&) = delete;
  PcieRgaPreview& operator=(const PcieRgaPreview&) = delete;

  bool resize(const cv::Mat& source, cv::Mat& destination) {
    if (source.type() != CV_8UC3 || source.cols != 1920 || source.rows != 1080 ||
        source.step != 1920U * 3 || !source.isContinuous())
      throw std::runtime_error("RGA preview requires packed 1920x1080 BGR888");
    cv::Mat output = pool_.acquire();
    if (output.empty()) return false; // 消费者占满三块时保留旧预览，不覆盖、不额外分配。
    const auto input = wrapbuffer_virtualaddr_t(const_cast<unsigned char*>(source.data),
        source.cols, source.rows, source.step / 3, source.rows, RK_FORMAT_BGR_888);
    const auto target = wrapbuffer_virtualaddr_t(output.data, output.cols, output.rows,
        output.step / 3, output.rows, RK_FORMAT_BGR_888);
    // 输入和输出都是整页独占的普通 BGR 内存，由匹配的 RGA 驱动维护缓存。
    // sync=1：函数返回前，不能让 UI 绘图或复用输出；也不能释放输入引用。
    const IM_STATUS status = imresize_t(input, target, 0, 0, IM_INTERP_DEFAULT, 1);
    if (status != IM_STATUS_SUCCESS)
      throw std::runtime_error(std::string("PCIe RGA preview resize failed: ") + imStrError_t(status));
    destination = output;
    return true;
  }

  bool resize_tile(const cv::Mat& source, cv::Mat& destination,
                   cv::Size size = cv::Size(672, 378)) {
    if (source.empty() || source.type() != CV_8UC3)
      throw std::runtime_error("RGA tile resize requires a BGR888 source");
    const bool supported=size==cv::Size(672,378)||size==cv::Size(1356,762)||
      size==cv::Size(656,369)||size==cv::Size(1328,747)||
      size==cv::Size(960,540)||size==cv::Size(432,243);
    if(!supported)throw std::runtime_error("Unsupported RGA display tile size");
    const auto key=std::make_pair(size.width,size.height);
    auto& pool=display_pools_[key];
    if(!pool)pool.reset(new PcieBgrPool(3,size.width,size.height));
    cv::Mat output=pool->acquire();
    if (output.empty()) return false;
    rga_image_ops::resize_bgr(source, output);
    destination = output;
    return true;
  }

  static void split(const cv::Mat& preview, std::array<cv::Mat,4>& views) {
    CV_Assert(preview.type() == CV_8UC3 && preview.size() == cv::Size(1280,720));
    for (int i=0; i<4; ++i)
      views[i] = preview(cv::Rect((i%2)*640, (i/2)*360, 640,360));
    // ROI 仅增加共享引用，不做四路 copyTo。绘制仍使用原来的独立 clone，
    // GLES 收到的帧保持连续内存，原推理/HSV 路径也不会改读这份预览。
  }

 private:
  std::map<std::pair<int,int>,std::unique_ptr<PcieBgrPool>> display_pools_;
  PcieBgrPool pool_;
};
} // namespace adas
