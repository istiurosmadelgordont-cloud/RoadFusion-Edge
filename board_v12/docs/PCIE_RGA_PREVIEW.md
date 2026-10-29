# board_v12 PCIe 图像处理路径

board_v12 的 PCIe 实时采集固定使用 DMA-BUF + 同步 RGA；预览和模型输入的可硬件加速缩放、通道转换也走 RGA。启动入口为 `sh scripts/run_pcie.sh`，没有 read/CPU 预览切换选项。

```text
FPGA RGB565
  -> driver 8.2 DMA-BUF
  -> 同步 RGA 转换为 1920x1080 BGR
      -> RGA 缩放为 1280x720 预览，拆成四个 640x360 视图
      -> YOLO/MO/UFLDv2 输入缩放与 BGR->RGB 转换由 RGA 完成
      -> OpenCV 负责检测/车道/灯态算法、跟踪、文字与图形叠加
  -> 实时预览放大由 RGA 完成，再由 GLES 显示
```

采集、预览、YOLO、MO 和 PCIe 实时 UFLDv2 的 RGA 输出都使用有界、页对齐的 BGR/RGB 池，并在同步转换结束后才交给消费者。缩放操作支持带步长的图像视图，UFLDv2 可直接读取四宫格帧中的单路 ROI。池中图像仍由 OpenCV `cv::Mat` 引用计数持有，避免异步推理或显示期间覆盖数据。

RGA 执行失败时，PCIe 实时采集和模型输入会明确报错，不切换到 CPU 图像转换。文件回放仍由原 OpenCV 解码入口读取；回放尺寸不受 RGA 支持时，UFLD 保留 CPU 兼容处理。

RGA 负责像素搬运、格式转换和缩放，OpenCV 继续承载 `cv::Mat`、目标解码/NMS、HSV/连通域、车道拟合、跟踪和绘制。两者职责不同，当前版本已把实时 PCIe 路径中可直接替换的缩放及通道转换迁到 RGA。

## 构建与启动

```sh
cd /home/cat/rk3568_adas/board_v12
sh scripts/build.sh
sh scripts/run_pcie.sh
```

模型仍从 `models/` 或 `../models/` 读取，PCIe 设备多于一个时通过 `PCIE_DEVICE=/dev/fpga_frame_<BDF>` 指定。运行时需要板端驱动 8.2 的 DMA-BUF 导出能力和本目录配套的 librga。

## 回归和验证

`pcie_rga_capture_regression` 使用 PCIe/RGA mock 验证固定采集路径、同步转换、缓冲池寿命、STOP 及错误处理。`pcie_rga_color_hardware` 在板端检查 RGB565 全量颜色、BGR/RGB 通道转换、带步长 ROI 的 RGA 缩放和预览颜色。该硬件回归不访问 PCIe 设备，也不代表完整模型 FPS 或视频端到端帧率。

```sh
cmake --build build --target pcie_rga_capture_regression pcie_rga_color_hardware -- -j2
./build/pcie_rga_capture_regression
./build/pcie_rga_color_hardware
```
