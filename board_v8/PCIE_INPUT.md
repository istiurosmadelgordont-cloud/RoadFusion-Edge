# board_v8 PCIe 实时输入

数据流：FPGA 四路画面拼成一个 1920×1080 RGB565 帧 → HS2 驱动
`/dev/pcie_hdmi_host` → 独立采集线程转 BGR → 最近一帧 → v8 识别及四宫格 UI。
左上前视、右上后视、左下左视、右下右视。YOLO 每次处理完整合成帧，
保持模型原有 640 输入预处理；前后视 UFLD 使用原始 960×540 象限。

## 板端编译和启动

```sh
cd ~/rk3568_adas/board_v8
cmake -S . -B build
cmake --build build --target adas_four_view -- -j2
export DISPLAY=:0
export XAUTHORITY=/home/cat/.Xauthority
sh scripts/run_pcie.sh
```

模型放在 `models/`（也可链接已有模型目录）：

- `unified21_p2_v7_640_int8.rknn`
- `ufldv2_culane_res18_1600x320_int8.rknn`

可通过 `MODEL`、`UFLD_MODEL`、`PCIE_DEVICE` 环境变量覆盖路径。
脚本参数追加到程序参数后面，例如 `--max-frames 300 --snapshot /tmp/v8.png`。
`--headless` 关闭窗口，`--benchmark` 取消显示节流。默认显示上限 15 FPS，
实际帧率由日志统计，不代表每种模型均以 15 FPS 推理。

## 调度与性能

- 主线程在创建 RKNN 上下文前绑定 CPU 2、3，逻辑、显示及应用推理工作线程继承该亲和性。
- PCIe 采集线程单独绑定 CPU 0、1，负责阻塞读取和 RGB565 转 BGR。
- YOLO、1600 UFLD 通过 RKNN 在 NPU 执行，沿用 v8 单 NPU 队列调度，避免互相排队。
- 只保留最新完整帧，较慢的消费者跳过旧帧；引用计数保证推理中的图像不会被覆盖。
- 这仍有驱动读拷贝和颜色转换，不是零拷贝。慢速采集、模型耗时均独立记录。
- UFLD 源码与 GitHub `ed129600bd1c20aaffc475cd012b82c8700489e1` 一致。
  PCIe 的原始象限以带行跨度的 ROI 传入，按上游条件走 CPU 缩放预处理，
  避开旧 RGA 虚拟地址接口失败；车道模型本身仍由 NPU 执行。
- PCIe 模式强制 1600 UFLDv2，前后视均使用神经网络车道；缺模型或尺寸不符报错，
  不进入传统 OpenCV 车道检测分支。OpenCV 仍作为图像处理依赖。
- 预览使用固定 3:2 双线性缩放；已加入与 OpenCV 结果的像素对比测试。
- PCIe 车道结果接收期限为真实采集时刻起 450 ms，包含预览及工作线程交接；
  MP4 保留 300 ms。原有前视 1000 ms、后视 1400 ms 显示过期规则保留。
- `pcie_perf` 是完整帧采集率；`fps_1s` 是实际显示率；`npu_ms`、`lane_ms`
  是各识别任务耗时。显示使用最近检测结果及 v8 原有跟踪/过期规则。

实时输入不使用 MP4 时钟，不支持回放跳转、切换场景或逐路 YOLO 模式。
不传 `--pcie` 仍保留原 MP4 功能。`q`、Esc、SIGINT、SIGTERM 正常退出时
等待采集线程结束并发送 HS2 STOP。驱动超时会报错退出，STOP 失败会明确记录。
设备应只被一个采集程序打开，先关闭旧版 Qt 或旧识别进程。

如驱动支持 `direct_copy`，可检查 `/sys/module/pcie_hdmi_host/parameters/direct_copy`。
启用该驱动选项可减少中间拷贝；它属于驱动运行参数，重载驱动后可能恢复默认。
程序不会自动修改系统驱动参数。
