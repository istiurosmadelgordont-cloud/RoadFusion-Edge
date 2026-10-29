# board_v11 鲁班猫部署记录：2026-10-27

## 位置与构建

目标：192.168.137.17，用户 cat，EmbedFire LubanCat-2 / RK3568。
内核：4.19.232，ARM64；GCC 8.3.0；OpenCV 3.2.0；FreeType 2.9.1。
工程：/home/cat/rk3568_adas/board_v11。

在板子本机执行 sh scripts/build.sh，构建 adas_four_view、adas_demo、ufld_benchmark。
驱动通过 make -C driver KERNEL_DIR=/usr/src/linux-headers-4.19.232 -j2 编译。
最终识别程序为 ARM aarch64 ELF；驱动 vermagic 为 4.19.232 SMP mod_unload aarch64。
已有驱动与新模块的 srcversion 都是 EF6F51B5763CCA26C5CB4C6，版本 8.2，未重载模块。

第一阶段（加入 RGA 预览前）的 SHA256：

- build/adas_four_view：aefe677112c06a0c625fe78b8fcc4d7f2d5e7da5a46f2a3ce7f221602cfe4640
- driver/my_pciedriver.ko：e6cc5a0ebd31eef835ed6f155b140663cae204d3e1b682b348f6b1d307f71f02
- third_party/librga/lib/librga.a：853159434f487f1bd852db4c1d45eeb4a6b1629ba4c93f2cb2fbd5b20f11ec67

同日后续接入 RGA 预览的构建与验证见 [预览接入说明](PCIE_RGA_PREVIEW.md)，
下文的真实视频帧率属于第一阶段，不能视作新预览路径的结果。

板上 models 链接到 ../board_v10/models；four_view_sample 链接到
../board_v10/four_view_sample，不复制或改动原模型/素材。不要删除这些链接的目标。

## 本次修复的真实兼容问题

首次构建可通过，但系统 librga API 1.3.1 与 RGA multicore 内核 v1.2.26 不匹配。
RGA 转换报 Invalid argument；dmesg 显示输出虚拟地址被当作 DMA-BUF fd：

    rga_mm_get_channel_external_buffer dma_buf_get fail fd[547606290432]

项目增加 third_party/librga 的官方 1.10.6_[3] 配套头文件和 ARM64 静态库，
CMake 在 ARM64 默认选择这套文件。上游动态库要求 GLIBC_2.29，而板子是
GLIBC_2.28，因此用静态库在板子工具链上链接；不升级系统 glibc 或替换系统 RGA。
完整程序运行时确认打印 rga_api version 1.10.6_[3]。
启动日志也改为实际 PCIe mode=rga/read，不再固定误报 pcie_v3 read。

依据：[Rockchip 官方兼容表](https://github.com/airockchip/librga/blob/main/docs/Rockchip_FAQ_RGA_CN.md#版本对应关系)。
上游许可、版本和快照哈希见 ../third_party/librga/README.md。

## 实机检查

1. 实际 PcieCapture 类 + 配套 RGA：连续转换 90 帧，输出 1920x1080 BGR，
   返回 RGA_CAPTURE_SMOKE_PASS frames=90。正常 STOP、释放所有导出引用。
2. 完整程序：在板子桌面运行下面命令，90 帧后自动退出，进程返回 0。

```sh
cd /home/cat/rk3568_adas/board_v11
export DISPLAY=:0 XAUTHORITY=/home/cat/.Xauthority
sh scripts/run_pcie.sh --max-frames 90 \
  --snapshot /tmp/board-v11-rga-validation.jpg --snapshot-frame 30
```

观察：

- PCIe capture mode=rga BGR_pool=6；rga_api version 1.10.6_[3]。
- YOLO/MO 模型初始化成功；YOLO 持续输出检测结果，示例为后视汽车检测框。
- GLES renderer: Mali-G52；窗口可呈现四路面板、输入画面和检测叠加。
- pcie_perf：fps=59.8681，get_ms=8.53085，rga_ms=8.14323，pool_drops=0。
- Done. shown=90 wall_fps=10.6096；YOLO 结果更新样本约 5.5 Hz。
- 退出后 fuser 不再发现采集设备占用。

这些数据为约 8 秒的当前输入短测。车道与 TTC 默认关闭，输入包含测试画面及黑区，
并非标准四路同素材性能对比；wall_fps 不是物理屏幕刷新测量。
没有测试 read/RGA 加速比、UFLD 开启后的性能、MO 灯态准确率、所有按钮交互、
参数 RECEIVED/APPLIED 回传或长期稳定性。截图只确认输出和叠加存在，不等于准确率通过。

板端临时日志：/tmp/board-v11-build-final.log、/tmp/board-v11-driver-build.log、
/tmp/board-v11-rga-smoke.log、/tmp/board-v11-rga-validation.log。
验证截图为 /tmp/board-v11-rga-validation.jpg。临时文件重启后可能丢失。

## 用户启动

在鲁班猫桌面终端执行：

```sh
cd /home/cat/rk3568_adas/board_v11
sh scripts/run_pcie.sh
```

默认 RGA，按 Q / Esc 正常退出；对比 read 添加 --pcie-io read。
Qt 查看器和识别程序不能同时打开同一个采集设备。板子当前已加载匹配驱动，
无需再次 insmod；重启后若没有设备节点，需要先加载 driver/my_pciedriver.ko 并设置权限。
