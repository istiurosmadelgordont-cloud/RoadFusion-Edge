# PCIe 视频 v4：DMA-BUF + RGA 按需预览缩放

面向鲁班猫 RK3568、ARM64 Linux 4.19.232、Qt 5 / Qt 6。
保留开始/停止、FPS、图像参数编辑、两阶段回执和底部日志。
这次只改本目录的上位机与构建/测试；驱动仍是和 board_v11 相同的 8.2，
视频 ABI 仍为 2，FPGA 协议、board_v11 与识别算法没有改动。

## 这次 RGA 做什么

```text
FPGA -> 原四槽 RGB565 DMA
          -> GET_FRAME 取得完整帧
          -> GUI 需要新预览？
               是：RGA 同步缩放到预览缓冲 -> PUT_FRAME -> Qt 显示
               否：直接 PUT_FRAME，不访问像素
```

四个槽在 START 前导出为 DMA-BUF fd，不再建立 misc mmap。
Qt 可以直接显示 RGB565，因此这里不转 BGR，直接缩放 1920x1080 RGB565。
默认图像窗口为 960x540；后台根据窗口需求选择 16:9 预览尺寸，最大为原图。
尺寸按 32x18 为单位向下取整，最小 128x72，以满足对齐与缩放范围。
窗口不是正好 16:9、发生 DPI 缩放或大于原图时，Qt 仍可能做最后一点缩放。
所以这不是完全取消 Qt 绘图，也没有把显示缓冲直接交给显示控制器。

后台仅准备 GUI 需要的预览，不在采集线程无条件生成每一帧预览。
三个普通 RGB565 预览缓冲采用整页独占内存和 QImage 共享引用管理，
仍被窗口/消费者使用的图像不会被覆盖。RGA 成功同步返回后才归还 FPGA 槽。
输出是普通用户内存，由 RGA 驱动负责页映射和缓存维护，仍有相应开销。
出现 RGA 错误会先停止采集，不在同一次打开中静默改用 read。

窗口关闭或停止时：请求 STOP -> 等接收线程和同步 RGA 结束 ->
处理持有帧归还 -> 释放导出 fd -> 关闭设备；不强制终止工作线程。
预览副本不持有 FPGA 槽，停止之后仍可由普通 QImage 引用保护。

## 四种方式与切换

ARM64 完整构建默认选择 RGA。点击停止，等会话结束后可切换其他方式。

| 界面选项 | 像素路径 |
| --- | --- |
| DMA-BUF + RGA（预览缩放） | 原 DMA 槽 -> RGA 小尺寸 RGB565 -> Qt 绘制 |
| read（复制） | DMA -> 内核副本 -> 原尺寸 QImage -> Qt 缩放/绘制 |
| mmap（显示缓冲） | DMA 映射 -> 原尺寸 QImage 副本 -> Qt 缩放/绘制 |
| mmap（直接显示，对照） | QImage 引用 DMA 映射，Qt 直接访问并缩放；最后消费者结束才归还槽 |

mmap 的两种方式和 read 不调用 RGA，作为对照保留。减少显式复制不保证更快。
RGA 预览缓冲是另外的较小输出图像，不复制一套原图四槽，也不改变原四槽协议。
本查看器没有 YOLO、MobileNet 或其他识别模型，不依赖 OpenCV 或 libdrm。

## 在板子上编译和启动

将整个 pcie_v4 目录复制到板子，包含 third_party/librga，在桌面用户终端运行：

```sh
cd /你的路径/pcie_v4
bash run.sh
```

脚本编译驱动和 Qt，检查已加载模块的源版本，必要时加载驱动、设置节点读权限，
最后打开上位机。不要使用 sudo bash run.sh；仅必要操作在脚本内使用 sudo。
默认采用匹配的 /lib/modules/$(uname -r)/build；如需指定头文件：

```sh
KERNEL_DIR=/usr/src/linux-headers-$(uname -r) bash run.sh
```

首次打开点击开始；也可指定模式并自动开始：

```sh
bash run.sh --mode rga --start
bash run.sh --mode read --start
bash run.sh --mode mmap --start
bash run.sh --mode mmap-direct --start
```

只编译：bash run.sh --build-only。
多设备可加 /dev/fpga_frame_0000:01:00.0。
如果已编译且驱动/权限均就绪，也能直接执行：

```sh
./build/pcie_viewer --mode rga
```

本次内核代码不变，已加载同一份 8.2 驱动可继续复用，不必为了 RGA 预览重载。
脚本不自动卸载已加载模块。若源版本不符，先正常退出所有采集程序，
再 sudo rmmod my_pciedriver 后重新运行；不要强制卸载。
Qt 查看器和 board_v11 识别不能同时打开同一个采集设备。

## 构建依赖和私有 RGA

需要 build-essential、CMake、Qt Widgets 开发包及当前内核的完整构建头文件。
兼容板上的 CMake 3.13.4 与 Qt 5.11.3。
ARM64 默认将 third_party/librga 的官方 1.10.6 配套静态库/头文件链接到上位机。
这是为避免系统旧 librga 1.3.1 与 multicore 内核不匹配；不替换系统库或升级 glibc。
许可证、来源和哈希见 third_party/librga/README.md。

普通 x86 Linux 只能编译不启用 RGA 的 read/mmap 版本；这不模拟 PCIe 设备：

```sh
cmake -S . -B build -DPCIE_ENABLE_RGA=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
```

## FPS、日志和图像参数

采集 FPS 是完整 GET/read 数量；RGA 模式也包括为了保持实时而直接归还的帧。
显示 FPS 只统计新预览真正调用 paintEvent 绘制的次数，二者不能混为一谈。
每约 5 秒记录一次 RGA 预览尺寸、缩放次数和平均同步调用耗时，
以及采集/显示 FPS 与平均绘图耗时。绘图计时不包含 paintEvent 之后的系统呈现。
GUI 每约 16ms 取一次预览，属于调度方式，不保证每次都完成绘图或实际达到 60FPS。

图像参数仍通过独立 ioctl 提交五字配置，使用匹配 FPGA 的 RECEIVED/APPLIED。
编辑不会自动发送；点击应用才提交。超时不自动重发，也不清除在途状态。
RGA 缩放只改变本地预览大小，图像处理参数仍作用于 FPGA 整幅图像。
驱动能力不能证明 FPGA 位流版本；APPLIED 不代表对应图像已显示到屏幕。

## 验证

```sh
bash tests/run_local.sh
```

测试执行实际 Qt/C++ 源码并替换 PCIe/RGA 边界，不需要 root 或 FPGA。
覆盖原 read/mmap、两阶段回执、DMA-BUF 部分导出失败、页对齐、RGB565 缩放、
池满时跳帧、窗口尺寸更新、预览引用寿命、RGA 并发 STOP 和异常退出。
模拟回归只能检查程序逻辑，不能代替实机像素、缓存一致性和性能验证。

2026-10-27：已在鲁班猫的 GCC 8.3、Qt 5.11.3 环境完成 ARM64 上位机构建，
驱动/启动脚本和 read/mmap/RGA 的软件模拟回归均通过。
RGA 模拟使用随工程提供的正式头文件，执行边界由测试替换。
用户选择先只更新文件，因此本次没有运行真实 FPGA/RGA 预览或 read/RGA 性能对照，
没有把新界面强行覆盖到当前运行会话；重新启动查看器后才使用新版本。
当前性能变化尚无实测结论。实际采集的 8.2 驱动文件没有变化。
