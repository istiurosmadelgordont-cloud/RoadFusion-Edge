# ARM fast v2

只改变 ARM 驱动，不改 FPGA。保留四地址初始化、不发 DMA CLEAR、增强写接口、两次内存复制及复制前后标记检查。

默认选择较新的未交付候选帧，而不是第一个轮询命中。按 1..255 帧号作半周期比较，零无效；各缓冲区标记相差超过半周期或存在长期未完成缓冲区时先后顺序可能不准确。并非可靠的时间戳，也不消除原协议撕裂风险。禁止把性能提升等同于完整性提升。

每约 5 秒或超时输出 perf；各字段是这段统计窗口累计值。frames=交付帧数，attempts=图像复制次数，retries=复制前后标记改变次数，timeouts=超时数。read_us 为读取函数测量区间总时间，dma_copy_us/user_copy_us 为两次复制总时间（包含调度影响）。总读取时间也包含复制时间，不能相加。

复制 driver/pcie_hdmi_host.c 到 ARM 后 clean/build，modinfo -F version driver/pcie_hdmi_host.ko 必须为 2026.09.15-arm-fast-v2。旧流启动后模块驻留，需要双方关机/FPGA复位并先配置 FPGA，再启动 ARM；不改 FPGA 代码。

加载驱动后先仅运行 benchmark_capture.py --seconds 15，再看 dmesg | grep 'perf '。普通用户需 sudo。

对比原轮询：关闭 GUI/测速后，echo N | sudo tee /sys/module/pcie_hdmi_host/parameters/newest_first，再运行相同测速；恢复使用 Y。每种模式建议至少两次，统计日志跨切换的第一条可能混合两种模式。

源码结构检查完成；此环境没有 ARM 内核构建工具链，未编译或验证硬件提升。
