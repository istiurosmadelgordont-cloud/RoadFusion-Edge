# 驱动 8.1：等待精度与设备锁优化

本版只修改 board_v10/driver，视频 ABI 仍是 2，支持原 read/mmap 用户程序。
默认 read、1920×1080 RGB565、四槽、帧序选择、STOP cookie、两阶段图像回执及异常隔离协议不变。
原工作区 pcie_v3、pcie_v4 和 FPGA 源码未在这次优化中修改。

## 改动

1. 等待新帧使用 wait_event_interruptible_hrtimeout，默认间隔 1000us。
   板端 CONFIG_HIGH_RES_TIMERS=y、HZ=300；旧 msecs_to_jiffies(1) 会按一个约 3.33ms 的 tick 等待。
   新实现不是忙等，实际唤醒时间仍受系统负载和定时器余量影响。
   STOP/remove 仍通过原 waitq 提前唤醒，总取帧超时仍是 5 秒；单次 -ETIME 只表示重新检查帧尾。
2. DMA → snapshot 的第一次复制仍持有设备锁，复制完后立即归还槽。
   snapshot → 用户的第二次复制期间释放设备锁，只保留 read_lock，避免用户缺页拖住控制命令。
   snapshot 的普通内存由打开文件的引用保护，不能在 read 返回前释放，也不会被另一 read 覆盖。
   复制期间如发生 STOP、重启或解绑，仅向原调用者返回已复制的旧帧字节，不把旧进度写回新会话。
   应用执行 STOP 后仍应结束/丢弃在途读帧，再开始新会话；STOP 不表示用户线程已经退出 read。
3. 图像回执仅在 image_pending 为真时扫描邮箱；RECEIVED 已确认后只继续检查 APPLIED。
   已完成结果使用缓存，迟到回执、超时禁止覆盖及 STOP 排空流程保留。

read 仍是两次复制，mmap 的映射及所有权规则不变。本次没有修改 DMA 缓存属性，
也没有改变 YOLO/MO/UFLD 调度、视频缩放或 GUI 绘制；不能用这次改动推断显示 FPS 一定提高。

## 编译和切换

板端已完成编译，新的模块位于当前工程 driver/my_pciedriver.ko。
当前加载的模块不会因替换文件而改变；先退出所有采集程序，再切换：

```sh
cd /home/cat/rk3568_adas/board_v10
# 如在另一台同内核板上部署源码，先执行 make -C driver
sudo rmmod my_pciedriver && sudo insmod driver/my_pciedriver.ko
sudo chown "$(id -un):$(id -gn)" /dev/fpga_frame_*
cat /sys/module/my_pciedriver/version
sh scripts/run_pcie.sh
```

卸载失败时先处理占用或未完成 STOP，不强制卸载。
应用、Qt 和 FPGA 位流无需因驱动 8.1 重新编译或烧录。

## 轮询间隔与对比

默认 1000us；非零输入实际钳位到 100~20000us，0 使用旧 tick 等待方式。
可在加载时设置 insmod driver/my_pciedriver.ko frame_poll_us=1000，或运行时设置：

```sh
# 旧 tick 等待，仅用于比较；不改变视频/STOP 协议
printf '0\n' | sudo tee /sys/module/my_pciedriver/parameters/frame_poll_us
# 恢复默认高精度等待
printf '1000\n' | sudo tee /sys/module/my_pciedriver/parameters/frame_poll_us
```

更短的间隔会增加无帧时的唤醒次数；500us 或更短不保证提高吞吐。
同一测试视频、同一 NPU/车道开关下，比较 pcie_perf 的 fps/read_ms、控制按钮响应和 CPU 占用。
0 与 1000 的比较只隔离等待方式，完整比较 8.0/8.1 还需分别加载两个版本。
目前尚未加载 8.1 进行实际视频性能对比，不提供提升幅度结论。

## 已完成的检查

- 用鲁班猫实际 4.19.232 ARM64 内核头文件成功编译，modinfo 为 8.1，vermagic 与板端匹配。
- tests/driver_io_regression.py 抽取真实 read/等待函数：覆盖短读、部分复制/EFAULT、
  复制期间另一线程 STOP/重启/解绑、旧进度不回写、新旧等待方式、信号/会话/超时边界。
- tests/driver_control_regression.py 抽取真实控制函数：覆盖部分/旧回执、单次在途、
  完成结果缓存、STOP 超时及排空、cookie 生命周期、DMA 地址重叠和 ABI。

运行软件检查：python3 tests/driver_io_regression.py && python3 tests/driver_control_regression.py。
它们使用模拟内核依赖，不代替真实 DMA、性能及板级联调。
