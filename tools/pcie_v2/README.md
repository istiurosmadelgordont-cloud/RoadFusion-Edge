# PCIe 视频与图像参数控制：驱动 7.0

> 当前入口说明（2026-10-26）：本目录保留驱动 **7.0 单向参数提交**的独立 Qt 查看器。最新 ADAS 与 pcie_v3 8.0/8.1 双阶段回执入口见 [board_v10](https://github.com/istiurosmadelgordont-cloud/RoadFusion-Edge/blob/a6ab14ee72b9bc163f0428a35f9c44413b2e2eaa/board_v10/CURRENT.md) 和 [图像控制说明](https://github.com/istiurosmadelgordont-cloud/RoadFusion-Edge/blob/a6ab14ee72b9bc163f0428a35f9c44413b2e2eaa/board_v10/docs/PCIE_V3_IMAGE_CONTROLS.md)。以下 7.0 操作与验证记录按原版本保留；两个程序不能同时占用同一设备。

目标为 LubanCat ARM64、Linux 4.19.232、Qt 5.11.3 或兼容版本。
本版对应 `board2_sfp_pcie` 的 20261015 FPGA 工程：图像参数控制合入
`pcie_dma_ctrl`，没有 0x220/0x224 配置状态 DMA。目录名仍为 pcie_v2，
视频 ABI 保持 2，模块版本更新为 7.0。旧 20261011 位流不适用本版调参。

## 编译与启动

将本目录完整复制到板子，在普通桌面用户终端运行：

```sh
cd /你的路径/pcie_v2
bash run.sh
# 只编译，不加载驱动、不打开窗口：
bash run.sh --build-only
```

脚本编译驱动和 Qt、核对模块源版本、加载模块、设置节点读取权限，
然后打开查看器。不要用 sudo bash run.sh；仅必要操作由脚本调用 sudo。
不会自动卸载已加载的旧模块。正常停止并关闭全部采集程序后可以正常
卸载旧模块；如果旧 6.0 曾登记配置 DMA，必须按旧版约束重启主机并可靠
复位 FPGA，不能强制卸载。首次绑定仍要求 FPGA 已可靠复位且无旧 DMA。

设备为 `/dev/fpga_frame_<BDF>`；可 `bash run.sh /dev/fpga_frame_0000:01:00.0`。
Qt 和 board_v9 不能同时打开同一设备。board_v9 继续使用原采集接口，
更新后的启动脚本要求模块 7.0。用户 UAPI 副本已同步到 board_v9/include/adas。

## Qt 操作

1. 选择 read 或 mmap，点击开始，图像窗口弹出，主窗口统计采集/显示 FPS。
2. 编辑参数或选择预设不会自动发送；点击“应用参数”才提交整包。
3. 只在提交 ioctl 期间禁止重复点击；返回后可以再次手动提交。
4. 成功显示“参数已提交；FPGA 不回传生效确认”。这不是 APPLIED。
5. 错误会显示“参数提交失败”，不会自动重发，也不会轮询旧配置接口。
6. 停止或关闭图像窗口：STOP、归还所有持有帧、解除映射、关闭设备。

停止视频不清除图像处理参数，参数也作用于 FPGA HDMI 输出。四个槽承载
同一合成视频流；所有参数对整张 1920x1080 图像生效。正常调参不会再使
模块一直保留到重启，但 STOP 故障或强制解绑的隔离保护仍存在。

当前 FPGA 仅能在内部确认帧边界应用，软件没有接受/拒绝/生效反馈。
快速连续提交时中间配置可能被合并，只保留最新待处理项。观察三个完整
视频帧可用于比较画面，但不能代替硬件配置 ACK。预设仍为联调起点，
没有实际画质验收；Gamma 使用固定 LUT。

## 保持不变的视频接口

- `GET_INFO`：72 字节，视频 ABI=2；`GET_FRAME/PUT_FRAME`：32 字节。
- `START/STOP/GET_INFO/GET_FRAME/PUT_FRAME` 的命令号 0x01~0x05 不变。
- BAR1：0x110 四次登记图像地址，0x180 登记独立 STOP 状态区。
- 0x190 START，0x130 STOP(cookie)，0x170 RELEASE(token)，0x1A0 清地址配置。
- 图像尾部 64 字节，含 16 个相同 LE32 token；STOP 也需 16 字全匹配。
- read：DMA -> 内核副本 -> 用户图像；mmap：只读映射四槽，GET/PUT 管理使用权。
- STOP cookie 继续使用唯一编号，超时重试只等原 cookie。

STOP 失败时继续保留 DMA 与模块引用；还有消费者或 mmap 时不得提前释放。
强制解绑时如无法安全释放，保留资源并拒绝重新绑定。本版没有增加未经
板级验证的自动复位或热插拔恢复。没有 BAR 读完成和 MSI，禁止 readl 刷新。

## 新的参数 UAPI

`FPGA_IOC_GET_CAPS`（0x13）返回 16 字节驱动能力：
`version=1`、`flags & FPGA_CAP_IMAGE_SUBMIT`。这只是驱动能力声明，不能
证明实际 FPGA 位流版本；实际位流仍需由部署流程匹配。

`FPGA_IOC_SUBMIT_IMAGE`（0x12，_IOW，32 字节）：

```c
struct fpga_image_submit {
    __u32 words[5];
    __u32 reserved[3]; /* 全部为 0 */
};
```

驱动在同一 mutex 内验证状态、钳位参数、写 0x200/204/208/20C/210，
最后写 0x214=1。返回 0 只说明执行了提交写操作，不保证硬件已经接受或
应用。没有提交后的用户复制，没有配置 DMA、配置 cookie 或软件 APPLIED。

旧 `FPGA_IOC_SET_IMAGE/GET_IMAGE`（0x10/0x11）保留原编号和结构布局，
调用立即返回 EOPNOTSUPP；新程序不可再用。Qt 在 START 前查询 GET_CAPS，
避免搭配旧驱动时使用错误配置协议。驱动仍需与对应 FPGA 位流配套。

BAR 只需覆盖至 0x217（长度 0x218），不再分配 image_ack 或写 0x220/224。
原 0x140/150/160 命令可由 FPGA 兼容，但本驱动统一通过五字整包设置，
避免混用控制入口产生半包参数。

## 文件

- my_pciedriver.c：四槽视频、STOP 安全和单向参数提交。
- fpga_uapi.h：固定宽度接口；image_protocol.h：参数钳位。
- main.cpp：Qt 采集、参数编辑与单向提交、帧率。
- run.sh / Makefile / CMakeLists.txt：加载和构建流程。
- tests/：不连接 FPGA 的控制、Qt 及视频兼容回归，验证结果见本次交付说明。

本次软件修改不会自动解决此前真实 FPGA 无完整帧的问题；像素字节序、
DMA 时序、实际帧边界应用与画质仍需上板验证。

## 2026-10-16 本机验证

本次未连接鲁班猫、未加载模块、未启动真实 FPGA 采集。

- WSL Ubuntu 22.04，GCC 11.4，Qt 5.15.3：CMake Release 完整构建通过。
- Linux 5.15.0-25 x86 内核头文件：完整模块构建通过，产物仅在临时目录。
  此模块不能加载到 ARM64 板子；交付源文件必须在板子上按 4.19.232 构建。
- `bash tests/run_local.sh` 通过。真实驱动函数的用户态模拟覆盖参数钳位、
  六次写入顺序、非法保留位/错误状态/用户复制失败不写硬件、旧 ioctl 拒绝、
  视频 ABI 不变、STOP 确认释放引用、超时保留和 mmap 解绑隔离。
- Qt 回归编译真实 main.cpp，只替换设备系统调用，覆盖十个预设、只提交
  消息、失败后手动再提交、没有旧查询和自动重发、旧驱动拒绝、read/mmap
  帧引用归还以及 FPS 重绘排除/清零。
- `basic_read/mmap/get_frame/put_frame_locked/frame_ready/start_locked/
  init_protocol/stop_ack` 八个关键函数与改动前逐字一致。
- board_v9 的 UAPI 头与本目录逐字一致，两个启动脚本语法检查通过。

复现模拟测试（普通 Linux 用户，无需设备，需 C/C++、Python3、pkg-config、
Qt Widgets 开发包）：

```sh
bash tests/run_local.sh
```

Linux 4.19.232 ARM64 原生编译、真实 DMA 收帧、参数帧边界应用和实际
画质尚未验证。测试只证明软件行为符合新接口，不证明 FPGA 已在运行
对应位流，也不证明此前黑屏原因已消除。


## 2026-10-16 板端 Qt 启动修复

随后用户开机，已在 `/home/cat/pcie_v2` 检查并修复 Qt 启动：
原 build/pcie_viewer 和 main.cpp.o 均为 0 字节，空可执行文件可能被 shell
当作空脚本执行，导致无窗口、无报错。执行 Qt clean-first 完整重编译，
得到有效 ARM64 ELF，Qt 5.11.3 主窗口已确认 IsViewable。

run.sh 已同步空产物保护：编译前如发现空的 Qt 目标文件或可执行文件，
自动 clean-first；编译后再次检查程序非空且可执行。新的启动检查已在
本地覆盖正常产物、空程序、空目标文件、重建后仍空及不可执行五种情况。
原脚本在板子保留为 run.sh.before_empty_build_fix；工程源代码和视频协议
没有因本次启动修复改变。该记录仅确认窗口启动，不代替硬件收帧验收。
