# pango_pcie_dma_alloc — 紫光同创 PCIe 测试平台（主机侧）

紫光同创官方 PCIe 测试平台套件，包含 **Linux 内核驱动 + GTK 上位机 + 参考 RTL + 应用指南**，
用于验证主机（Root Complex）与 FPGA Endpoint 之间的 PCIe 链路、PIO 读写，以及 DMA 回环与带宽性能。

## 目录结构

```text
app_pcie/                  GTK+-2.0 图形上位机（C 源码 + Makefile）
  includes/                头文件：color.h（打印颜色）、config_gui.h（GUI 配置）
  sources/main.c           主程序
doc/
  PCIe测试平台应用指南_v1.0.pdf
driver/                    Linux 内核模块，字符设备 /dev/pango_pci_driver
  id_config.h             VID / DID 定义
  pango_pci_driver.c / .h
pcie_test_rtl/             三个参考硬件工程（均为 PG2L100H / Logos-2）
  PG2L100H_PCIe_DMA          DMA Auto / DMA Manual / PIO Test
  PG2L100H_PCIe_performance  DMA 性能测试
  PG2L100H_PCIe_Tandem       通过 PCIe 加载位流（Tandem）
run.sh                     一键脚本：编译驱动 → 装载 → 编译上位机 → 运行
文件目录.txt                官方原始目录说明
```

## 关键参数

| 项目 | 值 |
|---|---|
| Vendor ID / Device ID | `0x0755` / `0x0755`（见 `driver/id_config.h`，必须与 FPGA 端 IP 一致） |
| 字符设备节点 | `/dev/pango_pci_driver` |
| ioctl 类型 | `'S'`，命令号 0~9 |
| 主要 ioctl | 2 `MAP_ADDR`、3 `WRITE_TO_KERNEL`、4 `DMA_READ`、5 `DMA_WRITE`、6 `READ_FROM_KERNEL`、7 `UMAP_ADDR`、8/9 `PERFORMANCE_START/END` |
| 上位机依赖 | `gtk+-2.0`、`glib-2.0`、`gcc`（`pkg-config` 取编译参数） |

## 用法

```sh
# 必须 root（脚本内会校验当前用户名为 root）
sudo sh run.sh
```

脚本流程：`make -C driver` → `insmod pango_pci_driver.ko` → `make -C app_pcie` → 运行 `app_pcie/build/app`。

界面起来后先看 **Endpoint Status** 页：Link Status / Speed / Width、BAR0~5 基地址与长度、MPS、MRRS ——
一页即可判断链路是否正常。

## 相对官方原版的本地适配

以下三个文件的时间戳为 2026-09-13，与包内其余文件（2021 年）不一致，属于本地适配改动：

| 文件 | 改动内容 |
|---|---|
| `run.sh` | 增加 `set -e` 失败即停、在 sudo 前预解析脚本路径、非 root 时自动 `sudo` 重入并透传 `DISPLAY` / `XAUTHORITY` —— 便于在 ARM 主机上通过 SSH 直接运行 |
| `driver/pango_pci_driver.c` | 对 `copy_from_user` / `copy_to_user` 的返回值做检查。厂家内核（Android/Rockchip BSP 一系）的 `scripts/gcc-wrapper.py` 会把任何 warning 升级为致命错误，未使用返回值会触发 `-Wunused-result` 导致编译中断、`.ko` 无法生成 |
| `app_pcie/sources/main.c` | 有本地修改（具体差异未与原始包逐行比对） |

未纳入版本控制：`app_pcie/build/`、`driver/` 下的编译中间产物，见 `.gitignore`。

## 注意事项

- **RTL 不通用**：`pcie_test_rtl/` 三个工程都是 **PG2L100H（Logos-2）**。紫光不同系列的 PCIe IP 互不兼容，
  搬到 **PGL50H（Logos 系列）** 上时只能参考，不能直接综合。真正通用的是 `driver/` 与 `app_pcie/`，
  前提是 FPGA 端把寄存器偏移和位定义对齐。
- **与 `fpga/pcie_video/` 的协议完全不同，别混用**：
  - 本包走**寄存器触发式 DMA**：`0x100` 写命令（长度 / 地址位宽 / 方向）、`0x110` 写地址低 32 位、`0x120` 写高 32 位；
  - 仓库内 `fpga/pcie_video/` 的 PGL50H 彩条工程是**自研协议**：`0x110` 连写 4 个缓冲区地址即提交、`0x130` 停止，
    没有命令寄存器，而且 `0x110` 在两套协议里含义**完全不同**。
    两者都会用 `0x110`，接错工程时表现是「打了命令但毫无反应」。
- 该平台默认**不使用中断**（IP 不支持），传输完成靠用户态延时/轮询。
- 驱动使用 `set_dma_mask()`（先试 64 位、失败退 32 位），对大内存 ARM 平台是好事。

## 相关

- 仓库内 `fpga/pcie_video/` — PGL50H 的 HDMI 彩条经 PCIe 传输工程（自研协议）
- 链路 bring-up 与排障流程见技能 `pango-pcie-arm-bringup`
