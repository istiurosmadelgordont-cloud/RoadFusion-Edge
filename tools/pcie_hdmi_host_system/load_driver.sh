#!/bin/sh
# ---------------------------------------------------------------------------
# 加载 PCIe HDMI 采集驱动，并确认设备节点可用。
#
# 这个脚本刻意**不抢占**旧驱动：如果检测到 pango_pci_driver 或
# pango_video_test 已经加载，直接报错退出，让用户自己决定怎么处理。
# 原因是这三套驱动盯的是同一块 0755:0755 设备，而且各自会持有总线主控、
# 持续往自己的缓冲区写；强行抢占可能让 FPGA 写到已经没人管的内存。
# 正确做法是重启机器，开机时只加载需要的那一个。
# ---------------------------------------------------------------------------
set -eu

cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"

# 需要 root 才能 insmod；非 root 时自动提权重入
if [ "$(id -u)" -ne 0 ]; then exec sudo sh ./load_driver.sh "$@"; fi

# 旧驱动仍在时不做任何抢占，直接失败退出
for old in pango_pci_driver pango_video_test; do
    if [ -d "/sys/module/$old" ]; then
        echo "旧驱动 $old 已加载。关闭旧程序，重启后直接运行本项目，不运行旧 run.sh。" >&2
        exit 1
    fi
done

# 本驱动尚未加载才编译+装载，避免重复加载或覆盖已在运行的模块
if [ ! -d /sys/module/pcie_hdmi_host ]; then
    make -C driver
    insmod ./driver/pcie_hdmi_host.ko
fi

# 最后确认字符设备真的出现了——只有 probe 成功并绑上设备才会创建它
if [ ! -c /dev/pcie_hdmi_host ]; then
    echo "未绑定图像设备：检查 lspci -nnk、0755:0755、BAR1 及 dmesg。" >&2
    exit 1
fi
echo "设备就绪：/dev/pcie_hdmi_host"
