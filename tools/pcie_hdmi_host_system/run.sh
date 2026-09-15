#!/bin/sh
# ---------------------------------------------------------------------------
# 一键启动脚本：加载驱动 → 启动上位机
#
#   ./run.sh --demo      演示模式（合成彩条，不需要 root/FPGA/驱动）
#   ./run.sh             硬件模式（自动 sudo，加载驱动后读 PCIe）
#   ./run.sh --headless --count 3    无显示器时批量采集
# ---------------------------------------------------------------------------
set -eu

# 先切到脚本所在目录，之后所有相对路径（driver/、captures/）都以这里为基准
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"

# --demo provides a GUI without root, FPGA or kernel-module installation.
if [ "${1:-}" = "--demo" ]; then
    shift
    exec python3 ./host.py --source demo "$@"
fi

# 图形界面要读 X11，而 sudo 默认会清掉 DISPLAY/XAUTHORITY。
# 因此这里显式把当前用户的显示环境透传进去，让 MobaXterm / SSH 会话下也能弹出窗口。
if [ "$(id -u)" -ne 0 ]; then
    exec sudo env DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" sh ./run.sh "$@"
fi

# 驱动必须先就绪，否则 host.py 打开 /dev/pcie_hdmi_host 会失败
sh ./load_driver.sh
exec python3 ./host.py "$@"
