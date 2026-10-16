#!/usr/bin/env bash
# 在已安装 Qt Widgets 开发包的 Linux 中运行；无需 FPGA、无需 root。
set -euo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
python3 "$project_dir/tests/driver_control_regression.py"
python3 "$project_dir/tests/launcher_regression.py"
build_dir=$(mktemp -d /tmp/pcie-control-test.XXXXXX)
# 临时目录仅保留编译产物，失败时也便于查看，不触碰工程源文件。
pkg=${QT_WIDGETS_PKG:-Qt5Widgets}
# Qt5 的 -reduce-relocations 要求 PIC；Qt6 也可使用对应 pkg-config 包。
${CXX:-c++} -std=c++17 -fPIC -O2 -Wall -Wextra -Wpedantic \
    $(pkg-config --cflags "$pkg") "$project_dir/tests/qt_control_regression.cpp" \
    -o "$build_dir/qt-control-test" $(pkg-config --libs "$pkg") -pthread \
    -Wl,--wrap=open -Wl,--wrap=close -Wl,--wrap=read -Wl,--wrap=ioctl \
    -Wl,--wrap=mmap -Wl,--wrap=munmap
QT_QPA_PLATFORM=offscreen timeout 15s "$build_dir/qt-control-test"
bash -n "$project_dir/run.sh"
echo 'PASS: driver, Qt and shell regression'
