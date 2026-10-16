#!/usr/bin/env bash
# 在板子的桌面终端执行：bash run.sh [设备节点]
# 编译驱动和 Qt -> 加载驱动 -> 设置节点权限 -> 启动 Qt。
# 不保存密码；仅加载模块和调整权限时用 sudo，Qt 仍以当前用户运行。
set -euo pipefail

die() { printf '\n错误：%s\n' "$*" >&2; exit 1; }

build_only=0
device=""
for arg in "$@"; do
    case "$arg" in
        -h|--help)
            printf '%s\n' \
                '用法：bash run.sh [--build-only] [/dev/fpga_frame_<BDF>]' \
                '默认：编译驱动和上位机、加载驱动、打开窗口。' \
                '--build-only：只编译，不加载驱动、不修改权限、不启动窗口。' \
                'JOBS=2 设置并行任务数；KERNEL_DIR=/path/to/kernel 指定内核目录。' \
                '重复运行复用源版本标识一致的已加载模块；不会自动卸载模块。'
            exit 0 ;;
        --build-only) build_only=1 ;;
        /dev/fpga_frame_*)
            [[ -z "$device" ]] || die '最多指定一个设备节点。'
            device="$arg" ;;
        *) die "不支持的参数：$arg；使用 --help 查看用法。" ;;
    esac
done

[[ "$(uname -s)" == Linux && "$(uname -m)" == aarch64 ]] ||
    die '此启动脚本用于 ARM64 Linux 板子；交叉编译请单独使用 Makefile。'
(( EUID != 0 )) || die '请以普通桌面用户执行 bash run.sh，不要使用 sudo bash run.sh。'

# 从脚本所在位置定位工程，允许从其他工作目录启动。
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="$project_dir/build"
kernel_release="$(uname -r)"
kernel_dir="${KERNEL_DIR:-/lib/modules/$kernel_release/build}"
jobs="${JOBS:-2}"
module_name=my_pciedriver
module_file="$project_dir/$module_name.ko"
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || die 'JOBS 必须是正整数。'

# Debian 普通用户 PATH 可能不含 /usr/sbin，其中有 modinfo、insmod。
export PATH="$PATH:/usr/sbin:/sbin"
for tool in make gcc g++ cmake modinfo; do
    command -v "$tool" >/dev/null 2>&1 ||
        die "缺少 $tool。Debian/Ubuntu 可安装：sudo apt install build-essential cmake qtbase5-dev kmod"
done
for file in Makefile my_pciedriver.c CMakeLists.txt main.cpp fpga_uapi.h image_protocol.h; do
    [[ -f "$project_dir/$file" ]] || die "工程缺少 $file。"
done
for file in Makefile .config Module.symvers include/generated/autoconf.h include/generated/utsrelease.h; do
    [[ -f "$kernel_dir/$file" ]] || die "内核构建环境缺少 $kernel_dir/$file。"
done

printf '\n[1/5] 编译驱动（内核 %s）……\n' "$kernel_release"
# 命令行参数覆盖旧环境中的交叉编译设置，明确在板子上本机编译。
make -C "$project_dir" KERNEL_DIR="$kernel_dir" ARCH=arm64 CROSS_COMPILE= -j"$jobs" ||
    die '驱动编译失败，请查看上面的错误。'
[[ -f "$module_file" ]] || die "没有生成 $module_file。"
vermagic="$(modinfo -F vermagic "$module_file")"
[[ "${vermagic%% *}" == "$kernel_release" ]] ||
    die "模块内核版本与当前内核不符：$vermagic；请使用匹配的内核构建目录。"

printf '\n[2/5] 编译 Qt 上位机……\n'
cmake -S "$project_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release ||
    die 'CMake 配置失败；若缺少 Qt 5 开发包，可执行 sudo apt install qtbase5-dev。'
# 掉电或传输中断可能留下时间戳较新、大小却为 0 的产物，增量构建会误判。
# 发现这种产物时清理 Qt 构建目标后重建，不影响源文件和驱动模块。
qt_clean=()
for artifact in "$build_dir/pcie_viewer" "$build_dir/CMakeFiles/pcie_viewer.dir/main.cpp.o"; do
    if [[ -f "$artifact" && ! -s "$artifact" ]]; then
        printf '发现空的 Qt 构建产物，执行完整重建：%s\n' "$artifact"
        qt_clean=(--clean-first)
        break
    fi
done
cmake --build "$build_dir" "${qt_clean[@]}" --parallel "$jobs" || die '上位机编译失败。'
[[ -s "$build_dir/pcie_viewer" && -x "$build_dir/pcie_viewer" ]] ||
    die 'Qt 可执行文件为空或不可执行；请重新构建，不要直接运行损坏的产物。'
if (( build_only )); then
    printf '\n编译完成：\n  %s\n  %s\n' "$module_file" "$build_dir/pcie_viewer"
    exit 0
fi

# 从桌面终端运行时环境已设置好；不擅自选择其他用户的桌面。
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}${QT_QPA_PLATFORM:-}" ]] ||
    die '编译成功，但没有图形环境。请在板子的桌面终端运行，或使用 --build-only。'
command -v sudo >/dev/null 2>&1 || die '缺少 sudo，无法加载驱动。'

printf '\n[3/5] 加载驱动……\n'
printf '驱动 7.0 对应 20261015 FPGA：参数单向提交，视频 ABI 保持 2。\n'
if [[ -d "/sys/module/$module_name" ]]; then
    # 不自动 rmmod：模块可能仍有用户、DMA 或超时后保留的引用。
    disk_version="$(modinfo -F srcversion "$module_file")"
    live_version="$(cat "/sys/module/$module_name/srcversion" 2>/dev/null || true)"
    if [[ -z "$disk_version" || "$disk_version" != "$live_version" ]]; then
        die '已加载模块与刚编译的源版本不同或无法确认。请先停止采集并关闭旧上位机，再执行 sudo rmmod my_pciedriver，成功后重新运行。若运行的是旧 6.0 且曾启用配置 DMA，需重启主机并可靠复位 FPGA；不要强制卸载。'
    fi
    printf '模块已加载，源版本标识一致，继续使用。\n'
else
    # insmod 成功仅代表模块已注册，仍需检查 probe 是否生成设备节点。
    sudo insmod "$module_file" ||
        die '驱动加载失败。请执行 sudo dmesg | tail -n 60 查看原因。'
fi

printf '\n[4/5] 检查设备并设置访问权限……\n'
command -v udevadm >/dev/null 2>&1 && udevadm settle --timeout=5 || true
shopt -s nullglob
nodes=(/dev/fpga_frame_*)
if [[ -n "$device" ]]; then
    [[ -c "$device" ]] || die "未找到设备 $device。请检查 FPGA 连接和驱动日志。"
    nodes=("$device")
fi

if (( ${#nodes[@]} == 0 )); then
    found_fpga=0
    for pci in /sys/bus/pci/devices/*; do
        if [[ "$(cat "$pci/vendor")" == 0x0755 && "$(cat "$pci/device")" == 0x0755 ]]; then
            found_fpga=1
        fi
    done
    if (( found_fpga )); then
        printf '检测到 FPGA，但没有设备节点；请检查是否被其他驱动占用或 probe 失败。\n'
        printf '查看日志：sudo dmesg | tail -n 60\n'
    else
        printf '未枚举到 FPGA（0755:0755），请检查供电、连接和 PCIe 链路。\n'
    fi
    printf '仍会打开上位机界面；当前无法开始采集。\n'
fi

for node in "${nodes[@]}"; do
    # 只处理本驱动真实注册的节点，不对任意路径执行提权 chmod。
    name="${node##*/}"
    [[ "$node" == "/dev/$name" && ! -L "$node" && -c "$node" ]] ||
        die "设备路径不符合要求：$node"
    bdf="${name#fpga_frame_}"
    [[ "$bdf" =~ ^[[:xdigit:]]{4}:[[:xdigit:]]{2}:[[:xdigit:]]{2}\.[0-7]$ ]] ||
        die "设备名称不符合要求：$name"
    owner="$(readlink -f "/sys/bus/pci/devices/$bdf/driver/module" || true)"
    [[ "$owner" == "/sys/module/$module_name" && -f "/sys/class/misc/$name/dev" ]] ||
        die "设备不属于当前驱动：$node"
    IFS=: read -r major minor < "/sys/class/misc/$name/dev"
    expected_dev="$(printf '%x:%x' "$major" "$minor")"
    [[ "$(stat -c '%t:%T' "$node")" == "$expected_dev" ]] || die "设备号不匹配：$node"
    if [[ ! -r "$node" ]]; then
        # 上位机 O_RDONLY（含 mmap/ioctl）只需读权限，授予当前用户主组。
        # 权限随节点删除而失效，脚本下次运行时重新检查。
        sudo chgrp -- "$(id -g)" "$node"
        sudo chmod 0640 -- "$node"
        [[ -r "$node" ]] || die "当前用户仍无权读取 $node。"
    fi
done

printf '\n[5/5] 启动上位机……\n'
# 退出 Qt 后保留模块，下次可直接复用；不在退出时自动卸载。
if [[ -n "$device" ]]; then
    exec "$build_dir/pcie_viewer" "$device"
else
    exec "$build_dir/pcie_viewer"
fi
