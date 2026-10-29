#!/bin/sh
# Prepare the driver and print only the selected device on stdout.
set -eu
PATH=/usr/sbin:/sbin:$PATH
export PATH
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
driver_dir=$PROJECT_DIR/driver
module=$driver_dir/my_pciedriver.ko
required_version=8.2

fail() { printf '%s\n' "$*" >&2; exit 1; }
as_root() {
  if [ "$(id -u)" -eq 0 ]; then "$@"; else sudo -- "$@"; fi
}
check_loaded_version() {
  loaded_version=$(cat /sys/module/my_pciedriver/version 2>/dev/null || true)
  [ "$loaded_version" = "$required_version" ] ||
    fail "Loaded my_pciedriver version is ${loaded_version:-unknown}; board_v12 requires $required_version. Exit capture programs and unload the old driver first."
}

if [ -d /sys/module/my_pciedriver ]; then
  check_loaded_version
  printf 'PCIe: reusing loaded driver %s.\n' "$required_version" >&2
else
  command -v modinfo >/dev/null 2>&1 || fail 'Missing modinfo; install kmod.'
  command -v insmod >/dev/null 2>&1 || fail 'Missing insmod; install kmod.'
  # Serialize first-time builds/loads from this project. Never unload a live driver.
  command -v flock >/dev/null 2>&1 || fail 'Missing flock; install util-linux.'
  exec 9>"$driver_dir/.load.lock"
  flock -w 60 9 || fail 'Another driver preparation is still running.'
  if [ ! -d /sys/module/my_pciedriver ]; then
    kernel=$(uname -r)
    rebuild=false
    if [ ! -f "$module" ]; then
      rebuild=true
    else
      module_kernel=$(modinfo -F vermagic "$module" 2>/dev/null | cut -d ' ' -f 1)
      module_version=$(modinfo -F version "$module" 2>/dev/null || true)
      if [ "$module_kernel" != "$kernel" ] || [ "$module_version" != "$required_version" ] ||
         [ -n "$(find "$driver_dir" -maxdepth 1 -type f \( -name '*.c' -o -name '*.h' -o -name Makefile \) -newer "$module" -print)" ]; then
        rebuild=true
      fi
    fi
    if [ "$rebuild" = true ]; then
      kernel_dir=${KERNEL_DIR:-/lib/modules/$kernel/build}
      if [ ! -f "$kernel_dir/Makefile" ] && [ -z "${KERNEL_DIR:-}" ]; then
        kernel_dir=/usr/src/linux-headers-$kernel
      fi
      [ -f "$kernel_dir/Makefile" ] || fail "Missing kernel headers for $kernel; set KERNEL_DIR to the matching headers directory."
      printf 'PCIe: building driver for %s...\n' "$kernel" >&2
      make -C "$driver_dir" KERNEL_DIR="$kernel_dir" -j2 >&2
    fi
    module_kernel=$(modinfo -F vermagic "$module" | cut -d ' ' -f 1)
    [ "$module_kernel" = "$kernel" ] || fail "Driver kernel mismatch: $module_kernel / $kernel."
    [ "$(modinfo -F version "$module")" = "$required_version" ] || fail 'Driver version mismatch after build.'
    dependencies=$(modinfo -F depends "$module" | tr ',' ' ')
    for dependency in $dependencies; do as_root modprobe "$dependency" >&2; done
    printf 'PCIe: loading %s...\n' "$module" >&2
    if ! as_root insmod "$module" >&2; then
      [ -d /sys/module/my_pciedriver ] || fail 'Failed to load PCIe driver; see the kernel/module error above.'
    fi
  fi
  check_loaded_version
  flock -u 9
  exec 9>&-
fi

# Wait for devtmpfs/udev after insertion, including device ownership rules.
if command -v udevadm >/dev/null 2>&1; then udevadm settle --timeout=5 || true; fi
device=${PCIE_DEVICE:-}
if [ -n "$device" ]; then
  case "$device" in /dev/fpga_frame_*) ;; *) fail 'PCIE_DEVICE must be /dev/fpga_frame_<BDF>.' ;; esac
else
  for candidate in /dev/fpga_frame_*; do
    [ -c "$candidate" ] || continue
    [ -z "$device" ] || fail 'Multiple PCIe devices; set PCIE_DEVICE=/dev/fpga_frame_<BDF>.'
    device=$candidate
  done
fi
[ -n "$device" ] && [ -c "$device" ] || fail 'No FPGA capture device found after loading the driver; check PCIe enumeration and board connection.'
if [ ! -r "$device" ]; then
  as_root chgrp "$(id -gn)" "$device" >&2
  as_root chmod g+r "$device" >&2
fi
[ -r "$device" ] || fail "PCIe device is not readable: $device"

# RGA and RKNN use the board's platform drivers; load RGA if provided as a module.
if [ ! -c /dev/rga ]; then
  as_root modprobe rga >&2 || fail 'RGA device is missing and its platform driver could not be loaded.'
  if command -v udevadm >/dev/null 2>&1; then udevadm settle --timeout=5 || true; fi
fi
[ -c /dev/rga ] || fail 'RGA driver did not create /dev/rga.'
if [ ! -r /dev/rga ] || [ ! -w /dev/rga ]; then
  as_root chgrp "$(id -gn)" /dev/rga >&2
  as_root chmod g+rw /dev/rga >&2
fi
[ -r /dev/rga ] && [ -w /dev/rga ] || fail 'RGA device is not accessible.'
if [ ! -d /sys/module/rknpu ]; then
  as_root modprobe rknpu >&2 || fail 'RKNN NPU platform driver is unavailable.'
fi
[ -d /sys/module/rknpu ] || fail 'RKNN NPU driver did not initialize.'
printf '%s\n' "$device"
