#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
# 与回放使用相同模型。SCENE_DIR 只用于可选 AVM 元数据，选灯不依赖场景配置。
# 环境变量或末尾参数仍可覆盖；采集保持 read，显示默认不限帧率。
if [ -z "${MODEL:-}" ]; then
  MODEL=models/yolo14_960x544_qat_int8.rknn
  [ -f "$MODEL" ] || MODEL=../models/yolo14_960x544_qat_int8.rknn
fi
if [ -z "${UFLD_MODEL:-}" ]; then
  UFLD_MODEL=models/ufldv2_culane_res18_1600x320_int8.rknn
  [ -f "$UFLD_MODEL" ] || UFLD_MODEL=../models/ufldv2_culane_res18_1600x320_int8.rknn
fi
SCENE_DIR=${SCENE_DIR:-four_view_sample/candidate_nvidia_010722d8_near_left_straight_20261013}
PCIE_DEVICE=${PCIE_DEVICE:-}
test -f "$MODEL" || { echo "Missing YOLO model: $MODEL" >&2; exit 1; }
# pcie_v2 的节点名带 PCI 地址，采集类固定走 read。
# 多块设备时要求显式指定，避免选错板卡。
if [ -z "$PCIE_DEVICE" ]; then
  for device in /dev/fpga_frame_*; do
    [ -c "$device" ] || continue
    if [ -n "$PCIE_DEVICE" ]; then
      echo "Multiple PCIe devices; set PCIE_DEVICE=/dev/fpga_frame_<BDF>." >&2
      exit 1
    fi
    PCIE_DEVICE=$device
  done
fi
[ -n "$PCIE_DEVICE" ] && [ -c "$PCIE_DEVICE" ] || {
  echo "PCIe device not found; load the pcie_v2 driver in driver/ first." >&2
  exit 1
}
[ -r "$PCIE_DEVICE" ] || {
  echo "No read permission: $PCIE_DEVICE" >&2
  exit 1
}
# Affinity is set inside the application before RKNN creates helper threads.
# Only the independent PCIe worker moves to CPU 0,1.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
exec ./build/adas_four_view --pcie "$PCIE_DEVICE" --model "$MODEL" \
  --ufld-model "$UFLD_MODEL" --scene "$SCENE_DIR" --cpu-threads 1 \
  --detect-every 1 --no-lanes --no-ttc --display-fps 0 "$@"
