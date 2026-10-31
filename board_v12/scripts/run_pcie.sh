#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
# 与回放使用相同模型。SCENE_DIR 只用于可选 AVM 元数据，选灯不依赖场景配置。
# board_v12 PCIe 采集和预览固定使用 DMA-BUF + RGA。
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
PCIE_BUFFERS=${PCIE_BUFFERS:-6}
test -f "$MODEL" || { echo "Missing YOLO model: $MODEL" >&2; exit 1; }
test -x ./build/adas_four_view || {
  echo "Missing executable; run sh scripts/build.sh first." >&2
  exit 1
}
# 自动准备 8.2 模块和设备权限；已加载时复用，不卸载正在使用的驱动。
PCIE_DEVICE=$(PCIE_DEVICE="$PCIE_DEVICE" sh ./scripts/prepare_pcie.sh)
# Affinity is set inside the application before RKNN creates helper threads.
# Only the independent PCIe worker moves to CPU 0,1.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
# 保持此前实测的 NPU 工作频率；重启后驱动默认最低 200MHz 会降低检测吞吐。
# 只在支持 800MHz、上限允许且当前最低频率更低时恢复，不改变 governor/最高频率。
adas_npu_freq=/sys/class/devfreq/fde40000.npu
if [ -r "$adas_npu_freq/min_freq" ] && [ -r "$adas_npu_freq/max_freq" ] && \
   [ -r "$adas_npu_freq/available_frequencies" ] && \
   grep -qw 800000000 "$adas_npu_freq/available_frequencies" && \
   [ "$(cat "$adas_npu_freq/max_freq")" -ge 800000000 ] && \
   [ "$(cat "$adas_npu_freq/min_freq")" -lt 800000000 ]; then
  if ! printf 800000000 | sudo -n tee "$adas_npu_freq/min_freq" >/dev/null 2>&1; then
    echo "NPU minimum frequency unchanged: cannot restore 800MHz." >&2
  fi
fi
exec ./build/adas_four_view --pcie "$PCIE_DEVICE" --pcie-buffers "$PCIE_BUFFERS" --model "$MODEL" \
  --ufld-model "$UFLD_MODEL" --scene "$SCENE_DIR" --cpu-threads 1 \
  --detect-every 1 --npu-workers 2 --no-lanes --no-ttc --display-fps 0 --demo-navigation "$@"
