#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
MODEL=${MODEL:-models/unified21_p2_v7_640_int8.rknn}
UFLD_MODEL=${UFLD_MODEL:-models/ufldv2_culane_res18_1600x320_int8.rknn}
PCIE_DEVICE=${PCIE_DEVICE:-/dev/pcie_hdmi_host}
test -f "$MODEL" || { echo "Missing YOLO model: $MODEL" >&2; exit 1; }
test -f "$UFLD_MODEL" || { echo "Missing 1600 UFLD model: $UFLD_MODEL" >&2; exit 1; }
# Affinity is set inside the application before RKNN creates helper threads.
# Only the independent PCIe worker moves to CPU 0,1.
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
exec ./build/adas_four_view --pcie "$PCIE_DEVICE" --model "$MODEL" \
  --ufld-model "$UFLD_MODEL" --ufld-rear --cpu-threads 1 \
  --detect-every 6 --ufld-every 2 --display-fps 15 "$@"
