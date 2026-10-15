#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
cd "$PROJECT_DIR"

# Board deployments traditionally keep the model below board_v7/models. A
# fresh repository clone keeps the shared artifact in ../models instead.
MODEL=models/yolo14_960x544_qat_int8.rknn
if [ ! -f "$MODEL" ]; then
  MODEL=../models/yolo14_960x544_qat_int8.rknn
fi

UFLD_MODEL=models/ufldv2_culane_res18_1600x320_int8.rknn
if [ ! -f "$UFLD_MODEL" ]; then
  UFLD_MODEL=../models/ufldv2_culane_res18_1600x320_int8.rknn
fi

exec taskset -c 2,3 ./build/adas_four_view --model "$MODEL" --ufld-model "$UFLD_MODEL" \
  --scene four_view_sample/candidate_nvidia_010722d8_near_left_straight_20261013 --cpu-threads 1 --detect-every 1 --no-lanes --no-ttc "$@"
