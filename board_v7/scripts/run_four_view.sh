#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
cd "$PROJECT_DIR"

# Board deployments traditionally keep the model below board_v7/models. A
# fresh repository clone keeps the shared artifact in ../models instead.
MODEL=models/unified21_p2_v7_640_int8.rknn
if [ ! -f "$MODEL" ]; then
  MODEL=../models/unified21_p2_v7_640_int8.rknn
fi

exec taskset -c 2,3 ./build/adas_four_view --model "$MODEL" \
  --scene four_view_sample/66b5fa4b_30fps --cpu-threads 2 --detect-every 6 --software-decode "$@"
