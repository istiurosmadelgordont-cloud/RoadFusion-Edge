#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
cd "$PROJECT_DIR"

# 当前部署优先从 board_v8/models 加载 YOLO QAT INT8；否则查找共享 models。
# MO 固定从 YOLO 同目录加载 mo_fp16_baseline.rknn，两个文件必须放在一起。
# 车道线与 TTC 当前默认关闭；命令行追加参数可覆盖默认场景和调度配置。
MODEL=models/yolo14_960x544_qat_int8.rknn
if [ ! -f "$MODEL" ]; then
  MODEL=../models/yolo14_960x544_qat_int8.rknn
fi

UFLD_MODEL=models/ufldv2_culane_res18_1600x320_int8.rknn
if [ ! -f "$UFLD_MODEL" ]; then
  UFLD_MODEL=../models/ufldv2_culane_res18_1600x320_int8.rknn
fi

exec taskset -c 2,3 ./build/adas_four_view --model "$MODEL" --ufld-model "$UFLD_MODEL" \
  --scene four_view_sample/signal_ru_20261010 --cpu-threads 1 --detect-every 3 "$@"
