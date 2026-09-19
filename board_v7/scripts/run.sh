#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
cd "$PROJECT_DIR"

# Support both the existing board layout and a fresh repository clone.
MODEL=models/unified21_p2_v7_640_int8.rknn
if [ ! -f "$MODEL" ]; then
  MODEL=../models/unified21_p2_v7_640_int8.rknn
fi

# With no command-line arguments, open the bundled demonstration video so the
# dashboard is immediately available. The user can press O to choose another
# video. Passing --source still overrides this default.
if [ "$#" -eq 0 ]; then
  set -- --source samples/project_video.mp4
fi

exec taskset -c 2,3 ./build/adas_demo --model "$MODEL" --cpu-threads 2 "$@"
