#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
cd "$PROJECT_DIR"

# Prefer the deployment layout used on the board, then the repository layout.
MODEL_FILE="models/unified17_v10_candidate_640_int8.rknn"
if [ ! -f "$MODEL_FILE" ]; then
  MODEL_FILE="../models/unified17_v10_candidate_640_int8.rknn"
fi

# A deployed board contains the bundled demonstration video. Repository clones
# should pass --source explicitly because test videos are intentionally ignored.
if [ "$#" -eq 0 ]; then
  if [ -f samples/project_video.mp4 ]; then
    set -- --source samples/project_video.mp4
  else
    echo "Usage: $0 --source FILE|INDEX [other options]" >&2
    exit 2
  fi
fi

exec ./build/adas_demo --model "$MODEL_FILE" "$@"
