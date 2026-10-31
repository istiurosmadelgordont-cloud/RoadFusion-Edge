#!/bin/sh
set -eu

SCENE=${1:?usage: build_fpga_composite.sh SCENE_DIR}
OUT="$SCENE/sync_720p.mp4"

ffmpeg -hide_banner -loglevel warning -y \
  -i "$SCENE/front.mp4" -i "$SCENE/rear.mp4" \
  -i "$SCENE/left.mp4" -i "$SCENE/right.mp4" \
  -filter_complex \
  '[0:v][1:v]hstack[top];[2:v][3:v]hstack[bottom];[top][bottom]vstack[v]' \
  -map '[v]' -an -r 30 -c:v libx264 -preset ultrafast -crf 20 \
  -pix_fmt yuv420p -movflags +faststart "$OUT"

echo "$OUT"
