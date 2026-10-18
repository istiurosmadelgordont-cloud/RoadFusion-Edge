#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
mkdir -p "$PROJECT_DIR/build"
cd "$PROJECT_DIR/build"
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -- -j2
echo "Built: $PROJECT_DIR/build/adas_demo"
