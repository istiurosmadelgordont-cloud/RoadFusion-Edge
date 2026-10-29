#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PROJECT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
mkdir -p "$PROJECT_DIR/build"
cd "$PROJECT_DIR/build"
# Moving the board_v12 folder must not retain an absolute source path in CMake.
if [ -f CMakeCache.txt ] && ! grep -Fqx "CMAKE_HOME_DIRECTORY:INTERNAL=$PROJECT_DIR" CMakeCache.txt; then
  mv CMakeCache.txt "CMakeCache.before-move-$(date +%Y%m%d-%H%M%S).txt"
fi
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -- -j2
echo "Built: $PROJECT_DIR/build/adas_four_view"
