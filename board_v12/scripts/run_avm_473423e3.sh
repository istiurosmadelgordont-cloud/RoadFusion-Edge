#!/bin/sh
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"

# Match the seven-camera 473423e3 clip; other clips need their own calibration.
avm_scene=four_view_sample/avm_473423e3
for asset in avm.yml sync.yml rear.mp4 rear_left.mp4 rear_right.mp4 front_tele.mp4; do
  [ -s "$avm_scene/$asset" ] || {
    echo "Missing AVM asset: $avm_scene/$asset" >&2
    exit 1
  }
done

export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-/home/cat/.Xauthority}
exec sh ./scripts/run_pcie.sh \
  --enable-lanes --enable-fcw --pcie-camera-order left,front,right,rear \
  --avm-local-scene "$avm_scene" "$@"
