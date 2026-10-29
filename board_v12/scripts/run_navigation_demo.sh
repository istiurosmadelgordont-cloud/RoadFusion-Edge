#!/bin/sh
set -eu
# Navigation direction/distance are explicitly simulated; video stays on PCIe.
exec sh "$(dirname -- "$0")/run_pcie.sh" --demo-navigation --no-test4 "$@"
