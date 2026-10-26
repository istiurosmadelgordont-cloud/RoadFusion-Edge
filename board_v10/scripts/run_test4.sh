#!/bin/sh
set -eu
# Same models, PCIe transport and Test One/Two controls; only enable Test Four.
exec sh "$(dirname -- "$0")/run_pcie.sh" --enable-test4 "$@"
