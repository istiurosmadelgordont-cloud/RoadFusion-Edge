#!/bin/sh
set -eu
# Compatibility entry for the original board launch command.
exec sh "$(dirname -- "$0")/run_pcie.sh" "$@"
