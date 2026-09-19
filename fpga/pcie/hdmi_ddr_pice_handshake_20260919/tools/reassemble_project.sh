#!/bin/sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
package_root=$(dirname -- "$script_dir")
output="$package_root/hdmi_ddr_pice_handshake_20260919.zip"
expected="1766b7285fcafec8c9e39230b77e2e83ce771b5f0e5c773dfb33d1f14680c576"

set -- "$package_root"/project_parts/hdmi_ddr_pice_handshake_20260919.zip.part*
if [ "$#" -ne 28 ]; then
    echo "Expected 28 archive parts, found $#." >&2
    exit 1
fi

cat "$@" > "$output"
printf '%s  %s\n' "$expected" "$output" | sha256sum -c -
echo "Created: $output"

