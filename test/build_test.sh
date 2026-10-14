#!/usr/bin/env bash
# Builds ./test_harness against the patched libfprint in ../driver/result (nothing is installed).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB="$(readlink -f "$HERE/../driver/result")"
[ -d "$LIB/lib" ] || { echo "build the driver first (nix-build, see driver/)"; exit 1; }
export HERE LIB
exec nix-shell -p gcc pkg-config glib gusb libgudev --run '
gcc -o "$HERE/test_harness" "$HERE/harness.c" -I"$LIB/include" -I"$LIB/include/libfprint-2" \
  $(pkg-config --cflags --libs gobject-2.0 gio-2.0) -L"$LIB/lib" -lfprint-2 -Wl,-rpath,"$LIB/lib"
echo built "$HERE/test_harness"
'
