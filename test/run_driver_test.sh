#!/usr/bin/env bash
# Enroll one finger with the new open-source driver, then try to verify it several times.
# Stop fprintd first (systemctl stop fprintd); the test talks to the reader directly.
# Usage: ./run_driver_test.sh [verify_attempts]   (default 6)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
[ -x ./test_harness ] || ./build_test.sh
echo "ENROLL: press the SAME finger flat on the sensor, hold ~1-2 s until it prints progress,"
echo "lift, wait a second, press again (slightly different spot each time). Repeat until"
echo "it says 'enroll finished OK'. Then VERIFY: touch the same finger again each time it asks."
sleep 2
exec ./test_harness enroll 150 verify "${1:-6}"
