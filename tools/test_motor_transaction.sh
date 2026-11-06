#!/usr/bin/env bash
# Hardware-free test using pinned SDK headers, never the SDK transport library.
set -euo pipefail
ROOT=$(dirname "$(dirname "$(realpath "$0")")")
SDK=$(realpath "${1:?Usage: bash tools/test_motor_transaction.sh /path/to/CANdle-SDK}")
BUILD=$(mktemp -d)
trap 'rm -rf "$BUILD"' EXIT
INCLUDES=()
while IFS= read -r -d '' directory; do
  INCLUDES+=(-isystem "$directory")
done < <(find "$SDK/candlelib" "$SDK/commons" -type d -print0)
"${CXX:-g++}" -std=c++17 -O3 -DNDEBUG -ffunction-sections -fdata-sections \
  -I"$ROOT/src" "${INCLUDES[@]}" -isystem /usr/include/libusb-1.0 \
  "$ROOT/tools/test_motor_transaction.cpp" "$ROOT/src/comm/candle_motor.cpp" \
  "$SDK/commons/logger/logger.cpp" -pthread -Wl,--gc-sections \
  -o "$BUILD/motor_transaction_test"
timeout 30s "$BUILD/motor_transaction_test"
