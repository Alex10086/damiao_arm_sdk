#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"

# Build only the read-only calibration tool; never configure a CAN interface here.
cmake -S "$ROOT_DIR" -B "$BUILD_DIR"
cmake --build "$BUILD_DIR" --target calibrate_joint_limits -j2

HAS_CONFIG=false
for arg in "$@"; do
    if [[ "$arg" == "--config" ]]; then
        HAS_CONFIG=true
        break
    fi
done
if "$HAS_CONFIG"; then
    exec "$BUILD_DIR/calibrate_joint_limits" "$@"
else
    exec "$BUILD_DIR/calibrate_joint_limits" --config "$ROOT_DIR/config/arm.yaml" "$@"
fi
