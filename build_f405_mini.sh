#!/usr/bin/env bash
set -euo pipefail
firmware_dir=$(cd "$(dirname "$0")" && pwd)
fsp_target=${FSP_TARGET_HZ:-600}
case "$fsp_target" in
    300|600|1100) ;;
    *) echo 'FSP_TARGET_HZ must be 300, 600, or 1100' >&2; exit 1 ;;
esac
imu_filter_flag="-DENABLE_PT2_FILTER_IMU=0"
case "${ENABLE_PT2_FILTER_IMU:-1}" in
    0) ;;
    1) imu_filter_flag="-DENABLE_PT2_FILTER_IMU=1" ;;
    *) echo 'ENABLE_PT2_FILTER_IMU must be 0 or 1' >&2; exit 1 ;;
esac
compiler_version=$(arm-none-eabi-gcc -dumpversion)
make -C "$firmware_dir" -j"${BUILD_JOBS:-8}" CONFIG=SPEEDYBEEF405MINI \
    GCC_REQUIRED_VERSION="$compiler_version" \
    EXTRA_FLAGS="-DCLOUD_BUILD -DUSE_DSHOT -DUSE_SERIALRX -DUSE_SERIALRX_CRSF -DUSE_TELEMETRY -DUSE_TELEMETRY_CRSF -DFSP_PERIOD_HZ=$fsp_target $imu_filter_flag"
