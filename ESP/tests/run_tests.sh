#!/bin/sh
# Host tests for the handheld IMU math and the radio packet. Run from anywhere: sh ESP/tests/run_tests.sh
cd "$(dirname "$0")" || exit 1
cmp ../handheld/imu_link.h ../turret/imu_link.h || { echo "imu_link.h differs between handheld and turret: copy one over the other"; exit 1; }
g++ -std=c++17 -O1 -Wall -Wextra -o "${TMPDIR:-/tmp}/test_aim_math" test_aim_math.cpp && "${TMPDIR:-/tmp}/test_aim_math"
