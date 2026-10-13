#!/usr/bin/env bash
# Integration probes belong in a container with --network none and no device mappings.
set -eo pipefail
hardware_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$hardware_root/source.sh"
cmake_tool_directory="$(python3 -c 'import cmake; print(cmake.CMAKE_BIN_DIR)')"
export PATH="$cmake_tool_directory:$PATH"
cd "$hardware_root"
cmake -S tests/integration -B build/slam-input-tests
cmake --build build/slam-input-tests -j"${BUILD_JOBS:-2}"
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=90 build/slam-input-tests/scan_gate_probe
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=90 build/slam-input-tests/ceres_anchor_probe
python3 tests/integration/base_feedback_probe.py
python3 tests/integration/map_save_service_probe.py
python3 tests/integration/recording_probe.py
probe_directory="$(mktemp -d /tmp/duojin01-slam-replay-XXXXXX)"
python3 tests/integration/slam_replay_probe.py "$probe_directory"
echo "SLAM integration checks passed; replay evidence: $probe_directory"
